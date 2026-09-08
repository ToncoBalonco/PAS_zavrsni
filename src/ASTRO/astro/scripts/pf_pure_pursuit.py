#!/usr/bin/env python3
"""
Potential Field Pure Pursuit
=============================
Novi pristup lokalnom upravljanju robotom koji kombinira:

    1) PURE PURSUIT (privlačna komponenta)
       - prati globalnu putanju iz A* planera (/plan)
       - traži lookahead točku i računa smjer prema njoj
         u lokalnom (robotovom) koordinatnom sustavu

    2) POTENCIJALNA POLJA (odbojna komponenta)
       - čita /scan direktno (bez dynamic_ob_layer/costmapa)
       - gleda SAMO prednji konus zraka (± pf_front_angle_deg/2
         oko smjera "naprijed", nakon scan_angle_offset_deg) -
         bočne i stražnje prepreke se ignoriraju za odbijanje
       - svaka laserska zraka u tom konusu bliža od
         `influence_radius` stvara odbojni vektor koji gura
         robota od prepreke
       - svi odbojni vektori se zbrajaju u jedan rezultantni

    Privlačni i odbojni vektor se zbroje (vektorski) i iz
    rezultante se očita:
        - željeni smjer kretanja (heading error, jer je vektor
          već u robotovom lokalnom frameu -> x=naprijed, y=lijevo)
        - jačina odbijanja (koristi se za usporavanje blizu
          prepreka)

    Rezultat je glađe, "tekuće" izbjegavanje prepreka umjesto
    naglog stani/kreni ponašanja starog cmd_safety gate-a. Node
    i dalje ima ugrađeni hard-stop failsafe (zadnja linija
    obrane) za slučaj da potencijalno polje ne stigne reagirati
    na vrijeme (npr. prepreka koja iskrsne jako blizu).

    Ovaj node NE mijenja postojeće pure_pursuit_follower.py i
    cmd_safety.py - to su i dalje dostupni kao stari pristup.
    Ovo je potpuno novi, samostalan node.

Topici:
    Pretplate:
        /plan   (nav_msgs/Path)          <- A* planer
        /scan   (sensor_msgs/LaserScan)  <- za potencijalno polje
    TF:
        map -> odom -> base_footprint
    Publicira:
        /cmd_vel (geometry_msgs/Twist)   <- direktno na robota
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import tf2_ros


from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy
)

class PotentialFieldPurePursuit(Node):

    def __init__(self):
        super().__init__('potential_field_pure_pursuit')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # TF frameovi
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # --- Pure Pursuit (privlačna komponenta) ---
        self.declare_parameter('lookahead_distance', 0.35)
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('min_linear_velocity', 0.05)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.10)

        # Usporavanje pri približavanju cilju
        self.declare_parameter('slowdown_distance', 0.850)
        self.declare_parameter('slowdown_min_factor', 0.30)

        # P-pojačanje za pretvorbu heading errora (iz kombiniranog
        # vektora) u kutnu brzinu
        self.declare_parameter('heading_kp', 1.8)

        # Iznad ovog headinga (rad) robot se prvo rotira u mjestu
        # umjesto da vozi krivudavo
        self.declare_parameter('rotate_in_place_threshold', 1.2)

        # Histereza za gornji prag (rad) - da robot ne treperi
        # ulazak/izlazak iz "rotiraj u mjestu" moda kad je heading
        # error baš na granici. Izlazi iz rotate-in-place tek kad
        # padne ispod (rotate_in_place_threshold - hysteresis).
        self.declare_parameter('rotate_in_place_hysteresis', 0.25)

        # Maksimalna promjena kutne brzine po sekundi [rad/s^2].
        # Sprječava da omega skoči trenutno s +max na -max kad
        # heading error promijeni predznak (uzrok "titranja").
        self.declare_parameter('max_angular_acceleration', 2.0)

        # --- Potential Field (odbojna komponenta) ---

        # Prepreke dalje od ovoga se ignoriraju za odbijanje [m]
        self.declare_parameter('influence_radius', 0.70)

        # Pojačanje odbojne sile
        self.declare_parameter('repulsive_gain', 0.45)

        # Pojačanje privlačnog vektora (obično 1.0)
        self.declare_parameter('attractive_gain', 1.0)

        # Eksponencijalno zaglađivanje odbojnog vektora između
        # /scan poruka (0-1). Manje = glađe ali sporije reagira,
        # veće = brže reagira ali osjetljivije na šum lasera.
        self.declare_parameter('repulsive_smoothing_alpha', 0.3)

        # Obradi svaku N-tu zraku (radi brzine)
        self.declare_parameter('scan_stride', 2)

        # Kut 0 skenera se pretpostavlja "naprijed" - ako laser
        # nije poravnat s robotom, podesi offset [deg]
        self.declare_parameter('scan_angle_offset_deg', 0.0)

        # Potencijalno polje gleda samo prednji konus oko smjera
        # "naprijed" (nakon primjene scan_angle_offset_deg).
        # Ukupna širina konusa u stupnjevima - npr. 170.0 = ±85°.
        # Zrake izvan ovog konusa (bokovi, iza robota) se
        # potpuno ignoriraju za odbojni vektor.
        self.declare_parameter('pf_front_angle_deg', 170.0)

        # --- Hard-stop failsafe (zadnja linija obrane) ---
        self.declare_parameter('hard_stop_distance', 0.22)
        self.declare_parameter('hard_stop_front_angle_deg', 60.0)

        self.declare_parameter('control_rate', 20.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        plan_topic = self.get_parameter('plan_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value
        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value
        self.min_linear_velocity = self.get_parameter(
            'min_linear_velocity'
        ).value
        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value
        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value
        self.slowdown_distance = self.get_parameter(
            'slowdown_distance'
        ).value
        self.slowdown_min_factor = self.get_parameter(
            'slowdown_min_factor'
        ).value
        self.heading_kp = self.get_parameter(
            'heading_kp'
        ).value
        self.rotate_in_place_threshold = self.get_parameter(
            'rotate_in_place_threshold'
        ).value
        self.rotate_in_place_hysteresis = self.get_parameter(
            'rotate_in_place_hysteresis'
        ).value
        self.max_angular_acceleration = self.get_parameter(
            'max_angular_acceleration'
        ).value

        self.influence_radius = self.get_parameter(
            'influence_radius'
        ).value
        self.repulsive_gain = self.get_parameter(
            'repulsive_gain'
        ).value
        self.attractive_gain = self.get_parameter(
            'attractive_gain'
        ).value
        self.repulsive_smoothing_alpha = self.get_parameter(
            'repulsive_smoothing_alpha'
        ).value
        self.scan_stride = max(
            1,
            int(self.get_parameter('scan_stride').value)
        )
        self.scan_angle_offset = math.radians(
            self.get_parameter('scan_angle_offset_deg').value
        )

        self.pf_front_half_cone = math.radians(
            self.get_parameter('pf_front_angle_deg').value
        ) / 2.0

        self.hard_stop_distance = self.get_parameter(
            'hard_stop_distance'
        ).value
        self.hard_stop_front_angle = math.radians(
            self.get_parameter('hard_stop_front_angle_deg').value
        )

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        # =====================================================
        # VARIJABLE
        # =====================================================

        self.path = None
        self.current_index = 0
        self.finished = True

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # Zadnji izračunati odbojni vektor iz /scan (lokalni frame)
        self.rep_x = 0.0
        self.rep_y = 0.0
        self.rep_magnitude = 0.0

        # Najbliža prepreka u prednjem konusu (za hard-stop)
        self.min_front_distance = float('inf')

        # Zadnja poslana kutna brzina - za rate-limit (sprječava
        # nagle skokove/titranje)
        self.prev_omega = 0.0

        # Da li je robot trenutno u "rotiraj u mjestu" modu -
        # koristi histerezu da ne treperi ulaz/izlaz
        self.rotating_in_place = False

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # SUB / PUB
        # =====================================================

        self.plan_sub = self.create_subscription(
            Path,
            plan_topic,
            self.plan_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'ASTRO POTENTIAL FIELD PURE PURSUIT'
        )
        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'Plan topic:     {plan_topic}'
        )
        self.get_logger().info(
            f'Scan topic:     {scan_topic}'
        )
        self.get_logger().info(
            f'Cmd vel topic:  {cmd_vel_topic}'
        )
        self.get_logger().info(
            f'TF:             '
            f'{self.map_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'Lookahead:      '
            f'{self.lookahead_distance:.2f} m'
        )
        self.get_logger().info(
            f'Influence rad.: '
            f'{self.influence_radius:.2f} m'
        )
        self.get_logger().info(
            f'PF konus:       '
            f'±{math.degrees(self.pf_front_half_cone):.1f} deg'
        )
        self.get_logger().info(
            f'Repulsive gain: '
            f'{self.repulsive_gain:.2f}'
        )
        self.get_logger().info(
            f'Hard stop dist: '
            f'{self.hard_stop_distance:.2f} m'
        )
        self.get_logger().info(
            'Čekam putanju na /plan...'
        )

    # =========================================================
    # PLAN CALLBACK
    # =========================================================

    def plan_callback(self, msg: Path):
        """
        Prima novu putanju od A* planera. Svaka nova putanja
        resetira praćenje od početka.
        """

        if len(msg.poses) == 0:
            self.get_logger().warn(
                'Primio sam praznu putanju. Zanemarujem.'
            )
            return

        self.path = msg
        self.current_index = 0
        self.finished = False

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'Nova putanja primljena: '
            f'{len(msg.poses)} waypointa.'
        )
        self.get_logger().info(
            'Počinjem praćenje (A* + potential field)...'
        )
        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # SCAN CALLBACK - POTENCIJALNO POLJE
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        """
        Iz sirovog laserskog skena računa:
            - odbojni vektor (rep_x, rep_y) u lokalnom frameu
              robota (x = naprijed, y = lijevo)
            - najbližu prepreku u prednjem konusu, za hard-stop

        Pretpostavka: kut 0 skenera (angle_min + i*angle_increment)
        je "naprijed" u odnosu na robota, uz mogući
        scan_angle_offset ako laser nije poravnat.
        """

        rep_x = 0.0
        rep_y = 0.0

        min_front = float('inf')
        half_front_cone = self.hard_stop_front_angle / 2.0

        r_min = max(msg.range_min, 0.03)

        for i in range(0, len(msg.ranges), self.scan_stride):

            r = msg.ranges[i]

            if not math.isfinite(r) or r < msg.range_min:
                continue

            angle = (
                msg.angle_min
                + i * msg.angle_increment
                - self.scan_angle_offset
            )

            angle = self.normalize_angle(angle)

            # ---- hard-stop provjera (prednji konus) ----

            if -half_front_cone <= angle <= half_front_cone:
                if r < min_front:
                    min_front = r

            # ---- odbojni vektor (potencijalno polje) ----
            # Samo prednji konus (npr. ±85°) - bočne i stražnje
            # prepreke se ignoriraju za odbijanje.

            if abs(angle) > self.pf_front_half_cone:
                continue

            if r >= self.influence_radius:
                continue

            r_clamped = max(r, r_min)

            # Klasična APF formula: sila raste kako se r -> 0
            # i nestaje na granici influence_radius.
            magnitude = self.repulsive_gain * (
                (1.0 / r_clamped)
                -
                (1.0 / self.influence_radius)
            ) / (r_clamped * r_clamped)

            magnitude = max(0.0, magnitude)

            # Smjer odbijanja = suprotno od smjera prepreke
            rep_x += -magnitude * math.cos(angle)
            rep_y += -magnitude * math.sin(angle)

        # Eksponencijalno zaglađivanje - umjesto da svaki novi
        # scan potpuno prepiše odbojni vektor (što uzrokuje
        # nagle skokove/titranje kod šumovitih očitanja), novi
        # rezultat se samo djelomično miješa sa starim.
        alpha = self.repulsive_smoothing_alpha

        self.rep_x = alpha * rep_x + (1.0 - alpha) * self.rep_x
        self.rep_y = alpha * rep_y + (1.0 - alpha) * self.rep_y
        self.rep_magnitude = math.hypot(self.rep_x, self.rep_y)
        self.min_front_distance = min_front

    # =========================================================
    # UPDATE ROBOT POSE (TF)
    # =========================================================

    def update_robot_pose(self) -> bool:

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            self.robot_x = t.x
            self.robot_y = t.y

            self.robot_yaw = self.quaternion_to_yaw(
                r.x, r.y, r.z, r.w
            )

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return False

    # =========================================================
    # POMOĆNE FUNKCIJE
    # =========================================================

    def quaternion_to_yaw(self, x, y, z, w) -> float:

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def world_to_local(self, wx, wy):
        """
        Pretvara točku iz map framea u lokalni frame robota
        (x = naprijed, y = lijevo).
        """

        dx = wx - self.robot_x
        dy = wy - self.robot_y

        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)

        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw

        return local_x, local_y

    # =========================================================
    # FIND LOOKAHEAD POINT
    # =========================================================

    def find_lookahead_point(self):
        """
        Traži prvu točku na putanji koja je udaljena barem
        lookahead_distance od robota, počevši od current_index.
        Napreduje current_index kako se robot kreće duž putanje.
        """

        poses = self.path.poses

        # Napreduj current_index do najbliže sljedeće točke
        while self.current_index < len(poses) - 1:

            px = poses[self.current_index].pose.position.x
            py = poses[self.current_index].pose.position.y

            dist = math.hypot(
                px - self.robot_x,
                py - self.robot_y
            )

            if dist >= self.lookahead_distance:
                break

            self.current_index += 1

        for idx in range(self.current_index, len(poses)):

            px = poses[idx].pose.position.x
            py = poses[idx].pose.position.y

            dist = math.hypot(
                px - self.robot_x,
                py - self.robot_y
            )

            if dist >= self.lookahead_distance:
                self.current_index = idx
                return px, py

        # Nema točke dovoljno daleko - uzmi zadnju (cilj)
        final_pose = poses[-1]

        return (
            final_pose.pose.position.x,
            final_pose.pose.position.y
        )

    # =========================================================
    # CHECK GOAL REACHED
    # =========================================================

    def check_goal_reached(self) -> bool:

        if self.path is None:
            return False

        poses = self.path.poses

        if len(poses) == 0:
            return False

        progress = self.current_index / float(len(poses))

        if progress < 0.90:
            return False

        final_pose = poses[-1]

        gx = final_pose.pose.position.x
        gy = final_pose.pose.position.y

        dist = math.hypot(
            gx - self.robot_x,
            gy - self.robot_y
        )

        return dist < self.goal_tolerance

    # =========================================================
    # STOP ROBOT
    # =========================================================

    def stop_robot(self, allow_rotation=False, omega=0.0):

        cmd = Twist()
        cmd.linear.x = 0.0

        if allow_rotation:
            cmd.angular.z = omega
        else:
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        # Reset rate-limit stanja - kad robot ponovno krene,
        # omega ne smije "pamtiti" staru vrijednost od prije stopa
        self.prev_omega = cmd.angular.z

    # =========================================================
    # GLAVNI IZRAČUN KOMANDE (PURE PURSUIT + POTENTIAL FIELD)
    # =========================================================

    def calculate_command(self, target_x, target_y):

        # ---- privlačni vektor (pure pursuit smjer) ----

        att_x, att_y = self.world_to_local(target_x, target_y)

        att_norm = math.hypot(att_x, att_y)

        if att_norm > 1e-6:
            att_x /= att_norm
            att_y /= att_norm

        att_x *= self.attractive_gain
        att_y *= self.attractive_gain

        # ---- odbojni vektor (već izračunat u scan_callback) ----

        comb_x = att_x + self.rep_x
        comb_y = att_y + self.rep_y

        if abs(comb_x) < 1e-6 and abs(comb_y) < 1e-6:
            # Privlačna i odbojna sila se poništavaju (lokalni
            # minimum) - lagano rotiraj da se izađe iz zastoja.
            omega = self.limit_omega_rate(self.max_angular_velocity * 0.3)
            return 0.0, omega

        heading_error = math.atan2(comb_y, comb_x)

        # ---- rotacija u mjestu ako je cilj/rezultanta jako
        #      izvan prednjeg konusa (s histerezom protiv
        #      treperenja ulaska/izlaska iz ovog moda) ----

        if self.rotating_in_place:
            exit_threshold = (
                self.rotate_in_place_threshold
                - self.rotate_in_place_hysteresis
            )
            if abs(heading_error) < exit_threshold:
                self.rotating_in_place = False
        else:
            if abs(heading_error) > self.rotate_in_place_threshold:
                self.rotating_in_place = True

        if self.rotating_in_place:

            omega = self.heading_kp * heading_error
            omega = max(
                -self.max_angular_velocity,
                min(omega, self.max_angular_velocity)
            )
            omega = self.limit_omega_rate(omega)

            self.get_logger().debug(
                f'Rotacija u mjestu. Heading error: '
                f'{math.degrees(heading_error):.1f} deg'
            )

            return 0.0, omega

        # ---- normalna vožnja ----

        v = self.max_linear_velocity

        # usporavanje pri velikom heading erroru
        v *= max(
            0.2,
            math.cos(heading_error)
        )

        # usporavanje blizu cilja
        final_pose = self.path.poses[-1]

        dist_to_goal = math.hypot(
            final_pose.pose.position.x - self.robot_x,
            final_pose.pose.position.y - self.robot_y
        )

        if dist_to_goal < self.slowdown_distance:
            scale = dist_to_goal / self.slowdown_distance
            v *= max(self.slowdown_min_factor, scale)

        # usporavanje kad je odbojna sila jaka (prepreka blizu)
        if self.rep_magnitude > 1e-6:
            rep_scale = 1.0 / (1.0 + self.rep_magnitude)
            v *= max(0.25, rep_scale)

        v = max(v, self.min_linear_velocity)

        omega = self.heading_kp * heading_error
        omega = max(
            -self.max_angular_velocity,
            min(omega, self.max_angular_velocity)
        )
        omega = self.limit_omega_rate(omega)

        self.get_logger().debug(
            f'att=({att_x:.2f},{att_y:.2f}) '
            f'rep=({self.rep_x:.2f},{self.rep_y:.2f}) '
            f'heading_err={math.degrees(heading_error):.1f}deg '
            f'v={v:.2f} omega={omega:.2f}'
        )

        return v, omega

    # =========================================================
    # RATE-LIMIT KUTNE BRZINE
    # =========================================================

    def limit_omega_rate(self, omega_target):
        """
        Ograničava koliko se omega smije promijeniti u jednom
        ciklusu kontrolne petlje (max_angular_acceleration),
        umjesto da skoči trenutno s +max na -max kad heading
        error promijeni predznak. To je glavni uzrok "titranja"
        lijevo-desno kod nailaska na prepreku.
        """

        max_domega = self.max_angular_acceleration / self.control_rate

        omega = max(
            self.prev_omega - max_domega,
            min(omega_target, self.prev_omega + max_domega)
        )

        self.prev_omega = omega

        return omega

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        tf_ok = self.update_robot_pose()

        if not tf_ok:
            self.stop_robot()
            return

        # ---- hard-stop failsafe (neovisno o potential fieldu) ----

        if self.min_front_distance < self.hard_stop_distance:

            self.get_logger().warn(
                f'HARD STOP! Prepreka na '
                f'{self.min_front_distance:.2f} m ispred.',
                throttle_duration_sec=1.0
            )

            self.stop_robot()
            return

        if self.path is None or self.finished:
            self.stop_robot()
            return

        if self.check_goal_reached():

            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info('GOAL DOSTIGNUT!')
            self.get_logger().info('Robot staje.')
            self.get_logger().info('Čekam novu putanju...')
            self.get_logger().info(
                '========================================'
            )

            self.finished = True
            self.path = None

            self.stop_robot()
            return

        target = self.find_lookahead_point()

        if target is None:
            self.stop_robot()
            return

        target_x, target_y = target

        v, omega = self.calculate_command(target_x, target_y)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega

        self.cmd_vel_pub.publish(cmd)


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PotentialFieldPurePursuit()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Keyboard interrupt. Gasim potential field pure pursuit.'
        )

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
