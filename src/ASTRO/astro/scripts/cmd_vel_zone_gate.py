#!/usr/bin/env python3
"""
Cmd Vel Zone Gate
==================
Prošireni nasljednik cmd_safety.py. Sjedi između pure pursuita
(/cmd_vel_nav) i stvarnog /cmd_vel i radi TRI stvari:

1. FRONT STOP + REVERSE/ROTATE RECOVERY STATE MACHINE (IZMIJENJENO)
   Zadnja linija obrane. Ima 4 stanja:

       NORMAL
           Standardni rad - front slow (linearno usporavanje ispod
           slow_distance) + lateralni bias na angular.z.

           Trigger za izlazak iz NORMAL-a više NIJE jedna izolirana
           zraka ispod stop_distance (to je bio šum-osjetljiv
           pristup naslijeđen iz cmd_safety.py) nego POTVRĐENI
           KLASTER: mora postojati min_cluster_points susjednih
           laserskih zraka na sličnoj udaljenosti (ista logika kao
           zone_obstacle_detector.find_best_cluster) unutar
           stop_distance. Uz to postoji i emergency_stop_distance
           kao dodatni fail-safe - ako se bilo koja pojedinačna
           zraka nađe ispod tog (vrlo malog) praga, manevar se
           okida ODMAH i bez potrebe za potvrdom klasterom (npr.
           tanka nožica stolice koja ne formira klaster, ali je
           opasno blizu).

       HARD_STOP  (okine se kad je klaster/emergency potvrđen)
           Robot potpuno staje - linear.x I angular.z na 0. Drži se
           puna nula hard_stop_hold_sec sekundi (kratka pauza da se
           scan/stanje stabiliziraju prije nego krenemo unatrag).

       REVERSING  (NOVO)
           Robot se vozi ravno unatrag (linear.x = -reverse_speed,
           angular.z = 0) TOČNO reverse_time_sec sekundi (open-loop,
           nema odometrijske povratne veze u ovom node-u - stvarno
           prijeđena udaljenost ovisi o reverse_speed-u, ali samo
           trajanje manevra se izravno podešava preko parametra).

       ROTATING  (NOVO)
           Nakon što se odmaknuo unatrag, robot se zarotira u mjestu
           (linear.x = 0, angular.z = ±rotate_speed) za otprilike
           rotate_angle_deg stupnjeva. Smjer rotacije se bira TAKO
           DA SE ROBOT OKREĆE OD KLASTERA (ako je klaster detektiran
           na lijevoj strani prednjeg konusa, robot rotira udesno, i
           obrnuto) - cilj je da nova A* putanja ima što više
           slobodnog prostora "vidljivog" za planiranje oko
           prepreke.

           Na kraju rotacije šalje se /force_replan (throttled preko
           replan_request_cooldown_sec) - astar_zone_planner tada
           odmah radi novi A* prema istom cilju koristeći trenutni
           front_zone_obstacles grid, koji sadrži upravo detektirani
           klaster (zone_obstacle_detector je cijelo vrijeme radio
           dok se manevar izvodio, pa je grid svjež).

           Nakon toga se node vraća u NORMAL, ali s kratkim "grace"
           periodom (post_maneuver_grace_sec) tijekom kojeg se novi
           reverse/rotate manevar NE okida iznova (daje se vremena
           novom planu i pure pursuitu da preuzmu) - proporcionalno
           usporavanje (slow_distance) i lateralni avoidance i dalje
           rade normalno tijekom tog perioda.

   NAPOMENA (VAŽNO - nema straga senzora):
       Ovaj node koristi samo prednji LIDAR konus za detekciju.
       Vožnja unatrag u REVERSING stanju je open-loop i ne provjerava
       je li iza robota slobodno. Ako robot ima i stražnji senzor,
       preporuka je dodati ekvivalentnu provjeru prije/tijekom
       REVERSING stanja.

2. LATERALNI AVOIDANCE STEER
   Sluša /left_avoidance_signal i /right_avoidance_signal (iz
   zone_obstacle_detector.py, 0.0-1.0 jačina) i DODAJE blagi
   angular.z bias na izlaz pure pursuita - robot polako skreće
   OD strane na kojoj je detektiran klaster, bez diranja globalne
   putanje/A*-a. Bias je:
       - proporcionalan jačini signala (lateral_gain)
       - ograničen na max_lateral_bias (rad/s)
       - nisko-propusno filtriran (bias_smoothing_alpha) da ne trza
       - timeout-a se ako signal ne stigne unutar signal_timeout_sec
       - lijevi i desni signal se oduzimaju (djelomično poništavaju
         u "hodniku")
   Ovaj bias je AKTIVAN samo u NORMAL stanju - tijekom HARD_STOP /
   REVERSING / ROTATING je isključen (kretanje unatrag i rotacija su
   čisti open-loop manevri, bez miješanja s reaktivnim steeringom).

3. FORCE REPLAN REQUEST
   Publisher na /force_replan (std_msgs/Empty) koji astar_zone_planner
   sluša i odmah (bez čekanja na path_monitor_callback ili cooldown)
   radi novi A* prema istom cilju, koristeći trenutni front_zone
   grid - dakle novi plan "vidi" upravo detektirani klaster. Šalje
   se na kraju ROTATING stanja (vidi gore).

    Gate je JEDINO mjesto koje ima zadnju riječ nad /cmd_vel -
    time je izbjegnut sukob autoriteta između pure pursuita i
    reaktivnog avoidancea.

NAPOMENA / PRETPOSTAVKA:
    Kut 0 u skener frame-u se pretpostavlja kao "naprijed" u
    odnosu na robota (isto kao cmd_safety.py). Ako je laser
    montiran zarotirano, podesi `angle_offset_deg`.

    Konvencija kuta: pozitivan kut = lijevo (standardna ROS/REP103
    konvencija, isto pretpostavljeno kao i u ostatku skener koda u
    ovom paketu). Ako se na terenu pokaže obrnuto, invertiraj predznak
    u compute_rotate_direction().

Topici:
    Pretplate:
        /scan                    (sensor_msgs/LaserScan)
        /cmd_vel_nav              (geometry_msgs/Twist)   <- izlaz pure pursuit-a
        /left_avoidance_signal    (std_msgs/Float32)      <- iz zone_obstacle_detector
        /right_avoidance_signal   (std_msgs/Float32)      <- iz zone_obstacle_detector
    Publicira:
        /cmd_vel                  (geometry_msgs/Twist)   <- stvarni izlaz na robota
        /force_replan             (std_msgs/Empty)        <- astar_zone_planner sluša
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty


class CmdVelZoneGate(Node):

    def __init__(self):
        super().__init__('cmd_vel_zone_gate')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('left_signal_topic', '/left_avoidance_signal')
        self.declare_parameter('right_signal_topic', '/right_avoidance_signal')
        self.declare_parameter('force_replan_topic', '/force_replan')

        # --- Front stop/slow (identično cmd_safety.py) ---
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('slow_distance', 0.70)
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('angle_offset_deg', 0.0)

        # --- Klaster potvrda za stop zonu (NOVO, ista logika kao
        #     zone_obstacle_detector.find_best_cluster) ---
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('max_range_jump', 0.15)
        self.declare_parameter('max_index_gap', 2)

        # Fail-safe: pojedinačna zraka ispod ovog praga okida manevar
        # ODMAH, bez potrebe za potvrdom klasterom (mora biti manji
        # od stop_distance).
        self.declare_parameter('emergency_stop_distance', 0.15)

        # --- Hard stop / reverse / rotate state machine (IZMIJENJENO) ---
        # Koliko dugo robot stoji potpuno nepomično (linear=0,
        # angular=0) prije nego krene unatrag
        self.declare_parameter('hard_stop_hold_sec', 1.0)

        # Koliko DUGO (s) i kojom brzinom (m/s, magnituda) se vozi
        # unatrag - trajanje se podešava IZRAVNO preko reverse_time_sec
        # (open-loop, nema odometrije), a stvarno prijeđena udaljenost
        # je otprilike reverse_speed * reverse_time_sec.
        self.declare_parameter('reverse_time_sec', 6.7)
        self.declare_parameter('reverse_speed', 0.15)

        # Za koliko se zarotira (stupnjeva) i kojom kutnom brzinom
        # (rad/s, magnituda) nakon što se odmakne unatrag
        self.declare_parameter('rotate_angle_deg', 25.0)
        self.declare_parameter('rotate_speed', 0.4)

        # Min. razmak između dva /force_replan zahtjeva
        self.declare_parameter('replan_request_cooldown_sec', 3.0)

        # Nakon završenog reverse+rotate manevra, koliko dugo se novi
        # manevar ne okida iznova (daje se vremena novom planu da
        # "stigne")
        self.declare_parameter('post_maneuver_grace_sec', 1.0)

        # --- Lateralni avoidance ---
        self.declare_parameter('lateral_gain', 0.6)
        self.declare_parameter('max_lateral_bias', 0.5)
        self.declare_parameter('bias_smoothing_alpha', 0.3)
        self.declare_parameter('signal_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        scan_topic = self.get_parameter('scan_topic').value
        cmd_in_topic = self.get_parameter('cmd_in_topic').value
        cmd_out_topic = self.get_parameter('cmd_out_topic').value
        left_signal_topic = self.get_parameter('left_signal_topic').value
        right_signal_topic = self.get_parameter('right_signal_topic').value
        force_replan_topic = self.get_parameter('force_replan_topic').value

        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.front_angle_deg = self.get_parameter('front_angle_deg').value
        self.angle_offset_deg = self.get_parameter('angle_offset_deg').value

        self.min_cluster_points = int(
            self.get_parameter('min_cluster_points').value
        )
        self.max_range_jump = self.get_parameter('max_range_jump').value
        self.max_index_gap = int(self.get_parameter('max_index_gap').value)
        self.emergency_stop_distance = self.get_parameter(
            'emergency_stop_distance'
        ).value

        self.hard_stop_hold_sec = self.get_parameter('hard_stop_hold_sec').value

        self.reverse_time_sec = self.get_parameter('reverse_time_sec').value
        self.reverse_speed = abs(self.get_parameter('reverse_speed').value)

        self.rotate_angle_deg = self.get_parameter('rotate_angle_deg').value
        self.rotate_speed = abs(self.get_parameter('rotate_speed').value)

        self.replan_request_cooldown_sec = self.get_parameter(
            'replan_request_cooldown_sec'
        ).value
        self.post_maneuver_grace_sec = self.get_parameter(
            'post_maneuver_grace_sec'
        ).value

        self.lateral_gain = self.get_parameter('lateral_gain').value
        self.max_lateral_bias = self.get_parameter('max_lateral_bias').value
        self.bias_smoothing_alpha = self.get_parameter('bias_smoothing_alpha').value
        self.signal_timeout_sec = self.get_parameter('signal_timeout_sec').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        # =====================================================
        # STANJE
        # =====================================================

        self.min_front_distance = float('inf')
        self.min_front_angle = 0.0

        # Klaster potvrđen unutar stop_distance u trenutnom scanu
        # (lista (index, angle, range) tuple-ova) ili None.
        self.front_cluster = None
        self.front_cluster_mean_angle = 0.0

        self.left_signal = 0.0
        self.right_signal = 0.0
        self.last_left_time = 0.0
        self.last_right_time = 0.0

        self.smoothed_bias = 0.0

        self.last_cmd_in = Twist()
        self.have_cmd_in = False

        # --- state machine ---
        # 'NORMAL' | 'HARD_STOP' | 'REVERSING' | 'ROTATING'
        self.gate_state = 'NORMAL'
        self.hard_stop_start_time = 0.0
        self.reverse_start_time = 0.0
        self.rotate_start_time = 0.0
        self.rotate_direction = 1.0  # +1.0 = ulijevo (CCW), -1.0 = udesno (CW)
        self.last_replan_request_time = 0.0
        self.maneuver_grace_until = 0.0

        # =====================================================
        # SUB / PUB
        # =====================================================

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )

        self.cmd_sub = self.create_subscription(
            Twist, cmd_in_topic, self.cmd_callback, 10
        )

        self.left_sub = self.create_subscription(
            Float32, left_signal_topic, self.left_signal_callback, 10
        )

        self.right_sub = self.create_subscription(
            Float32, right_signal_topic, self.right_signal_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_out_topic, 10)
        self.force_replan_pub = self.create_publisher(
            Empty, force_replan_topic, 10
        )

        self.control_timer = self.create_timer(
            1.0 / control_rate_hz, self.control_loop
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info(
            'CMD VEL ZONE GATE (stop/slow + reverse+rotate recovery + lateral steer)'
        )
        self.get_logger().info('========================================')
        self.get_logger().info(f'Ulaz:              {cmd_in_topic}')
        self.get_logger().info(f'Izlaz:             {cmd_out_topic}')
        self.get_logger().info(f'Force replan:      {force_replan_topic}')
        self.get_logger().info(f'Stop distance:     {self.stop_distance:.2f} m')
        self.get_logger().info(f'Slow distance:     {self.slow_distance:.2f} m')
        self.get_logger().info(f'Front cone:        {self.front_angle_deg:.1f} deg')
        self.get_logger().info(
            f'Min cluster pts:   {self.min_cluster_points} '
            f'(emergency < {self.emergency_stop_distance:.2f} m bez klastera)'
        )
        self.get_logger().info(f'Hard stop hold:    {self.hard_stop_hold_sec:.2f} s')
        self.get_logger().info(
            f'Reverse:           {self.reverse_time_sec:.2f} s @ '
            f'{self.reverse_speed:.2f} m/s '
            f'(~{self.reverse_time_sec * self.reverse_speed:.2f} m)'
        )
        self.get_logger().info(
            f'Rotate:            {self.rotate_angle_deg:.1f} deg @ '
            f'{self.rotate_speed:.2f} rad/s'
        )
        self.get_logger().info(f'Lateral gain:      {self.lateral_gain:.2f}')
        self.get_logger().info(f'Max lateral bias:  {self.max_lateral_bias:.2f} rad/s')

    # =========================================================
    # SCAN CALLBACK (front stop/slow + front-zone klaster potvrda)
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        half_cone = math.radians(self.front_angle_deg / 2.0)
        offset = math.radians(self.angle_offset_deg)

        min_dist = float('inf')
        min_angle = 0.0

        # Točke unutar prednjeg konusa koje su ISPOD stop_distance -
        # kandidati za klaster (isti kriterij kao HARD_STOP granica).
        cluster_candidates = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue

            angle = msg.angle_min + i * msg.angle_increment - offset

            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi

            if -half_cone <= angle <= half_cone:
                if r < min_dist:
                    min_dist = r
                    min_angle = angle

                if r < self.stop_distance:
                    cluster_candidates.append((i, angle, r))

        self.min_front_distance = min_dist
        self.min_front_angle = min_angle

        self.front_cluster = self.find_front_cluster(cluster_candidates)
        if self.front_cluster is not None:
            self.front_cluster_mean_angle = sum(
                p[1] for p in self.front_cluster
            ) / len(self.front_cluster)

    # =========================================================
    # CLUSTERING (ista logika kao
    # zone_obstacle_detector.find_best_cluster - kontinuirani niz
    # susjednih zraka na sličnoj udaljenosti)
    # =========================================================

    def find_front_cluster(self, points):
        """
        points: lista (index, angle, range) sortirana po indexu,
        već filtrirana na < stop_distance unutar prednjeg konusa.
        Vraća najveći "kontinuirani" klaster (lista istih tuple-a)
        ili None ako nijedan klaster ne prolazi min_cluster_points.
        """

        if not points:
            return None

        clusters = []
        current = [points[0]]

        for prev, cur in zip(points, points[1:]):
            idx_gap = cur[0] - prev[0]
            range_jump = abs(cur[2] - prev[2])

            if idx_gap <= self.max_index_gap and range_jump <= self.max_range_jump:
                current.append(cur)
            else:
                clusters.append(current)
                current = [cur]

        clusters.append(current)

        best = max(clusters, key=len)

        if len(best) >= self.min_cluster_points:
            return best

        return None

    # =========================================================
    # AVOIDANCE SIGNAL CALLBACKS
    # =========================================================

    def left_signal_callback(self, msg: Float32):
        self.left_signal = max(0.0, min(1.0, msg.data))
        self.last_left_time = time.time()

    def right_signal_callback(self, msg: Float32):
        self.right_signal = max(0.0, min(1.0, msg.data))
        self.last_right_time = time.time()

    # =========================================================
    # CMD VEL CALLBACK (samo pamti zadnju komandu iz pure pursuita)
    # =========================================================

    def cmd_callback(self, msg: Twist):
        self.last_cmd_in = msg
        self.have_cmd_in = True

    # =========================================================
    # LATERALNI BIAS (koristi se SAMO u NORMAL stanju)
    # =========================================================

    def update_lateral_bias(self, now: float) -> float:
        left = (
            self.left_signal
            if (now - self.last_left_time) <= self.signal_timeout_sec
            else 0.0
        )
        right = (
            self.right_signal
            if (now - self.last_right_time) <= self.signal_timeout_sec
            else 0.0
        )

        raw_bias = (right - left) * self.lateral_gain
        raw_bias = max(-self.max_lateral_bias, min(self.max_lateral_bias, raw_bias))

        alpha = self.bias_smoothing_alpha
        self.smoothed_bias = alpha * raw_bias + (1.0 - alpha) * self.smoothed_bias

        if abs(self.smoothed_bias) > 0.02:
            self.get_logger().info(
                f'Lateral avoidance: L={left:.2f} R={right:.2f} '
                f'bias={self.smoothed_bias:+.2f} rad/s',
                throttle_duration_sec=0.5
            )

        return self.smoothed_bias

    # =========================================================
    # SMJER ROTACIJE - okreni robota OD klastera
    # =========================================================

    def compute_rotate_direction(self) -> float:
        """
        Pozitivan kut (angle > 0) = klaster je lijevo od robota
        (standardna ROS/REP103 konvencija) -> rotiraj UDESNO
        (negativan angular.z) da se okreneš OD prepreke, i obrnuto.

        Ako nema potvrđenog klastera (samo emergency_stop_distance
        trigger na jednoj zraci), koristi kut najbliže zrake
        (min_front_angle) kao najbolju dostupnu procjenu smjera
        prepreke.
        """
        if self.front_cluster is not None:
            reference_angle = self.front_cluster_mean_angle
        else:
            reference_angle = self.min_front_angle

        return -1.0 if reference_angle > 0.0 else 1.0

    # =========================================================
    # KONTROLNA PETLJA - state machine
    # (NORMAL / HARD_STOP / REVERSING / ROTATING)
    # =========================================================

    def control_loop(self):
        if not self.have_cmd_in:
            return

        now = time.time()

        # --- NORMAL -> ulaz u HARD_STOP ako je klaster potvrđen u
        #     stop zoni (ili emergency fail-safe na jednoj zraci) ---
        if self.gate_state == 'NORMAL':
            obstacle_confirmed = self.front_cluster is not None
            emergency = self.min_front_distance < self.emergency_stop_distance

            if (obstacle_confirmed or emergency) and now >= self.maneuver_grace_until:
                self.gate_state = 'HARD_STOP'
                self.hard_stop_start_time = now
                self.rotate_direction = self.compute_rotate_direction()
                # Resetiraj lateralni bias - ne smije "curiti" iz NORMAL
                # stanja u manevar niti se nakupljati dok su lateralni
                # signali isključeni iz miksa.
                self.smoothed_bias = 0.0

                if obstacle_confirmed:
                    n_pts = len(self.front_cluster)
                    mean_r = sum(p[2] for p in self.front_cluster) / n_pts
                    self.get_logger().warn(
                        f'SAFETY STOP! Klaster potvrđen ({n_pts} tocaka, '
                        f'srednji range {mean_r:.2f} m). Zaustavljanje na '
                        f'{self.hard_stop_hold_sec:.1f} s, zatim reverse '
                        f'{self.reverse_time_sec:.1f} s + rotacija '
                        f'{self.rotate_angle_deg:.0f} deg.'
                    )
                else:
                    self.get_logger().warn(
                        f'EMERGENCY STOP! Zraka na {self.min_front_distance:.2f} m '
                        f'(< {self.emergency_stop_distance:.2f} m) bez potvrde '
                        f'klasterom. Isti reverse+rotate manevar.'
                    )

        # --- HARD_STOP: potpuna nula, pa nakon hold vremena reverse ---
        if self.gate_state == 'HARD_STOP':
            self.publish_zero()

            if (now - self.hard_stop_start_time) >= self.hard_stop_hold_sec:
                self.gate_state = 'REVERSING'
                self.reverse_start_time = now
                self.get_logger().info(
                    f'Krećem unatrag ({self.reverse_time_sec:.2f} s @ '
                    f'{self.reverse_speed:.2f} m/s)...'
                )

            return

        # --- REVERSING: ravno unatrag TOČNO reverse_time_sec sekundi
        #     (direktno podesivo preko parametra, open-loop) ---
        if self.gate_state == 'REVERSING':
            out = Twist()
            out.linear.x = -self.reverse_speed
            out.angular.z = 0.0
            self.cmd_pub.publish(out)

            if (now - self.reverse_start_time) >= self.reverse_time_sec:
                self.gate_state = 'ROTATING'
                self.rotate_start_time = now
                direction_txt = 'ulijevo' if self.rotate_direction > 0 else 'udesno'
                self.get_logger().info(
                    f'Reverse gotov. Rotiram {direction_txt} '
                    f'({self.rotate_angle_deg:.0f} deg @ '
                    f'{self.rotate_speed:.2f} rad/s)...'
                )

            return

        # --- ROTATING: čista rotacija u mjestu, smjer OD klastera,
        #     open-loop preko vremena (angle = speed * time). Na
        #     kraju zatraži novi A* replan i vrati se u NORMAL. ---
        if self.gate_state == 'ROTATING':
            out = Twist()
            out.linear.x = 0.0
            out.angular.z = self.rotate_direction * self.rotate_speed
            self.cmd_pub.publish(out)

            rotate_duration = math.radians(self.rotate_angle_deg) / max(
                self.rotate_speed, 1e-3
            )

            if (now - self.rotate_start_time) >= rotate_duration:
                self.publish_zero()
                self.request_force_replan(now)

                self.gate_state = 'NORMAL'
                self.maneuver_grace_until = now + self.post_maneuver_grace_sec
                self.smoothed_bias = 0.0

                self.get_logger().info(
                    'Reverse+rotate manevar gotov. Nazad u NORMAL '
                    f'(grace period {self.post_maneuver_grace_sec:.1f} s).'
                )

            return

        # --- NORMAL: standardni front slow + lateral bias ---
        bias = self.update_lateral_bias(now)
        out = Twist()

        if self.min_front_distance < self.slow_distance:
            scale = (
                (self.min_front_distance - self.stop_distance)
                / (self.slow_distance - self.stop_distance)
            )
            scale = max(0.0, min(1.0, scale))
            out.linear.x = self.last_cmd_in.linear.x * scale
        else:
            out.linear.x = self.last_cmd_in.linear.x

        out.angular.z = self.last_cmd_in.angular.z + bias

        self.cmd_pub.publish(out)

    # =========================================================
    # HELPERS
    # =========================================================

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def request_force_replan(self, now: float):
        if (now - self.last_replan_request_time) < self.replan_request_cooldown_sec:
            return

        self.last_replan_request_time = now

        self.get_logger().warn('========================================')
        self.get_logger().warn(
            'Reverse+rotate manevar gotov - tražim FORCE REPLAN oko klastera.'
        )
        self.get_logger().warn('========================================')

        self.force_replan_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelZoneGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim zone gate.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
