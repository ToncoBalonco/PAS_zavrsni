#!/usr/bin/env python3
"""
Cmd Vel Safety Gate + Reaktivni Obstacle Avoidance (scan-only)
================================================================
Sluša /scan i /cmd_vel_nav (izlaz pure pursuit kontrolera) i prije
nego što proslijedi komandu na /cmd_vel, koristi ISKLJUČIVO laser
scan da:

    1) pronađe najbliži slobodni "prolaz" (gap) u prednjem konusu
       i lagano skrene robota prema njemu (reaktivni avoidance,
       slično pojednostavljenom VFH-u),
    2) uspori/potpuno zaustavi robota ako je nešto preblizu ravno
       ispred, i
    3) kad je prepreka detektirana u stop zoni: robot NE nastavlja
       naprijed i ne rotira odmah na mjestu - prvo se kratko odmakne
       unatrag (uz sigurnosnu provjeru "leđa" preko punog 360°
       LIDAR-a), a tek onda rotira u mjestu prema najboljem
       raspoloživom smjeru dok se prolaz ne oslobodi.

ZAŠTO OVAKO (bez /map i bez TF-a):
    Prijašnji pristup je obstacle avoidance radio preko globalne
    karte (/map, AMCL) - dynamic_obstacle_layer je preko TF-a
    (map -> scan frame) upisivao laser pogotke u kartu, a A*
    planer je preko TF-a (map -> base_frame) dobivao poziciju
    robota za replanning. Kad računalo na robotu i operatersko
    računalo nisu vremenski usklađena (nema zajedničkog NTP-a
    između dva stroja u ROS 2 multi-machine setupu), TF stablo
    koje prolazi kroz AMCL (map->odom, vremenski promjenjiv,
    ovisi o kojem računalu) postaje nepouzdano - lookup_transform
    baca ExtrapolationException ili vraća zastarjele transformacije,
    pa se avoidance gasio/kasnio baš kad je najpotrebniji.

    Ovaj node ne koristi NIKAKAV TF. Sve što mu treba je /scan
    poruka - koordinate laserskih zraka su već u frame-u samog
    senzora (relativno na robota), pa nema potrebe za transformacijom
    u globalni frame da bismo znali "je li nešto blizu ispred robota
    i s koje strane je slobodno". To je čini potpuno neovisnim o
    kvaliteti lokalizacije/sinkronizaciji satova.

NAPOMENA / PRETPOSTAVKA:
    Kut 0 (angle_min + i*angle_increment) u skener frame-u se
    pretpostavlja kao "naprijed" u odnosu na robota. Ako je laser
    montiran zarotirano, podesi `angle_offset_deg`.

Topici:
    Pretplate:
        /scan          (sensor_msgs/LaserScan)
        /cmd_vel_nav   (geometry_msgs/Twist)   <- izlaz pure pursuit-a
    Publicira:
        /cmd_vel       (geometry_msgs/Twist)   <- stvarni izlaz na robota
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelSafetyGate(Node):

    def __init__(self):
        super().__init__('cmd_vel_safety_gate')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')

        # Topic na koji se javlja kad manevar izbjegavanja (unatrag +
        # rotacija) završi - astar_path_planner_ob.py na to reagira
        # preplaniranjem prema zadnjem zadanom goalu, jer stara
        # putanja nakon manevra više nije pouzdana.
        self.declare_parameter('replan_request_topic', '/replan_request')

        # Puna zaustava ako je prepreka bliža od ovoga [m]
        # (mjereno u uskom "keep straight" konusu ravno ispred)
        self.declare_parameter('stop_distance', 0.45)

        # Ispod ove udaljenosti počinje linearno usporavanje [m]
        self.declare_parameter('slow_distance', 0.70)

        # Širina uskog konusa za stop/slow provjeru [deg] (±45°)
        self.declare_parameter('front_angle_deg', 90.0)

        # Širina šireg konusa u kojem tražimo prolaz (gap) za
        # skretanje [deg] - treba biti šira od front_angle_deg
        self.declare_parameter('search_angle_deg', 200.0)

        # Offset ako laser nije poravnat s "naprijed" robota [deg].
        # VAŽNO: laser_joint u lidar.xacro ima rpy="0 0 pi" - LIDAR je
        # fizički montiran zarotiran 180° oko Z u odnosu na base_link.
        # Kut 0 u /scan poruci (lokalna +X os senzora) zato pokazuje
        # PREMA NATRAG robota, a kut 180° (pi) pokazuje prema naprijed.
        # Zato default MORA biti 180.0, inače je cijeli prednji konus
        # zapravo iza robota.
        self.declare_parameter('angle_offset_deg', 180.0)

        # Uključi/isključi reaktivno skretanje prema prolazu.
        # Ako False, node radi kao čisti stop/slow gate (staro
        # ponašanje) bez korekcije kuta.
        self.declare_parameter('avoidance_enabled', True)

        # Minimalna kutna širina prolaza da bude "vjerodostojan" [deg]
        self.declare_parameter('min_gap_width_deg', 25.0)

        # Efektivni polumjer robota (+ sigurnosna margina) [m], koristi
        # se da provjerimo stane li robot geometrijski kroz prolaz na
        # danoj udaljenosti (šira udaljenost -> potreban manji kut)
        self.declare_parameter('robot_safety_radius_m', 0.30)

        # Proporcionalno pojačanje za skretanje prema centru prolaza
        self.declare_parameter('angular_gain', 1.5)

        # Maksimalna korekcija kuta koju smijemo dodati na izvorni
        # angular.z iz pure pursuita [deg]
        self.declare_parameter('max_steer_correction_deg', 45.0)

        # Brzina rotacije u mjestu kad je prepreka detektirana ispred
        # (< stop_distance) [rad/s]. Robot (diff_drive_controller)
        # ima max angular.z = 2.78 rad/s - ovo je namjerno osjetno
        # jače (~43% max) da robot brzo i vidljivo okrene "leđa"
        # prepreci, ne da se sporo vrti dok ne udari.
        self.declare_parameter('rotate_in_place_speed', 1.2)

        # --- Ponašanje kad je prepreka u stop zoni: prvo unatrag, ---
        # --- pa tek onda rotacija u mjestu ---

        # Brzina vožnje unatrag [m/s] (pozitivan broj, primjenjuje
        # se kao negativan linear.x)
        self.declare_parameter('reverse_speed', 0.15)

        # Koliko dugo se vozi unatrag prije nego se počne rotirati [s]
        self.declare_parameter('reverse_duration_sec', 1.0)

        # Sigurnosna provjera "leđa" prije/tijekom vožnje unatrag -
        # LIDAR je pun 360°, pa provjeravamo i stražnji konus. Ako je
        # nešto preblizu iza robota, preskačemo unatrag i odmah
        # rotiramo u mjestu (ne želimo se zabiti u nešto iza sebe).
        self.declare_parameter('rear_check_angle_deg', 60.0)

        # Maksimalno trajanje same rotacije prije povratka na
        # normalno ponašanje, čak i ako se prolaz formalno nije
        # "razbistrio" (sigurnosni timeout protiv beskonačnog vrtnja) [s]
        self.declare_parameter('max_rotate_duration_sec', 4.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        scan_topic = self.get_parameter('scan_topic').value
        cmd_in_topic = self.get_parameter('cmd_in_topic').value
        cmd_out_topic = self.get_parameter('cmd_out_topic').value
        replan_request_topic = self.get_parameter('replan_request_topic').value

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.search_angle_deg = float(self.get_parameter('search_angle_deg').value)
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)

        self.avoidance_enabled = bool(self.get_parameter('avoidance_enabled').value)
        self.min_gap_width_deg = float(self.get_parameter('min_gap_width_deg').value)
        self.robot_safety_radius_m = float(self.get_parameter('robot_safety_radius_m').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_steer_correction_deg = float(self.get_parameter('max_steer_correction_deg').value)
        self.rotate_in_place_speed = float(self.get_parameter('rotate_in_place_speed').value)

        self.reverse_speed = abs(float(self.get_parameter('reverse_speed').value))
        self.reverse_duration_sec = float(self.get_parameter('reverse_duration_sec').value)
        self.rear_check_angle_deg = float(self.get_parameter('rear_check_angle_deg').value)
        self.max_rotate_duration_sec = float(self.get_parameter('max_rotate_duration_sec').value)

        # =====================================================
        # STANJE (osvježava scan_callback, čita cmd_callback)
        # =====================================================

        self.min_front_distance = float('inf')  # za stop/slow logiku
        self.min_rear_distance = float('inf')    # sigurnosna provjera za vožnju unatrag
        self.steering_angle = 0.0                # rad, centar odabranog prolaza
        self.path_blocked = False                # nema dovoljno širokog prolaza

        # FSM: 'NORMAL' -> (detekcija u stop zoni) -> 'REVERSING'
        #      -> 'ROTATING' -> natrag na 'NORMAL'
        self.state = 'NORMAL'
        self.state_start_time = 0.0

        # =====================================================
        # SUB / PUB
        # =====================================================

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )

        self.cmd_sub = self.create_subscription(
            Twist, cmd_in_topic, self.cmd_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_out_topic, 10)
        self.replan_pub = self.create_publisher(
            String, replan_request_topic, 10
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('CMD VEL SAFETY GATE + REAKTIVNI AVOIDANCE')
        self.get_logger().info('(samo /scan, bez TF-a i bez /map)')
        self.get_logger().info('========================================')
        self.get_logger().info(f'Ulaz:              {cmd_in_topic}')
        self.get_logger().info(f'Izlaz:             {cmd_out_topic}')
        self.get_logger().info(f'Replan request:    {replan_request_topic}')
        self.get_logger().info(f'Stop distance:     {self.stop_distance:.2f} m')
        self.get_logger().info(f'Slow distance:     {self.slow_distance:.2f} m')
        self.get_logger().info(f'Front cone:        {self.front_angle_deg:.1f} deg')
        self.get_logger().info(f'Search cone:       {self.search_angle_deg:.1f} deg')
        self.get_logger().info(f'Avoidance enabled: {self.avoidance_enabled}')
        self.get_logger().info(
            f'Reverse:           {self.reverse_speed:.2f} m/s za {self.reverse_duration_sec:.1f} s'
        )
        self.get_logger().info(
            f'Rotate:            {self.rotate_in_place_speed:.2f} rad/s, '
            f'max {self.max_rotate_duration_sec:.1f} s'
        )
        self.get_logger().info(f'Rear check cone:   {self.rear_check_angle_deg:.1f} deg')

    # =========================================================
    # POMOĆNE FUNKCIJE
    # =========================================================

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # =========================================================
    # SCAN CALLBACK - gradi lokalnu polarnu histogramu i traži gap
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        offset = math.radians(self.angle_offset_deg)
        half_search = math.radians(self.search_angle_deg / 2.0)
        half_front = math.radians(self.front_angle_deg / 2.0)
        half_rear = math.radians(self.rear_check_angle_deg / 2.0)

        # (kut, udaljenost) za svaku zraku unutar šireg "search" konusa
        samples = []
        rear_dists = []  # neovisno o search konusu - LIDAR je pun 360°
        for i, r in enumerate(msg.ranges):
            angle = self.wrap_angle(msg.angle_min + i * msg.angle_increment - offset)

            if math.isfinite(r) and r > 0.0:
                # VAŽNO: prihvaćamo očitanje i kad je r < msg.range_min.
                # Neki (pogotovo stvarni) LIDAR-ovi i dalje vraćaju
                # konačnu udaljenost za prepreke bliže od nominalnog
                # range_min (npr. 0.15-0.17 m kod range_min=0.30 m).
                # Stari kod je takva očitanja odbacivao kao "nema
                # echa" i tretirao ih kao SLOBODAN prostor do
                # range_max - što znači da je robot baš najbliže,
                # najopasnije prepreke ignorirao i nastavljao voziti
                # prema njoj. Zato ovdje NE filtriramo po range_min,
                # nego samo odbacujemo istinski nevaljana očitanja
                # (nan/inf/0.0).
                dist = min(r, msg.range_max)
            else:
                # istinski nema echa (nan/inf/0.0) = pretpostavljamo
                # da je slobodno do range_max
                dist = msg.range_max

            if -half_search <= angle <= half_search:
                samples.append((angle, dist))

            # "Straga" = kut blizu ±180° (wrap-around), neovisno o
            # search konusu - koristi se samo za sigurnosnu provjeru
            # prije/tijekom vožnje unatrag.
            if (math.pi - abs(angle)) <= half_rear:
                rear_dists.append(dist)

        self.min_rear_distance = min(rear_dists) if rear_dists else float('inf')

        if not samples:
            self.min_front_distance = float('inf')
            self.steering_angle = 0.0
            self.path_blocked = False
            return

        samples.sort(key=lambda s: s[0])

        # --- uski konus ravno ispred: samo za stop/slow udaljenost ---
        front_dists = [d for a, d in samples if -half_front <= a <= half_front]
        self.min_front_distance = min(front_dists) if front_dists else float('inf')

        if not self.avoidance_enabled:
            self.steering_angle = 0.0
            self.path_blocked = False
            return

        # --- traženje prolaza (gap) unutar šireg konusa ---
        # ćelija je "slobodna" ako je udaljenost veća od slow_distance
        # (dovoljno daleko da nema smisla usporavati/skretati zbog nje)
        gap_clear_distance = self.slow_distance
        free_mask = [d > gap_clear_distance for _, d in samples]

        gaps = []
        start = None
        for idx, free in enumerate(free_mask):
            if free and start is None:
                start = idx
            elif not free and start is not None:
                gaps.append((start, idx - 1))
                start = None
        if start is not None:
            gaps.append((start, len(free_mask) - 1))

        best_center = None
        best_score = None

        for s, e in gaps:
            a_start = samples[s][0]
            a_end = samples[e][0]
            width = a_end - a_start

            min_d_in_gap = min(d for _, d in samples[s:e + 1])
            # potreban polukut da robot (+ margina) prođe na toj udaljenosti
            required_half_angle = math.atan2(
                self.robot_safety_radius_m, max(min_d_in_gap, 0.05)
            )
            required_width = max(
                math.radians(self.min_gap_width_deg),
                2.0 * required_half_angle
            )

            if width < required_width:
                continue

            center = (a_start + a_end) / 2.0
            score = abs(center)  # bliže "ravno naprijed" = bolje

            if best_score is None or score < best_score:
                best_score = score
                best_center = center

        if best_center is not None:
            self.steering_angle = best_center
            self.path_blocked = False
        else:
            self.path_blocked = True
            if gaps:
                # nijedan gap nije dovoljno širok da robot prođe, ali
                # ipak biramo najširi kao smjer za rotaciju u mjestu
                widest = max(gaps, key=lambda g: samples[g[1]][0] - samples[g[0]][0])
                self.steering_angle = (
                    samples[widest[0]][0] + samples[widest[1]][0]
                ) / 2.0
            else:
                self.steering_angle = 0.0

    # =========================================================
    # CMD VEL CALLBACK (GATE + AVOIDANCE + REVERSE/ROTATE FSM)
    # =========================================================

    def cmd_callback(self, msg: Twist):
        now = time.time()

        # Ako pure_pursuit_follower nema aktivnu putanju (nema
        # zadanog goal_pose-a), on I DALJE kontinuirano publicira
        # na /cmd_vel_nav (praznu, nultu Twist poruku, ~20 Hz) -
        # vidi pure_pursuit_follower.py: control_loop() -> stop_robot()
        # kad je self.path is None. Bez ove provjere, safety gate bi
        # reagirao (rotirao/vozio unatrag) na sve što lidar vidi i
        # kad robot uopće ne pokušava nikamo voziti - npr. dok
        # mirno stoji pored zida ili čeka novi cilj.
        #
        # Zato: ako je ulazna komanda potpuno nula (nema stvarne
        # navigacijske namjere), samo proslijedi nulu i resetiraj
        # FSM - ne diraj ništa na temelju scan-a.
        if abs(msg.linear.x) < 1e-4 and abs(msg.angular.z) < 1e-4:
            self.state = 'NORMAL'
            self.cmd_pub.publish(Twist())
            return

        # Ako je reaktivni avoidance isključen, radi kao čisti
        # stop/slow gate (staro, jednostavno ponašanje) - bez FSM-a
        # za unatrag+rotaciju i bez korekcije kuta.
        if not self.avoidance_enabled:
            out = Twist()
            out.angular.z = msg.angular.z
            if self.min_front_distance < self.stop_distance:
                out.linear.x = 0.0
                self.get_logger().warn(
                    f'SAFETY STOP! Prepreka na {self.min_front_distance:.2f} m ispred.',
                    throttle_duration_sec=1.0
                )
            elif self.min_front_distance < self.slow_distance:
                scale = (
                    (self.min_front_distance - self.stop_distance)
                    / (self.slow_distance - self.stop_distance)
                )
                scale = max(0.0, min(1.0, scale))
                out.linear.x = msg.linear.x * scale
            else:
                out.linear.x = msg.linear.x
            self.cmd_pub.publish(out)
            return

        # --- FSM: NORMAL -> REVERSING -> ROTATING -> NORMAL ---
        #
        # Kad se prepreka pojavi u stop zoni, robot više NE nastavlja
        # naprijed niti se odmah počinje vrtjeti na mjestu (staro
        # ponašanje). Umjesto toga:
        #   1) prvo se malo odmakne unatrag (REVERSING, fiksno
        #      vrijeme), uz sigurnosnu provjeru "leđa" preko punog
        #      360° LIDAR-a - ako je i iza njega preblizu, taj korak
        #      se preskače,
        #   2) zatim rotira u mjestu (ROTATING) prema najboljem
        #      raspoloživom smjeru dok se prolaz ne oslobodi (ili dok
        #      ne istekne sigurnosni timeout),
        #   3) tek onda se vraća na normalno praćenje puta.

        if self.state == 'NORMAL' and self.min_front_distance < self.stop_distance:
            if self.min_rear_distance < self.stop_distance:
                # I iza je preblizu - ne idemo unatrag, ravno na rotaciju
                self.state = 'ROTATING'
                self.get_logger().warn(
                    f'SAFETY STOP! Prepreka ispred ({self.min_front_distance:.2f} m) '
                    f'I iza ({self.min_rear_distance:.2f} m) - preskačem unatrag, '
                    f'rotiram u mjestu.',
                    throttle_duration_sec=1.0
                )
            else:
                self.state = 'REVERSING'
                self.get_logger().warn(
                    f'SAFETY STOP! Prepreka na {self.min_front_distance:.2f} m ispred. '
                    f'Idem unatrag pa rotiram u mjestu.',
                    throttle_duration_sec=1.0
                )
            self.state_start_time = now

        if self.state == 'REVERSING':
            out = Twist()

            # Ako se u međuvremenu i iza pojavi prepreka - prekini
            # vožnju unatrag i idi odmah na rotaciju.
            if self.min_rear_distance < self.stop_distance:
                self.state = 'ROTATING'
                self.state_start_time = now
            elif (now - self.state_start_time) >= self.reverse_duration_sec:
                self.state = 'ROTATING'
                self.state_start_time = now
            else:
                out.linear.x = -self.reverse_speed
                out.angular.z = 0.0
                self.cmd_pub.publish(out)
                return

        if self.state == 'ROTATING':
            out = Twist()
            out.linear.x = 0.0

            if abs(self.steering_angle) > 1e-3:
                out.angular.z = math.copysign(self.rotate_in_place_speed, self.steering_angle)
            else:
                out.angular.z = self.rotate_in_place_speed

            elapsed = now - self.state_start_time
            path_clear = (
                self.min_front_distance >= self.stop_distance
                and not self.path_blocked
            )

            if path_clear or elapsed >= self.max_rotate_duration_sec:
                self.state = 'NORMAL'

                # Manevar (unatrag + rotacija) je gotov - robotova
                # pozicija/orijentacija više ne odgovara staroj
                # putanji. Javi planeru da preplanira prema zadnjem
                # goalu iz nove pozicije, umjesto da pure pursuit
                # nastavi slijepo pratiti stari /plan.
                reason = 'path_clear' if path_clear else 'rotate_timeout'
                notice = String()
                notice.data = reason
                self.replan_pub.publish(notice)

                # nastavljamo dolje na normalnu obradu u ovom istom
                # ciklusu (fallthrough), ne publiciramo rotaciju
            else:
                self.cmd_pub.publish(out)
                return

        # =====================================================
        # NORMALNO PONAŠANJE (state == 'NORMAL')
        # =====================================================

        out = Twist()

        # Reaktivno skretanje prema centru najbližeg valjanog prolaza
        steer_correction = 0.0
        if self.avoidance_enabled:
            max_corr = math.radians(self.max_steer_correction_deg)
            steer_correction = self.angular_gain * self.steering_angle
            steer_correction = max(-max_corr, min(max_corr, steer_correction))

        out.angular.z = msg.angular.z + steer_correction

        if self.min_front_distance < self.slow_distance:
            scale = (
                (self.min_front_distance - self.stop_distance)
                / (self.slow_distance - self.stop_distance)
            )
            scale = max(0.0, min(1.0, scale))
            out.linear.x = msg.linear.x * scale
        else:
            out.linear.x = msg.linear.x

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim safety gate.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
