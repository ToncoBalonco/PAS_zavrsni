#!/usr/bin/env python3
"""
Zone Obstacle Detector
=======================
Dijeli robotov LIDAR FOV na 3 zone (front / left / right) i za
svaku zonu radi jednostavan "cluster" detektor: obrazac se broji
kao stvarna prepreka tek kad postoji N susjednih laserskih točaka
na sličnoj udaljenosti unutar zadanog raspona (band) - jedna
izolirana točka (šum) se ignorira.

FRONT ZONA:
    Kad se detektira klaster u prednjoj zoni na front_distance
    rasponu, te točke se upisuju u lokalni "front_zone_obstacles"
    grid - ISTIH dimenzija/rezolucije/origina kao statička /map
    (isti trik kao dynamic_ob_layer, mark & clear raytracing).
    Taj grid koristi astar_zone_planner.py za merge + replan
    trigger (submapa u koordinatnom sustavu globalne karte, bez
    potrebe za posebnom transformacijom lokalne <-> globalne
    mreže).

    NAPOMENA: reaktivni "stop -> unatrag -> rotacija -> replan"
    manevar (kad robot uđe u vlastitu stop zonu) se izvodi u
    cmd_vel_zone_gate.py, koji radi SVOJU (nezavisnu, na bližem
    rasponu - stop_distance) klaster detekciju nad istim sirovim
    scanom, istom clustering logikom kao ovdje (find_best_cluster).
    Ovaj node samo osigurava da front_zone_obstacles grid bude
    svjež dok se taj manevar izvodi, tako da replan koji uslijedi
    "vidi" upravo taj klaster.

LIJEVA / DESNA ZONA:
    Kad se detektira klaster unutar side_distance raspona,
    publisha se "jačina" (0.0 - 1.0) na /left_avoidance_signal
    ili /right_avoidance_signal. To NE dira globalnu putanju -
    koristi ga cmd_vel_zone_gate.py za blago lateralno skretanje
    na razini brzinskih komandi (reaktivno, niska latencija).

KONVENCIJA KUTA (VAŽNO):
    Zone se računaju iz "sirovog" skener kuta (angle_min + i *
    angle_increment - angle_offset_deg), BEZ TF korekcije (isti
    pristup kao cmd_safety.py) - front/left/right marking mora
    biti brz i ne ovisi o TF-u.

    Parametar `positive_angle_is_left` (default True = standardna
    ROS/REP103 konvencija: pozitivan kut = lijevo od robota) NIJE
    potvrđen za ovaj robot - postavljen je kao parametar upravo
    zato da se može testirati/zamijeniti bez izmjene koda:
        ros2 param set /zone_obstacle_detector positive_angle_is_left false
    ili preko launch parametara.

Topici:
    Pretplate:
        /map    (nav_msgs/OccupancyGrid) - metapodaci za front grid
        /scan   (sensor_msgs/LaserScan)
    Publicira:
        /front_zone_obstacles   (nav_msgs/OccupancyGrid) - isti frame kao /map
        /left_avoidance_signal  (std_msgs/Float32)  0.0 - 1.0
        /right_avoidance_signal (std_msgs/Float32)  0.0 - 1.0
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class ZoneObstacleDetector(Node):

    def __init__(self):
        super().__init__('zone_obstacle_detector')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_grid_topic', '/front_zone_obstacles')
        self.declare_parameter('left_signal_topic', '/left_avoidance_signal')
        self.declare_parameter('right_signal_topic', '/right_avoidance_signal')

        self.declare_parameter('map_frame', 'map')

        # --- Zone (kutovi u stupnjevima, POLU-širina za front) ---
        self.declare_parameter('front_half_angle_deg', 20.0)   # front: -20..+20
        self.declare_parameter('side_angle_min_deg', 20.0)     # side: 20..110
        self.declare_parameter('side_angle_max_deg', 110.0)

        # Kompenzacija ako laser nije poravnat s "naprijed" robota
        self.declare_parameter('angle_offset_deg', 0.0)

        # Konvencija: True = standardni ROS/REP103 (pozitivan kut = lijevo)
        # False = obrnuto (pozitivan kut = desno)
        # NEPOTVRĐENO za ovaj robot - testirati na terenu!
        self.declare_parameter('positive_angle_is_left', True)

        # --- Distance bandovi (m) ---
        self.declare_parameter('front_distance_min', 0.40)
        self.declare_parameter('front_distance_max', 0.60)
        self.declare_parameter('side_distance_min', 0.30)
        self.declare_parameter('side_distance_max', 0.50)

        # --- Clustering ---
        # Min. broj susjednih točaka da se nešto smatra "pravom" preprekom
        self.declare_parameter('min_cluster_points', 5)
        # Max. dopušteni "skok" u range-u (m) između susjednih točaka
        # da se i dalje smatraju istim klasterom
        self.declare_parameter('max_range_jump', 0.15)
        # Max. razmak u indeksima zrake da se smatraju "susjednima"
        # (dopušta angle_stride > 1 na inputu, iako ovdje ne
        # radimo stride po defaultu)
        self.declare_parameter('max_index_gap', 2)

        # --- Front grid (isti princip kao dynamic_ob_layer) ---
        self.declare_parameter('max_range_used', 5.0)
        self.declare_parameter('obstacle_timeout_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('min_process_period_sec', 0.1)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        map_topic = self.get_parameter('map_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        front_grid_topic = self.get_parameter('front_grid_topic').value
        left_signal_topic = self.get_parameter('left_signal_topic').value
        right_signal_topic = self.get_parameter('right_signal_topic').value

        self.map_frame = self.get_parameter('map_frame').value

        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value
        )
        self.side_angle_min = math.radians(
            self.get_parameter('side_angle_min_deg').value
        )
        self.side_angle_max = math.radians(
            self.get_parameter('side_angle_max_deg').value
        )
        self.angle_offset = math.radians(
            self.get_parameter('angle_offset_deg').value
        )
        self.positive_angle_is_left = self.get_parameter(
            'positive_angle_is_left'
        ).value

        self.front_distance_min = self.get_parameter('front_distance_min').value
        self.front_distance_max = self.get_parameter('front_distance_max').value
        self.side_distance_min = self.get_parameter('side_distance_min').value
        self.side_distance_max = self.get_parameter('side_distance_max').value

        self.min_cluster_points = int(
            self.get_parameter('min_cluster_points').value
        )
        self.max_range_jump = self.get_parameter('max_range_jump').value
        self.max_index_gap = int(self.get_parameter('max_index_gap').value)

        self.max_range_used = self.get_parameter('max_range_used').value
        self.obstacle_timeout_sec = self.get_parameter('obstacle_timeout_sec').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.min_process_period_sec = self.get_parameter(
            'min_process_period_sec'
        ).value

        # =====================================================
        # STANJE - FRONT GRID (isti princip kao dynamic_ob_layer)
        # =====================================================

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_ready = False

        self.front_grid = None
        self.obstacle_timestamps = {}
        self.last_process_time = 0.0

        # =====================================================
        # TF (treba nam samo za front grid - world koordinate)
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =====================================================
        # QoS / SUB / PUB
        # =====================================================

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )

        self.front_grid_pub = self.create_publisher(
            OccupancyGrid, front_grid_topic, 10
        )
        self.left_signal_pub = self.create_publisher(
            Float32, left_signal_topic, 10
        )
        self.right_signal_pub = self.create_publisher(
            Float32, right_signal_topic, 10
        )

        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz, self.publish_front_grid
        )
        self.decay_timer = self.create_timer(0.5, self.apply_decay)

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('ZONE OBSTACLE DETECTOR')
        self.get_logger().info('========================================')
        self.get_logger().info(
            f'Front zona:  +/-{math.degrees(self.front_half_angle):.1f} deg, '
            f'band [{self.front_distance_min:.2f}, {self.front_distance_max:.2f}] m'
        )
        self.get_logger().info(
            f'Side zona:   {math.degrees(self.side_angle_min):.1f}..'
            f'{math.degrees(self.side_angle_max):.1f} deg, '
            f'band [{self.side_distance_min:.2f}, {self.side_distance_max:.2f}] m'
        )
        self.get_logger().info(
            f'positive_angle_is_left = {self.positive_angle_is_left} '
            f'(NEPOTVRĐENO - testirati na robotu!)'
        )
        self.get_logger().info(
            f'Min cluster points: {self.min_cluster_points}'
        )
        self.get_logger().info('Čekam kartu i scan...')

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.front_grid = np.full(
            (self.map_height, self.map_width), -1, dtype=np.int8
        )
        self.obstacle_timestamps = {}
        self.map_ready = True

        self.get_logger().info(
            f'Karta primljena za front zone grid: '
            f'{self.map_width}x{self.map_height} @ {self.map_resolution:.3f} m/ćelija'
        )

    # =========================================================
    # WORLD <-> GRID
    # =========================================================

    def world_to_grid(self, x, y):
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col

    def is_valid_cell(self, row, col):
        return 0 <= row < self.map_height and 0 <= col < self.map_width

    # =========================================================
    # KUT -> STRANA (LEFT / RIGHT / FRONT) I NORMALIZACIJA
    # =========================================================

    def classify_angle(self, angle: float) -> str:
        """Vraća 'front', 'left', 'right' ili None za dani (već
        offsetirani i normalizirani) kut u radijanima."""

        if -self.front_half_angle <= angle <= self.front_half_angle:
            return 'front'

        mag = abs(angle)
        if not (self.side_angle_min <= mag <= self.side_angle_max):
            return None

        is_positive_side = angle > 0.0

        if self.positive_angle_is_left:
            return 'left' if is_positive_side else 'right'
        else:
            return 'right' if is_positive_side else 'left'

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # =========================================================
    # CLUSTERING (generička funkcija za jednu zonu)
    # =========================================================

    def find_best_cluster(self, points):
        """
        points: lista (index, angle, range) sortirana po indexu,
        već filtrirana na jednu zonu i jedan distance band.
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
    # SCAN CALLBACK
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        now = time.time()

        front_points = []
        left_points = []
        right_points = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue

            raw_angle = msg.angle_min + i * msg.angle_increment
            angle = self.normalize_angle(raw_angle - self.angle_offset)

            zone = self.classify_angle(angle)
            if zone is None:
                continue

            if zone == 'front':
                if self.front_distance_min <= r <= self.front_distance_max:
                    front_points.append((i, angle, r))
            elif zone == 'left':
                if self.side_distance_min <= r <= self.side_distance_max:
                    left_points.append((i, angle, r))
            elif zone == 'right':
                if self.side_distance_min <= r <= self.side_distance_max:
                    right_points.append((i, angle, r))

        # --- LIJEVA / DESNA ZONA -> SIGNALI ---
        self.publish_side_signal(
            self.left_signal_pub, self.find_best_cluster(left_points)
        )
        self.publish_side_signal(
            self.right_signal_pub, self.find_best_cluster(right_points)
        )

        # --- PREDNJA ZONA -> GRID (throttled + treba TF) ---
        if not self.map_ready:
            return

        if (now - self.last_process_time) < self.min_process_period_sec:
            return
        self.last_process_time = now

        self.process_front_zone(msg, front_points, now)

    # =========================================================
    # SIDE SIGNAL PUBLISH
    # =========================================================

    def publish_side_signal(self, publisher, cluster):
        strength = 0.0

        if cluster is not None:
            ranges = [p[2] for p in cluster]
            mean_range = sum(ranges) / len(ranges)

            band = self.side_distance_max - self.side_distance_min
            if band > 1e-6:
                strength = (self.side_distance_max - mean_range) / band
                strength = max(0.0, min(1.0, strength))
            else:
                strength = 1.0

        msg = Float32()
        msg.data = strength
        publisher.publish(msg)

    # =========================================================
    # FRONT ZONA -> GRID (mark & clear raytracing, isti princip
    # kao dynamic_ob_layer, ali samo za front-zone zrake i samo
    # se MARKAJU ćelije koje pripadaju potvrđenom klasteru)
    # =========================================================

    def process_front_zone(self, msg: LaserScan, front_points, now: float):

        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF (map -> {msg.header.frame_id}) neuspješan: {e}',
                throttle_duration_sec=2.0
            )
            return

        sensor_x = t.transform.translation.x
        sensor_y = t.transform.translation.y

        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        sensor_yaw = math.atan2(siny_cosp, cosy_cosp)

        sensor_row, sensor_col = self.world_to_grid(sensor_x, sensor_y)

        # Koje zrake pripadaju potvrđenom klasteru (za marking) -
        # sve ostale front-zone zrake se samo koriste za čišćenje
        # (raytrace clear), isto kao u dynamic_ob_layer.
        cluster = self.find_best_cluster(front_points)
        cluster_indices = set(p[0] for p in cluster) if cluster else set()

        if cluster is not None:
            self.get_logger().info(
                f'FRONT klaster detektiran: {len(cluster)} točaka, '
                f'srednji range ~{sum(p[2] for p in cluster) / len(cluster):.2f} m',
                throttle_duration_sec=1.0
            )

        max_range = min(self.max_range_used, msg.range_max)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min:
                continue

            raw_angle = msg.angle_min + i * msg.angle_increment
            local_angle = self.normalize_angle(raw_angle - self.angle_offset)

            if not (-self.front_half_angle <= local_angle <= self.front_half_angle):
                continue

            # Koristimo apsolutni (world) kut za raytracing (TF yaw +
            # sirovi kut - offset). VAŽNO: mora se oduzeti isti
            # angle_offset kao i kod klasifikacije front/left/right
            # gore (local_angle), inače se klaster ISPRAVNO prepozna
            # kao "front" ali se u grid upiše na fizički pogrešno
            # (zrcaljeno/rotirano) mjesto na karti.
            world_angle = (
                msg.angle_min + i * msg.angle_increment
                - self.angle_offset + sensor_yaw
            )

            is_hit = r <= max_range
            ray_len = min(r, max_range)

            end_x = sensor_x + ray_len * math.cos(world_angle)
            end_y = sensor_y + ray_len * math.sin(world_angle)

            end_row, end_col = self.world_to_grid(end_x, end_y)

            self.raytrace_clear(
                sensor_row, sensor_col, end_row, end_col,
                clear_endpoint=not is_hit
            )

            if is_hit and i in cluster_indices and self.is_valid_cell(end_row, end_col):
                self.front_grid[end_row, end_col] = 100
                self.obstacle_timestamps[(end_row, end_col)] = now

    # =========================================================
    # RAYTRACING (BRESENHAM) - isto kao dynamic_ob_layer
    # =========================================================

    def raytrace_clear(self, r0, c0, r1, c1, clear_endpoint=False):
        points = self.bresenham(r0, c0, r1, c1)
        end_index = len(points) if clear_endpoint else len(points) - 1

        for r, c in points[:end_index]:
            if self.is_valid_cell(r, c) and self.front_grid[r, c] == 100:
                self.front_grid[r, c] = -1
                self.obstacle_timestamps.pop((r, c), None)

    @staticmethod
    def bresenham(r0, c0, r1, c1):
        points = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0

        while True:
            points.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

        return points

    # =========================================================
    # DECAY
    # =========================================================

    def apply_decay(self):
        if self.front_grid is None:
            return

        now = time.time()
        expired = [
            cell for cell, ts in self.obstacle_timestamps.items()
            if (now - ts) > self.obstacle_timeout_sec
        ]

        for r, c in expired:
            if self.is_valid_cell(r, c):
                self.front_grid[r, c] = -1
            self.obstacle_timestamps.pop((r, c), None)

    # =========================================================
    # PUBLISH FRONT GRID
    # =========================================================

    def publish_front_grid(self):
        if self.front_grid is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.front_grid.flatten().tolist()

        self.front_grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim zone obstacle detector.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
