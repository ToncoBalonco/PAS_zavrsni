#!/usr/bin/env python3
"""
Local Obstacle Layer (scan-only, bez /map i bez TF-a)
========================================================
Prati /scan i gradi lokalnu "rolling window" occupancy grid
centriranu na sam senzor (dakle i na robota, jer je senzor kruto
montiran na njega) - isti "mark & clear" raytracing princip kao
prije (costmap_2d obstacle layer stil):
    - Za svaku laser zraku koja pogodi nešto unutar
      `local_size_m`, ćelija pogotka se označi kao prepreka (100).
    - Sve ćelije DUŽ zrake (prije pogotka) se čiste
      (postavljaju na -1 = "nema prepreke"), jer laser "vidi kroz"
      njih pa znamo da su trenutno slobodne.
    - Oznake koje se ne osvježe unutar `obstacle_timeout_sec`
      automatski nestaju (decay) - sigurnosna mreža za slijepe
      kuteve / uski FOV senzora.

ZAŠTO VIŠE NEMA /map I TF-a (map -> scan frame):
    Stara verzija je za svaku scan poruku radila TF lookup
    (map -> scan frame) da bi znala gdje se senzor nalazi u
    globalnoj karti, i grid je bio poravnat s /map (ista
    rezolucija/origin). To je zahtijevalo cijeli TF lanac kroz
    AMCL (map -> odom), koji je vremenski promjenjiv i loman kad
    računalo na robotu i operatersko računalo nisu vremenski
    usklađena (nema NTP-a između strojeva) - lookup bi povremeno
    bacao ExtrapolationException ili vraćao zastarjelu poziciju.

    Ova verzija grid gradi ISKLJUČIVO iz /scan poruke, u frame-u
    samog senzora (msg.header.frame_id) - laserske zrake su već
    dane relativno na senzor/robota, pa nikakav TF lookup nije
    potreban za samo računanje grida. Grid se "vozi" s robotom
    automatski jer je izražen u robot-relativnom frame-u.

    Rezultat je i dalje koristan za RViz vizualizaciju (RViz sam
    koristi statičku, vremenski neosjetljivu transformaciju
    base_link -> laser iz robot_state_publishera da ga ispravno
    prikaže na robotu), ali za samo obstacle avoidance koristi se
    isključivo cmd_safety.py, koji čita /scan direktno.

Topici:
    Pretplate:
        /scan   (sensor_msgs/LaserScan)
    Publicira:
        /dynamic_obstacles (nav_msgs/OccupancyGrid) - lokalni grid,
        frame_id = frame senzora (npr. laser_frame), centriran na
        senzor/robota, veličine `local_size_m` x `local_size_m`.
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class DynamicObstacleLayer(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_layer')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/dynamic_obstacles')

        # Veličina lokalnog (kvadratnog) grida oko senzora [m].
        # Ne treba biti velika - koristi se za lokalni avoidance,
        # ne za globalno planiranje.
        self.declare_parameter('local_size_m', 6.0)

        # Rezolucija ćelije lokalnog grida [m]
        self.declare_parameter('resolution', 0.05)

        # Do koje udaljenosti (m) uzimamo laser očitanja u obzir.
        # Ograničeno i s local_size_m/2 (ne može biti izvan grida).
        self.declare_parameter('max_range_used', 5.0)

        # Obradi svaku N-tu zraku (radi brzine). 1 = sve zrake.
        self.declare_parameter('angle_stride', 2)

        # Koliko dugo prepreka "ostaje" ako je laser više ne vidi
        # (npr. izvan trenutnog FOV-a) [sekunde]
        self.declare_parameter('obstacle_timeout_sec', 2.0)

        # Min. razmak između obrade dva scana [sekunde] - throttle
        self.declare_parameter('min_process_period_sec', 0.1)

        self.declare_parameter('publish_rate_hz', 10.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        scan_topic = self.get_parameter('scan_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.local_size_m = float(self.get_parameter('local_size_m').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.max_range_used = float(self.get_parameter('max_range_used').value)
        self.angle_stride = max(1, int(self.get_parameter('angle_stride').value))
        self.obstacle_timeout_sec = float(self.get_parameter('obstacle_timeout_sec').value)
        self.min_process_period_sec = float(self.get_parameter('min_process_period_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # =====================================================
        # GRID GEOMETRIJA (fiksna, centrirana na senzor)
        # =====================================================

        self.grid_size = max(1, int(round(self.local_size_m / self.resolution)))
        # senzor je uvijek u centru grida
        self.sensor_row = self.grid_size // 2
        self.sensor_col = self.grid_size // 2
        # origin grida (donji-lijevi kut) relativno na senzor/robota
        self.grid_origin = -(self.grid_size / 2.0) * self.resolution

        # dynamic_grid: -1 = nema prepreke, 100 = prepreka
        self.dynamic_grid = np.full(
            (self.grid_size, self.grid_size), -1, dtype=np.int8
        )
        self.obstacle_timestamps = {}
        self.last_process_time = 0.0
        self.sensor_frame_id = None  # postavlja se iz prve /scan poruke

        # =====================================================
        # SUB / PUB (bez TF, bez /map)
        # =====================================================

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )

        self.grid_pub = self.create_publisher(OccupancyGrid, output_topic, 10)

        self.publish_timer = self.create_timer(
            1.0 / publish_rate_hz, self.publish_grid
        )

        self.decay_timer = self.create_timer(0.5, self.apply_decay)

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('LOCAL OBSTACLE LAYER (samo /scan, bez /map i TF-a)')
        self.get_logger().info('========================================')
        self.get_logger().info(f'Scan topic:       {scan_topic}')
        self.get_logger().info(f'Output topic:     {output_topic}')
        self.get_logger().info(f'Local size:       {self.local_size_m:.2f} m ({self.grid_size}x{self.grid_size} ćelija)')
        self.get_logger().info(f'Resolution:       {self.resolution:.3f} m/ćelija')
        self.get_logger().info(f'Max range used:   {self.max_range_used:.2f} m')
        self.get_logger().info(f'Angle stride:     {self.angle_stride}')
        self.get_logger().info(f'Obstacle timeout: {self.obstacle_timeout_sec:.2f} s')
        self.get_logger().info('Čekam /scan...')

    # =========================================================
    # LOKALNE (senzor-centrirane) <-> GRID KOORDINATE
    # =========================================================

    def local_to_grid(self, x, y):
        """x, y su metri relativno na senzor (x naprijed, y lijevo)."""
        col = self.sensor_col + int(round(x / self.resolution))
        row = self.sensor_row + int(round(y / self.resolution))
        return row, col

    def is_valid_cell(self, row, col):
        return 0 <= row < self.grid_size and 0 <= col < self.grid_size

    # =========================================================
    # SCAN CALLBACK - direktno iz /scan, bez TF-a
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        if (now - self.last_process_time) < self.min_process_period_sec:
            return
        self.last_process_time = now

        self.sensor_frame_id = msg.header.frame_id

        max_range = min(self.max_range_used, msg.range_max, self.local_size_m / 2.0)

        for i in range(0, len(msg.ranges), self.angle_stride):
            r = msg.ranges[i]

            # VAŽNO: ne odbacujemo očitanja samo zato što su ispod
            # msg.range_min - stvarni LIDAR-ovi znaju vratiti valjanu
            # udaljenost i za prepreke bliže od nominalnog range_min
            # (npr. 0.15-0.17 m). Odbacujemo samo istinski nevaljana
            # očitanja (nan/inf/<=0), jer bi filtriranje po range_min
            # sakrilo baš najbliže, najopasnije prepreke iz grida.
            if not math.isfinite(r) or r <= 0.0:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            is_hit = r <= max_range
            ray_len = min(r, max_range)

            # x naprijed, y lijevo - direktno iz scan poruke, bez TF-a
            end_x = ray_len * math.cos(angle)
            end_y = ray_len * math.sin(angle)

            end_row, end_col = self.local_to_grid(end_x, end_y)

            self.raytrace_clear(
                self.sensor_row, self.sensor_col, end_row, end_col,
                clear_endpoint=not is_hit
            )

            if is_hit and self.is_valid_cell(end_row, end_col):
                self.dynamic_grid[end_row, end_col] = 100
                self.obstacle_timestamps[(end_row, end_col)] = now

    # =========================================================
    # RAYTRACING (BRESENHAM) - ČIŠĆENJE DUŽ ZRAKE
    # =========================================================

    def raytrace_clear(self, r0, c0, r1, c1, clear_endpoint=False):
        points = self.bresenham(r0, c0, r1, c1)

        end_index = len(points) if clear_endpoint else len(points) - 1

        for r, c in points[:end_index]:
            if self.is_valid_cell(r, c) and self.dynamic_grid[r, c] == 100:
                self.dynamic_grid[r, c] = -1
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
        now = time.time()
        expired = [
            cell for cell, ts in self.obstacle_timestamps.items()
            if (now - ts) > self.obstacle_timeout_sec
        ]

        for r, c in expired:
            if self.is_valid_cell(r, c):
                self.dynamic_grid[r, c] = -1
            self.obstacle_timestamps.pop((r, c), None)

    # =========================================================
    # PUBLISH
    # =========================================================

    def publish_grid(self):
        if self.sensor_frame_id is None:
            return  # još nismo primili niti jedan /scan

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Frame senzora - grid je uvijek centriran na robota/senzor,
        # RViz ga ispravno prikazuje preko postojeće (statičke)
        # base_link -> laser transformacije, bez da ovaj node treba
        # bilo kakav TF lookup.
        msg.header.frame_id = self.sensor_frame_id

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.grid_origin
        msg.info.origin.position.y = self.grid_origin
        msg.info.origin.orientation.w = 1.0

        msg.data = self.dynamic_grid.flatten().tolist()

        self.grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim local obstacle layer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
