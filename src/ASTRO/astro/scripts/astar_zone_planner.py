#!/usr/bin/env python3
"""
A* Path Planner - Zone Edition
================================
Ista logika kao astar_path_planner_ob.py (globalna A* putanja +
merge/inflate + throttled replan monitor), samo umjesto punog
360-stupanjskog /dynamic_obstacles sloja koristi /front_zone_obstacles
iz zone_obstacle_detector.py - dakle replan se okida SAMO kad je
potvrđen (klasterom) obstacle u PREDNJOJ zoni robota blizu putanje.

Bočne prepreke (left/right zone) se NE tiču ovog plannera - njih
rješava cmd_vel_zone_gate.py izravno na razini brzinskih komandi
(reaktivno skretanje), bez diranja globalne putanje.

FORCE REPLAN (IZMIJENJENO):
    Sluša /force_replan (std_msgs/Empty). cmd_vel_zone_gate sada,
    kad potvrdi klaster u stop zoni, radi cijeli manevar prije nego
    zatraži replan: HARD_STOP (potpuna nula, hard_stop_hold_sec) ->
    REVERSING (odmakne se ~reverse_distance_m unatrag) -> ROTATING
    (zarotira se ~rotate_angle_deg OD klastera). Tek na kraju tog
    manevra šalje /force_replan - planner tada odmah (BEZ čekanja
    na path_monitor_callback i BEZ replan_cooldown_sec throttlea,
    jer ga cmd_vel_zone_gate već throttlea svojim
    replan_request_cooldown_sec) radi novi A* prema istom cilju
    koristeći trenutni front_zone_obstacles grid - koji do tog
    trena sadrži upravo detektirani klaster (zone_obstacle_detector
    je cijelo vrijeme radio dok se manevar izvodio). Robot je u tom
    trenu već odmaknut i zarotiran OD prepreke, pa novi A* ima veću
    šansu naći put oko klastera nego kad bi replan tražio odmah
    pored prepreke.

Topici:
    Pretplate:
        /map                   (nav_msgs/OccupancyGrid)
        /goal_pose             (geometry_msgs/PoseStamped)  <- RViz 2D Goal Pose
        /front_zone_obstacles  (nav_msgs/OccupancyGrid)     <- iz zone_obstacle_detector
        /force_replan          (std_msgs/Empty)             <- iz cmd_vel_zone_gate

    Publicira:
        /plan         (nav_msgs/Path)
"""

import math
import time
import heapq

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

import tf2_ros

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class AStarZonePlanner(Node):

    def __init__(self):
        super().__init__('astar_zone_planner')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('front_zone_obstacles_topic', '/front_zone_obstacles')
        self.declare_parameter('force_replan_topic', '/force_replan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Obstacle inflation - proširuje prepreke radi sigurnosti
        self.declare_parameter('inflation_radius_m', 0.35)

        # Prag iznad kojeg se ćelija smatra preprekom (0-100)
        self.declare_parameter('obstacle_threshold', 50)

        # Zasebna (obično manja) inflacija SAMO za front-zone
        # klaster točke - neovisna od inflation_radius_m koji vrijedi
        # za statičku kartu. Klaster točke su svježe/potvrđene ali
        # rijetke (par ćelija), pa ne treba isti "debeli" sigurnosni
        # rub kao za zidove.
        self.declare_parameter('front_zone_inflation_m', 0.10)

        # --- replanning / path monitoring parametri ---

        # Koliko često provjeravamo je li putanja blokirana [s]
        self.declare_parameter('path_monitor_period_sec', 0.3)

        # Koliko daleko ispred robota (po putanji) provjeravamo [m]
        self.declare_parameter('check_horizon_m', 2.0)

        # Min. razmak između dva replana [s]
        self.declare_parameter('replan_cooldown_sec', 1.5)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        front_zone_obstacles_topic = self.get_parameter(
            'front_zone_obstacles_topic'
        ).value
        force_replan_topic = self.get_parameter('force_replan_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.inflation_radius_m = self.get_parameter('inflation_radius_m').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.front_zone_inflation_m = self.get_parameter(
            'front_zone_inflation_m'
        ).value

        self.path_monitor_period_sec = self.get_parameter(
            'path_monitor_period_sec'
        ).value
        self.check_horizon_m = self.get_parameter('check_horizon_m').value
        self.replan_cooldown_sec = self.get_parameter('replan_cooldown_sec').value

        # =====================================================
        # VARIJABLE KARTE
        # =====================================================

        self.map_data = None        # trenutna PLANNING grid (static+front, inflated)
        self.map_raw = None         # numpy array (original, static, bez inflacije)
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.front_zone_raw = None          # numpy array iz /front_zone_obstacles
        self.last_goal_world = None
        self.current_full_grid_path = None
        self.last_replan_time = 0.0

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.plan_pub = self.create_publisher(Path, plan_topic, 10)

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )

        front_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.front_zone_sub = self.create_subscription(
            OccupancyGrid,
            front_zone_obstacles_topic,
            self.front_zone_callback,
            front_qos
        )

        self.force_replan_sub = self.create_subscription(
            Empty,
            force_replan_topic,
            self.force_replan_callback,
            10
        )

        self.path_monitor_timer = self.create_timer(
            self.path_monitor_period_sec, self.path_monitor_callback
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('ASTRO A* PATH PLANNER - ZONE EDITION')
        self.get_logger().info('========================================')
        self.get_logger().info(f'Map topic:       {map_topic}')
        self.get_logger().info(f'Goal topic:      {goal_topic}')
        self.get_logger().info(f'Plan topic:      {plan_topic}')
        self.get_logger().info(f'Front zone obst: {front_zone_obstacles_topic}')
        self.get_logger().info(f'Force replan:    {force_replan_topic}')
        self.get_logger().info(f'TF:              {self.map_frame} -> {self.base_frame}')
        self.get_logger().info(f'Inflation static:{self.inflation_radius_m:.2f} m')
        self.get_logger().info(
            f'Inflation front: {self.front_zone_inflation_m:.2f} m'
        )
        self.get_logger().info(f'Obstacle thr:    {self.obstacle_threshold}')
        self.get_logger().info(
            f'Check horizon:   {self.check_horizon_m:.2f} m, '
            f'replan cooldown: {self.replan_cooldown_sec:.2f} s'
        )
        self.get_logger().info('Čekam kartu i goal_pose...')

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width)
        )

        self.map_raw = raw

        # Nova statička karta -> stari front-zone sloj više ne
        # odgovara dimenzijama, resetiraj ga.
        self.front_zone_raw = None

        self.map_data = self.build_planning_grid()

        self.get_logger().info(
            f'Karta primljena: {self.map_width}x{self.map_height} '
            f'@ {self.map_resolution:.3f} m/ćelija',
            throttle_duration_sec=5.0
        )

    # =========================================================
    # FRONT ZONE OBSTACLES CALLBACK
    # =========================================================

    def front_zone_callback(self, msg: OccupancyGrid):
        if self.map_raw is None:
            return

        arr = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        if arr.shape != self.map_raw.shape:
            self.get_logger().warn(
                'Dimenzije /front_zone_obstacles ne odgovaraju statičkoj karti! '
                'Ignoriram poruku (provjeri jesu li oba node-a spojena na istu /map).',
                throttle_duration_sec=5.0
            )
            return

        self.front_zone_raw = arr

    # =========================================================
    # BUILD PLANNING GRID
    # =========================================================

    def build_planning_grid(self) -> np.ndarray:
        """
        Statička karta i front-zone klaster točke se sada
        inflate-aju ODVOJENO, svaka sa svojim radijusom, pa se
        rezultati spoje OR logikom:

            static  -> inflate_grid(map_raw, inflation_radius_m)
            front   -> inflate_grid(front_zone_raw, front_zone_inflation_m)
            merged  -> static_inflated OR front_inflated

        Ovo omogućuje da klaster točke (svježe, potvrđene, ali
        malobrojne) imaju svoj, obično manji, sigurnosni rub bez
        utjecaja na inflaciju cijele statičke karte (zidova).
        """

        static_inflated = self.inflate_grid(self.map_raw, self.inflation_radius_m)

        if self.front_zone_raw is not None and self.front_zone_raw.shape == self.map_raw.shape:
            front_inflated = self.inflate_grid(
                self.front_zone_raw, self.front_zone_inflation_m
            )

            merged = static_inflated.copy()
            merged[front_inflated > self.obstacle_threshold] = 100

            n_front = int(np.sum(front_inflated > self.obstacle_threshold))
            self.get_logger().info(
                f'Front-zone inflation ({self.front_zone_inflation_m:.2f} m) '
                f'gotova. Ćelija: {n_front}',
                throttle_duration_sec=2.0
            )

            return merged

        return static_inflated

    # =========================================================
    # INFLATE GRID (generička funkcija, radijus kao parametar)
    # =========================================================

    def inflate_grid(self, grid: np.ndarray, radius_m: float) -> np.ndarray:
        """Proširuje prepreke u zadanom gridu za radius_m metara."""

        inflation_cells = int(math.ceil(radius_m / self.map_resolution))

        inflated = grid.copy()

        obstacle_rows, obstacle_cols = np.where(grid > self.obstacle_threshold)

        for r, c in zip(obstacle_rows, obstacle_cols):
            r_min = max(0, r - inflation_cells)
            r_max = min(self.map_height - 1, r + inflation_cells)
            c_min = max(0, c - inflation_cells)
            c_max = min(self.map_width - 1, c + inflation_cells)

            inflated[r_min:r_max + 1, c_min:c_max + 1] = 100

        n_inflated = int(np.sum(inflated > self.obstacle_threshold))

        self.get_logger().info(
            f'Inflation ({radius_m:.2f} m) gotova. Ćelija prepreka: {n_inflated}',
            throttle_duration_sec=2.0
        )

        return inflated

    # =========================================================
    # WORLD <-> GRID KONVERZIJA
    # =========================================================

    def world_to_grid(self, x: float, y: float):
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        return row, col

    def grid_to_world(self, row: int, col: int):
        x = col * self.map_resolution + self.map_origin_x + (self.map_resolution / 2.0)
        y = row * self.map_resolution + self.map_origin_y + (self.map_resolution / 2.0)
        return x, y

    # =========================================================
    # VALIDACIJA GRID POZICIJE
    # =========================================================

    def is_valid_cell(self, row: int, col: int) -> bool:
        return 0 <= row < self.map_height and 0 <= col < self.map_width

    def is_free_cell(self, row: int, col: int) -> bool:
        if not self.is_valid_cell(row, col):
            return False
        val = self.map_data[row, col]
        return 0 <= val <= self.obstacle_threshold

    # =========================================================
    # GET ROBOT POSE
    # =========================================================

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:
            self.get_logger().warn(
                f'TF lookup neuspješan: {e}', throttle_duration_sec=2.0
            )
            return None

    # =========================================================
    # GOAL CALLBACK
    # =========================================================

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info('========================================')
        self.get_logger().info('Goal primljen iz RViz-a.')

        if self.map_data is None:
            self.get_logger().error('Karta još nije primljena! Odbacujem goal.')
            return

        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            self.get_logger().error('Ne mogu dobiti poziciju robota! Odbacujem goal.')
            return

        rx, ry = robot_pos
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        self.get_logger().info(f'Robot:  ({rx:.3f}, {ry:.3f})')
        self.get_logger().info(f'Goal:   ({gx:.3f}, {gy:.3f})')

        self.last_goal_world = (gx, gy)

        self.plan_to_goal((rx, ry), (gx, gy))

        self.get_logger().info('========================================')

    # =========================================================
    # PLAN TO GOAL
    # =========================================================

    def plan_to_goal(self, robot_world: tuple, goal_world: tuple) -> bool:
        rx, ry = robot_world
        gx, gy = goal_world

        self.map_data = self.build_planning_grid()

        start_grid = self.world_to_grid(rx, ry)
        goal_grid = self.world_to_grid(gx, gy)

        self.get_logger().info(
            f'Start grid: row={start_grid[0]}, col={start_grid[1]}'
        )
        self.get_logger().info(
            f'Goal grid:  row={goal_grid[0]}, col={goal_grid[1]}'
        )

        if not self.is_valid_cell(*start_grid):
            self.get_logger().error('Start je izvan granica karte!')
            return False

        if not self.is_valid_cell(*goal_grid):
            self.get_logger().error('Goal je izvan granica karte!')
            return False

        if not self.is_free_cell(*goal_grid):
            self.get_logger().warn(
                'Goal je u prepreci ili nepoznatom području! '
                'Pokušavam svejedno planirati...'
            )

        self.get_logger().info('Pokrećem A* planiranje...')

        grid_path = self.run_astar(start_grid, goal_grid)

        if grid_path is None:
            self.get_logger().error('A* nije pronašao putanju!')
            return False

        self.get_logger().info(f'A* pronašao putanju s {len(grid_path)} ćelija.')

        self.current_full_grid_path = grid_path

        pruned = self.prune_path(grid_path)

        self.get_logger().info(f'Nakon čišćenja: {len(pruned)} waypointa.')

        world_points = [self.grid_to_world(r, c) for r, c in pruned]

        self.publish_plan(world_points)

        self.get_logger().info('Putanja objavljena na /plan.')

        return True

    # =========================================================
    # PATH MONITOR - provjerava je li putanja blokirana front-zone
    # klasterom
    # =========================================================

    def path_monitor_callback(self):
        if self.current_full_grid_path is None:
            return

        if self.front_zone_raw is None or self.map_raw is None:
            return

        if self.front_zone_raw.shape != self.map_raw.shape:
            return

        obstacle_cells = np.argwhere(self.front_zone_raw == 100)

        if obstacle_cells.shape[0] == 0:
            return

        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            return

        rx, ry = robot_pos
        r_row, r_col = self.world_to_grid(rx, ry)

        path_arr = np.array(self.current_full_grid_path, dtype=np.float64)

        dists_to_robot = np.hypot(
            path_arr[:, 0] - r_row, path_arr[:, 1] - r_col
        )
        nearest_idx = int(np.argmin(dists_to_robot))

        horizon_cells = int(math.ceil(self.check_horizon_m / self.map_resolution))
        horizon_path = path_arr[nearest_idx: nearest_idx + horizon_cells]

        if horizon_path.shape[0] == 0:
            return

        diff_r = horizon_path[:, 0:1] - obstacle_cells[None, :, 0]
        diff_c = horizon_path[:, 1:2] - obstacle_cells[None, :, 1]
        dist_cells = np.sqrt(diff_r ** 2 + diff_c ** 2)

        min_dist_m = float(np.min(dist_cells)) * self.map_resolution
        safety_margin = self.inflation_radius_m + self.map_resolution

        if min_dist_m < safety_margin:
            self.trigger_replan(
                f'Front-zone prepreka na putanji (razmak {min_dist_m:.2f} m, '
                f'horizon {self.check_horizon_m:.1f} m)'
            )

    # =========================================================
    # FORCE REPLAN (NOVO) - direktan zahtjev od cmd_vel_zone_gate
    # nakon hard-stop perioda, BEZ replan_cooldown_sec throttlea
    # (taj throttle je već primijenjen na strani gate-a preko
    # replan_request_cooldown_sec)
    # =========================================================

    def force_replan_callback(self, msg: Empty):
        if self.last_goal_world is None:
            self.get_logger().warn(
                'Force replan zatražen, ali nema aktivnog cilja. Ignoriram.'
            )
            return

        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            self.get_logger().warn(
                'Force replan zatražen, ali TF pozicija robota nije dostupna. '
                'Ignoriram.'
            )
            return

        self.get_logger().warn('========================================')
        self.get_logger().warn(
            'FORCE REPLAN (hard stop) - nova A* putanja oko klastera.'
        )
        self.get_logger().warn('========================================')

        # Ažuriraj i last_replan_time da se path_monitor_callback
        # (throttled preko replan_cooldown_sec) ne pobije s ovim
        # tek izvršenim replanom.
        self.last_replan_time = time.time()

        self.plan_to_goal(robot_pos, self.last_goal_world)

    # =========================================================
    # TRIGGER REPLAN
    # =========================================================

    def trigger_replan(self, reason: str):
        now = time.time()

        if (now - self.last_replan_time) < self.replan_cooldown_sec:
            return

        if self.last_goal_world is None:
            return

        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            return

        self.last_replan_time = now

        self.get_logger().warn('========================================')
        self.get_logger().warn(f'REPLAN OKINUT: {reason}')
        self.get_logger().warn('========================================')

        self.plan_to_goal(robot_pos, self.last_goal_world)

    # =========================================================
    # A* ALGORITAM
    # =========================================================

    def run_astar(self, start: tuple, goal: tuple):
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        neighbors = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)),
            ( 1,  1, math.sqrt(2)),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            cr, cc = current

            for dr, dc, move_cost in neighbors:
                nr = cr + dr
                nc = cc + dc
                neighbor = (nr, nc)

                if not self.is_free_cell(nr, nc):
                    continue

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    h = math.hypot(nr - goal[0], nc - goal[1])
                    f = tentative_g + h

                    heapq.heappush(open_set, (f, neighbor))

        return None

    # =========================================================
    # REKONSTRUKCIJA PUTANJE
    # =========================================================

    def reconstruct_path(self, came_from: dict, current: tuple):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return path

    # =========================================================
    # ČIŠĆENJE PUTANJE (PRUNE)
    # =========================================================

    def prune_path(self, grid_path: list) -> list:
        if len(grid_path) <= 2:
            return grid_path

        pruned = [grid_path[0]]

        for i in range(1, len(grid_path) - 1):
            prev_r, prev_c = pruned[-1]
            curr_r, curr_c = grid_path[i]
            next_r, next_c = grid_path[i + 1]

            dir1 = (curr_r - prev_r, curr_c - prev_c)
            dir2 = (next_r - curr_r, next_c - curr_c)

            if dir1 != dir2:
                pruned.append(grid_path[i])

        pruned.append(grid_path[-1])
        return pruned

    # =========================================================
    # PUBLISH PLAN
    # =========================================================

    def publish_plan(self, world_points: list):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in world_points:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.plan_pub.publish(path_msg)


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = AStarZonePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim planer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
