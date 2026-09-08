#!/usr/bin/env python3

import math
import heapq

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import tf2_ros

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class AStarPathPlanner(Node):
    """
    Planira putanju A* algoritmom i publishuje je na /plan.
    Sluša goal_pose iz RViz-a, uzima trenutnu poziciju robota
    iz TF-a (map -> base_footprint) i računa putanju kroz kartu.

    Topici:
        Pretplate:
            /map          (nav_msgs/OccupancyGrid)
            /goal_pose    (geometry_msgs/PoseStamped)  <- RViz 2D Goal Pose

        Publicira:
            /plan         (nav_msgs/Path)
    """

    def __init__(self):
        super().__init__('astar_path_planner')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Obstacle inflation - proširuje prepreke radi sigurnosti
        self.declare_parameter('inflation_radius_m', 0.35)

        # Prag iznad kojeg se ćelija smatra preprekom (0-100)
        self.declare_parameter('obstacle_threshold', 50) #bio je 50

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.inflation_radius_m = self.get_parameter(
            'inflation_radius_m'
        ).value

        self.obstacle_threshold = self.get_parameter(
            'obstacle_threshold'
        ).value

        # =====================================================
        # VARIJABLE KARTE
        # =====================================================

        self.map_data = None        # numpy array (inflated)
        self.map_raw = None         # numpy array (original)
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.plan_pub = self.create_publisher(
            Path,
            plan_topic,
            10
        )

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        # Nav2 map_server publishuje s transient_local QoS —
        # subscriber mora koristiti isti profil da dobije kartu
        # čak i ako se pretplati nakon što je karta objavljena.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('ASTRO A* PATH PLANNER')
        self.get_logger().info('========================================')
        self.get_logger().info(f'Map topic:    {map_topic}')
        self.get_logger().info(f'Goal topic:   {goal_topic}')
        self.get_logger().info(f'Plan topic:   {plan_topic}')
        self.get_logger().info(f'TF:           {self.map_frame} -> {self.base_frame}')
        self.get_logger().info(
            f'Inflation:    {self.inflation_radius_m:.2f} m'
        )
        self.get_logger().info(f'Obstacle thr: {self.obstacle_threshold}')
        self.get_logger().info('Čekam kartu i goal_pose...')

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg: OccupancyGrid):
        """Prima kartu iz AMCL-a / map_server-a i primjenjuje inflation."""

        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width)
        )

        self.map_raw = raw
        self.map_data = self.inflate_obstacles(raw)

        self.get_logger().info(
            f'Karta primljena: {self.map_width}x{self.map_height} '
            f'@ {self.map_resolution:.3f} m/ćelija',
            throttle_duration_sec=5.0
        )

    # =========================================================
    # INFLATE OBSTACLES
    # =========================================================

    def inflate_obstacles(self, grid: np.ndarray) -> np.ndarray:
        """
        Proširuje prepreke za inflation_radius_m metara.
        Radi se jednom pri primanju karte – ne usporava planiranje.
        """

        inflation_cells = int(
            math.ceil(self.inflation_radius_m / self.map_resolution)
        )

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
            f'Inflation gotova. Ćelija prepreka: {n_inflated}'
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
        return (
            0 <= row < self.map_height
            and 0 <= col < self.map_width
        )

    def is_free_cell(self, row: int, col: int) -> bool:
        """Slobodna ćelija = nije prepreka i nije nepoznato područje."""
        if not self.is_valid_cell(row, col):
            return False
        val = self.map_data[row, col]
        # -1 = nepoznato, >threshold = prepreka
        return 0 <= val <= self.obstacle_threshold

    # =========================================================
    # GET ROBOT POSE
    # =========================================================

    def get_robot_pose(self):
        """Vraća (x, y) robota u map frame-u ili None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
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
                f'TF lookup neuspješan: {e}',
                throttle_duration_sec=2.0
            )
            return None

    # =========================================================
    # GOAL CALLBACK
    # =========================================================

    def goal_callback(self, msg: PoseStamped):
        """Poziva se kad RViz pošalje 2D Goal Pose."""

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

        # -----------------------------------------------
        # Grid koordinate
        # -----------------------------------------------

        start_grid = self.world_to_grid(rx, ry)
        goal_grid = self.world_to_grid(gx, gy)

        self.get_logger().info(
            f'Start grid: row={start_grid[0]}, col={start_grid[1]}'
        )
        self.get_logger().info(
            f'Goal grid:  row={goal_grid[0]}, col={goal_grid[1]}'
        )

        # -----------------------------------------------
        # Provjera granica
        # -----------------------------------------------

        if not self.is_valid_cell(*start_grid):
            self.get_logger().error('Start je izvan granica karte!')
            return

        if not self.is_valid_cell(*goal_grid):
            self.get_logger().error('Goal je izvan granica karte!')
            return

        if not self.is_free_cell(*goal_grid):
            self.get_logger().warn(
                'Goal je u prepreci ili nepoznatom području! '
                'Pokušavam svejedno planirati...'
            )

        # -----------------------------------------------
        # A* planiranje
        # -----------------------------------------------

        self.get_logger().info('Pokrećem A* planiranje...')

        grid_path = self.run_astar(start_grid, goal_grid)

        if grid_path is None:
            self.get_logger().error('A* nije pronašao putanju!')
            return

        self.get_logger().info(
            f'A* pronašao putanju s {len(grid_path)} ćelija.'
        )

        # -----------------------------------------------
        # Ukloni redundantne točke
        # -----------------------------------------------

        pruned = self.prune_path(grid_path)

        self.get_logger().info(
            f'Nakon čišćenja: {len(pruned)} waypointa.'
        )

        # -----------------------------------------------
        # World koordinate i publish
        # -----------------------------------------------

        world_points = [
            self.grid_to_world(r, c) for r, c in pruned
        ]

        self.publish_plan(world_points)

        self.get_logger().info('Putanja objavljena na /plan.')
        self.get_logger().info('========================================')

    # =========================================================
    # A* ALGORITAM
    # =========================================================

    def run_astar(self, start: tuple, goal: tuple):
        """
        Klasični A* s 8-smjernim susjedima i euklidskom heuristikom.
        Vraća listu (row, col) od starta do cilja ili None.
        """

        # open_set: (f_score, (row, col))
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        # 8 smjera: gore, dolje, lijevo, desno + dijagonale
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

                    # Euklidska heuristika (u ćelijama)
                    h = math.hypot(nr - goal[0], nc - goal[1])
                    f = tentative_g + h

                    heapq.heappush(open_set, (f, neighbor))

        return None  # Putanja nije pronađena

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
        """
        Uklanja točke koje leže na istom pravcu kako bi
        pure pursuit dobio čišće waypointe.
        """

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
        """Kreira i publishuje nav_msgs/Path poruku."""

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
    node = AStarPathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim planer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
