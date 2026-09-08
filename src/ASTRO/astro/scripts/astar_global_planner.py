#!/usr/bin/env python3
"""
A* Global Planner (nova, čista struktura)
=============================================
Globalni planer za A* + Potential Field Pure Pursuit (PFPP) arhitekturu:

    MAP -> A* -> [prune] -> [smoothing] -> GLOBALNI PUT (/plan)

Zadužen ISKLJUČIVO za globalno planiranje kroz statičku (inflatanu)
kartu. Lokalno praćenje puta i izbjegavanje prepreka radi
pfpp_local_controller.py, koji čita /plan i /scan.

Razlike u odnosu na astar_planner_pf.py (prijašnja verzija):
    - Kružna (a ne kvadratna) inflacija prepreka - kvadratna
      inflacija umjetno blokira dijagonalne prolaze pored uglova
      prepreka.
    - Put se nakon A* pretrage čisti (uklanjaju se kolinearne
      točke) i zatim GLADI (gradient descent smoothing), umjesto
      da se svaka ćelija mreže objavi kao zaseban waypoint. Grubi
      "stepenasti" put (dijagonalni/ortogonalni skokovi svake
      ćelije) inače unosi nagle promjene smjera u Pure Pursuit,
      što je jedan od uzroka trzave vožnje.
    - Zaglađeni put se provjerava natrag protiv inflatane karte -
      točka koja bi smoothingom završila preblizu preprekе vraća se
      na svoju "sirovu" (prunanu) poziciju.
    - Orijentacija svake točke u /plan sada pokazuje smjer prema
      sljedećoj točki (korisno za vizualizaciju/debug), umjesto
      identity quaterniona.

Topici:
    Pretplate:
        /map        (nav_msgs/OccupancyGrid)
        /goal_pose  (geometry_msgs/PoseStamped)  <- RViz "2D Goal Pose"
    TF:
        map -> base_frame (samo za start poziciju robota)
    Publicira:
        /plan       (nav_msgs/Path)
"""

import math
import heapq

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import tf2_ros


# =================================================================
# OCCUPANCY GRID - kartа, inflacija, world<->grid konverzija
# =================================================================

class OccupancyGridMap:
    """
    Drži zadnju primljenu kartu (inflatanu prema robot_radius) i
    pretvara između world (metri, map frame) i grid (row, col)
    koordinata.
    """

    def __init__(self, robot_radius: float, obstacle_threshold: int):
        self.robot_radius = robot_radius
        self.obstacle_threshold = obstacle_threshold

        self.ready = False
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.occupancy = None

        # Predizračunati kružni "disk" pomaka za inflaciju - radi se
        # jednom (ovisi samo o resolution/robot_radius), a ne po
        # svakoj ćeliji prepreke.
        self._disk_offsets = None
        self._disk_resolution = None

    def update_from_msg(self, msg: OccupancyGrid):

        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int16).reshape(
            (self.height, self.width)
        )

        occupancy = np.zeros((self.height, self.width), dtype=np.uint8)

        # Nepoznato (-1) i iznad praga = prepreka (sigurnije).
        occupancy[raw < 0] = 1
        occupancy[raw >= self.obstacle_threshold] = 1

        self.occupancy = self._inflate_circular(occupancy)
        self.ready = True

    def _disk_offsets_for(self, resolution: float):
        """Popis (dr, dc) pomaka unutar robot_radius, kružno (ne
        kvadratno) - kvadratna inflacija nepotrebno blokira
        dijagonalne prolaze uz uglove prepreka."""

        if self._disk_offsets is not None and (
            self._disk_resolution == resolution
        ):
            return self._disk_offsets

        radius_cells = int(math.ceil(self.robot_radius / resolution))

        offsets = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if math.hypot(dr, dc) * resolution <= self.robot_radius:
                    offsets.append((dr, dc))

        self._disk_offsets = offsets
        self._disk_resolution = resolution

        return offsets

    def _inflate_circular(self, occupancy: np.ndarray) -> np.ndarray:

        inflated = occupancy.copy()
        obstacle_cells = np.argwhere(occupancy == 1)
        offsets = self._disk_offsets_for(self.resolution)

        for row, col in obstacle_cells:
            for dr, dc in offsets:
                r = row + dr
                c = col + dc
                if 0 <= r < self.height and 0 <= c < self.width:
                    inflated[r, c] = 1

        return inflated

    def world_to_grid(self, x: float, y: float):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row: int, col: int) -> bool:
        if not self.in_bounds(row, col):
            return False
        return self.occupancy[row, col] == 0


# =================================================================
# A* PRETRAGA - čist algoritam, ne zna ništa o ROS-u
# =================================================================

class AStarSearch:

    NEIGHBORS = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    def __init__(self, grid_map: OccupancyGridMap):
        self.grid_map = grid_map

    @staticmethod
    def _heuristic(a, b) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def search(self, start, goal):

        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        visited = set()

        while open_set:

            _, current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return self._reconstruct(came_from, current)

            for dr, dc in self.NEIGHBORS:

                neighbor = (current[0] + dr, current[1] + dc)

                if not self.grid_map.is_free(neighbor[0], neighbor[1]):
                    continue

                step_cost = math.sqrt(2.0) if dr and dc else 1.0
                tentative_g = g_score[current] + step_cost

                if (
                    neighbor not in g_score
                    or tentative_g < g_score[neighbor]
                ):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    @staticmethod
    def _reconstruct(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


# =================================================================
# POST-PROCESSING PUTANJE - čišćenje (prune) + glađenje (smooth)
# =================================================================

class PathPostProcessor:
    """
    Sirova A* putanja (jedna točka po ćeliji mreže) je "stepenasta"
    - svaki zaokret je 45°/90° skok. To u Pure Pursuitu izaziva
    nagle promjene traženog smjera na svakom zaokretu. Ovdje se
    put prvo čisti (kolinearne točke se uklanjaju), a zatim gladi
    (gradient descent smoothing) tako da zavoji postanu kontinuirane
    krivulje, a ne oštri kutovi.
    """

    def __init__(
        self,
        grid_map: OccupancyGridMap,
        smoothing_weight_data: float,
        smoothing_weight_smooth: float,
        smoothing_iterations: int,
        smoothing_tolerance: float,
    ):
        self.grid_map = grid_map
        self.weight_data = smoothing_weight_data
        self.weight_smooth = smoothing_weight_smooth
        self.iterations = smoothing_iterations
        self.tolerance = smoothing_tolerance

    def prune_collinear(self, grid_path: list) -> list:
        """Uklanja točke koje leže na pravcu između susjeda - putanja
        i dalje prolazi kroz iste ćelije, samo s manje waypointa."""

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

    def smooth(self, world_points: list) -> list:
        """Gradient descent smoothing (standardna tehnika): svaka
        točka se pomiče između svoje originalne pozicije (fidelity,
        `weight_data`) i prosjeka susjeda (glatkoća, `weight_smooth`).
        Početna i krajnja točka se ne pomiču. Nakon svake iteracije,
        svaka pomaknuta točka se provjeri protiv inflatane karte -
        ako bi smoothing gurnuo točku preblizu prepreke, ta točka se
        vrati na svoju predsmoothing (prunanu) poziciju."""

        n = len(world_points)

        if n <= 2:
            return world_points

        original = [list(p) for p in world_points]
        smoothed = [list(p) for p in world_points]

        for _ in range(self.iterations):

            max_change = 0.0

            for i in range(1, n - 1):

                for dim in range(2):

                    before = smoothed[i][dim]

                    smoothed[i][dim] += self.weight_data * (
                        original[i][dim] - smoothed[i][dim]
                    )
                    smoothed[i][dim] += self.weight_smooth * (
                        smoothed[i - 1][dim]
                        + smoothed[i + 1][dim]
                        - 2.0 * smoothed[i][dim]
                    )

                    max_change = max(
                        max_change, abs(smoothed[i][dim] - before)
                    )

            if max_change < self.tolerance:
                break

        # Sigurnosna provjera: zaglađena točka ne smije upasti u
        # inflatanu prepreku. Ako upadne, vrati je na "sirovu"
        # (prunanu) poziciju koja je po definiciji sigurna.
        for i in range(1, n - 1):
            row, col = self.grid_map.world_to_grid(
                smoothed[i][0], smoothed[i][1]
            )
            if not self.grid_map.is_free(row, col):
                smoothed[i] = original[i]

        return [tuple(p) for p in smoothed]


# =================================================================
# ROS NODE
# =================================================================

class AStarGlobalPlanner(Node):

    def __init__(self):
        super().__init__('astar_global_planner')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Radijus robota u metrima - koliko se prepreke naduvaju
        # (kružno, ne kvadratno).
        self.declare_parameter('robot_radius', 0.43)

        # Prag iznad kojeg se ćelija smatra preprekom (0-100).
        self.declare_parameter('obstacle_threshold', 50)

        # --- Post-processing puta ---

        # Koliko jako put ostaje vjeran originalnoj (sirovoj) A*
        # putanji tijekom glađenja (0-1). Veće = bliže originalu.
        self.declare_parameter('smoothing_weight_data', 0.5)

        # Koliko jako se put gladi/ravna prema susjedima (0-1).
        # Veće = glađi put, ali dalje od originalne A* putanje.
        self.declare_parameter('smoothing_weight_smooth', 0.25)

        # Maksimalan broj iteracija smoothing algoritma.
        self.declare_parameter('smoothing_iterations', 100)

        # Ako se nijedna točka ne pomakne više od ovoga [m] u jednoj
        # iteraciji, smoothing prekida ranije (konvergirao).
        self.declare_parameter('smoothing_tolerance', 1e-4)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        robot_radius = self.get_parameter('robot_radius').value
        obstacle_threshold = self.get_parameter(
            'obstacle_threshold'
        ).value

        self.grid_map = OccupancyGridMap(robot_radius, obstacle_threshold)
        self.astar = AStarSearch(self.grid_map)
        self.post_processor = PathPostProcessor(
            self.grid_map,
            smoothing_weight_data=self.get_parameter(
                'smoothing_weight_data'
            ).value,
            smoothing_weight_smooth=self.get_parameter(
                'smoothing_weight_smooth'
            ).value,
            smoothing_iterations=self.get_parameter(
                'smoothing_iterations'
            ).value,
            smoothing_tolerance=self.get_parameter(
                'smoothing_tolerance'
            ).value,
        )

        # =====================================================
        # STANJE
        # =====================================================

        self.goal_x = None
        self.goal_y = None

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =====================================================
        # SUB / PUB
        # =====================================================

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10
        )

        self.plan_pub = self.create_publisher(Path, plan_topic, 10)

        self.get_logger().info(
            'ASTRO A* GLOBAL PLANNER spreman. '
            f'robot_radius={robot_radius:.2f}m. '
            'Čekam kartu i goal_pose...'
        )

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg: OccupancyGrid):
        self.grid_map.update_from_msg(msg)
        self.get_logger().info(
            f'Karta primljena: {self.grid_map.width}x'
            f'{self.grid_map.height} @ '
            f'{self.grid_map.resolution:.3f} m/ćelija'
        )

    # =========================================================
    # GOAL CALLBACK
    # =========================================================

    def goal_callback(self, msg: PoseStamped):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.get_logger().info(
            f'Novi goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}'
        )

        if not self.grid_map.ready:
            self.get_logger().warn('Karta još nije primljena.')
            return

        robot_pose = self._get_robot_pose()

        if robot_pose is None:
            self.get_logger().warn(
                'TF (map -> base_frame) trenutno nedostupan.'
            )
            return

        robot_x, robot_y = robot_pose
        self._plan_path(robot_x, robot_y, self.goal_x, self.goal_y)

    def _get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            t = transform.transform.translation
            return t.x, t.y
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    # =========================================================
    # PLANIRANJE
    # =========================================================

    def _plan_path(self, robot_x, robot_y, goal_x, goal_y):

        start = self.grid_map.world_to_grid(robot_x, robot_y)
        goal = self.grid_map.world_to_grid(goal_x, goal_y)

        if not self.grid_map.in_bounds(*start):
            self.get_logger().error('Robot je izvan karte.')
            return

        if not self.grid_map.in_bounds(*goal):
            self.get_logger().error('Cilj je izvan karte.')
            return

        if not self.grid_map.is_free(*start):
            self.get_logger().error(
                'Pozicija robota je unutar (naduvane) prepreke!'
            )
            return

        if not self.grid_map.is_free(*goal):
            self.get_logger().error(
                'Cilj je unutar (naduvane) prepreke!'
            )
            return

        self.get_logger().info('Računam A*...')

        grid_path = self.astar.search(start, goal)

        if grid_path is None:
            self.get_logger().error('A* nije pronašao putanju.')
            return

        pruned = self.post_processor.prune_collinear(grid_path)

        world_points = [
            self.grid_map.grid_to_world(r, c) for r, c in pruned
        ]

        smoothed_points = self.post_processor.smooth(world_points)

        self.get_logger().info(
            f'Putanja pronađena: {len(grid_path)} ćelija -> '
            f'{len(pruned)} waypointa nakon čišćenja '
            f'(zaglađeno).'
        )

        self._publish_plan(smoothed_points)

    def _publish_plan(self, world_points: list):

        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.map_frame

        n = len(world_points)

        for i, (x, y) in enumerate(world_points):

            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orijentacija = smjer prema sljedećoj točki (korisno za
            # vizualizaciju; PFPP kontroler orijentaciju ne koristi).
            if i < n - 1:
                dx = world_points[i + 1][0] - x
                dy = world_points[i + 1][1] - y
            else:
                dx = x - world_points[i - 1][0] if i > 0 else 1.0
                dy = y - world_points[i - 1][1] if i > 0 else 0.0

            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            ros_path.poses.append(pose)

        self.plan_pub.publish(ros_path)
        self.get_logger().info('Putanja objavljena na /plan.')


def main(args=None):

    rclpy.init(args=args)
    node = AStarGlobalPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim A* planer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

