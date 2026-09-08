#!/usr/bin/env python3
"""
A* Planner (nova struktura)
=============================
Čist, samostalan globalni A* planer. Dio nove strukture zajedno s
potential_field_pure_pursuit.py - bez dynamic obstacle layera,
bez cmd_safety gate-a, bez replanning monitora. Lokalno
izbjegavanje prepreka radi potential_field_pure_pursuit.py
(direktno iz /scan), ovaj node je zadužen samo za globalni put
kroz statičku kartu.

Kako radi:
    - Sluša /map (nav_msgs/OccupancyGrid) - kartu dobiva od
      map_servera, ne učitava PGM/YAML ručno.
    - Kartu inflatea prema robot_radius (sigurnosni razmak od
      zidova).
    - Poziciju robota uzima iz TF-a (map -> base_footprint),
      isto kao potential_field_pure_pursuit.py.
    - Na svaki novi /goal_pose (RViz "2D Goal Pose") pokreće A*
      pretragu i publica putanju na /plan.
    - Novi goal jednostavno prekine/zamijeni prethodno planiranje.

Topici:
    Pretplate:
        /map        (nav_msgs/OccupancyGrid)
        /goal_pose  (geometry_msgs/PoseStamped)  <- RViz 2D Goal Pose
    TF:
        map -> base_footprint
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


class AStarPlannerPF(Node):

    def __init__(self):
        super().__init__('astar_planner_pf')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Robot radius u metrima - koliko se prepreke naduvaju.
        self.declare_parameter('robot_radius', 0.20)

        # Prag iznad kojeg se ćelija smatra preprekom (0-100).
        self.declare_parameter('obstacle_threshold', 50)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        map_topic = self.get_parameter('map_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.robot_radius = self.get_parameter('robot_radius').value
        self.obstacle_threshold = self.get_parameter(
            'obstacle_threshold'
        ).value

        # =====================================================
        # STANJE
        # =====================================================

        self.map_ready = False

        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Inflatana occupancy mreža (0 = slobodno, 100 = prepreka)
        self.occupancy = None

        self.goal_x = None
        self.goal_y = None

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

        self.plan_pub = self.create_publisher(
            Path,
            plan_topic,
            10
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'ASTRO A* PLANNER (nova struktura)'
        )
        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'Map topic:   {map_topic}'
        )
        self.get_logger().info(
            f'Goal topic:  {goal_topic}'
        )
        self.get_logger().info(
            f'Plan topic:  {plan_topic}'
        )
        self.get_logger().info(
            f'TF:          {self.map_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'Robot radius:{self.robot_radius:.2f} m'
        )
        self.get_logger().info(
            'Čekam kartu i goal_pose...'
        )

    # =========================================================
    # MAP CALLBACK
    # =========================================================

    def map_callback(self, msg: OccupancyGrid):

        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int16).reshape(
            (self.height, self.width)
        )

        occupancy = np.zeros(
            (self.height, self.width),
            dtype=np.int8
        )

        # Nepoznato (-1) i iznad praga = prepreka (sigurnije).
        occupancy[raw < 0] = 100
        occupancy[raw >= self.obstacle_threshold] = 100

        self.occupancy = self.inflate(occupancy)
        self.map_ready = True

        self.get_logger().info(
            f'Karta primljena: {self.width}x{self.height} '
            f'@ {self.resolution:.3f} m/ćelija'
        )

    # =========================================================
    # OBSTACLE INFLATION
    # =========================================================

    def inflate(self, occupancy):

        radius_cells = int(
            math.ceil(self.robot_radius / self.resolution)
        )

        inflated = occupancy.copy()

        obstacle_cells = np.argwhere(occupancy == 100)

        for row, col in obstacle_cells:

            r_min = max(0, row - radius_cells)
            r_max = min(self.height, row + radius_cells + 1)
            c_min = max(0, col - radius_cells)
            c_max = min(self.width, col + radius_cells + 1)

            inflated[r_min:r_max, c_min:c_max] = 100

        return inflated

    # =========================================================
    # GOAL CALLBACK
    # =========================================================

    def goal_callback(self, msg: PoseStamped):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.get_logger().info(
            f'Novi goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}'
        )

        if not self.map_ready:
            self.get_logger().warn('Karta još nije primljena.')
            return

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            self.get_logger().warn(
                'TF (map -> base_footprint) trenutno nedostupan.'
            )
            return

        robot_x, robot_y = robot_pose

        self.plan_path(robot_x, robot_y, self.goal_x, self.goal_y)

    # =========================================================
    # ROBOT POSE (TF)
    # =========================================================

    def get_robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation

            return t.x, t.y

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return None

    # =========================================================
    # WORLD <-> GRID
    # =========================================================

    def world_to_grid(self, x, y):

        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)

        return row, col

    def grid_to_world(self, row, col):

        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution

        return x, y

    def is_valid(self, row, col):

        if row < 0 or row >= self.height:
            return False

        if col < 0 or col >= self.width:
            return False

        return self.occupancy[row, col] == 0

    # =========================================================
    # A*
    # =========================================================

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start, goal):

        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        neighbors = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]

        while open_set:

            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dr, dc in neighbors:

                neighbor = (current[0] + dr, current[1] + dc)

                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                step_cost = (
                    math.sqrt(2.0) if dr != 0 and dc != 0 else 1.0
                )

                tentative_g = g_score[current] + step_cost

                if (
                    neighbor not in g_score
                    or tentative_g < g_score[neighbor]
                ):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self.heuristic(
                        neighbor, goal
                    )

                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path

    # =========================================================
    # PLAN PATH
    # =========================================================

    def plan_path(self, robot_x, robot_y, goal_x, goal_y):

        start = self.world_to_grid(robot_x, robot_y)
        goal = self.world_to_grid(goal_x, goal_y)

        if not (0 <= start[0] < self.height and 0 <= start[1] < self.width):
            self.get_logger().error('Robot je izvan karte.')
            return

        if not (0 <= goal[0] < self.height and 0 <= goal[1] < self.width):
            self.get_logger().error('Cilj je izvan karte.')
            return

        if not self.is_valid(start[0], start[1]):
            self.get_logger().error(
                'Pozicija robota je unutar (naduvane) prepreke!'
            )
            return

        if not self.is_valid(goal[0], goal[1]):
            self.get_logger().error(
                'Cilj je unutar (naduvane) prepreke!'
            )
            return

        self.get_logger().info('Računam A*...')

        grid_path = self.astar(start, goal)

        if grid_path is None:
            self.get_logger().error('A* nije pronašao putanju.')
            return

        self.get_logger().info(
            f'Putanja pronađena: {len(grid_path)} ćelija.'
        )

        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.map_frame

        for row, col in grid_path:

            x, y = self.grid_to_world(row, col)

            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            ros_path.poses.append(pose)

        self.plan_pub.publish(ros_path)

        self.get_logger().info('Putanja objavljena na /plan.')


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlannerPF()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Gasim A* planer.')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
