#!/usr/bin/env python3

import math
import heapq
import os

import numpy as np
from PIL import Image
import yaml

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):

    def __init__(self):

        super().__init__('astar_planner')

        # ==========================================================
        # MAP PATH
        # ==========================================================

        self.map_yaml = (
            '/home/tona/astro_ws_pas/'
            'worlds/02 CRTA_simplified/'
            'crta_pojednostavljeno/CRTA_v4/map/'
            'crta_map_v2.yaml'
        )

        # ==========================================================
        # ROBOT PARAMETERS
        # ==========================================================

        # Robot radius in meters.
        #
        # 0.20 m means that obstacles will be expanded
        # by approximately 20 cm.
        #
        self.robot_radius = 0.20

        # ==========================================================
        # STATE
        # ==========================================================

        self.robot_x = None
        self.robot_y = None

        self.goal_x = None
        self.goal_y = None

        self.path = None

        # ==========================================================
        # LOAD MAP
        # ==========================================================

        self.load_map()

        # ==========================================================
        # ROS INTERFACES
        # ==========================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/astar_path',
            10
        )

        self.get_logger().info(
            'A* planner started.'
        )

        self.get_logger().info(
            'Waiting for robot odometry and /goal_pose...'
        )

    # ==============================================================
    # MAP LOADING
    # ==============================================================

    def load_map(self):

        if not os.path.exists(self.map_yaml):

            self.get_logger().error(
                f'Map YAML does not exist:\n{self.map_yaml}'
            )

            raise FileNotFoundError(self.map_yaml)

        # ----------------------------------------------------------
        # Read YAML
        # ----------------------------------------------------------

        with open(self.map_yaml, 'r') as file:

            map_config = yaml.safe_load(file)

        self.resolution = float(
            map_config['resolution']
        )

        self.origin_x = float(
            map_config['origin'][0]
        )

        self.origin_y = float(
            map_config['origin'][1]
        )

        self.occupied_thresh = float(
            map_config.get('occupied_thresh', 0.65)
        )

        self.free_thresh = float(
            map_config.get('free_thresh', 0.25)
        )

        image_name = map_config['image']

        # Resolve relative image path.
        image_path = os.path.join(
            os.path.dirname(self.map_yaml),
            image_name
        )

        if not os.path.exists(image_path):

            self.get_logger().error(
                f'Map image does not exist:\n{image_path}'
            )

            raise FileNotFoundError(image_path)

        # ----------------------------------------------------------
        # Load PGM
        # ----------------------------------------------------------

        image = Image.open(image_path).convert('L')

        self.image = np.array(image)

        self.height, self.width = self.image.shape

        # ----------------------------------------------------------
        # Convert image into occupancy grid
        #
        # PGM:
        #
        # 0   = black
        # 255 = white
        #
        # ROS occupancy:
        #
        # 0   = free
        # 100 = occupied
        # -1  = unknown
        # ----------------------------------------------------------

        self.occupancy = np.full(
            (self.height, self.width),
            -1,
            dtype=np.int8
        )

        for row in range(self.height):

            for col in range(self.width):

                pixel = self.image[row, col]

                probability = 1.0 - (
                    float(pixel) / 255.0
                )

                if probability >= self.occupied_thresh:

                    self.occupancy[row, col] = 100

                elif probability <= self.free_thresh:

                    self.occupancy[row, col] = 0

                else:

                    self.occupancy[row, col] = -1

        # ----------------------------------------------------------
        # Inflate obstacles
        # ----------------------------------------------------------

        self.inflate_obstacles()

        self.get_logger().info(
            f'Map loaded: '
            f'{self.width} x {self.height} cells'
        )

        self.get_logger().info(
            f'Map size: '
            f'{self.width * self.resolution:.2f} m x '
            f'{self.height * self.resolution:.2f} m'
        )

        self.get_logger().info(
            f'Resolution: {self.resolution:.3f} m'
        )

        self.get_logger().info(
            f'Origin: '
            f'({self.origin_x:.2f}, '
            f'{self.origin_y:.2f})'
        )

    # ==============================================================
    # OBSTACLE INFLATION
    # ==============================================================

    def inflate_obstacles(self):

        radius_cells = int(
            math.ceil(
                self.robot_radius /
                self.resolution
            )
        )

        original = self.occupancy.copy()

        obstacle_cells = np.argwhere(
            original == 100
        )

        for row, col in obstacle_cells:

            r_min = max(
                0,
                row - radius_cells
            )

            r_max = min(
                self.height,
                row + radius_cells + 1
            )

            c_min = max(
                0,
                col - radius_cells
            )

            c_max = min(
                self.width,
                col + radius_cells + 1
            )

            self.occupancy[
                r_min:r_max,
                c_min:c_max
            ] = 100

        # Unknown cells remain obstacles for safety.

        self.occupancy[
            original == -1
        ] = 100

        self.get_logger().info(
            f'Obstacle inflation: '
            f'{self.robot_radius:.2f} m '
            f'({radius_cells} cells)'
        )

    # ==============================================================
    # ODOM CALLBACK
    # ==============================================================

    def odom_callback(self, msg):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )

    # ==============================================================
    # GOAL CALLBACK
    # ==============================================================

    def goal_callback(self, msg):

        self.goal_x = (
            msg.pose.position.x
        )

        self.goal_y = (
            msg.pose.position.y
        )

        self.get_logger().info(
            f'New goal received: '
            f'x={self.goal_x:.3f}, '
            f'y={self.goal_y:.3f}'
        )

        if self.robot_x is None:

            self.get_logger().warn(
                'Robot odometry not available yet.'
            )

            return

        self.plan_path()

    # ==============================================================
    # WORLD -> GRID
    # ==============================================================

    def world_to_grid(self, x, y):

        col = int(
            math.floor(
                (x - self.origin_x) /
                self.resolution
            )
        )

        row_from_bottom = int(
            math.floor(
                (y - self.origin_y) /
                self.resolution
            )
        )

        # PGM row 0 is at the TOP.
        row = (
            self.height -
            1 -
            row_from_bottom
        )

        return row, col

    # ==============================================================
    # GRID -> WORLD
    # ==============================================================

    def grid_to_world(self, row, col):

        x = (
            self.origin_x +
            (col + 0.5) *
            self.resolution
        )

        row_from_bottom = (
            self.height -
            1 -
            row
        )

        y = (
            self.origin_y +
            (row_from_bottom + 0.5) *
            self.resolution
        )

        return x, y

    # ==============================================================
    # VALID CELL
    # ==============================================================

    def is_valid(self, row, col):

        if row < 0:
            return False

        if row >= self.height:
            return False

        if col < 0:
            return False

        if col >= self.width:
            return False

        return self.occupancy[row, col] == 0

    # ==============================================================
    # HEURISTIC
    # ==============================================================

    def heuristic(self, a, b):

        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    # ==============================================================
    # A*
    # ==============================================================

    def astar(self, start, goal):

        # ----------------------------------------------------------
        # Priority queue
        # ----------------------------------------------------------

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        # ----------------------------------------------------------
        # 8-connected grid
        # ----------------------------------------------------------

        neighbors = [

            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),

            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1)
        ]

        while open_set:

            _, current = heapq.heappop(
                open_set
            )

            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )

            for dr, dc in neighbors:

                neighbor = (
                    current[0] + dr,
                    current[1] + dc
                )

                if not self.is_valid(
                    neighbor[0],
                    neighbor[1]
                ):
                    continue

                if dr != 0 and dc != 0:

                    movement_cost = math.sqrt(2.0)

                else:

                    movement_cost = 1.0

                tentative_g = (
                    g_score[current]
                    +
                    movement_cost
                )

                if (
                    neighbor not in g_score
                    or tentative_g <
                    g_score[neighbor]
                ):

                    came_from[neighbor] = current

                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        +
                        self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            neighbor
                        )
                    )

        return None

    # ==============================================================
    # RECONSTRUCT PATH
    # ==============================================================

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [current]

        while current in came_from:

            current = came_from[current]

            path.append(current)

        path.reverse()

        return path

    # ==============================================================
    # PLAN PATH
    # ==============================================================

    def plan_path(self):

        start = self.world_to_grid(
            self.robot_x,
            self.robot_y
        )

        goal = self.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        self.get_logger().info(
            f'Start grid: {start}'
        )

        self.get_logger().info(
            f'Goal grid: {goal}'
        )

        # ----------------------------------------------------------
        # Check boundaries
        # ----------------------------------------------------------

        if not (
            0 <= start[0] < self.height
            and
            0 <= start[1] < self.width
        ):

            self.get_logger().error(
                'Robot position is outside the map.'
            )

            return

        if not (
            0 <= goal[0] < self.height
            and
            0 <= goal[1] < self.width
        ):

            self.get_logger().error(
                'Goal is outside the map.'
            )

            return

        # ----------------------------------------------------------
        # Check occupancy
        # ----------------------------------------------------------

        if not self.is_valid(
            start[0],
            start[1]
        ):

            self.get_logger().error(
                'Robot start position is occupied!'
            )

            return

        if not self.is_valid(
            goal[0],
            goal[1]
        ):

            self.get_logger().error(
                'Goal position is occupied!'
            )

            return

        # ----------------------------------------------------------
        # Run A*
        # ----------------------------------------------------------

        self.get_logger().info(
            'Running A*...'
        )

        grid_path = self.astar(
            start,
            goal
        )

        if grid_path is None:

            self.get_logger().error(
                'A* could not find a path.'
            )

            return

        self.get_logger().info(
            f'A* found path with '
            f'{len(grid_path)} cells.'
        )

        # ----------------------------------------------------------
        # Convert to ROS Path
        # ----------------------------------------------------------

        ros_path = Path()

        ros_path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        ros_path.header.frame_id = 'odom'

        for row, col in grid_path:

            x, y = self.grid_to_world(
                row,
                col
            )

            pose = PoseStamped()

            pose.header = ros_path.header

            pose.pose.position.x = x

            pose.pose.position.y = y

            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            ros_path.poses.append(
                pose
            )

        self.path = ros_path

        self.path_pub.publish(
            ros_path
        )

        self.get_logger().info(
            'A* path published on /astar_path'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
