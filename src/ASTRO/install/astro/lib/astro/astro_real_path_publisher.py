#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class AstroRealPathPublisher(Node):

    def __init__(self):
        super().__init__('astro_real_path_publisher')

        # =====================================================
        # PARAMETRI
        # =====================================================

        # Isti parametri koje koristi Gazebo publisher
        self.declare_parameter('width', 1.5)
        self.declare_parameter('height', 1.0)
        self.declare_parameter('num_points', 1500)

        # Putanja je definirana lokalno u odnosu na
        # početnu poziciju stvarnog robota.
        self.declare_parameter(
            'frame_id',
            'trajectory_start'
        )

        self.declare_parameter(
            'publish_rate',
            1.0
        )

        self.width = self.get_parameter(
            'width'
        ).value

        self.height = self.get_parameter(
            'height'
        ).value

        self.num_points = self.get_parameter(
            'num_points'
        ).value

        self.frame_id = self.get_parameter(
            'frame_id'
        ).value

        self.publish_rate = self.get_parameter(
            'publish_rate'
        ).value

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.path_pub = self.create_publisher(
            Path,
            '/figure8_real_path',
            10
        )

        # =====================================================
        # GENERIRANJE PUTANJE
        # =====================================================

        self.path = self.generate_figure8()

        # =====================================================
        # TIMER
        # =====================================================

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_path
        )

        self.get_logger().info(
            'ASTRO REAL figure-8 path publisher started.'
        )

        self.get_logger().info(
            f'Width: {self.width:.2f} m'
        )

        self.get_logger().info(
            f'Height: {self.height:.2f} m'
        )

        self.get_logger().info(
            f'Points: {self.num_points}'
        )

        self.get_logger().info(
            'Publishing: /figure8_real_path'
        )

    # =========================================================
    # GENERIRANJE FIGURE 8
    # =========================================================

    def generate_figure8(self):

        path = Path()

        path.header.frame_id = self.frame_id

        for i in range(self.num_points):

            # 0 -> 2*pi
            t = (
                2.0 *
                math.pi *
                i /
                self.num_points
            )

            # Ista formula kao u Gazebo simulaciji
            x = self.width * math.sin(t)

            y = (
                self.height *
                math.sin(t) *
                math.cos(t)
            )

            # -------------------------------------------------
            # SLJEDEĆA TOČKA
            # -------------------------------------------------

            t_next = (
                2.0 *
                math.pi *
                (i + 1) /
                self.num_points
            )

            x_next = (
                self.width *
                math.sin(t_next)
            )

            y_next = (
                self.height *
                math.sin(t_next) *
                math.cos(t_next)
            )

            dx = x_next - x
            dy = y_next - y

            yaw = math.atan2(dy, dx)

            # -------------------------------------------------
            # POSE
            # -------------------------------------------------

            pose = PoseStamped()

            pose.header.frame_id = self.frame_id

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Quaternion iz yaw-a

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

            pose.pose.orientation.z = math.sin(
                yaw / 2.0
            )

            pose.pose.orientation.w = math.cos(
                yaw / 2.0
            )

            path.poses.append(pose)

        return path

    # =========================================================
    # OBJAVA
    # =========================================================

    def publish_path(self):

        now = self.get_clock().now().to_msg()

        self.path.header.stamp = now

        for pose in self.path.poses:
            pose.header.stamp = now

        self.path_pub.publish(self.path)


def main(args=None):

    rclpy.init(args=args)

    node = AstroRealPathPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
