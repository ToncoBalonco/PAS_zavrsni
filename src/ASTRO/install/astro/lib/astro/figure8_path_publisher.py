#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Figure8PathPublisher(Node):

    def __init__(self):
        super().__init__('figure8_path_publisher')

        self.declare_parameter('width', 1.0)
        self.declare_parameter('height', 0.6)
        self.declare_parameter('num_points', 500)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate', 1.0)

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

        self.path_pub = self.create_publisher(
            Path,
            '/figure8_path',
            10
        )

        self.path = self.generate_figure8()

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_path
        )

        self.get_logger().info(
            'Figure 8 path publisher started.'
        )

        self.get_logger().info(
            f'Width: {self.width:.2f} m | '
            f'Height: {self.height:.2f} m | '
            f'Points: {self.num_points}'
        )

        self.get_logger().info(
            f'Publishing: /figure8_path | '
            f'Frame: {self.frame_id}'
        )

    def generate_figure8(self):

        path = Path()

        path.header.frame_id = self.frame_id

        for i in range(self.num_points):

            t = (
                2.0 *
                math.pi *
                i /
                float(self.num_points - 1)
            )

            x = (
                self.width *
                math.sin(t)
            )

            y = (
                self.height *
                math.sin(t) *
                math.cos(t)
            )

            if i < self.num_points - 1:

                t_next = (
                    2.0 *
                    math.pi *
                    (i + 1) /
                    float(self.num_points - 1)
                )

            else:

                t_next = t

            x_next = (
                self.width *
                math.sin(t_next)
            )

            y_next = (
                self.height *
                math.sin(t_next) *
                math.cos(t_next)
            )

            if i < self.num_points - 1:

                dx = x_next - x
                dy = y_next - y

                yaw = math.atan2(
                    dy,
                    dx
                )

            else:

                yaw = 0.0

            pose = PoseStamped()

            pose.header.frame_id = self.frame_id

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

            pose.pose.orientation.z = math.sin(
                yaw / 2.0
            )

            pose.pose.orientation.w = math.cos(
                yaw / 2.0
            )

            path.poses.append(
                pose
            )

        return path

    def publish_path(self):

        self.path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for pose in self.path.poses:

            pose.header.stamp = (
                self.path.header.stamp
            )

        self.path_pub.publish(
            self.path
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = Figure8PathPublisher()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()


