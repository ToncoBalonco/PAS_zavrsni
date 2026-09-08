#!/usr/bin/env python3

"""
sim_epoch_runner.py

Replays cmd_vel from bag_data.npz into Gazebo and records simulated odometry.

Usage:
    ros2 run astro sim_epoch_runner.py \
        --bag-data /path/to/bag_data.npz \
        --output /path/to/sim_odom_result.npz \
        --duration 30.0
"""

import argparse
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def euler_from_quaternion(x, y, z, w):
    """Extract yaw from quaternion."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class SimEpochRunner(Node):

    def __init__(self, bag_data_path, output_path, duration):

        super().__init__('sim_epoch_runner')

        # ---------------------------------------------------------
        # Load bag data
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Loading bag data from {bag_data_path}'
        )

        bag_data = np.load(
            bag_data_path,
            allow_pickle=True
        )

        self.cmd_vel_times = bag_data['cmd_vel_times']
        self.cmd_vel_linear = bag_data['cmd_vel_linear']
        self.cmd_vel_angular = bag_data['cmd_vel_angular']

        self.duration = float(duration)
        self.output_path = output_path

        self.get_logger().info(
            f'Loaded {len(self.cmd_vel_times)} cmd_vel messages, '
            f'duration={self.duration:.2f}s'
        )

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.sim_time = 0.0
        self.sim_start_time = None

        self.cmd_idx = 0

        self.started = False
        self.finished = False

        self.controller_ready = False
        self.ready_check_count = 0

        # ---------------------------------------------------------
        # Odom storage
        # ---------------------------------------------------------

        self.odom_times = []
        self.odom_x = []
        self.odom_y = []
        self.odom_yaw = []

        # ---------------------------------------------------------
        # QoS
        # ---------------------------------------------------------

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive_base_controller/cmd_vel_unstamped',
            10
        )

        # ---------------------------------------------------------
        # Clock subscriber
        # ---------------------------------------------------------

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        # ---------------------------------------------------------
        # Odometry subscriber
        # ---------------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            odom_qos
        )

        # ---------------------------------------------------------
        # Main timer
        #
        # 100 Hz is more than enough for cmd_vel replay.
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            0.01,
            self.timer_callback
        )

        self.get_logger().info(
            'SimEpochRunner initialized, waiting for simulation...'
        )

    # =============================================================
    # CLOCK
    # =============================================================

    def clock_callback(self, msg):

        self.sim_time = (
            msg.clock.sec +
            msg.clock.nanosec * 1e-9
        )

    # =============================================================
    # ODOMETRY
    # =============================================================

    def odom_callback(self, msg):

        if self.finished:
            return

        # ---------------------------------------------------------
        # IMPORTANT:
        #
        # Receiving odometry means the diff-drive controller
        # is working.
        # ---------------------------------------------------------

        if not self.controller_ready:

            self.controller_ready = True

            self.get_logger().info(
                'Odometry received! '
                f'Controller ready at sim_time={self.sim_time:.2f}s'
            )

        # ---------------------------------------------------------
        # Do not record odometry before replay starts
        # ---------------------------------------------------------

        if not self.started:
            return

        # ---------------------------------------------------------
        # Relative epoch time
        # ---------------------------------------------------------

        t = self.sim_time - self.sim_start_time

        # ---------------------------------------------------------
        # Extract pose
        # ---------------------------------------------------------

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        yaw = euler_from_quaternion(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # ---------------------------------------------------------
        # Store odometry
        # ---------------------------------------------------------

        self.odom_times.append(t)
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.odom_yaw.append(yaw)

    # =============================================================
    # MAIN TIMER
    # =============================================================

    def timer_callback(self):

        if self.finished:
            return

        # ---------------------------------------------------------
        # Wait until simulation clock starts
        # ---------------------------------------------------------

        if self.sim_time < 0.1:
            return

        # ---------------------------------------------------------
        # Wait until odometry is received
        # ---------------------------------------------------------

        if not self.controller_ready:

            self.ready_check_count += 1

            # Log once per second
            if self.ready_check_count % 100 == 0:

                self.get_logger().info(
                    'Waiting for controller '
                    f'(sim_time={self.sim_time:.1f}s)...'
                )

                # Send zero velocity while waiting
                stop_msg = Twist()

                self.cmd_vel_pub.publish(
                    stop_msg
                )

            return

        # ---------------------------------------------------------
        # Start replay
        # ---------------------------------------------------------

        if not self.started:

            self.started = True

            self.sim_start_time = self.sim_time

            self.get_logger().info(
                'Controller ready! '
                f'Starting cmd_vel replay at '
                f'sim_time={self.sim_time:.2f}s'
            )

        # ---------------------------------------------------------
        # Calculate elapsed epoch time
        # ---------------------------------------------------------

        elapsed = (
            self.sim_time -
            self.sim_start_time
        )

        # ---------------------------------------------------------
        # Check if replay is finished
        # ---------------------------------------------------------

        if elapsed >= self.duration:

            self.finish()

            return

        # ---------------------------------------------------------
        # Replay cmd_vel
        # ---------------------------------------------------------

        while (
            self.cmd_idx < len(self.cmd_vel_times)
            and
            self.cmd_vel_times[self.cmd_idx] <= elapsed
        ):

            msg = Twist()

            msg.linear.x = float(
                self.cmd_vel_linear[self.cmd_idx]
            )

            msg.angular.z = float(
                self.cmd_vel_angular[self.cmd_idx]
            )

            self.cmd_vel_pub.publish(msg)

            self.cmd_idx += 1

    # =============================================================
    # FINISH
    # =============================================================

    def finish(self):

        if self.finished:
            return

        self.finished = True

        # ---------------------------------------------------------
        # Stop robot
        # ---------------------------------------------------------

        stop_msg = Twist()

        self.cmd_vel_pub.publish(
            stop_msg
        )

        # ---------------------------------------------------------
        # Save result
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Epoch complete! '
            f'Recorded {len(self.odom_times)} odom samples.'
        )

        self.get_logger().info(
            f'Saving to {self.output_path}'
        )

        np.savez(
            self.output_path,

            odom_times=np.array(
                self.odom_times
            ),

            odom_x=np.array(
                self.odom_x
            ),

            odom_y=np.array(
                self.odom_y
            ),

            odom_yaw=np.array(
                self.odom_yaw
            )
        )

        self.get_logger().info(
            'Results saved.'
        )

        # ---------------------------------------------------------
        # Shutdown shortly after saving
        # ---------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.5,
            self.shutdown_callback
        )

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def shutdown_callback(self):

        self.get_logger().info(
            'Shutting down node.'
        )

        raise SystemExit(0)


# =================================================================
# MAIN
# =================================================================

def main():

    parser = argparse.ArgumentParser(
        description='Sim Epoch Runner'
    )

    parser.add_argument(
        '--bag-data',
        required=True,
        help='Path to bag_data.npz'
    )

    parser.add_argument(
        '--output',
        required=True,
        help='Path to save simulated odometry'
    )

    parser.add_argument(
        '--duration',
        type=float,
        required=True,
        help='Bag duration in seconds'
    )

    args, unknown = parser.parse_known_args()

    rclpy.init(
        args=unknown
    )

    node = SimEpochRunner(
        args.bag_data,
        args.output,
        args.duration
    )

    try:

        rclpy.spin(node)

    except SystemExit:

        pass

    finally:

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':

    main()
