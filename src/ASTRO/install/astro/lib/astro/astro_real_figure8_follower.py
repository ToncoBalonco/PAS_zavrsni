#!/usr/bin/env python3

import math
import csv
from pathlib import Path as FilePath
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

import tf2_ros


class AstroRealFigure8Follower(Node):

    def __init__(self):

        super().__init__(
            'astro_real_figure8_follower'
        )

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter(
            'lookahead_distance',
            0.2
        )

        self.declare_parameter(
            'max_linear_velocity',
            0.25
        )

        self.declare_parameter(
            'max_angular_velocity',
            1.0
        )

        self.declare_parameter(
            'min_linear_velocity',
            0.01
        )

        self.declare_parameter(
            'control_rate',
            30.0
        )

        self.declare_parameter(
            'goal_tolerance',
            0.01
        )

        self.declare_parameter(
            'path_topic',
            '/figure8_real_path'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel_nav'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        self.declare_parameter(
            'tf_timeout',
            0.5
        )

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.min_linear_velocity = self.get_parameter(
            'min_linear_velocity'
        ).value

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        path_topic = self.get_parameter(
            'path_topic'
        ).value

        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.tf_timeout = self.get_parameter(
            'tf_timeout'
        ).value

        # =====================================================
        # DATA DIRECTORY
        # =====================================================

        self.data_root = FilePath(
            '/home/tona/astro_ws_pas/src/ASTRO/astro/data'
        )

        self.data_root.mkdir(
            parents=True,
            exist_ok=True
        )

        # =====================================================
        # VARIJABLE
        # =====================================================

        self.path = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.finished = False

        self.current_index = 0

        self.last_tf_time = None

        # =====================================================
        # DATA LOGGING
        # =====================================================

        self.recording_started = False

        self.recording_start_time = None

        self.actual_data = []

        self.ideal_data = []

        self.run_directory = None

        self.data_saved = False

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'ASTRO REAL FIGURE-8 FOLLOWER'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'Path topic: {path_topic}'
        )

        self.get_logger().info(
            f'Command topic: {cmd_vel_topic}'
        )

        self.get_logger().info(
            f'TF: {self.odom_frame} -> '
            f'{self.base_frame}'
        )

        self.get_logger().info(
            f'Lookahead: '
            f'{self.lookahead_distance:.2f} m'
        )

        self.get_logger().info(
            f'Max velocity: '
            f'{self.max_linear_velocity:.2f} m/s'
        )

        self.get_logger().info(
            f'Data directory: {self.data_root}'
        )

        self.get_logger().info(
            'Waiting for path and TF...'
        )

    # =========================================================
    # PATH CALLBACK
    # =========================================================

    def path_callback(self, msg):

        if self.path is None:

            self.path = msg

            self.current_index = 0

            self.finished = False

            self.get_logger().info(
                f'Received figure-8 path with '
                f'{len(msg.poses)} points.'
            )

    # =========================================================
    # GET ROBOT POSE FROM TF
    # =========================================================

    def update_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            translation = transform.transform.translation

            rotation = transform.transform.rotation

            self.robot_x = translation.x
            self.robot_y = translation.y

            self.robot_yaw = (
                self.quaternion_to_yaw(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w
                )
            )

            self.last_tf_time = (
                self.get_clock().now()
            )

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            return False

    # =========================================================
    # QUATERNION -> YAW
    # =========================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w
    ):

        siny_cosp = (
            2.0 *
            (w * z + x * y)
        )

        cosy_cosp = (
            1.0 -
            2.0 *
            (y * y + z * z)
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =========================================================
    # NORMALIZACIJA KUTA
    # =========================================================

    def normalize_angle(
        self,
        angle
    ):

        while angle > math.pi:

            angle -= 2.0 * math.pi

        while angle < -math.pi:

            angle += 2.0 * math.pi

        return angle

    # =========================================================
    # SPREMI POČETNU POZICIJU
    # =========================================================

    def capture_start_pose(self):

        if self.start_x is not None:
            return

        if self.robot_x is None:
            return

        self.start_x = self.robot_x
        self.start_y = self.robot_y
        self.start_yaw = self.robot_yaw

        # =====================================================
        # KREIRAJ MAPU ZA OVU VOŽNJU
        # =====================================================

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        self.run_directory = (
            self.data_root /
            f'figure8_{timestamp}'
        )

        self.run_directory.mkdir(
            parents=True,
            exist_ok=True
        )

        self.recording_start_time = (
            self.get_clock().now()
        )

        self.recording_started = True

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'START POSE CAPTURED'
        )

        self.get_logger().info(
            f'x = {self.start_x:.3f} m'
        )

        self.get_logger().info(
            f'y = {self.start_y:.3f} m'
        )

        self.get_logger().info(
            f'yaw = {self.start_yaw:.3f} rad'
        )

        self.get_logger().info(
            f'Data directory: {self.run_directory}'
        )

        self.get_logger().info(
            'Figure-8 will start from this position.'
        )

        self.get_logger().info(
            '========================================'
        )

        # =====================================================
        # KREIRAJ IDEALNU PUTANJU
        # =====================================================

        self.generate_ideal_path_data()

    # =========================================================
    # LOKALNA PUTANJA -> ODOM
    # =========================================================

    def local_to_odom(
        self,
        local_x,
        local_y
    ):

        cos_yaw = math.cos(
            self.start_yaw
        )

        sin_yaw = math.sin(
            self.start_yaw
        )

        odom_x = (
            self.start_x +
            cos_yaw * local_x -
            sin_yaw * local_y
        )

        odom_y = (
            self.start_y +
            sin_yaw * local_x +
            cos_yaw * local_y
        )

        return odom_x, odom_y

    # =========================================================
    # GENERIRANJE IDEALNE PUTANJE
    # =========================================================

    def generate_ideal_path_data(self):

        if self.path is None:
            return

        if self.start_x is None:
            return

        self.ideal_data = []

        for i, pose_stamped in enumerate(
            self.path.poses
        ):

            local_x = (
                pose_stamped.pose.position.x
            )

            local_y = (
                pose_stamped.pose.position.y
            )

            ideal_x, ideal_y = (
                self.local_to_odom(
                    local_x,
                    local_y
                )
            )

            self.ideal_data.append({
                'index': i,
                'local_x': local_x,
                'local_y': local_y,
                'x': ideal_x,
                'y': ideal_y
            })

    # =========================================================
    # SPREMANJE STVARNE POZICIJE
    # =========================================================

    def record_actual_pose(
        self,
        target_x=None,
        target_y=None,
        linear_velocity=0.0,
        angular_velocity=0.0
    ):

        if not self.recording_started:
            return

        if self.robot_x is None:
            return

        current_time = (
            self.get_clock().now()
        )

        elapsed = (
            current_time -
            self.recording_start_time
        ).nanoseconds / 1e9

        if target_x is None:
            target_x = float('nan')

        if target_y is None:
            target_y = float('nan')

        # ---------------------------------------------
        # Greška prema trenutno odabranoj target točki
        # ---------------------------------------------

        if (
            not math.isnan(target_x)
            and not math.isnan(target_y)
        ):

            target_error = math.sqrt(
                (target_x - self.robot_x) ** 2 +
                (target_y - self.robot_y) ** 2
            )

        else:

            target_error = float('nan')

        self.actual_data.append({
            'time': elapsed,
            'x': self.robot_x,
            'y': self.robot_y,
            'yaw': self.robot_yaw,
            'target_x': target_x,
            'target_y': target_y,
            'target_error': target_error,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        })

    # =========================================================
    # LOOKAHEAD
    # =========================================================

    def find_lookahead_point(self):

        if self.path is None:
            return None

        poses = self.path.poses

        if len(poses) == 0:
            return None

        for i in range(
            self.current_index,
            len(poses)
        ):

            local_x = (
                poses[i].pose.position.x
            )

            local_y = (
                poses[i].pose.position.y
            )

            target_x, target_y = (
                self.local_to_odom(
                    local_x,
                    local_y
                )
            )

            dx = (
                target_x -
                self.robot_x
            )

            dy = (
                target_y -
                self.robot_y
            )

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            if distance >= self.lookahead_distance:

                self.current_index = i

                return target_x, target_y

        last = poses[-1]

        return self.local_to_odom(
            last.pose.position.x,
            last.pose.position.y
        )

    # =========================================================
    # PURE PURSUIT
    # =========================================================

    def calculate_command(
        self,
        target_x,
        target_y
    ):

        dx = (
            target_x -
            self.robot_x
        )

        dy = (
            target_y -
            self.robot_y
        )

        cos_yaw = math.cos(
            self.robot_yaw
        )

        sin_yaw = math.sin(
            self.robot_yaw
        )

        local_x = (
            cos_yaw * dx +
            sin_yaw * dy
        )

        local_y = (
            -sin_yaw * dx +
            cos_yaw * dy
        )

        # -------------------------------------------------
        # Ako je cilj iza robota
        # -------------------------------------------------

        if local_x <= 0.0:

            self.get_logger().debug(
                'Target behind robot, stopping.'
            )

            return 0.0, 0.0

        distance_squared = (
            local_x * local_x +
            local_y * local_y
        )

        curvature = (
            2.0 *
            local_y /
            max(
                distance_squared,
                1e-6
            )
        )

        v = self.max_linear_velocity

        omega = curvature * v

        omega = max(
            -self.max_angular_velocity,
            min(
                omega,
                self.max_angular_velocity
            )
        )

        distance = math.sqrt(
            distance_squared
        )

        # -------------------------------------------------
        # Smanjenje brzine blizu cilja
        # -------------------------------------------------

        if distance < 0.80:

            scale = (
                distance /
                0.80
            )

            v *= max(
                0.3,
                scale
            )

        if v > 0.0:

            v = max(
                v,
                self.min_linear_velocity
            )

        # -------------------------------------------------
        # DEBUG LOGGING
        # -------------------------------------------------

        self.get_logger().info(
            f'Target: ({target_x:.3f}, {target_y:.3f})  '
            f'Robot: ({self.robot_x:.3f}, {self.robot_y:.3f})  '
            f'Yaw: {self.robot_yaw:.3f}  '
            f'local: ({local_x:.3f}, {local_y:.3f})  '
            f'curvature: {curvature:.3f}  '
            f'v: {v:.3f}, omega: {omega:.3f}'
        )

        return v, omega

    # =========================================================
    # GOAL
    # =========================================================

    def check_goal(self):

        if self.path is None:
            return False

        if len(self.path.poses) == 0:
            return False

        progress = (
            self.current_index /
            float(len(self.path.poses))
        )

        if progress < 0.90:
            return False

        goal = self.path.poses[-1]

        goal_x, goal_y = (
            self.local_to_odom(
                goal.pose.position.x,
                goal.pose.position.y
            )
        )

        dx = (
            goal_x -
            self.robot_x
        )

        dy = (
            goal_y -
            self.robot_y
        )

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        return (
            distance <
            self.goal_tolerance
        )

    # =========================================================
    # STOP
    # =========================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    # =========================================================
    # PATH LENGTH
    # =========================================================

    def calculate_path_length(
        self,
        points
    ):

        if len(points) < 2:
            return 0.0

        total = 0.0

        for i in range(
            1,
            len(points)
        ):

            dx = (
                points[i][0] -
                points[i - 1][0]
            )

            dy = (
                points[i][1] -
                points[i - 1][1]
            )

            total += math.sqrt(
                dx * dx +
                dy * dy
            )

        return total

    # =========================================================
    # NAJBLIŽA TOČKA IDEALNE PUTANJE
    # =========================================================

    def nearest_ideal_distance(
        self,
        x,
        y
    ):

        if not self.ideal_data:
            return float('nan')

        min_distance = float('inf')

        for point in self.ideal_data:

            dx = (
                x -
                point['x']
            )

            dy = (
                y -
                point['y']
            )

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            if distance < min_distance:
                min_distance = distance

        return min_distance

    # =========================================================
    # ANALIZA PODATAKA
    # =========================================================

    def calculate_metrics(self):

        if not self.actual_data:
            return None

        # =====================================================
        # IDEALNA PUTANJA
        # =====================================================

        ideal_points = [
            (
                p['x'],
                p['y']
            )
            for p in self.ideal_data
        ]

        ideal_length = (
            self.calculate_path_length(
                ideal_points
            )
        )

        # =====================================================
        # STVARNA PUTANJA
        # =====================================================

        actual_points = [
            (
                p['x'],
                p['y']
            )
            for p in self.actual_data
        ]

        actual_length = (
            self.calculate_path_length(
                actual_points
            )
        )

        # =====================================================
        # VRIJEME
        # =====================================================

        start_time = (
            self.actual_data[0]['time']
        )

        end_time = (
            self.actual_data[-1]['time']
        )

        duration = end_time - start_time

        # =====================================================
        # GREŠKE
        # =====================================================

        errors = []

        for sample in self.actual_data:

            error = (
                self.nearest_ideal_distance(
                    sample['x'],
                    sample['y']
                )
            )

            if not math.isnan(error):

                errors.append(error)

        if errors:

            mean_error = (
                sum(errors) /
                len(errors)
            )

            rmse_error = math.sqrt(
                sum(
                    e * e
                    for e in errors
                ) /
                len(errors)
            )

            max_error = max(errors)

        else:

            mean_error = float('nan')
            rmse_error = float('nan')
            max_error = float('nan')

        # =====================================================
        # ZAVRŠNA GREŠKA
        # =====================================================

        final_actual = self.actual_data[-1]

        if self.ideal_data:

            final_ideal = self.ideal_data[-1]

            final_error = math.sqrt(
                (
                    final_actual['x'] -
                    final_ideal['x']
                ) ** 2
                +
                (
                    final_actual['y'] -
                    final_ideal['y']
                ) ** 2
            )

        else:

            final_error = float('nan')

        # =====================================================
        # MAX TARGET ERROR
        # =====================================================

        target_errors = []

        for sample in self.actual_data:

            error = sample['target_error']

            if not math.isnan(error):

                target_errors.append(error)

        if target_errors:

            mean_target_error = (
                sum(target_errors) /
                len(target_errors)
            )

            max_target_error = max(
                target_errors
            )

        else:

            mean_target_error = float('nan')
            max_target_error = float('nan')

        return {
            'duration': duration,
            'ideal_length': ideal_length,
            'actual_length': actual_length,
            'mean_error': mean_error,
            'rmse_error': rmse_error,
            'max_error': max_error,
            'final_error': final_error,
            'mean_target_error': mean_target_error,
            'max_target_error': max_target_error,
            'actual_samples': len(
                self.actual_data
            ),
            'ideal_samples': len(
                self.ideal_data
            )
        }

    # =========================================================
    # SAVE ACTUAL CSV
    # =========================================================

    def save_actual_csv(self):

        if self.run_directory is None:
            return

        file_path = (
            self.run_directory /
            'actual_path.csv'
        )

        with open(
            file_path,
            'w',
            newline=''
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                'time_s',
                'x_m',
                'y_m',
                'yaw_rad',
                'target_x_m',
                'target_y_m',
                'target_error_m',
                'linear_velocity_m_s',
                'angular_velocity_rad_s'
            ])

            for sample in self.actual_data:

                writer.writerow([
                    f"{sample['time']:.6f}",
                    f"{sample['x']:.6f}",
                    f"{sample['y']:.6f}",
                    f"{sample['yaw']:.6f}",
                    f"{sample['target_x']:.6f}",
                    f"{sample['target_y']:.6f}",
                    f"{sample['target_error']:.6f}",
                    f"{sample['linear_velocity']:.6f}",
                    f"{sample['angular_velocity']:.6f}"
                ])

        self.get_logger().info(
            f'Actual path saved: {file_path}'
        )

    # =========================================================
    # SAVE IDEAL CSV
    # =========================================================

    def save_ideal_csv(self):

        if self.run_directory is None:
            return

        file_path = (
            self.run_directory /
            'ideal_path.csv'
        )

        with open(
            file_path,
            'w',
            newline=''
        ) as file:

            writer = csv.writer(file)

            writer.writerow([
                'index',
                'local_x_m',
                'local_y_m',
                'ideal_x_m',
                'ideal_y_m'
            ])

            for point in self.ideal_data:

                writer.writerow([
                    point['index'],
                    f"{point['local_x']:.6f}",
                    f"{point['local_y']:.6f}",
                    f"{point['x']:.6f}",
                    f"{point['y']:.6f}"
                ])

        self.get_logger().info(
            f'Ideal path saved: {file_path}'
        )

    # =========================================================
    # SAVE METRICS
    # =========================================================

    def save_metrics(
        self,
        metrics,
        status='COMPLETED'
    ):

        if self.run_directory is None:
            return

        file_path = (
            self.run_directory /
            'metrics.txt'
        )

        with open(
            file_path,
            'w'
        ) as file:

            file.write(
                'ASTRO FIGURE-8 PATH ANALYSIS\n'
            )

            file.write(
                '========================================\n'
            )

            file.write(
                f'Status: {status}\n'
            )

            file.write(
                f'Date: '
                f'{datetime.now().isoformat()}\n'
            )

            file.write(
                '\n'
            )

            file.write(
                'START POSE\n'
            )

            file.write(
                '----------------------------------------\n'
            )

            file.write(
                f'x: {self.start_x:.6f} m\n'
            )

            file.write(
                f'y: {self.start_y:.6f} m\n'
            )

            file.write(
                f'yaw: {self.start_yaw:.6f} rad\n'
            )

            file.write(
                '\n'
            )

            if metrics is not None:

                file.write(
                    'PATH METRICS\n'
                )

                file.write(
                    '----------------------------------------\n'
                )

                file.write(
                    f'Duration: '
                    f'{metrics["duration"]:.6f} s\n'
                )

                file.write(
                    f'Ideal path length: '
                    f'{metrics["ideal_length"]:.6f} m\n'
                )

                file.write(
                    f'Actual path length: '
                    f'{metrics["actual_length"]:.6f} m\n'
                )

                file.write(
                    f'Mean path error: '
                    f'{metrics["mean_error"]:.6f} m\n'
                )

                file.write(
                    f'RMSE path error: '
                    f'{metrics["rmse_error"]:.6f} m\n'
                )

                file.write(
                    f'Maximum path error: '
                    f'{metrics["max_error"]:.6f} m\n'
                )

                file.write(
                    f'Final position error: '
                    f'{metrics["final_error"]:.6f} m\n'
                )

                file.write(
                    f'Mean target error: '
                    f'{metrics["mean_target_error"]:.6f} m\n'
                )

                file.write(
                    f'Maximum target error: '
                    f'{metrics["max_target_error"]:.6f} m\n'
                )

                file.write(
                    f'Actual samples: '
                    f'{metrics["actual_samples"]}\n'
                )

                file.write(
                    f'Ideal samples: '
                    f'{metrics["ideal_samples"]}\n'
                )

        self.get_logger().info(
            f'Metrics saved: {file_path}'
        )

    # =========================================================
    # GENERATE PLOT
    # =========================================================

    def generate_plot(self):

        if self.run_directory is None:
            return

        if not self.actual_data:
            return

        if not self.ideal_data:
            return

        try:

            import matplotlib.pyplot as plt

        except ImportError:

            self.get_logger().error(
                'matplotlib is not installed. '
                'Cannot generate path plot.'
            )

            return

        ideal_x = [
            p['x']
            for p in self.ideal_data
        ]

        ideal_y = [
            p['y']
            for p in self.ideal_data
        ]

        actual_x = [
            p['x']
            for p in self.actual_data
        ]

        actual_y = [
            p['y']
            for p in self.actual_data
        ]

        plt.figure(
            figsize=(10, 8)
        )

        plt.plot(
            ideal_x,
            ideal_y,
            linestyle='--',
            linewidth=2,
            label='Ideal path'
        )

        plt.plot(
            actual_x,
            actual_y,
            linewidth=2,
            label='Real robot path'
        )

        # -------------------------------------------------
        # START
        # -------------------------------------------------

        plt.scatter(
            [actual_x[0]],
            [actual_y[0]],
            s=80,
            label='Start'
        )

        # -------------------------------------------------
        # END
        # -------------------------------------------------

        plt.scatter(
            [actual_x[-1]],
            [actual_y[-1]],
            s=80,
            label='End'
        )

        plt.xlabel(
            'X [m]'
        )

        plt.ylabel(
            'Y [m]'
        )

        plt.title(
            'ASTRO Figure-8: Ideal vs Real Path'
        )

        plt.axis(
            'equal'
        )

        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()

        file_path = (
            self.run_directory /
            'path_comparison.png'
        )

        plt.savefig(
            file_path,
            dpi=200
        )

        plt.close()

        self.get_logger().info(
            f'Path plot saved: {file_path}'
        )

    # =========================================================
    # SAVE ALL DATA
    # =========================================================

    def save_all_data(
        self,
        status='COMPLETED'
    ):

        if self.data_saved:
            return

        if self.run_directory is None:
            return

        if not self.actual_data:
            self.get_logger().warning(
                'No trajectory samples to save.'
            )
            return

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'SAVING FIGURE-8 DATA'
        )

        self.get_logger().info(
            '========================================'
        )

        # -------------------------------------------------
        # CSV datoteke
        # -------------------------------------------------

        self.save_actual_csv()

        self.save_ideal_csv()

        # -------------------------------------------------
        # METRIKE
        # -------------------------------------------------

        metrics = (
            self.calculate_metrics()
        )

        self.save_metrics(
            metrics,
            status
        )

        # -------------------------------------------------
        # GRAF
        # -------------------------------------------------

        self.generate_plot()

        self.data_saved = True

        # -------------------------------------------------
        # Ispis rezultata
        # -------------------------------------------------

        if metrics is not None:

            self.get_logger().info(
                '----------------------------------------'
            )

            self.get_logger().info(
                f'Duration: '
                f'{metrics["duration"]:.3f} s'
            )

            self.get_logger().info(
                f'Ideal path length: '
                f'{metrics["ideal_length"]:.3f} m'
            )

            self.get_logger().info(
                f'Actual path length: '
                f'{metrics["actual_length"]:.3f} m'
            )

            self.get_logger().info(
                f'Mean path error: '
                f'{metrics["mean_error"]:.4f} m'
            )

            self.get_logger().info(
                f'RMSE path error: '
                f'{metrics["rmse_error"]:.4f} m'
            )

            self.get_logger().info(
                f'Max path error: '
                f'{metrics["max_error"]:.4f} m'
            )

            self.get_logger().info(
                f'Final position error: '
                f'{metrics["final_error"]:.4f} m'
            )

            self.get_logger().info(
                '----------------------------------------'
            )

        self.get_logger().info(
            f'ALL DATA SAVED TO:\n'
            f'{self.run_directory}'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        # -----------------------------------------------
        # Uvijek prvo pokušaj dobiti TF
        # -----------------------------------------------

        tf_ok = self.update_robot_pose()

        if not tf_ok:

            self.stop_robot()

            return

        # -----------------------------------------------
        # Ako nemamo putanju
        # -----------------------------------------------

        if self.path is None:

            self.stop_robot()

            return

        # -----------------------------------------------
        # Uhvatimo početnu poziciju samo jednom
        # -----------------------------------------------

        self.capture_start_pose()

        # -----------------------------------------------
        # Ako smo završili
        # -----------------------------------------------

        if self.finished:

            self.stop_robot()

            return

        # -----------------------------------------------
        # Provjera kraja
        # -----------------------------------------------

        if self.check_goal():

            # ------------------------------------------------
            # Zabilježi posljednju stvarnu poziciju
            # ------------------------------------------------

            self.record_actual_pose()

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'FIGURE-8 COMPLETED'
            )

            self.get_logger().info(
                'Stopping ASTRO.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.finished = True

            self.stop_robot()

            # ------------------------------------------------
            # Spremi sve podatke
            # ------------------------------------------------

            self.save_all_data(
                status='COMPLETED'
            )

            return

        # -----------------------------------------------
        # Pronađi lookahead
        # -----------------------------------------------

        target = (
            self.find_lookahead_point()
        )

        if target is None:

            self.stop_robot()

            return

        target_x, target_y = target

        # -----------------------------------------------
        # Izračun brzina
        # -----------------------------------------------

        v, omega = (
            self.calculate_command(
                target_x,
                target_y
            )
        )

        # -----------------------------------------------
        # SPREMI TRENUTNU POZICIJU
        # -----------------------------------------------

        self.record_actual_pose(
            target_x=target_x,
            target_y=target_y,
            linear_velocity=v,
            angular_velocity=omega
        )

        # -----------------------------------------------
        # POŠALJI NA STVARNI ASTRO
        # -----------------------------------------------

        cmd = Twist()

        cmd.linear.x = v

        cmd.angular.z = omega

        self.cmd_vel_pub.publish(
            cmd
        )


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = AstroRealFigure8Follower()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt received.'
        )

    finally:

        node.stop_robot()

        # -------------------------------------------------
        # Ako je vožnja prekinuta prije završetka,
        # ipak spremi sve do tada prikupljene podatke.
        # -------------------------------------------------

        if (
            node.recording_started
            and not node.data_saved
            and node.actual_data
        ):

            node.get_logger().warning(
                'Figure-8 interrupted. '
                'Saving partial trajectory data.'
            )

            node.save_all_data(
                status='INTERRUPTED'
            )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()


