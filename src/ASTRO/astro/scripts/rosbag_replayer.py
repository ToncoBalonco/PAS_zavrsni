#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

import rosbag2_py

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class RosbagCmdVelReplayer(Node):

    def __init__(self):
        super().__init__('rosbag_cmdvel_replayer')

        # ============================================================
        # PARAMETERS
        # ============================================================

        # Bag sa stvarnim robotom:
        self.declare_parameter(
            'bag_path',
            '/home/tona/astro_ws_pas/bags/rosbag2_2026_08_30-18_33_37'
        )

        # Bag sa OptiTrack podacima:
        self.declare_parameter(
            'optitrack_bag_path',
            '/home/tona/astro_ws_pas/bags/rosbag2_2026_08_30-20_33_37_opt'
        )

        self.declare_parameter(
            'input_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'output_topic',
            '/diff_drive_base_controller/cmd_vel'
        )

        self.declare_parameter(
            'gazebo_pose_topic',
            '/world/empty/dynamic_pose/info'
        )

        self.declare_parameter(
            'optitrack_topic',
            '/vrpn_mocap/astro_robot_5/pose'
        )

        self.declare_parameter(
            'robot_frame_id',
            'astro'
        )

        self.declare_parameter(
            'rate',
            1.0
        )

        self.declare_parameter(
            'loop',
            False
        )

        self.declare_parameter(
            'output_dir',
            '/home/tona/astro_ws_pas/results'
        )

        # ============================================================
        # READ PARAMETERS
        # ============================================================

        self.bag_path = self.get_parameter('bag_path').value
        self.optitrack_bag_path = self.get_parameter(
            'optitrack_bag_path'
        ).value

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value

        self.output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.gazebo_pose_topic = self.get_parameter(
            'gazebo_pose_topic'
        ).value

        self.optitrack_topic = self.get_parameter(
            'optitrack_topic'
        ).value

        self.robot_frame_id = self.get_parameter(
            'robot_frame_id'
        ).value

        self.rate = float(
            self.get_parameter('rate').value
        )

        self.loop = bool(
            self.get_parameter('loop').value
        )

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )

        os.makedirs(
            self.output_dir,
            exist_ok=True
        )

        # ============================================================
        # LOG
        # ============================================================

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            'ROSbag + OptiTrack bag replay'
        )
        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            f'Real robot bag     : {self.bag_path}'
        )

        self.get_logger().info(
            f'OptiTrack bag      : {self.optitrack_bag_path}'
        )

        self.get_logger().info(
            f'Input topic        : {self.input_topic}'
        )

        self.get_logger().info(
            f'Output topic       : {self.output_topic}'
        )

        self.get_logger().info(
            f'OptiTrack topic    : {self.optitrack_topic}'
        )

        self.get_logger().info(
            f'Gazebo pose topic  : {self.gazebo_pose_topic}'
        )

        self.get_logger().info(
            f'Output directory   : {self.output_dir}'
        )

        self.get_logger().info(
            '=============================================='
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        self.last_cmd = TwistStamped()
        self.last_cmd.header.frame_id = self.robot_frame_id

        # ============================================================
        # LOGS
        # ============================================================

        self.bag_cmd_log = []
        self.sent_cmd_log = []

        # ============================================================
        # GAZEBO SUBSCRIBERS
        # ============================================================

        self.gazebo_pose_sub = self.create_subscription(
            TFMessage,
            self.gazebo_pose_topic,
            self.gazebo_pose_callback,
            50
        )

        self.sim_odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.sim_odom_callback,
            50
        )

        self.sim_joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.sim_joint_states_callback,
            50
        )

        # ============================================================
        # SIMULATION DATA
        # ============================================================

        self.sim_pose_data = []
        self.sim_odom_data = []
        self.sim_joint_states_data = []

        self.first_sim_pose_time = None
        self.first_sim_odom_time = None
        self.first_sim_joint_time = None

        self._prev_pose_sample = None
        self._warned_missing_frame = False
        self._detected_frame_id = None

        # ============================================================
        # REAL BAG DATA
        # ============================================================

        self.messages = []

        self.bag_odom_data = []
        self.bag_joint_states_data = []

        # ============================================================
        # OPTITRACK DATA
        # ============================================================

        self.optitrack_data = []

        # ============================================================
        # PLAYBACK STATE
        # ============================================================

        self.current_index = 0
        self.first_bag_time = None
        self.playback_start_ns = None

        self.is_playing = False
        self.finished = False

        self.timer = None
        self.hold_timer = None
        self.finish_timer = None

        # ============================================================
        # LOAD BAGS
        # ============================================================

        self.load_bag()

        self.load_optitrack_bag()

        if not self.messages:
            self.get_logger().error(
                'Nema pronađenih /cmd_vel poruka u realnom bagu.'
            )
            return

        if not self.optitrack_data:
            self.get_logger().error(
                'Nema pronađenih OptiTrack poruka.'
            )
            return

        self.get_logger().info(
            f'Učitano {len(self.messages)} /cmd_vel poruka.'
        )

        self.get_logger().info(
            f'Učitano {len(self.bag_odom_data)} /odom poruka.'
        )

        self.get_logger().info(
            f'Učitano {len(self.bag_joint_states_data)} '
            f'/joint_states poruka.'
        )

        self.get_logger().info(
            f'Učitano {len(self.optitrack_data)} OptiTrack poruka.'
        )

        # ============================================================
        # TIMERS
        # ============================================================

        self.timer = self.create_timer(
            0.001,
            self.playback_loop
        )

        self.hold_timer = self.create_timer(
            0.02,
            self.hold_last_cmd
        )

        self.get_logger().info(
            'Replay node spreman.'
        )

    # ================================================================
    # QUATERNION
    # ================================================================

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ================================================================
    # HOLD LAST COMMAND
    # ================================================================

    def hold_last_cmd(self):

        if not self.is_playing or self.finished:
            return

        now_msg = self.get_clock().now().to_msg()

        self.last_cmd.header.stamp = now_msg

        self.cmd_pub.publish(
            self.last_cmd
        )

        if self.playback_start_ns is not None:

            rel_t = (
                self.get_clock().now().nanoseconds -
                self.playback_start_ns
            ) / 1e9

        else:
            rel_t = 0.0

        self.sent_cmd_log.append({
            'time': rel_t,
            'linear_x': self.last_cmd.twist.linear.x,
            'angular_z': self.last_cmd.twist.angular.z
        })

    # ================================================================
    # GAZEBO POSE CALLBACK
    # ================================================================

    def gazebo_pose_callback(self, msg):

        if not self.is_playing or self.finished:
            return

        match = None
        seen_frames = []

        candidate_frames = [
            self.robot_frame_id,
            'astro',
            'astro/base_footprint',
            'base_footprint',
            'astro/base_link',
            'base_link'
        ]

        for transform_stamped in msg.transforms:

            cid = transform_stamped.child_frame_id

            seen_frames.append(cid)

            if (
                self._detected_frame_id is not None
                and cid == self._detected_frame_id
            ):

                match = transform_stamped
                break

            elif self._detected_frame_id is None:

                for cand in candidate_frames:

                    if (
                        cid == cand
                        or cid.endswith('/' + cand)
                        or cand in cid
                    ):

                        match = transform_stamped

                        self._detected_frame_id = cid

                        self.get_logger().info(
                            f'Gazebo robot frame: "{cid}"'
                        )

                        break

                if match:
                    break

        if match is None:

            if not self._warned_missing_frame:

                self.get_logger().warn(
                    f'Robot frame "{self.robot_frame_id}" '
                    f'nije pronađen.'
                )

                self.get_logger().warn(
                    f'Dostupni frameovi: '
                    f'{sorted(set(seen_frames))}'
                )

                self._warned_missing_frame = True

            return

        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        if self.first_sim_pose_time is None:

            self.first_sim_pose_time = timestamp

        relative_time = (
            timestamp -
            self.first_sim_pose_time
        )

        translation = match.transform.translation
        rotation = match.transform.rotation

        x = translation.x
        y = translation.y

        yaw = self.quaternion_to_yaw(
            rotation
        )

        linear_velocity = 0.0
        angular_velocity = 0.0

        if self._prev_pose_sample is not None:

            (
                prev_time,
                prev_x,
                prev_y,
                prev_yaw
            ) = self._prev_pose_sample

            dt = relative_time - prev_time

            if dt > 1e-6:

                dx = x - prev_x
                dy = y - prev_y

                linear_velocity = (
                    math.sqrt(dx * dx + dy * dy) /
                    dt
                )

                dyaw = math.atan2(
                    math.sin(yaw - prev_yaw),
                    math.cos(yaw - prev_yaw)
                )

                angular_velocity = dyaw / dt

        self._prev_pose_sample = (
            relative_time,
            x,
            y,
            yaw
        )

        self.sim_pose_data.append({
            'time': relative_time,
            'x': x,
            'y': y,
            'yaw': yaw,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        })

    # ================================================================
    # SIMULATION ODOM
    # ================================================================

    def sim_odom_callback(self, msg):

        if not self.is_playing or self.finished:
            return

        timestamp = (
            self.get_clock().now().nanoseconds /
            1e9
        )

        if self.first_sim_odom_time is None:
            self.first_sim_odom_time = timestamp

        relative_time = (
            timestamp -
            self.first_sim_odom_time
        )

        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        self.sim_odom_data.append({
            'time': relative_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        })

    # ================================================================
    # SIMULATION JOINT STATES
    # ================================================================

    def sim_joint_states_callback(self, msg):

        if not self.is_playing or self.finished:
            return

        timestamp = (
            self.get_clock().now().nanoseconds /
            1e9
        )

        if self.first_sim_joint_time is None:
            self.first_sim_joint_time = timestamp

        relative_time = (
            timestamp -
            self.first_sim_joint_time
        )

        velocities = {}

        for i, name in enumerate(msg.name):

            if i < len(msg.velocity):

                velocities[name] = msg.velocity[i]

        self.sim_joint_states_data.append({
            'time': relative_time,
            'velocities': velocities
        })

    # ================================================================
    # LOAD REAL ROBOT BAG
    # ================================================================

    def load_bag(self):

        if not os.path.exists(self.bag_path):

            self.get_logger().error(
                f'Bag ne postoji: {self.bag_path}'
            )

            return

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id='sqlite3'
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = rosbag2_py.SequentialReader()

        try:

            reader.open(
                storage_options,
                converter_options
            )

        except Exception as exc:

            self.get_logger().error(
                f'Greška pri otvaranju realnog baya: {exc}'
            )

            return

        topics = reader.get_all_topics_and_types()

        topic_types = {
            topic.name: topic.type
            for topic in topics
        }

        required_topics = [
            self.input_topic,
            '/odom',
            '/joint_states'
        ]

        for topic in required_topics:

            if topic not in topic_types:

                self.get_logger().warn(
                    f'Topic {topic} ne postoji u bagu.'
                )

        while reader.has_next():

            topic_name, data, timestamp = (
                reader.read_next()
            )

            if topic_name == self.input_topic:

                try:

                    msg = deserialize_message(
                        data,
                        Twist
                    )

                except Exception:

                    continue

                self.messages.append(
                    (timestamp, msg)
                )

            elif topic_name == '/odom':

                try:

                    msg = deserialize_message(
                        data,
                        Odometry
                    )

                except Exception:

                    continue

                self.bag_odom_data.append(
                    (timestamp, msg)
                )

            elif topic_name == '/joint_states':

                try:

                    msg = deserialize_message(
                        data,
                        JointState
                    )

                except Exception:

                    continue

                self.bag_joint_states_data.append(
                    (timestamp, msg)
                )

        self.messages.sort(
            key=lambda item: item[0]
        )

        self.bag_odom_data.sort(
            key=lambda item: item[0]
        )

        self.bag_joint_states_data.sort(
            key=lambda item: item[0]
        )

    # ================================================================
    # LOAD OPTITRACK BAG
    # ================================================================

    def load_optitrack_bag(self):

        self.optitrack_data = []

        if not os.path.exists(
            self.optitrack_bag_path
        ):

            self.get_logger().error(
                'OptiTrack bag ne postoji: '
                f'{self.optitrack_bag_path}'
            )

            return

        storage_options = rosbag2_py.StorageOptions(
            uri=self.optitrack_bag_path,
            storage_id='sqlite3'
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = rosbag2_py.SequentialReader()

        try:

            reader.open(
                storage_options,
                converter_options
            )

        except Exception as exc:

            self.get_logger().error(
                f'Greška pri otvaranju OptiTrack baya: {exc}'
            )

            return

        topics = reader.get_all_topics_and_types()

        topic_types = {
            topic.name: topic.type
            for topic in topics
        }

        if self.optitrack_topic not in topic_types:

            self.get_logger().error(
                f'OptiTrack topic {self.optitrack_topic} '
                f'nije pronađen u bagu.'
            )

            self.get_logger().error(
                f'Dostupni topicovi: '
                f'{list(topic_types.keys())}'
            )

            return

        first_timestamp = None
        first_x = None
        first_y = None
        first_yaw = None

        while reader.has_next():

            topic_name, data, timestamp = (
                reader.read_next()
            )

            if topic_name != self.optitrack_topic:
                continue

            try:

                msg = deserialize_message(
                    data,
                    PoseStamped
                )

            except Exception:

                continue

            x = msg.pose.position.x
            y = msg.pose.position.y

            yaw = self.quaternion_to_yaw(
                msg.pose.orientation
            )

            if first_timestamp is None:

                first_timestamp = timestamp
                first_x = x
                first_y = y
                first_yaw = yaw

            relative_time = (
                timestamp -
                first_timestamp
            ) / 1e9

            self.optitrack_data.append({
                'time': relative_time,
                'x': x - first_x,
                'y': y - first_y,
                'yaw': yaw
            })

        self.get_logger().info(
            f'OptiTrack poruka učitano: '
            f'{len(self.optitrack_data)}'
        )

        if self.optitrack_data:

            self.get_logger().info(
                f'OptiTrack trajanje: '
                f'{self.optitrack_data[-1]["time"]:.3f} s'
            )

    # ================================================================
    # PLAYBACK
    # ================================================================

    def playback_loop(self):

        if not self.messages:
            return

        if not self.is_playing:

            self.is_playing = True

            self.current_index = 0

            self.first_bag_time = (
                self.messages[0][0]
            )

            self.playback_start_ns = (
                self.get_clock().now().nanoseconds
            )

            self.get_logger().info(
                '=============================================='
            )

            self.get_logger().info(
                'POČETAK REPLAYA'
            )

            self.get_logger().info(
                '=============================================='
            )

        if self.current_index >= len(
            self.messages
        ):

            if not self.finished:

                self.finished = True

                self.get_logger().info(
                    'Bag playback završen.'
                )

                self.stop_robot()

                if self.timer is not None:
                    self.timer.cancel()

                self.finish_timer = self.create_timer(
                    2.0,
                    self.finish_and_save_once
                )

            return

        elapsed = (
            self.get_clock().now().nanoseconds -
            self.playback_start_ns
        ) / 1e9

        elapsed *= self.rate

        while self.current_index < len(
            self.messages
        ):

            timestamp, msg = (
                self.messages[self.current_index]
            )

            message_time = (
                timestamp -
                self.first_bag_time
            ) / 1e9

            if message_time > elapsed:
                break

            stamped_msg = TwistStamped()

            stamped_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            stamped_msg.header.frame_id = (
                self.robot_frame_id
            )

            stamped_msg.twist = msg

            self.last_cmd = stamped_msg

            self.cmd_pub.publish(
                stamped_msg
            )

            self.bag_cmd_log.append({
                'time': message_time,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })

            self.current_index += 1

    # ================================================================
    # STOP ROBOT
    # ================================================================

    def stop_robot(self):

        stop_msg = TwistStamped()

        stop_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        stop_msg.header.frame_id = (
            self.robot_frame_id
        )

        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.last_cmd = stop_msg

        for _ in range(10):

            self.cmd_pub.publish(
                stop_msg
            )

    # ================================================================
    # NORMALIZATION
    # ================================================================

    def normalize_xy_yaw(
        self,
        x_list,
        y_list,
        yaw_list
    ):

        if not x_list:
            return [], [], []

        x0 = x_list[0]
        y0 = y_list[0]
        yaw0 = yaw_list[0]

        cos_a = math.cos(-yaw0)
        sin_a = math.sin(-yaw0)

        x_norm = []
        y_norm = []
        yaw_norm = []

        for x, y, yaw in zip(
            x_list,
            y_list,
            yaw_list
        ):

            dx = x - x0
            dy = y - y0

            rx = (
                dx * cos_a -
                dy * sin_a
            )

            ry = (
                dx * sin_a +
                dy * cos_a
            )

            dyaw = math.atan2(
                math.sin(yaw - yaw0),
                math.cos(yaw - yaw0)
            )

            x_norm.append(rx)
            y_norm.append(ry)
            yaw_norm.append(dyaw)

        return (
            x_norm,
            y_norm,
            yaw_norm
        )

    # ================================================================
    # SIMULATION TRAJECTORY
    # ================================================================

    def get_normalized_sim_xy_yaw(self):

        if not self.sim_pose_data:

            return [], [], []

        x = [
            d['x']
            for d in self.sim_pose_data
        ]

        y = [
            d['y']
            for d in self.sim_pose_data
        ]

        yaw = [
            d['yaw']
            for d in self.sim_pose_data
        ]

        return self.normalize_xy_yaw(
            x,
            y,
            yaw
        )

    # ================================================================
    # PLOT CMD VEL
    # ================================================================

    def plot_cmd_vel_comparison(self):

        if (
            not self.bag_cmd_log
            and not self.sent_cmd_log
        ):

            return None

        fig, (ax1, ax2) = plt.subplots(
            2,
            1,
            figsize=(12, 8),
            sharex=True
        )

        if self.bag_cmd_log:

            t = [
                d['time']
                for d in self.bag_cmd_log
            ]

            vx = [
                d['linear_x']
                for d in self.bag_cmd_log
            ]

            ax1.plot(
                t,
                vx,
                label='Ulazni /cmd_vel iz baya',
                linewidth=2
            )

        if self.sent_cmd_log:

            t = [
                d['time']
                for d in self.sent_cmd_log
            ]

            vx = [
                d['linear_x']
                for d in self.sent_cmd_log
            ]

            ax1.plot(
                t,
                vx,
                label='Naredba prema kontroleru',
                linestyle='--',
                linewidth=1.5
            )

        ax1.set_ylabel(
            'Linearna brzina v_x [m/s]'
        )

        ax1.set_title(
            'Usporedba ulaznog i poslanog cmd_vel'
        )

        ax1.grid(True)
        ax1.legend()

        if self.bag_cmd_log:

            wz = [
                d['angular_z']
                for d in self.bag_cmd_log
            ]

            ax2.plot(
                t,
                wz,
                label='Ulazni /cmd_vel iz baya',
                linewidth=2
            )

        if self.sent_cmd_log:

            wz = [
                d['angular_z']
                for d in self.sent_cmd_log
            ]

            ax2.plot(
                t,
                wz,
                label='Naredba prema kontroleru',
                linestyle='--',
                linewidth=1.5
            )

        ax2.set_xlabel(
            'Vrijeme [s]'
        )

        ax2.set_ylabel(
            'Kutna brzina w_z [rad/s]'
        )

        ax2.grid(True)
        ax2.legend()

        fig.tight_layout()

        path = os.path.join(
            self.output_dir,
            'cmd_vel_input_vs_controller.png'
        )

        fig.savefig(
            path,
            dpi=200
        )

        plt.close(fig)

        return path

    # ================================================================
    # PLOT XY SIMULATION
    # ================================================================

    def plot_simulation_xy(self):

        x, y, yaw = (
            self.get_normalized_sim_xy_yaw()
        )

        if not x:
            return None

        fig, ax = plt.subplots(
            figsize=(10, 8)
        )

        ax.plot(
            x,
            y,
            linewidth=2,
            label='Simulacija - Gazebo'
        )

        ax.scatter(
            0,
            0,
            s=100,
            label='Start'
        )

        ax.scatter(
            x[-1],
            y[-1],
            s=100,
            label='Kraj'
        )

        ax.set_xlabel(
            'X [m]'
        )

        ax.set_ylabel(
            'Y [m]'
        )

        ax.set_title(
            'ASTRO - Simulacijska XY putanja'
        )

        ax.axis('equal')
        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path = os.path.join(
            self.output_dir,
            'sim_trajectory_xy.png'
        )

        fig.savefig(
            path,
            dpi=200
        )

        plt.close(fig)

        return path

    # ================================================================
    # XY: SIMULATION VS OPTITRACK VS ODOM
    # ================================================================

    def plot_all_trajectories(self):

        if (
            not self.sim_pose_data
            or not self.optitrack_data
        ):

            self.get_logger().warn(
                'Nema simulacijskih ili OptiTrack podataka.'
            )

            return None

        # ------------------------------------------------------------
        # SIMULATION GROUND TRUTH
        # ------------------------------------------------------------

        sim_x, sim_y, sim_yaw = (
            self.get_normalized_sim_xy_yaw()
        )

        # ------------------------------------------------------------
        # OPTITRACK
        # ------------------------------------------------------------

        opt_x_raw = [
            d['x']
            for d in self.optitrack_data
        ]

        opt_y_raw = [
            d['y']
            for d in self.optitrack_data
        ]

        opt_yaw_raw = [
            d['yaw']
            for d in self.optitrack_data
        ]

        opt_x, opt_y, opt_yaw = (
            self.normalize_xy_yaw(
                opt_x_raw,
                opt_y_raw,
                opt_yaw_raw
            )
        )

        # ------------------------------------------------------------
        # REAL ODOM
        # ------------------------------------------------------------

        bag_odom_x = []
        bag_odom_y = []
        bag_odom_yaw = []

        if self.bag_odom_data:

            bag_odom_x = [
                msg.pose.pose.position.x
                for _, msg in self.bag_odom_data
            ]

            bag_odom_y = [
                msg.pose.pose.position.y
                for _, msg in self.bag_odom_data
            ]

            bag_odom_yaw = [
                self.quaternion_to_yaw(
                    msg.pose.pose.orientation
                )
                for _, msg in self.bag_odom_data
            ]

            (
                bag_odom_x,
                bag_odom_y,
                bag_odom_yaw
            ) = self.normalize_xy_yaw(
                bag_odom_x,
                bag_odom_y,
                bag_odom_yaw
            )

        # ------------------------------------------------------------
        # SIMULATION ODOM
        # ------------------------------------------------------------

        sim_odom_x = []
        sim_odom_y = []
        sim_odom_yaw = []

        if self.sim_odom_data:

            raw_x = [
                d['x']
                for d in self.sim_odom_data
            ]

            raw_y = [
                d['y']
                for d in self.sim_odom_data
            ]

            raw_yaw = [
                d['yaw']
                for d in self.sim_odom_data
            ]

            (
                sim_odom_x,
                sim_odom_y,
                sim_odom_yaw
            ) = self.normalize_xy_yaw(
                raw_x,
                raw_y,
                raw_yaw
            )

        # ------------------------------------------------------------
        # PLOT
        # ------------------------------------------------------------

        fig, ax = plt.subplots(
            figsize=(11, 9)
        )

        ax.plot(
            opt_x,
            opt_y,
            linewidth=2,
            label='OptiTrack - stvarnost'
        )

        ax.plot(
            sim_x,
            sim_y,
            linewidth=2,
            linestyle='--',
            label='Gazebo - ground truth'
        )

        if bag_odom_x:

            ax.plot(
                bag_odom_x,
                bag_odom_y,
                linewidth=1.8,
                linestyle='-.',
                label='Stvarna odometrija - /odom'
            )

        if sim_odom_x:

            ax.plot(
                sim_odom_x,
                sim_odom_y,
                linewidth=1.8,
                linestyle=':',
                label='Simulacijska odometrija'
            )

        ax.scatter(
            0,
            0,
            s=120,
            label='Start',
            zorder=5
        )

        ax.set_xlabel(
            'X [m]'
        )

        ax.set_ylabel(
            'Y [m]'
        )

        ax.set_title(
            'ASTRO - Usporedba stvarne i simulacijske putanje'
        )

        ax.axis('equal')
        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path = os.path.join(
            self.output_dir,
            'all_trajectories_xy.png'
        )

        fig.savefig(
            path,
            dpi=200
        )

        plt.close(fig)

        return path

    # ================================================================
    # X, Y, YAW SIMULATION VS REALITY
    # ================================================================

    def plot_xyz_yaw_sim_vs_real(self):

        if (
            not self.sim_pose_data
            or not self.optitrack_data
        ):

            return None

        # ------------------------------------------------------------
        # SIMULATION
        # ------------------------------------------------------------

        sim_x = [
            d['x']
            for d in self.sim_pose_data
        ]

        sim_y = [
            d['y']
            for d in self.sim_pose_data
        ]

        sim_yaw = [
            d['yaw']
            for d in self.sim_pose_data
        ]

        sim_t = [
            d['time']
            for d in self.sim_pose_data
        ]

        (
            sim_x,
            sim_y,
            sim_yaw
        ) = self.normalize_xy_yaw(
            sim_x,
            sim_y,
            sim_yaw
        )

        # ------------------------------------------------------------
        # OPTITRACK
        # ------------------------------------------------------------

        real_x = [
            d['x']
            for d in self.optitrack_data
        ]

        real_y = [
            d['y']
            for d in self.optitrack_data
        ]

        real_yaw = [
            d['yaw']
            for d in self.optitrack_data
        ]

        real_t = [
            d['time']
            for d in self.optitrack_data
        ]

        (
            real_x,
            real_y,
            real_yaw
        ) = self.normalize_xy_yaw(
            real_x,
            real_y,
            real_yaw
        )

        # ------------------------------------------------------------
        # PLOT X
        # ------------------------------------------------------------

        fig, ax = plt.subplots(
            figsize=(12, 6)
        )

        ax.plot(
            sim_t,
            sim_x,
            linewidth=2,
            label='Simulacija - Gazebo'
        )

        ax.plot(
            real_t,
            real_x,
            linewidth=2,
            linestyle='--',
            label='Stvarnost - OptiTrack'
        )

        ax.set_xlabel(
            'Vrijeme [s]'
        )

        ax.set_ylabel(
            'X [m]'
        )

        ax.set_title(
            'X koordinata - simulacija vs. stvarnost'
        )

        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path_x = os.path.join(
            self.output_dir,
            'x_sim_vs_real.png'
        )

        fig.savefig(
            path_x,
            dpi=200
        )

        plt.close(fig)

        # ------------------------------------------------------------
        # PLOT Y
        # ------------------------------------------------------------

        fig, ax = plt.subplots(
            figsize=(12, 6)
        )

        ax.plot(
            sim_t,
            sim_y,
            linewidth=2,
            label='Simulacija - Gazebo'
        )

        ax.plot(
            real_t,
            real_y,
            linewidth=2,
            linestyle='--',
            label='Stvarnost - OptiTrack'
        )

        ax.set_xlabel(
            'Vrijeme [s]'
        )

        ax.set_ylabel(
            'Y [m]'
        )

        ax.set_title(
            'Y koordinata - simulacija vs. stvarnost'
        )

        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path_y = os.path.join(
            self.output_dir,
            'y_sim_vs_real.png'
        )

        fig.savefig(
            path_y,
            dpi=200
        )

        plt.close(fig)

        # ------------------------------------------------------------
        # PLOT YAW
        # ------------------------------------------------------------

        fig, ax = plt.subplots(
            figsize=(12, 6)
        )

        ax.plot(
            sim_t,
            [
                math.degrees(v)
                for v in sim_yaw
            ],
            linewidth=2,
            label='Simulacija - Gazebo'
        )

        ax.plot(
            real_t,
            [
                math.degrees(v)
                for v in real_yaw
            ],
            linewidth=2,
            linestyle='--',
            label='Stvarnost - OptiTrack'
        )

        ax.set_xlabel(
            'Vrijeme [s]'
        )

        ax.set_ylabel(
            'Yaw [deg]'
        )

        ax.set_title(
            'Yaw - simulacija vs. stvarnost'
        )

        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path_yaw = os.path.join(
            self.output_dir,
            'yaw_sim_vs_real.png'
        )

        fig.savefig(
            path_yaw,
            dpi=200
        )

        plt.close(fig)

        # ------------------------------------------------------------
        # SVA TRI NA JEDNOM GRAFU
        # ------------------------------------------------------------

        fig, ax = plt.subplots(
            figsize=(12, 8)
        )

        ax.plot(
            sim_t,
            sim_x,
            label='Sim X',
            linewidth=1.8
        )

        ax.plot(
            real_t,
            real_x,
            label='Real X',
            linestyle='--',
            linewidth=1.8
        )

        ax.plot(
            sim_t,
            sim_y,
            label='Sim Y',
            linewidth=1.8
        )

        ax.plot(
            real_t,
            real_y,
            label='Real Y',
            linestyle='--',
            linewidth=1.8
        )

        ax.set_xlabel(
            'Vrijeme [s]'
        )

        ax.set_ylabel(
            'Položaj [m]'
        )

        ax.set_title(
            'X i Y - simulacija vs. stvarnost'
        )

        ax.grid(True)
        ax.legend()

        fig.tight_layout()

        path_xy_time = os.path.join(
            self.output_dir,
            'xy_sim_vs_real_time.png'
        )

        fig.savefig(
            path_xy_time,
            dpi=200
        )

        plt.close(fig)

        # ------------------------------------------------------------
        # SAVE NORMALIZED POSE CSV
        # ------------------------------------------------------------

        csv_path = os.path.join(
            self.output_dir,
            'simulation_vs_optitrack_pose.csv'
        )

        with open(
            csv_path,
            'w',
            newline=''
        ) as f:

            import csv

            writer = csv.writer(f)

            writer.writerow([
                'source',
                'time',
                'x',
                'y',
                'yaw_rad',
                'yaw_deg'
            ])

            for t, x, y, yaw in zip(
                sim_t,
                sim_x,
                sim_y,
                sim_yaw
            ):

                writer.writerow([
                    'simulation',
                    t,
                    x,
                    y,
                    yaw,
                    math.degrees(yaw)
                ])

            for t, x, y, yaw in zip(
                real_t,
                real_x,
                real_y,
                real_yaw
            ):

                writer.writerow([
                    'optitrack',
                    t,
                    x,
                    y,
                    yaw,
                    math.degrees(yaw)
                ])

        return (
            path_x,
            path_y,
            path_yaw,
            path_xy_time,
            csv_path
        )

    # ================================================================
    # WHEEL JOINT STATES
    # ================================================================

    def _find_wheel_joint_name(
        self,
        names,
        side
    ):

        exact = f'{side}_wheel_joint'

        if exact in names:
            return exact

        candidates = [
            name
            for name in names
            if side in name.lower()
            and 'wheel' in name.lower()
        ]

        return (
            candidates[0]
            if candidates
            else None
        )

    def plot_joint_states_bag_vs_simulation(self):

        if (
            not self.bag_joint_states_data
            or not self.sim_joint_states_data
        ):

            return None

        # ------------------------------------------------------------
        # BAG TIME
        # ------------------------------------------------------------

        bag_t0 = (
            self.bag_joint_states_data[0][0]
        )

        bag_names = []

        for _, msg in self.bag_joint_states_data:

            for name in msg.name:

                if name not in bag_names:
                    bag_names.append(name)

        sim_names = []

        for data in self.sim_joint_states_data:

            for name in data['velocities']:

                if name not in sim_names:
                    sim_names.append(name)

        bag_left = self._find_wheel_joint_name(
            bag_names,
            'left'
        )

        bag_right = self._find_wheel_joint_name(
            bag_names,
            'right'
        )

        sim_left = self._find_wheel_joint_name(
            sim_names,
            'left'
        )

        sim_right = self._find_wheel_joint_name(
            sim_names,
            'right'
        )

        if (
            not bag_left
            or not bag_right
            or not sim_left
            or not sim_right
        ):

            self.get_logger().warn(
                f'Wheel jointovi nisu pronađeni.'
            )

            return None

        def extract_bag_velocity(
            data,
            joint_name
        ):

            times = []
            values = []

            for timestamp, msg in data:

                if joint_name not in msg.name:
                    continue

                idx = msg.name.index(
                    joint_name
                )

                if idx >= len(msg.velocity):
                    continue

                times.append(
                    (timestamp - bag_t0) / 1e9
                )

                values.append(
                    msg.velocity[idx]
                )

            return times, values

        left_bag_t, left_bag = (
            extract_bag_velocity(
                self.bag_joint_states_data,
                bag_left
            )
        )

        right_bag_t, right_bag = (
            extract_bag_velocity(
                self.bag_joint_states_data,
                bag_right
            )
        )

        sim_time = [
            d['time']
            for d in self.sim_joint_states_data
        ]

        sim_left_values = [
            d['velocities'].get(
                sim_left,
                float('nan')
            )
            for d in self.sim_joint_states_data
        ]

        sim_right_values = [
            d['velocities'].get(
                sim_right,
                float('nan')
            )
            for d in self.sim_joint_states_data
        ]

        fig, (ax1, ax2) = plt.subplots(
            2,
            1,
            figsize=(12, 9),
            sharex=True
        )

        ax1.plot(
            left_bag_t,
            left_bag,
            label='Stvarnost - lijevi kotač',
            linewidth=1.8
        )

        ax1.plot(
            sim_time,
            sim_left_values,
            label='Simulacija - lijevi kotač',
            linestyle='--',
            linewidth=1.6
        )

        ax1.set_ylabel(
            'Kutna brzina [rad/s]'
        )

        ax1.set_title(
            'Lijevi kotač'
        )

        ax1.grid(True)
        ax1.legend()

        ax2.plot(
            right_bag_t,
            right_bag,
            label='Stvarnost - desni kotač',
            linewidth=1.8
        )

        ax2.plot(
            sim_time,
            sim_right_values,
            label='Simulacija - desni kotač',
            linestyle='--',
            linewidth=1.6
        )

        ax2.set_xlabel(
            'Vrijeme [s]'
        )

        ax2.set_ylabel(
            'Kutna brzina [rad/s]'
        )

        ax2.set_title(
            'Desni kotač'
        )

        ax2.grid(True)
        ax2.legend()

        fig.tight_layout()

        path = os.path.join(
            self.output_dir,
            'joint_states_bag_vs_simulation.png'
        )

        fig.savefig(
            path,
            dpi=200
        )

        plt.close(fig)

        return path

    # ================================================================
    # SAVE SIM CSV
    # ================================================================

    def save_sim_csv(self):

        import csv

        path = os.path.join(
            self.output_dir,
            'sim_pose.csv'
        )

        with open(
            path,
            'w',
            newline=''
        ) as csvfile:

            writer = csv.writer(csvfile)

            writer.writerow([
                'time',
                'x',
                'y',
                'yaw',
                'linear_velocity',
                'angular_velocity'
            ])

            for data in self.sim_pose_data:

                writer.writerow([
                    data['time'],
                    data['x'],
                    data['y'],
                    data['yaw'],
                    data['linear_velocity'],
                    data['angular_velocity']
                ])

        return path

    # ================================================================
    # SAVE RESULTS
    # ================================================================

    def save_results(self):

        self.get_logger().info(
            'Generiranje rezultata...'
        )

        self.save_sim_csv()

        self.plot_cmd_vel_comparison()

        self.plot_simulation_xy()

        self.plot_all_trajectories()

        self.plot_xyz_yaw_sim_vs_real()

        self.plot_joint_states_bag_vs_simulation()

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            f'Rezultati spremljeni u: {self.output_dir}'
        )

        self.get_logger().info(
            '=============================================='
        )

    # ================================================================
    # FINISH
    # ================================================================

    def finish_and_save_once(self):

        if self.finish_timer is not None:

            self.finish_timer.cancel()

            self.finish_timer = None

        self.save_results()

        rclpy.shutdown()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = RosbagCmdVelReplayer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if not node.finished:

            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
