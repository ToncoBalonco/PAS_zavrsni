#!/usr/bin/env python3
import os
import time
import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

import rosbag2_py

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class RosbagCmdVelReplayer(Node):

    def __init__(self):
        super().__init__('rosbag_cmdvel_replayer')

        # ========================================================
        # PARAMETERS
        # ========================================================
        self.declare_parameter('bag_path', '/home/tona/astro_ws_pas/rosbag2_2026_08_21-17_43_13')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_base_controller/cmd_vel')
        self.declare_parameter('gazebo_pose_topic', '/world/empty/dynamic_pose/info')
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('optitrack_csv', '/home/tona/astro_ws_pas/astro_optitrack_cuk_001.csv')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('output_dir', '/home/tona/astro_ws_pas/results')

        # READ PARAMETERS
        self.bag_path = self.get_parameter('bag_path').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.gazebo_pose_topic = self.get_parameter('gazebo_pose_topic').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.optitrack_csv = self.get_parameter('optitrack_csv').value
        self.rate = float(self.get_parameter('rate').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('========================================')
        self.get_logger().info('ROSbag + OptiTrack replay (TwistStamped)')
        self.get_logger().info(f'Bag              : {self.bag_path}')
        self.get_logger().info(f'Input Topic      : {self.input_topic}')
        self.get_logger().info(f'Output Topic     : {self.output_topic}')
        self.get_logger().info(f'Gazebo pose topic: {self.gazebo_pose_topic}')
        self.get_logger().info(f'Robot frame id   : {self.robot_frame_id}')
        self.get_logger().info(f'Results          : {self.output_dir}')
        self.get_logger().info('========================================')

        # ========================================================
        # PUBLISHER & LOGS
        # ========================================================
        self.cmd_pub = self.create_publisher(TwistStamped, self.output_topic, 10)

        self.last_cmd = TwistStamped()
        self.last_cmd.header.frame_id = self.robot_frame_id

        # Zapisi brzina
        self.bag_cmd_log = []   # Ulazni /cmd_vel iz baga
        self.sent_cmd_log = []  # Brzine poslane na kontroler

        # Timer za održavanje naredbe (50 Hz hold)
        self.hold_timer = self.create_timer(0.02, self.hold_last_cmd)

        # ========================================================
        # GAZEBO SUBSCRIBER
        # ========================================================
        self.gazebo_pose_sub = self.create_subscription(
            TFMessage,
            self.gazebo_pose_topic,
            self.gazebo_pose_callback,
            50
        )

        # Simulacijska odometrija i joint states
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

        self.first_sim_odom_time = None
        self.first_sim_joint_time = None

        self.sim_pose_data = []
        self.sim_odom_data = []
        self.sim_joint_states_data = []
        self.first_sim_pose_time = None
        self._prev_pose_sample = None
        self._warned_missing_frame = False
        self._detected_frame_id = None

        self.optitrack_data = []
        self.messages = []
        self.bag_odom_data = []
        self.bag_joint_states_data = []

        self.current_index = 0
        self.first_bag_time = None
        self.playback_start_ns = None
        self.is_playing = False
        self.finished = False
        self.timer = None
        self.finish_timer = None

        self.load_bag()

        if not self.messages:
            self.get_logger().error('Nema pronađenih cmd_vel poruka u bagu.')
            return

        self.get_logger().info(f'Učitano {len(self.messages)} cmd_vel poruka iz baga.')
        self.timer = self.create_timer(0.001, self.playback_loop)
        self.get_logger().info('Replay node spreman.')

    def hold_last_cmd(self):
        if self.is_playing and not self.finished:
            now_msg = self.get_clock().now().to_msg()
            self.last_cmd.header.stamp = now_msg
            self.cmd_pub.publish(self.last_cmd)

            if self.first_sim_pose_time is not None:
                rel_t = (self.get_clock().now().nanoseconds / 1e9) - self.first_sim_pose_time
            else:
                rel_t = (self.get_clock().now().nanoseconds - self.playback_start_ns) / 1e9

            self.sent_cmd_log.append({
                'time': rel_t,
                'linear_x': self.last_cmd.twist.linear.x,
                'angular_z': self.last_cmd.twist.angular.z
            })

    def gazebo_pose_callback(self, msg: TFMessage):
        if not self.is_playing or self.finished:
            return

        match = None
        seen_frames = []

        # Lista potencijalnih naziva frame-ova iz Gazeba
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

            if self._detected_frame_id and cid == self._detected_frame_id:
                match = transform_stamped
                break
            elif not self._detected_frame_id:
                for cand in candidate_frames:
                    if cid == cand or cid.endswith('/' + cand) or cand in cid:
                        match = transform_stamped
                        self._detected_frame_id = cid
                        self.get_logger().info(f'Uspješno prepoznat i odabran robot pose frame: "{cid}"')
                        break
                if match:
                    break

        if match is None:
            if not self._warned_missing_frame:
                self.get_logger().warn(
                    f'Traženi frame "{self.robot_frame_id}" nije pronađen u {self.gazebo_pose_topic}. '
                    f'Dostupni frameovi: {sorted(set(seen_frames))}'
                )
                self._warned_missing_frame = True
            return

        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        if self.first_sim_pose_time is None:
            self.first_sim_pose_time = timestamp

        relative_time = timestamp - self.first_sim_pose_time

        translation = match.transform.translation
        rotation = match.transform.rotation

        x = translation.x
        y = translation.y
        yaw = self.quaternion_to_yaw(rotation)

        linear_velocity = 0.0
        angular_velocity = 0.0

        if self._prev_pose_sample is not None:
            prev_time, prev_x, prev_y, prev_yaw = self._prev_pose_sample
            dt = relative_time - prev_time
            if dt > 1e-6:
                dx = x - prev_x
                dy = y - prev_y
                linear_velocity = math.sqrt(dx*dx + dy*dy) / dt
                dyaw = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
                angular_velocity = dyaw / dt

        self._prev_pose_sample = (relative_time, x, y, yaw)

        self.sim_pose_data.append({
            'time': relative_time,
            'x': x,
            'y': y,
            'yaw': yaw,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity
        })

    def sim_odom_callback(self, msg: Odometry):
        if not self.is_playing or self.finished:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        if self.first_sim_odom_time is None:
            self.first_sim_odom_time = timestamp

        relative_time = timestamp - self.first_sim_odom_time

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        self.sim_odom_data.append({
            'time': relative_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        })

    def sim_joint_states_callback(self, msg: JointState):
        if not self.is_playing or self.finished:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        if self.first_sim_joint_time is None:
            self.first_sim_joint_time = timestamp

        relative_time = timestamp - self.first_sim_joint_time

        velocities = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                velocities[name] = msg.velocity[i]

        self.sim_joint_states_data.append({
            'time': relative_time,
            'velocities': velocities
        })

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def load_optitrack_csv(self):
        self.optitrack_data = []
        if not os.path.exists(self.optitrack_csv):
            self.get_logger().error(f'OptiTrack CSV ne postoji: {self.optitrack_csv}')
            return False

        try:
            with open(self.optitrack_csv, 'r', newline='', encoding='utf-8-sig') as csvfile:
                sample = csvfile.read(4096)
                csvfile.seek(0)
                try:
                    dialect = csv.Sniffer().sniff(sample, delimiters=',;\t')
                    delimiter = dialect.delimiter
                except csv.Error:
                    delimiter = ','
                reader = csv.reader(csvfile, delimiter=delimiter)

                for row_number, row in enumerate(reader, start=1):
                    if row_number < 8 or len(row) < 5:
                        continue
                    try:
                        time_value = float(row[1].strip())
                        x_value = float(row[2].strip()) / 1000.0
                        z_value = float(row[4].strip()) / 1000.0
                    except ValueError:
                        continue
                    self.optitrack_data.append({
                        'time': time_value,
                        'x': x_value,
                        'z': z_value
                    })
        except Exception as exc:
            self.get_logger().error(f'Greška pri učitavanju OptiTrack CSV-a: {exc}')
            return False

        if not self.optitrack_data:
            return False

        t0 = self.optitrack_data[0]['time']
        x0 = self.optitrack_data[0]['x']
        z0 = self.optitrack_data[0]['z']
        for data in self.optitrack_data:
            data['time'] -= t0
            data['x'] -= x0
            data['z'] -= z0

        return True

    def load_bag(self):
        if not os.path.exists(self.bag_path):
            self.get_logger().error(f'Bag ne postoji: {self.bag_path}')
            return

        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as exc:
            self.get_logger().error(f'Greška pri otvaranju baga: {exc}')
            return

        topics = reader.get_all_topics_and_types()
        topic_types = {topic.name: topic.type for topic in topics}

        required_topics = [self.input_topic, '/odom', '/joint_states']
        for required_topic in required_topics:
            if required_topic not in topic_types:
                self.get_logger().warn(f'Topic {required_topic} ne postoji u bagu.')

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()

            if topic_name == self.input_topic:
                try:
                    msg = deserialize_message(data, Twist)
                except Exception:
                    continue
                self.messages.append((timestamp, msg))

            elif topic_name == '/odom':
                try:
                    msg = deserialize_message(data, Odometry)
                except Exception:
                    continue
                self.bag_odom_data.append((timestamp, msg))

            elif topic_name == '/joint_states':
                try:
                    msg = deserialize_message(data, JointState)
                except Exception:
                    continue
                self.bag_joint_states_data.append((timestamp, msg))

        self.messages.sort(key=lambda item: item[0])
        self.bag_odom_data.sort(key=lambda item: item[0])
        self.bag_joint_states_data.sort(key=lambda item: item[0])

        self.get_logger().info(f'Učitano /odom poruka iz baga: {len(self.bag_odom_data)}')
        self.get_logger().info(f'Učitano /joint_states poruka iz baga: {len(self.bag_joint_states_data)}')

    def playback_loop(self):
        if not self.messages:
            return

        if not self.is_playing:
            self.is_playing = True
            self.current_index = 0
            self.first_bag_time = self.messages[0][0]
            self.playback_start_ns = self.get_clock().now().nanoseconds

        if self.current_index >= len(self.messages):
            if not self.finished:
                self.finished = True
                self.get_logger().info('Bag playback završen.')
                self.stop_robot()
                if self.timer is not None:
                    self.timer.cancel()
                self.finish_timer = self.create_timer(2.0, self.finish_and_save_once)
            return

        elapsed = (self.get_clock().now().nanoseconds - self.playback_start_ns) / 1e9
        elapsed *= self.rate

        while self.current_index < len(self.messages):
            timestamp, msg = self.messages[self.current_index]
            message_time = (timestamp - self.first_bag_time) / 1e9
            if message_time > elapsed:
                break

            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = self.get_clock().now().to_msg()
            stamped_msg.header.frame_id = self.robot_frame_id
            stamped_msg.twist = msg

            self.last_cmd = stamped_msg
            self.cmd_pub.publish(stamped_msg)

            self.bag_cmd_log.append({
                'time': message_time,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })

            self.current_index += 1

    def stop_robot(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = self.robot_frame_id
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.last_cmd = stop_msg
        for _ in range(10):
            self.cmd_pub.publish(stop_msg)

    def finish_and_save_once(self):
        if self.finish_timer is not None:
            self.finish_timer.cancel()
            self.finish_timer = None

        self.load_optitrack_csv()
        self.save_results()
        self.get_logger().info('Rezultati i grafovi spremljeni.')
        rclpy.shutdown()

    def save_sim_csv(self):
        csv_path = os.path.join(self.output_dir, 'sim_pose.csv')
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])
            for data in self.sim_pose_data:
                writer.writerow([data['time'], data['x'], data['y'], data['yaw'],
                                 data['linear_velocity'], data['angular_velocity']])
        return csv_path

    def calculate_path_length(self, x, y):
        if len(x) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            length += math.sqrt(dx*dx + dy*dy)
        return length

    def get_normalized_sim_xy(self):
        if not self.sim_pose_data:
            return [], []
        x0 = self.sim_pose_data[0]['x']
        y0 = self.sim_pose_data[0]['y']
        yaw0 = self.sim_pose_data[0]['yaw']
        x = [data['x'] - x0 for data in self.sim_pose_data]
        y = [data['y'] - y0 for data in self.sim_pose_data]
        
        cos_a = math.cos(-yaw0)
        sin_a = math.sin(-yaw0)
        rx = [xi * cos_a - yi * sin_a for xi, yi in zip(x, y)]
        ry = [xi * sin_a + yi * cos_a for xi, yi in zip(x, y)]
        return rx, ry

    def _normalize_xy_yaw(self, x_list, y_list, yaw_list):
        """Pomakni trajektoriju (x, y, yaw) da počinje u ishodištu s yaw=0,
        na isti način kao get_normalized_sim_xy, kako bi se odometrijske
        trajektorije mogle prikazati u istom referentnom okviru."""
        if not x_list:
            return [], []
        x0 = x_list[0]
        y0 = y_list[0]
        yaw0 = yaw_list[0]
        cos_a = math.cos(-yaw0)
        sin_a = math.sin(-yaw0)
        rx, ry = [], []
        for xi, yi in zip(x_list, y_list):
            dx = xi - x0
            dy = yi - y0
            rx.append(dx * cos_a - dy * sin_a)
            ry.append(dx * sin_a + dy * cos_a)
        return rx, ry

    # ============================================================
    # GRAFOVI
    # ============================================================

    def plot_cmd_vel_comparison(self):
        """Graf usporedbe: Ulazni bag topic (/cmd_vel) vs Topic za kontroler"""
        if not self.bag_cmd_log and not self.sent_cmd_log:
            self.get_logger().warn('Nema cmd_vel podataka za graf.')
            return None

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # Linearna brzina
        if self.bag_cmd_log:
            t_bag = [d['time'] for d in self.bag_cmd_log]
            vx_bag = [d['linear_x'] for d in self.bag_cmd_log]
            ax1.plot(t_bag, vx_bag, label='Ulazni bag topic (/cmd_vel)', color='blue', linewidth=2)

        if self.sent_cmd_log:
            t_sent = [d['time'] for d in self.sent_cmd_log]
            vx_sent = [d['linear_x'] for d in self.sent_cmd_log]
            ax1.plot(t_sent, vx_sent, label='Topic za kontroler (/diff_drive_base_controller/cmd_vel)', 
                     color='red', linestyle='--', alpha=0.8, linewidth=1.5)

        ax1.set_ylabel('Linearna brzina v_x [m/s]')
        ax1.set_title('Usporedba ulaznog cmd_vel i signala prema kontroleru')
        ax1.grid(True)
        ax1.legend()

        # Kutna brzina
        if self.bag_cmd_log:
            wz_bag = [d['angular_z'] for d in self.bag_cmd_log]
            ax2.plot(t_bag, wz_bag, label='Ulazni bag topic (/cmd_vel)', color='blue', linewidth=2)

        if self.sent_cmd_log:
            wz_sent = [d['angular_z'] for d in self.sent_cmd_log]
            ax2.plot(t_sent, wz_sent, label='Topic za kontroler (/diff_drive_base_controller/cmd_vel)', 
                     color='red', linestyle='--', alpha=0.8, linewidth=1.5)

        ax2.set_xlabel('Vrijeme [s]')
        ax2.set_ylabel('Kutna brzina w_z [rad/s]')
        ax2.grid(True)
        ax2.legend()

        fig.tight_layout()
        path = os.path.join(self.output_dir, 'cmd_vel_input_vs_controller.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        self.get_logger().info(f'Usporedni cmd_vel graf spremljen: {path}')
        return path

    def plot_simulation_xy(self):
        if not self.sim_pose_data:
            self.get_logger().warn('Nema simulacijskih podataka za X-Y trajektoriju.')
            return None
        x, y = self.get_normalized_sim_xy()
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(x, y, linewidth=2, color='green', label='Simulacija (Gazebo pose)')
        ax.scatter(0.0, 0.0, color='blue', s=100, label='Start (0,0)', zorder=5)
        ax.scatter(x[-1], y[-1], color='red', s=100, label='Kraj', zorder=5)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('ASTRO - Simulacijska putanja (Gazebo ground-truth pose)')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        path = os.path.join(self.output_dir, 'sim_trajectory_xy.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        return path

    def compute_sim_vs_optitrack_rmse(self, sim_t, sim_x_graf, sim_y_graf):
        """Računa pozicijski RMSE između simulacijske (Gazebo ground-truth)
        i OptiTrack trajektorije. Simulacijske x/y (već u istom koordinatnom
        sustavu kao graf, tj. nakon zamjene osi) interpoliraju se linearno
        na OptiTrack vremenske oznake unutar zajedničkog vremenskog
        preklapanja. Vraća (rmse, n_uzoraka) ili (None, 0) ako preklapanje
        ne postoji."""
        if not self.optitrack_data or len(sim_t) < 2:
            return None, 0

        real_t = np.array([data['time'] for data in self.optitrack_data])
        real_x = np.array([data['x'] for data in self.optitrack_data])
        real_y = np.array([data['z'] for data in self.optitrack_data])

        sim_t = np.array(sim_t)
        sim_x_graf = np.array(sim_x_graf)
        sim_y_graf = np.array(sim_y_graf)

        t_min = max(sim_t[0], real_t[0])
        t_max = min(sim_t[-1], real_t[-1])
        if t_max <= t_min:
            self.get_logger().warn(
                'RMSE: nema vremenskog preklapanja između simulacije i OptiTracka.')
            return None, 0

        mask = (real_t >= t_min) & (real_t <= t_max)
        eval_t = real_t[mask]
        if len(eval_t) < 5:
            self.get_logger().warn('RMSE: premalo OptiTrack uzoraka u preklapanju.')
            return None, 0

        sim_x_i = np.interp(eval_t, sim_t, sim_x_graf)
        sim_y_i = np.interp(eval_t, sim_t, sim_y_graf)
        real_x_i = real_x[mask]
        real_y_i = real_y[mask]

        pos_err = np.sqrt((sim_x_i - real_x_i) ** 2 + (sim_y_i - real_y_i) ** 2)
        rmse = float(np.sqrt(np.mean(pos_err ** 2)))
        return rmse, len(eval_t)

    def plot_sim_vs_optitrack_trajectory(self):
        """Jedan graf sa 4 trajektorije: OptiTrack (stvarni ground-truth),
        simulacija (Gazebo pose ground-truth), odometrija iz baga (/odom)
        i odometrija iz simulacije (/diff_drive_base_controller/odom)."""
        if not self.sim_pose_data or not self.optitrack_data:
            return None

        sim_x, sim_y = self.get_normalized_sim_xy()
        sim_x_graf = [-y for y in sim_y]
        sim_y_graf = [x for x in sim_x]
        sim_t = [data['time'] for data in self.sim_pose_data]

        real_x = [data['x'] for data in self.optitrack_data]
        real_y = [data['z'] for data in self.optitrack_data]

        # RMSE između simulacije (Gazebo ground-truth) i OptiTracka
        rmse, n_rmse = self.compute_sim_vs_optitrack_rmse(sim_t, sim_x_graf, sim_y_graf)
        if rmse is not None:
            self.get_logger().info(
                f'RMSE (simulacija vs OptiTrack): {rmse:.4f} m  (n={n_rmse} uzoraka)')
        else:
            self.get_logger().warn('RMSE (simulacija vs OptiTrack): nije moguće izračunati.')

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(real_x, real_y, linewidth=2, label='OptiTrack', color='orange')
        ax.plot(sim_x_graf, sim_y_graf, linewidth=2, linestyle='--', label='Simulacija (Gazebo pose)', color='green')

        # Odometrijska trajektorija iz baga (/odom, stvarni sustav)
        if self.bag_odom_data:
            bag_x_raw = [msg.pose.pose.position.x for _, msg in self.bag_odom_data]
            bag_y_raw = [msg.pose.pose.position.y for _, msg in self.bag_odom_data]
            bag_yaw_raw = [self.quaternion_to_yaw(msg.pose.pose.orientation) for _, msg in self.bag_odom_data]
            bag_x_n, bag_y_n = self._normalize_xy_yaw(bag_x_raw, bag_y_raw, bag_yaw_raw)
            bag_odom_x_graf = [y for y in bag_y_n]
            bag_odom_y_graf = [x for x in bag_x_n]
            ax.plot(bag_odom_x_graf, bag_odom_y_graf, linewidth=1.8, linestyle='-.',
                    label='Odometrija iz baga (/odom)', color='blue')

        # Odometrijska trajektorija iz simulacije (/diff_drive_base_controller/odom)
        if self.sim_odom_data:
            sim_odom_x_raw = [d['x'] for d in self.sim_odom_data]
            sim_odom_y_raw = [d['y'] for d in self.sim_odom_data]
            sim_odom_yaw_raw = [d['yaw'] for d in self.sim_odom_data]
            sim_odom_x_n, sim_odom_y_n = self._normalize_xy_yaw(sim_odom_x_raw, sim_odom_y_raw, sim_odom_yaw_raw)
            sim_odom_x_graf = [y for y in sim_odom_y_n]
            sim_odom_y_graf = [x for x in sim_odom_x_n]
            ax.plot(sim_odom_x_graf, sim_odom_y_graf, linewidth=1.8, linestyle=':',
                    label='Odometrija iz simulacije (/diff_drive_base_controller/odom)', color='red')

        ax.scatter(0.0, 0.0, s=120, color='black', label='Start (0,0)', zorder=5)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        title = 'ASTRO - Usporedba trajektorija: OptiTrack, simulacija i odometrija'
        if rmse is not None:
            title += f'\nRMSE (sim vs OptiTrack) = {rmse:.4f} m'
        ax.set_title(title)
        ax.axis('equal')
        ax.grid(True)
        ax.legend()

        if rmse is not None:
            ax.text(
                0.02, 0.02, f'RMSE = {rmse:.4f} m\n(n={n_rmse} uzoraka)',
                transform=ax.transAxes, fontsize=11, va='bottom', ha='left',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        fig.tight_layout()
        path = os.path.join(self.output_dir, 'sim_vs_optitrack_trajectory.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        self.get_logger().info(f'Graf usporedbe trajektorija spremljen: {path}')
        return path

    def _find_wheel_joint_name(self, names, side):
        exact = f'{side}_wheel_joint'
        if exact in names:
            return exact

        candidates = [
            name for name in names
            if side in name.lower() and 'wheel' in name.lower()
        ]
        return candidates[0] if candidates else None

    def plot_joint_positions_bag_vs_simulation(self):
        if not self.bag_joint_states_data or not self.sim_joint_states_data:
            self.get_logger().warn('Nema dovoljno /joint_states podataka za usporedbu.')
            return None

        bag_t0 = self.bag_joint_states_data[0][0]
        bag_time = [(t - bag_t0) / 1e9 for t, _ in self.bag_joint_states_data]

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

        bag_left = self._find_wheel_joint_name(bag_names, 'left')
        bag_right = self._find_wheel_joint_name(bag_names, 'right')
        sim_left = self._find_wheel_joint_name(sim_names, 'left')
        sim_right = self._find_wheel_joint_name(sim_names, 'right')

        if not bag_left or not bag_right or not sim_left or not sim_right:
            self.get_logger().warn(
                f'Nisu pronađeni potrebni wheel jointovi. Bag: {bag_names}; Sim: {sim_names}'
            )
            return None

        def extract_bag_position(data, joint_name):
            values_t = []
            values = []
            for timestamp, msg in data:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.velocity):
                        values_t.append((timestamp - bag_t0) / 1e9)
                        values.append(msg.velocity[idx])
            return values_t, values

        left_bag_t, left_bag = extract_bag_position(self.bag_joint_states_data, bag_left)
        right_bag_t, right_bag = extract_bag_position(self.bag_joint_states_data, bag_right)

        sim_time = [d['time'] for d in self.sim_joint_states_data]
        sim_left = [d['velocities'].get(sim_left, float('nan')) for d in self.sim_joint_states_data]
        sim_right = [d['velocities'].get(sim_right, float('nan')) for d in self.sim_joint_states_data]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

        ax1.plot(left_bag_t, left_bag, label=f'Bag /joint_states ({bag_left})', linewidth=1.8)
        ax1.plot(sim_time, sim_left, label=f'Simulacija /joint_states ({self._find_wheel_joint_name(sim_names, "left")})', linestyle='--', linewidth=1.6)
        ax1.set_ylabel('Kut [rad]')
        ax1.set_title('Usporedba pozicije lijevog kotača')
        ax1.grid(True)
        ax1.legend()

        ax2.plot(right_bag_t, right_bag, label=f'Bag /joint_states ({bag_right})', linewidth=1.8)
        ax2.plot(sim_time, sim_right, label=f'Simulacija /joint_states ({self._find_wheel_joint_name(sim_names, "right")})', linestyle='--', linewidth=1.6)
        ax2.set_xlabel('Vrijeme [s]')
        ax2.set_ylabel('Kut [rad]')
        ax2.set_title('Usporedba pozicije desnog kotača')
        ax2.grid(True)
        ax2.legend()

        fig.tight_layout()
        path = os.path.join(self.output_dir, 'joint_positions_bag_vs_simulation.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        self.get_logger().info(f'Usporedni joint position graf spremljen: {path}')
        return path

    def save_results(self):
        if self.sim_pose_data:
            self.save_sim_csv()
            self.plot_simulation_xy()

        # Generiraj graf usporedbe ulaznog i izlaznog cmd_vel
        self.plot_cmd_vel_comparison()

        if self.sim_pose_data and self.optitrack_data:
            self.plot_sim_vs_optitrack_trajectory()


        if self.bag_joint_states_data and self.sim_joint_states_data:
            self.plot_joint_positions_bag_vs_simulation()


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
