#!/usr/bin/env python3
#
# ================================================================
# NAPOMENA - GAZEBO FORTRESS BRIDGE
# ... (same header comments, can keep) ...
# ================================================================

import os
import time
import csv
import math

import rclpy
from rclpy.node import Node

from rclpy.serialization import deserialize_message

from geometry_msgs.msg import Twist, TwistStamped
from tf2_msgs.msg import TFMessage

import rosbag2_py

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt


class RosbagCmdVelReplayer(Node):

    def __init__(self):

        super().__init__('rosbag_cmdvel_replayer')

        # ========================================================
        # PARAMETERS
        # ========================================================

        self.declare_parameter(
            'bag_path',
            '/home/tona/astro_ws_pas/rosbag2_2026_08_21-17_43_13'
        )

        self.declare_parameter(
            'input_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'output_topic',
            '/diff_drive_base_controller/cmd_vel'   # <-- changed to stamped topic
        )

        self.declare_parameter(
            'gazebo_pose_topic',
            '/world/empty/dynamic_pose/info'
        )

        self.declare_parameter(
            'robot_frame_id',
            'base_footprint'   # frame_id for TwistStamped header
        )

        self.declare_parameter(
            'optitrack_csv',
            '/home/tona/astro_ws_pas/astro_optitrack_cuk_001.csv'
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

        # ========================================================
        # READ PARAMETERS
        # ========================================================

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

        # ========================================================
        # LOG PARAMETERS
        # ========================================================

        self.get_logger().info('========================================')
        self.get_logger().info('ROSbag + OptiTrack replay (TwistStamped)')
        self.get_logger().info(f'Bag              : {self.bag_path}')
        self.get_logger().info(f'Input            : {self.input_topic}')
        self.get_logger().info(f'Output           : {self.output_topic}')
        self.get_logger().info(f'Gazebo pose topic: {self.gazebo_pose_topic}')
        self.get_logger().info(f'Robot frame id   : {self.robot_frame_id}')
        self.get_logger().info(f'OptiTrack CSV    : {self.optitrack_csv}')
        self.get_logger().info(f'Rate             : {self.rate}')
        self.get_logger().info(f'Loop             : {self.loop}')
        self.get_logger().info(f'Results          : {self.output_dir}')
        self.get_logger().info('========================================')

        # ========================================================
        # CMD_VEL PUBLISHER (TwistStamped)
        # ========================================================

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        self.last_cmd = TwistStamped()          # store as TwistStamped
        self.last_cmd.header.frame_id = self.robot_frame_id
        self.cmd_log = []                       # store (time, linear_x, angular_z)

        # 50 Hz hold timer
        self.hold_timer = self.create_timer(
            0.02,
            self.hold_last_cmd
        )

        # ========================================================
        # GAZEBO GROUND-TRUTH POSE SUBSCRIBER
        # ========================================================

        self.gazebo_pose_sub = self.create_subscription(
            TFMessage,
            self.gazebo_pose_topic,
            self.gazebo_pose_callback,
            50
        )

        # ========================================================
        # GAZEBO POSE STORAGE
        # ========================================================

        self.sim_pose_data = []
        self.first_sim_pose_time = None
        self._prev_pose_sample = None
        self._warned_missing_frame = False

        # ========================================================
        # OPTITRACK DATA
        # ========================================================

        self.optitrack_data = []

        # ========================================================
        # CMD_VEL BAG MESSAGES
        # ========================================================

        self.messages = []   # list of (timestamp_ns, Twist)

        # ========================================================
        # PLAYBACK STATE
        # ========================================================

        self.current_index = 0
        self.first_bag_time = None
        self.playback_start_ns = None   # sim time (not wall time)
        self.is_playing = False
        self.finished = False
        self.timer = None
        self.finish_timer = None

        # ========================================================
        # LOAD BAG
        # ========================================================

        self.load_bag()

        if not self.messages:
            self.get_logger().error('Nema pronađenih /cmd_vel poruka.')
            return

        self.get_logger().info(f'Učitano {len(self.messages)} /cmd_vel poruka.')

        # ========================================================
        # START PLAYBACK TIMER
        # ========================================================

        self.timer = self.create_timer(0.001, self.playback_loop)
        self.get_logger().info('Replay node spreman.')

    def hold_last_cmd(self):
        # Publish the last stamped command (with updated timestamp)
        if self.is_playing and not self.finished:
            # Update stamp to current simulation time
            self.last_cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(self.last_cmd)

    # ============================================================
    # GAZEBO POSE CALLBACK (unchanged)
    # ============================================================

    def gazebo_pose_callback(self, msg: TFMessage):
        if not self.is_playing or self.finished:
            return

        match = None
        seen_frames = []
        for transform_stamped in msg.transforms:
            seen_frames.append(transform_stamped.child_frame_id)
            if transform_stamped.child_frame_id == self.robot_frame_id:
                match = transform_stamped
                break

        if match is None:
            if not self._warned_missing_frame:
                self.get_logger().warn(
                    f'Frame "{self.robot_frame_id}" nije pronađen u {self.gazebo_pose_topic}. '
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

    # ============================================================
    # QUATERNION -> YAW
    # ============================================================

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ============================================================
    # LOAD OPTITRACK CSV (unchanged)
    # ============================================================

    def load_optitrack_csv(self):
        self.optitrack_data = []
        if not os.path.exists(self.optitrack_csv):
            self.get_logger().error(f'OptiTrack CSV ne postoji: {self.optitrack_csv}')
            return False

        self.get_logger().info(f'Učitavam OptiTrack CSV: {self.optitrack_csv}')
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
                    if row_number < 8:
                        continue
                    if len(row) < 5:
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
            self.get_logger().error('OptiTrack CSV nije sadržavao valjane podatke.')
            return False

        # Normalize time
        t0 = self.optitrack_data[0]['time']
        for data in self.optitrack_data:
            data['time'] -= t0

        # Normalize position
        x0 = self.optitrack_data[0]['x']
        z0 = self.optitrack_data[0]['z']
        for data in self.optitrack_data:
            data['x'] -= x0
            data['z'] -= z0

        self.get_logger().info(f'OptiTrack podaci učitani. Broj uzoraka: {len(self.optitrack_data)}')
        return True

    # ============================================================
    # LOAD BAG (unchanged)
    # ============================================================

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
            self.get_logger().error(f'Greška pri otvaranju baya: {exc}')
            return

        topics = reader.get_all_topics_and_types()
        topic_types = {topic.name: topic.type for topic in topics}

        if self.input_topic not in topic_types:
            self.get_logger().error(f'Topic {self.input_topic} ne postoji u bagu.')
            return

        self.get_logger().info(f'Pronađen topic: {self.input_topic} [{topic_types[self.input_topic]}]')

        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            if topic_name == self.input_topic:
                try:
                    msg = deserialize_message(data, Twist)
                except Exception as exc:
                    self.get_logger().error(f'Greška pri deserializaciji /cmd_vel: {exc}')
                    continue
                self.messages.append((timestamp, msg))

        self.messages.sort(key=lambda item: item[0])
        self.get_logger().info(f'/cmd_vel poruka: {len(self.messages)}')

    # ============================================================
    # PLAYBACK LOOP (publishes TwistStamped)
    # ============================================================

    def playback_loop(self):
        if not self.messages:
            return

        if not self.is_playing:
            self.is_playing = True
            self.current_index = 0
            self.first_bag_time = self.messages[0][0]
            self.playback_start_ns = self.get_clock().now().nanoseconds
            self.get_logger().info('Pokrećem /cmd_vel playback (use_sim_time).')

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

            # Create TwistStamped message
            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = self.get_clock().now().to_msg()
            stamped_msg.header.frame_id = self.robot_frame_id
            stamped_msg.twist = msg   # copy the Twist

            self.last_cmd = stamped_msg
            self.cmd_pub.publish(stamped_msg)

            self.cmd_log.append({
                'time': message_time,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })

            self.current_index += 1

    # ============================================================
    # STOP ROBOT (publishes zero TwistStamped)
    # ============================================================

    def stop_robot(self):
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = self.robot_frame_id
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.last_cmd = stop_msg
        for _ in range(10):
            self.cmd_pub.publish(stop_msg)
        self.get_logger().info('Robot zaustavljen.')

    # ============================================================
    # FINISH (unchanged except method names)
    # ============================================================

    def finish_and_save_once(self):
        if self.finish_timer is not None:
            self.finish_timer.cancel()
            self.finish_timer = None

        self.load_optitrack_csv()
        self.save_results()
        self.get_logger().info('Rezultati spremljeni.')
        self.get_logger().info('Rosbag replay završen.')
        rclpy.shutdown()

    # ============================================================
    # SAVE SIMULATION CSV (unchanged)
    # ============================================================

    def save_sim_csv(self):
        csv_path = os.path.join(self.output_dir, 'sim_pose.csv')
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])
            for data in self.sim_pose_data:
                writer.writerow([data['time'], data['x'], data['y'], data['yaw'],
                                 data['linear_velocity'], data['angular_velocity']])
        self.get_logger().info(f'Simulacijski (Gazebo pose) CSV spremljen: {csv_path}')
        return csv_path

    # ============================================================
    # SAVE OPTITRACK CSV (unchanged)
    # ============================================================

    def save_optitrack_csv(self):
        if not self.optitrack_data:
            return None
        csv_path = os.path.join(self.output_dir, 'optitrack_processed.csv')
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'z'])
            for data in self.optitrack_data:
                writer.writerow([data['time'], data['x'], data['z']])
        self.get_logger().info(f'Obrađeni OptiTrack CSV: {csv_path}')
        return csv_path

    # ============================================================
    # PATH LENGTH (unchanged)
    # ============================================================

    def calculate_path_length(self, x, y):
        if len(x) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            length += math.sqrt(dx*dx + dy*dy)
        return length

    # ============================================================
    # NORMALIZE SIM (unchanged)
    # ============================================================

    def get_normalized_sim_xy(self):
        if not self.sim_pose_data:
            return [], []
        x0 = self.sim_pose_data[0]['x']
        y0 = self.sim_pose_data[0]['y']
        yaw0 = self.sim_pose_data[0]['yaw']
        x = [data['x'] - x0 for data in self.sim_pose_data]
        y = [data['y'] - y0 for data in self.sim_pose_data]
        x, y = self.rotate_xy(x, y, -yaw0)
        return x, y

    def rotate_xy(self, x, y, angle):
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotated_x = []
        rotated_y = []
        for xi, yi in zip(x, y):
            rotated_x.append(xi * cos_a - yi * sin_a)
            rotated_y.append(xi * sin_a + yi * cos_a)
        return rotated_x, rotated_y

    # ============================================================
    # PLOTTING FUNCTIONS (unchanged, they use self.cmd_log)
    # ============================================================

    def plot_simulation_xy(self):
        if not self.sim_pose_data:
            self.get_logger().warn('Nema simulacijskih (Gazebo pose) podataka za graf.')
            return None
        x, y = self.get_normalized_sim_xy()
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(x, y, linewidth=2, label='Simulacija (Gazebo pose)')
        ax.scatter(0.0, 0.0, s=100, label='Start (0,0)', zorder=5)
        ax.scatter(x[-1], y[-1], s=100, label='End', zorder=5)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('ASTRO - simulacijska putanja (Gazebo ground-truth pose)')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        path = os.path.join(self.output_dir, 'sim_trajectory_xy.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        self.get_logger().info(f'Simulacijski X-Y graf: {path}')
        return path

    def plot_sim_vs_optitrack_trajectory(self):
        if not self.sim_pose_data or not self.optitrack_data:
            return None

        sim_x, sim_y = self.get_normalized_sim_xy()

        # Original fixed rotation: X_graf = Y_sim, Y_graf = -X_sim
        sim_x_graf = sim_y
        sim_y_graf = [-x for x in sim_x]

        real_x = [data['x'] for data in self.optitrack_data]
        real_y = [data['z'] for data in self.optitrack_data]

        sim_length = self.calculate_path_length(sim_x_graf, sim_y_graf)
        real_length = self.calculate_path_length(real_x, real_y)
        path_length_difference = abs(sim_length - real_length)

        sim_final_x = sim_x_graf[-1]
        sim_final_y = sim_y_graf[-1]
        real_final_x = real_x[-1]
        real_final_y = real_y[-1]
        final_position_error = math.hypot(sim_final_x - real_final_x, sim_final_y - real_final_y)

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(real_x, real_y, linewidth=2, label='OptiTrack')
        ax.plot(sim_x_graf, sim_y_graf, linewidth=2, linestyle='--', label='Simulacija (Gazebo pose)')
        ax.scatter(0.0, 0.0, s=120, label='Start (0,0)', zorder=5)
        ax.scatter(real_final_x, real_final_y, s=100, zorder=5)
        ax.scatter(sim_final_x, sim_final_y, s=100, zorder=5)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('ASTRO - simulacija (Gazebo pose) vs OptiTrack trajektorija')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()

        statistics = (f'OptiTrack put: {real_length:.3f} m\n'
                      f'Simulacijski put: {sim_length:.3f} m\n'
                      f'Razlika puta: {path_length_difference:.3f} m\n'
                      f'Greška završne pozicije: {final_position_error:.3f} m')
        ax.text(0.02, 0.98, statistics, transform=ax.transAxes, verticalalignment='top')

        fig.tight_layout()
        path = os.path.join(self.output_dir, 'sim_vs_optitrack_trajectory.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)

        self.get_logger().info('========================================')
        self.get_logger().info('SIMULACIJA (GAZEBO POSE) VS OPTITRACK TRAJEKTORIJA')
        self.get_logger().info('========================================')
        self.get_logger().info(f'OptiTrack put: {real_length:.3f} m')
        self.get_logger().info(f'Simulacijski put: {sim_length:.3f} m')
        self.get_logger().info(f'Razlika puta: {path_length_difference:.3f} m')
        self.get_logger().info(f'Greška završne pozicije: {final_position_error:.3f} m')
        self.get_logger().info(f'Trajektorija: {path}')
        return path

    def plot_position_vs_time(self):
        if not self.sim_pose_data or not self.optitrack_data:
            return None

        sim_times = [data['time'] for data in self.sim_pose_data]
        sim_x = [data['x'] for data in self.sim_pose_data]
        sim_y = [data['y'] for data in self.sim_pose_data]
        sim_x0 = sim_x[0]; sim_y0 = sim_y[0]
        sim_x = [v - sim_x0 for v in sim_x]
        sim_y = [v - sim_y0 for v in sim_y]

        real_times = [data['time'] for data in self.optitrack_data]
        real_x = [data['x'] for data in self.optitrack_data]
        real_z = [data['z'] for data in self.optitrack_data]

        fig, ax = plt.subplots(figsize=(12, 7))
        ax.plot(sim_times, sim_x, linewidth=2, label='Simulacija (Gazebo pose) X')
        ax.plot(real_times, real_x, linewidth=2, linestyle='--', label='OptiTrack X')
        ax.plot(sim_times, sim_y, linewidth=2, label='Simulacija (Gazebo pose) Y')
        ax.plot(real_times, real_z, linewidth=2, linestyle='--', label='OptiTrack Z')

        ax.set_xlabel('Vrijeme [s]')
        ax.set_ylabel('Pozicija [m]')
        ax.set_title('ASTRO - simulacija (Gazebo pose) vs OptiTrack pozicija kroz vrijeme')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()

        path = os.path.join(self.output_dir, 'simulation_vs_optitrack_position_vs_time.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)

        self.get_logger().info('========================================')
        self.get_logger().info('SIMULACIJA (GAZEBO POSE) VS OPTITRACK')
        self.get_logger().info('========================================')
        self.get_logger().info(f'Simulacijski put: {self.calculate_path_length(sim_x, sim_y):.3f} m')
        self.get_logger().info(f'OptiTrack put: {self.calculate_path_length(real_x, real_z):.3f} m')
        self.get_logger().info(f'Graf: {path}')
        return path

    def plot_cmd_vel(self):
        if not self.cmd_log:
            self.get_logger().warn('Nema cmd_vel loga za graf.')
            return None

        t = [d['time'] for d in self.cmd_log]
        vx = [d['linear_x'] for d in self.cmd_log]

        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(t, vx, linewidth=1.5)
        ax.set_xlabel('Vrijeme [s]')
        ax.set_ylabel('linear.x [m/s]')
        ax.set_title('ASTRO - replayani /cmd_vel iz baga (TwistStamped)')
        ax.grid(True)
        fig.tight_layout()
        path = os.path.join(self.output_dir, 'cmd_vel_from_bag.png')
        fig.savefig(path, dpi=200)
        plt.close(fig)
        self.get_logger().info(f'cmd_vel graf: {path}')
        return path

    # ============================================================
    # SAVE ALL RESULTS
    # ============================================================

    def save_results(self):
        self.get_logger().info('========================================')
        self.get_logger().info('SPREMANJE REZULTATA')
        self.get_logger().info('========================================')

        if self.sim_pose_data:
            self.save_sim_csv()
        else:
            self.get_logger().warn('Nema simulacijskih (Gazebo pose) podataka.')

        if self.optitrack_data:
            self.save_optitrack_csv()
        else:
            self.get_logger().warn('Nema OptiTrack podataka.')

        if self.sim_pose_data:
            self.plot_simulation_xy()

        self.plot_cmd_vel()   # always plot

        if self.sim_pose_data and self.optitrack_data:
            self.plot_sim_vs_optitrack_trajectory()
            self.plot_position_vs_time()
        else:
            self.get_logger().warn('Nije moguće napraviti simulacija-vs-OptiTrack grafove.')

        self.get_logger().info('========================================')


# ================================================================
# MAIN
# ================================================================

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
