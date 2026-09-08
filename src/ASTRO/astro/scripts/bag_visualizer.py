#!/usr/bin/env python3

import sys
import rclpy
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_all_topics_from_bag(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    topic_types = {t.name: t.type for t in topics_and_types}

    target_topics = [
        '/absoulte_current',
        '/cmd_vel',
        '/cmd_vel_joy',
        '/joint_states',
        '/odom'
    ]

    msg_types = {}
    for topic in target_topics:
        if topic in topic_types:
            msg_types[topic] = get_message(topic_types[topic])
        else:
            print(f"Upozorenje: Topic '{topic}' nije pronađen u ROSbagu.")

    data = {
        '/absoulte_current': {'t': [], 'current': []},
        '/cmd_vel': {'t': [], 'lin_x': [], 'ang_z': []},
        '/cmd_vel_joy': {'t': [], 'lin_x': [], 'ang_z': []},
        '/joint_states': {'t': [], 'position': {}, 'velocity': {}},
        '/odom': {'t': [], 'x': [], 'y': [], 'lin_x': [], 'ang_z': []}
    }

    t0 = None

    while reader.has_next():
        topic, raw_data, timestamp = reader.read_next()

        if topic not in msg_types:
            continue

        t_sec = timestamp * 1e-9
        if t0 is None:
            t0 = t_sec

        t_rel = t_sec - t0
        msg = deserialize_message(raw_data, msg_types[topic])

        if topic == '/absoulte_current':
            data[topic]['t'].append(t_rel)
            data[topic]['current'].append(msg.data)

        elif topic in ['/cmd_vel', '/cmd_vel_joy']:
            data[topic]['t'].append(t_rel)
            data[topic]['lin_x'].append(msg.linear.x)
            data[topic]['ang_z'].append(msg.angular.z)

        elif topic == '/joint_states':
            data[topic]['t'].append(t_rel)
            for i, name in enumerate(msg.name):
                if name not in data[topic]['position']:
                    data[topic]['position'][name] = []
                    data[topic]['velocity'][name] = []

                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0

                data[topic]['position'][name].append(pos)
                data[topic]['velocity'][name].append(vel)

        elif topic == '/odom':
            data[topic]['t'].append(t_rel)
            data[topic]['x'].append(msg.pose.pose.position.x)
            data[topic]['y'].append(msg.pose.pose.position.y)
            data[topic]['lin_x'].append(msg.twist.twist.linear.x)
            data[topic]['ang_z'].append(msg.twist.twist.angular.z)

    return data


def plot_all_data(data):
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 2)

    # 1. Struja motora (/absoulte_current)
    ax1 = fig.add_subplot(gs[0, 0])
    if data['/absoulte_current']['t']:
        ax1.plot(data['/absoulte_current']['t'], data['/absoulte_current']['current'], color='r', label='Struja')
        ax1.set_ylabel('Struja [A]')
        ax1.set_title('/absoulte_current')
        ax1.grid(True)
        ax1.legend()

    # 2. Usporedba naredbi brzine (/cmd_vel vs /cmd_vel_joy)
    ax2 = fig.add_subplot(gs[0, 1])
    if data['/cmd_vel']['t']:
        ax2.plot(data['/cmd_vel']['t'], data['/cmd_vel']['lin_x'], 'b-', label='cmd_vel linear.x')
        ax2.plot(data['/cmd_vel']['t'], data['/cmd_vel']['ang_z'], 'b--', label='cmd_vel angular.z')
    if data['/cmd_vel_joy']['t']:
        ax2.plot(data['/cmd_vel_joy']['t'], data['/cmd_vel_joy']['lin_x'], 'g:', label='cmd_vel_joy linear.x')
        ax2.plot(data['/cmd_vel_joy']['t'], data['/cmd_vel_joy']['ang_z'], 'g-.', label='cmd_vel_joy angular.z')
    ax2.set_ylabel('Brzina [m/s, rad/s]')
    ax2.set_title('/cmd_vel vs /cmd_vel_joy')
    ax2.grid(True)
    ax2.legend()

    # 3. Brzine zglobova (/joint_states)
    ax3 = fig.add_subplot(gs[1, 0])
    if data['/joint_states']['velocity']:
        for joint_name, vels in data['/joint_states']['velocity'].items():
            ax3.plot(data['/joint_states']['t'], vels, label=f'{joint_name} vel')
        ax3.set_ylabel('Brzina [rad/s]')
        ax3.set_title('/joint_states (Brzina zglobova)')
        ax3.grid(True)
        ax3.legend()

    # 4. Procijenjene brzine iz odometrije (/odom twist)
    ax4 = fig.add_subplot(gs[1, 1])
    if data['/odom']['t']:
        ax4.plot(data['/odom']['t'], data['/odom']['lin_x'], color='magenta', label='Odom lin_x')
        ax4.plot(data['/odom']['t'], data['/odom']['ang_z'], color='cyan', label='Odom ang_z')
        ax4.set_ylabel('Brzina [m/s, rad/s]')
        ax4.set_title('/odom (Brzina robota)')
        ax4.grid(True)
        ax4.legend()

    # 5. 2D Putanja robota iz odometrije (X vs Y)
    ax5 = fig.add_subplot(gs[2, :])
    if data['/odom']['x']:
        ax5.plot(data['/odom']['x'], data['/odom']['y'], 'k-', linewidth=2, label='Putanja')
        ax5.scatter(data['/odom']['x'][0], data['/odom']['y'][0], color='green', s=80, label='Start', zorder=5)
        ax5.scatter(data['/odom']['x'][-1], data['/odom']['y'][-1], color='red', s=80, label='Kraj', zorder=5)
        ax5.set_xlabel('X [m]')
        ax5.set_ylabel('Y [m]')
        ax5.set_title('/odom 2D Trajektorija')
        ax5.axis('equal')
        ax5.grid(True)
        ax5.legend()

    plt.tight_layout()


def main():
    if len(sys.argv) != 2:
        print("Upotreba: python3 plot_all_bag.py <rosbag_folder>")
        sys.exit(1)

    bag_path = sys.argv[1]
    rclpy.init()

    try:
        data = read_all_topics_from_bag(bag_path)
    finally:
        rclpy.shutdown()

    plot_all_data(data)
    plt.show()


if __name__ == '__main__':
    main()
