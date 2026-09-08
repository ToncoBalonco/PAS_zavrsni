#!/usr/bin/env python3

import sys
import rclpy
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import matplotlib.pyplot as plt


def read_odom_from_bag(bag_path):

    storage_options = StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )

    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(
        storage_options,
        converter_options
    )

    topics = reader.get_all_topics_and_types()

    topic_types = {
        topic.name: topic.type
        for topic in topics
    }

    if '/odom' not in topic_types:
        print("Greška: /odom nije pronađen u ROSbagu.")
        return None

    odom_msg_type = get_message(
        topic_types['/odom']
    )

    data = {
        't': [],
        'x': [],
        'y': [],
        'z': []
    }

    while reader.has_next():

        topic, raw_data, timestamp = reader.read_next()

        if topic != '/odom':
            continue

        msg = deserialize_message(
            raw_data,
            odom_msg_type
        )

        t = timestamp * 1e-9

        data['t'].append(t)

        data['x'].append(
            msg.pose.pose.position.x
        )

        data['y'].append(
            msg.pose.pose.position.y
        )

        data['z'].append(
            msg.pose.pose.position.z
        )

    return data


def calculate_path_length(x, y):

    length = 0.0

    for i in range(1, len(x)):

        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]

        length += math.sqrt(
            dx * dx +
            dy * dy
        )

    return length


def plot_odom(data):

    if data is None or not data['t']:
        print("Nema /odom podataka.")
        return

    x = data['x']
    y = data['y']

    # =========================================================
    # TRANSLACIJA POČETNE TOČKE U (0, 0)
    # =========================================================

    x0 = x[0]
    y0 = y[0]

    x_relative = [
        value - x0
        for value in x
    ]

    y_relative = [
        value - y0
        for value in y
    ]

    # =========================================================
    # STATISTIKA
    # =========================================================

    path_length = calculate_path_length(
        x_relative,
        y_relative
    )

    duration = (
        data['t'][-1] -
        data['t'][0]
    )

    # =========================================================
    # GRAF
    # =========================================================

    plt.figure(
        figsize=(10, 8)
    )

    plt.plot(
        x_relative,
        y_relative,
        linewidth=2,
        label='Odometrija'
    )

    # Početna točka
    plt.scatter(
        0,
        0,
        s=100,
        label='Start (0, 0)',
        zorder=5
    )

    # Završna točka
    plt.scatter(
        x_relative[-1],
        y_relative[-1],
        s=100,
        label='End',
        zorder=5
    )

    plt.xlabel(
        'X [m]'
    )

    plt.ylabel(
        'Y [m]'
    )

    plt.title(
        'ASTRO - Odometrijska putanja'
    )

    plt.axis(
        'equal'
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()

    # =========================================================
    # ISPIS
    # =========================================================

    print()
    print("========================================")
    print("        ODOMETRIJSKA ANALIZA")
    print("========================================")

    print()
    print(f"Broj /odom poruka: {len(data['t'])}")

    print(
        f"Trajanje:          {duration:.3f} s"
    )

    print(
        f"Ukupni prijeđeni put: {path_length:.3f} m"
    )

    print()
    print(
        f"Početna točka:     x=0.000, y=0.000"
    )

    print(
        f"Završna točka:     "
        f"x={x_relative[-1]:.3f}, "
        f"y={y_relative[-1]:.3f}"
    )

    print()
    print("========================================")


def main():

    if len(sys.argv) != 2:

        print(
            "Upotreba:"
        )

        print(
            "python3 plot_bag.py <rosbag_folder>"
        )

        sys.exit(1)

    bag_path = sys.argv[1]

    rclpy.init()

    try:

        data = read_odom_from_bag(
            bag_path
        )

    finally:

        rclpy.shutdown()

    plot_odom(
        data
    )

    plt.show()


if __name__ == '__main__':

    main()
