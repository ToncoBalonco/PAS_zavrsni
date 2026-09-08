import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # PATHS
    # ============================================================

    pkg_astro = get_package_share_directory('astro')

    # Existing Gazebo launch file
    gazebo_launch_file = os.path.join(
        pkg_astro,
        'launch',
        'sim_gazebo.launch.py'
    )

    # RViz configuration
    rviz_config_file = os.path.join(
        pkg_astro,
        'config',
        'astro_sim_config.rviz'
    )

    # ============================================================
    # 1. GAZEBO SIMULATION
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # ============================================================
    # 2. RVIZ2
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ============================================================
    # 3. FIGURE-8 PATH PUBLISHER
    # ============================================================

    publisher = Node(
        package='astro',
        executable='figure8_path_publisher.py',
        name='figure8_path_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # ============================================================
    # 4. FIGURE-8 PATH FOLLOWER
    # ============================================================

    follower = Node(
        package='astro',
        executable='figure8_path_follower.py',
        name='figure8_path_follower',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # ============================================================
    # STARTUP SEQUENCE
    # ============================================================

    # Gazebo + RViz start immediately.
    #
    # Publisher starts after 8 seconds.
    #
    # Follower starts after 10 seconds.
    #
    # Therefore:
    #
    #   t = 0 s    -> Gazebo
    #              -> RViz2
    #
    #   t = 8 s    -> Publisher
    #
    #   t = 10 s   -> Follower
    #
    # ============================================================

    publisher_delayed = TimerAction(
        period=8.0,
        actions=[publisher]
    )

    follower_delayed = TimerAction(
        period=10.0,
        actions=[follower]
    )

    return LaunchDescription([
        gazebo,
        rviz,
        publisher_delayed,
        follower_delayed
    ])


