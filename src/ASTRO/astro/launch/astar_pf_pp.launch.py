#!/usr/bin/env python3
"""
A* + Potential Field Pure Pursuit - nova struktura
=====================================================
Minimalan launch za novi pristup upravljanju:

    rsp -> map_server -> amcl -> astar_planner_pf.py
        -> potential_field_pure_pursuit.py -> rviz

Namjerno BEZ dynamic_ob_layer.py i cmd_safety.py - lokalno
izbjegavanje prepreka radi isključivo potential_field_pure_pursuit.py
direktno iz /scan. astar_planner_pf.py planira samo globalni put
kroz statičku (inflatanu) kartu, bez replanning monitora.

Stvarni robot (use_sim_time: False).
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # PATHS
    # ============================================================

    map_yaml = (
        "/home/tona/astro_ws_pas/src/ASTRO/"
        "astro_slam/maps/mapa_crte_new.yaml"
    )

    rviz_config = (
        Path(get_package_share_directory("astro"))
        / "rviz"
        / "astar_planer_v1.rviz"
    )

    # ============================================================
    # 1. ROBOT STATE PUBLISHER
    # ============================================================

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("astro"))
                / "launch"
                / "rsp.launch.py"
            )
        )
    )

    # ============================================================
    # 2. MAP SERVER
    # ============================================================

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": False,
            }
        ],
    )

    # ============================================================
    # 3. LIFECYCLE MANAGER
    # ============================================================

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl"
                ],
                "bond_timeout": 60.0,
            }
        ],
    )

    # ============================================================
    # 4. AMCL
    # ============================================================

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,

                "odom_frame_id": "odom",
                "global_frame_id": "map",
                "base_frame_id": "base_footprint",
                "scan_topic": "/scan",

                "laser_model_type": "likelihood_field",
                "laser_max_range": 8.0,
                "laser_max_beams": 80,
                "laser_sigma_hit": 0.2,
                "laser_z_hit": 0.95,
                "laser_z_rand": 0.05,

                "alpha1": 0.2,
                "alpha2": 0.2,
                "alpha3": 0.2,
                "alpha4": 0.2,

                "min_particles": 500,
                "max_particles": 2000,

                "save_pose_rate": 0.5,
                "tf_broadcast": True,
            }
        ],
    )

    # ============================================================
    # 5. A* PLANNER (nova struktura, samo globalni put)
    # ============================================================

    astar = Node(
        package="astro",
        executable="astar_planner_pf.py",
        name="astar_planner_pf",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "goal_topic": "/goal_pose",
                "plan_topic": "/plan",

                "map_frame": "map",
                "base_frame": "base_footprint",

                "robot_radius": 0.43,
                "obstacle_threshold": 50,
            }
        ],
    )

    # ============================================================
    # 6. POTENTIAL FIELD PURE PURSUIT (lokalno praćenje + izbjegavanje)
    # ============================================================

    potential_field_pure_pursuit = Node(
        package="astro",
        executable="pf_pure_pursuit.py",
        name="potential_field_pure_pursuit",
        output="screen",
        parameters=[
            {
                "plan_topic": "/plan",
                "scan_topic": "/scan",
                "cmd_vel_topic": "/cmd_vel",

                "map_frame": "map",
                "base_frame": "base_footprint",

                "lookahead_distance": 0.55,
                "max_linear_velocity": 0.25,
                "min_linear_velocity": 0.05,
                "max_angular_velocity": 2.1,
                "goal_tolerance": 0.10,

                "slowdown_distance": 0.85,
                "slowdown_min_factor": 0.30,

                "heading_kp": 1.2,
                "rotate_in_place_threshold": 1.5,
                "rotate_in_place_hysteresis": 0.7,
                "max_angular_acceleration": 3.2,

                "influence_radius": 0.9,
                "repulsive_gain": 0.7,
                "attractive_gain": 1.1,
                "repulsive_smoothing_alpha": 0.3,
                "scan_stride": 2,
                "scan_angle_offset_deg": 180.0,
                "pf_front_angle_deg": 60.0,

                "hard_stop_distance": 0.24,
                "hard_stop_front_angle_deg": 25.0,

                "control_rate": 20.0,
            }
        ],
    )

    # ============================================================
    # 7. RVIZ
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
        arguments=[
            "-d",
            str(rviz_config),
        ],
    )

    # ============================================================
    # STARTUP SEQUENCE
    # ============================================================

    return LaunchDescription(
        [

            # RSP
            rsp,

            # MAP SERVER
            TimerAction(
                period=2.0,
                actions=[
                    map_server
                ],
            ),

            # LIFECYCLE MANAGER
            TimerAction(
                period=4.0,
                actions=[
                    lifecycle_manager
                ],
            ),

            # AMCL
            TimerAction(
                period=7.0,
                actions=[
                    amcl
                ],
            ),

            # A*
            TimerAction(
                period=10.0,
                actions=[
                    astar
                ],
            ),

            # POTENTIAL FIELD PURE PURSUIT
            TimerAction(
                period=11.0,
                actions=[
                    potential_field_pure_pursuit
                ],
            ),

            # RVIZ
            TimerAction(
                period=13.0,
                actions=[
                    rviz
                ],
            ),
        ]
    )
