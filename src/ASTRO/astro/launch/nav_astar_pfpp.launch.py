#!/usr/bin/env python3
"""
A* + Potential Field Pure Pursuit v2 - launch
=================================================
    rsp -> map_server -> amcl -> astar_global_planner.py
        -> pfpp_local_controller.py -> rviz

Nasljeđuje strukturu iz astar_pf_pp.launch.py (bez
dynamic_ob_layer.py i cmd_safety.py - lokalno izbjegavanje prepreka
radi isključivo pfpp_local_controller.py direktno iz /scan).

Svi kontroler/planer parametri su u
astro/config/astar_pfpp_params.yaml, ne u ovoj datoteci.

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

    pkg_share = Path(get_package_share_directory("astro"))

    rviz_config = pkg_share / "rviz" / "astar_planer_v1.rviz"
    params_file = pkg_share / "config" / "astar_pfpp_params.yaml"

    # ============================================================
    # 1. ROBOT STATE PUBLISHER
    # ============================================================

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "rsp.launch.py")
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
                "node_names": ["map_server", "amcl"],
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
    # 5. A* GLOBAL PLANNER
    # ============================================================

    astar = Node(
        package="astro",
        executable="astar_global_planner.py",
        name="astar_global_planner",
        output="screen",
        parameters=[str(params_file)],
    )

    # ============================================================
    # 6. PFPP LOCAL CONTROLLER
    # ============================================================

    pfpp_controller = Node(
        package="astro",
        executable="pfpp_local_controller.py",
        name="pfpp_local_controller",
        output="screen",
        parameters=[str(params_file)],
    )

    # ============================================================
    # 7. RVIZ
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=["-d", str(rviz_config)],
    )

    # ============================================================
    # STARTUP SEQUENCE
    # ============================================================

    return LaunchDescription(
        [
            rsp,
            TimerAction(period=2.0, actions=[map_server]),
            TimerAction(period=4.0, actions=[lifecycle_manager]),
            TimerAction(period=7.0, actions=[amcl]),
            TimerAction(period=10.0, actions=[astar]),
            TimerAction(period=11.0, actions=[pfpp_controller]),
            TimerAction(period=13.0, actions=[rviz]),
        ]
    )
