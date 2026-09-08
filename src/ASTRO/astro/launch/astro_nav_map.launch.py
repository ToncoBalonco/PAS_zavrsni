#!/usr/bin/env python3

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

    # Putanja do RViz konfiguracije unutar paketa 'astro'
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
    #
    # Only map_server is managed here.
    # autostart=True means:
    #
    # unconfigured -> inactive -> active
    #
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
		    "laser_max_range": 6.0,
		    "laser_max_beams": 60,
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
    # 5. A* PATH PLANNER
    # ============================================================

    astar = Node(
        package="astro",
        executable="astar_path_planner_dod.py",
        output="screen",
    )

    # ============================================================
    # 6. PURE PURSUIT
    # ============================================================

    pure_pursuit = Node(
        package="astro",
        executable="pure_pursuit_follower.py",
        output="screen",
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
            #
            # Starts after map_server exists.
            # It automatically activates map_server.
            #
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
                period=9.0,
                actions=[
                    astar
                ],
            ),

            # PURE PURSUIT
            TimerAction(
                period=10.0,
                actions=[
                    pure_pursuit
                ],
            ),

            # RVIZ
            TimerAction(
                period=12.0,
                actions=[
                    rviz
                ],
            ),
        ]
    )
