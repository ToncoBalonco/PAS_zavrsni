#!/usr/bin/env python3
"""
A* Zone Navigation Launch
==========================
Analogno astar_ob_nav_launch.py, ali koristi "zone" verziju stacka:

    zone_obstacle_detector.py  - front klaster -> /front_zone_obstacles
                                  grid; left/right klaster -> avoidance
                                  signali
    astar_zone_planner.py      - A* + replan monitor + /force_replan
                                  handler (koristi front_zone_obstacles)
    pure_pursuit_follower.py   - prati /plan
    cmd_vel_zone_gate.py       - zadnja linija obrane: front slow +
                                  lateral steer (NORMAL), i kad se
                                  klaster potvrdi u stop zoni:
                                  HARD_STOP -> REVERSING (reverse_time_sec
                                  sekundi unatrag) -> ROTATING (rotacija
                                  za rotate_angle_deg, OD klastera) ->
                                  /force_replan -> nazad u NORMAL
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
    # 5. ZONE OBSTACLE DETECTOR
    #
    # Sluša /map i /scan. Front klaster -> /front_zone_obstacles
    # grid (koristi astar_zone_planner za inflate+replan). Left/
    # right klaster -> /left_avoidance_signal, /right_avoidance_signal
    # (koristi cmd_vel_zone_gate za reaktivni lateralni steer).
    # Mora biti gore prije A*-a da planer od starta ima front-zone
    # sloj dostupan.
    # ============================================================

    zone_obstacle_detector = Node(
        package="astro",
        executable="zone_obstacle_detector.py",
        name="zone_obstacle_detector",
        output="screen",
        parameters=[
            {
                "map_topic": "/map",
                "scan_topic": "/scan",
                "front_grid_topic": "/front_zone_obstacles",
                "left_signal_topic": "/left_avoidance_signal",
                "right_signal_topic": "/right_avoidance_signal",

                "front_half_angle_deg": 20.0,
                "side_angle_min_deg": 20.0,
                "side_angle_max_deg": 110.0,
                "positive_angle_is_left": True,

                # LIDAR je fizički montiran zarotiran 180 stupnjeva
                # (ono što je "nazad" u sirovom /scan-u je zapravo
                # "naprijed" na robotu) - kompenzira se ovdje.
                "angle_offset_deg": 180.0,

                "front_distance_min": 0.40,
                "front_distance_max": 0.60,
                "side_distance_min": 0.30,
                "side_distance_max": 0.50,

                "min_cluster_points": 5,
                "max_range_jump": 0.15,
                "max_index_gap": 2,

                "max_range_used": 5.0,
                "obstacle_timeout_sec": 2.0,
                "publish_rate_hz": 10.0,
            }
        ],
    )

    # ============================================================
    # 6. A* PATH PLANNER - ZONE EDITION
    # ============================================================

    astar_zone = Node(
        package="astro",
        executable="astar_zone_planner.py",
        name="astar_zone_planner",
        output="screen",
        parameters=[
            {
                "front_zone_obstacles_topic": "/front_zone_obstacles",
                "force_replan_topic": "/force_replan",
                "inflation_radius_m": 0.35,
                "front_zone_inflation_m": 0.10,
                "check_horizon_m": 2.0,
                "replan_cooldown_sec": 1.5,
            }
        ],
    )

    # ============================================================
    # 7. PURE PURSUIT
    # ============================================================

    pure_pursuit = Node(
        package="astro",
        executable="pure_pursuit_follower.py",
        name="pure_pursuit_follower",
        output="screen",
    )

    # ============================================================
    # 8. CMD VEL ZONE GATE
    #
    # Nakon pure pursuita - sluša njegov /cmd_vel_nav izlaz.
    # Front slow + lateral steer u NORMAL stanju; kad potvrdi
    # klaster u stop zoni: HARD_STOP -> REVERSING (reverse_time_sec s
    # unatrag) -> ROTATING (rotate_angle_deg, od klastera) ->
    # /force_replan -> NORMAL.
    # ============================================================

    cmd_vel_zone_gate = Node(
        package="astro",
        executable="cmd_vel_zone_gate.py",
        name="cmd_vel_zone_gate",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "cmd_in_topic": "/cmd_vel_nav",
                "cmd_out_topic": "/cmd_vel",
                "left_signal_topic": "/left_avoidance_signal",
                "right_signal_topic": "/right_avoidance_signal",
                "force_replan_topic": "/force_replan",

                "stop_distance": 0.30,
                "slow_distance": 0.70,
                "front_angle_deg": 60.0,
                # Isti fizički offset kao u zone_obstacle_detector -
                # LIDAR montiran zarotiran 180 stupnjeva.
                "angle_offset_deg": 180.0,

                "min_cluster_points": 5,
                "max_range_jump": 0.15,
                "max_index_gap": 2,
                "emergency_stop_distance": 0.15,

                "hard_stop_hold_sec": 1.0,
                # Koliko DUGO (s) se vozi unatrag - direktno podesivo
                "reverse_time_sec": 6.7,
                "reverse_speed": 0.15,
                # Za koliko stupnjeva se zarotira (od klastera)
                "rotate_angle_deg": 25.0,
                "rotate_speed": 0.4,
                "replan_request_cooldown_sec": 3.0,
                "post_maneuver_grace_sec": 1.0,

                "lateral_gain": 0.6,
                "max_lateral_bias": 0.5,
                "bias_smoothing_alpha": 0.3,
                "signal_timeout_sec": 0.5,
                "control_rate_hz": 20.0,
            }
        ],
    )

    # ============================================================
    # 9. RVIZ
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

            # ZONE OBSTACLE DETECTOR
            #
            # Prije A*-a - treba mu /map (transient_local, pa je
            # ok i ako se pretplati kasnije) i TF koji već postoji.
            #
            TimerAction(
                period=15.0,
                actions=[
                    zone_obstacle_detector
                ],
            ),

            # A* - ZONE EDITION
            TimerAction(
                period=16.0,
                actions=[
                    astar_zone
                ],
            ),

            # PURE PURSUIT
            TimerAction(
                period=18.0,
                actions=[
                    pure_pursuit
                ],
            ),

            # CMD VEL ZONE GATE
            #
            # Nakon pure pursuita - sluša njegov /cmd_vel_nav izlaz.
            #
            TimerAction(
                period=19.0,
                actions=[
                    cmd_vel_zone_gate
                ],
            ),

            # RVIZ
            TimerAction(
                period=20.0,
                actions=[
                    rviz
                ],
            ),
        ]
    )
