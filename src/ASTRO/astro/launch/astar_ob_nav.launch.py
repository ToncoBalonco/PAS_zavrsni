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
		    "laser_max_beams": 100,
		    "laser_sigma_hit": 0.2,
		    "laser_z_hit": 0.95,
		    "laser_z_rand": 0.05,

		    "alpha1": 0.2,
		    "alpha2": 0.2,
		    "alpha3": 0.2,
		    "alpha4": 0.2,

		    "min_particles": 500,
		    "max_particles": 2000,

		    # KLD-sampling error/z parametri - koliko "labavo" filter
		    # smije procijeniti potreban broj čestica
		    "pf_err": 0.05,
		    "pf_z": 0.99,

		    # Koliko se robot mora pomaknuti/zarotirati prije nego
		    # AMCL ažurira filter - manje = češće (točnije) ažuriranje
		    # kod spore vožnje, malo veće opterećenje CPU-a
		    "update_min_d": 0.15,
		    "update_min_a": 0.15,

		    # Recovery (augmented MCL) - ako filter "izgubi" robota
		    # (npr. nagli odom skok, kidnapping), ovo omogućava da se
		    # sam oporavi ubacivanjem nasumičnih čestica, umjesto da
		    # ostane trajno zaglavljen dok se ručno ne postavi pose
		    # u RViz-u.
		    "recovery_alpha_slow": 0.001,
		    "recovery_alpha_fast": 0.1,

		    # Produženo - proširuje "prozor valjanosti" objavljene
		    # map -> odom transformacije. Kod dva vremenski
		    # neusklađena računala (robot + operater) ovo smanjuje
		    # broj ExtrapolationException grešaka nizvodno (npr. u
		    # pure_pursuit_follower.py i astar_path_planner_ob.py
		    # koji rade TF lookup map -> base_footprint).
		    "transform_tolerance": 1.0,

		    "save_pose_rate": 0.5,
		    "tf_broadcast": True,
		}
	    ],
	)

    # ============================================================
    # 5. LOCAL OBSTACLE LAYER (PROMIJENJENO)
    #
    # Sluša SAMO /scan (bez /map i bez TF-a) i publishuje lokalni,
    # robot-centrirani /dynamic_obstacles grid - korisno za
    # vizualizaciju u RViz-u. Više NIJE ulaz u A* planer (vidi
    # napomenu kod astar node-a niže) jer bi merge s globalnom
    # kartom opet zahtijevao map-frame TF koji je nepouzdan kad
    # računalo na robotu i operatersko računalo nisu vremenski
    # usklađena.
    # ============================================================

    dynamic_obstacle_layer = Node(
        package="astro",
        executable="dynamic_ob_layer.py",
        name="dynamic_ob_layer",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "output_topic": "/dynamic_obstacles",
                "local_size_m": 6.0,
                "resolution": 0.05,
                "max_range_used": 5.0,
                "angle_stride": 2,
                "obstacle_timeout_sec": 2.0,
            }
        ],
    )

    # ============================================================
    # 6. A* PATH PLANNER (PROMIJENJENO - čisti globalni planer)
    #
    # Planira samo na statičkoj karti (/map). Auto-replan preko
    # dinamičkog sloja i map-frame TF-a je uklonjen - obstacle
    # avoidance u stvarnom vremenu radi cmd_vel_safety_gate niže,
    # preko /scan, bez TF-a.
    # ============================================================

    astar = Node(
        package="astro",
        executable="astar_path_planner_ob.py",
        name="astar_path_planner",
        output="screen",
        parameters=[
            {
                "replan_request_topic": "/replan_request",
                "replan_cooldown_sec": 2.0,
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
    # 8. CMD VEL SAFETY GATE + REAKTIVNI AVOIDANCE (PROMIJENJENO)
    #
    # Sjedi između pure pursuita (/cmd_vel_nav) i stvarnog
    # /cmd_vel. Ovo je sada GLAVNI mehanizam za izbjegavanje
    # prepreka u stvarnom vremenu: čita ISKLJUČIVO /scan (bez
    # TF-a, bez /map), traži slobodan prolaz i skreće robota
    # prema njemu, uz stop/slow zaštitu. Zato ostaje pouzdan i
    # kad TF (map -> odom preko AMCL-a) zakaže zbog vremenski
    # neusklađenih računala.
    # ============================================================

    cmd_vel_safety_gate = Node(
        package="astro",
        executable="cmd_safety.py",
        name="cmd_vel_safety_gate",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "cmd_in_topic": "/cmd_vel_nav",
                "cmd_out_topic": "/cmd_vel",
                "stop_distance": 0.45,
                "slow_distance": 0.70,
                "front_angle_deg": 90.0,
                "search_angle_deg": 200.0,
                # laser_joint u lidar.xacro: rpy="0 0 pi" -> LIDAR je
                # fizički zarotiran 180° oko Z u odnosu na base_link,
                # pa kut 0 u /scan gleda prema natrag. Bez ovoga bi
                # "prednji" konus zapravo bio iza robota.
                "angle_offset_deg": 180.0,
                "avoidance_enabled": True,
                "min_gap_width_deg": 25.0,
                "robot_safety_radius_m": 0.30,
                "angular_gain": 1.5,
                "max_steer_correction_deg": 45.0,
                "rotate_in_place_speed": 1.2,
                # Kad je prepreka u stop zoni: prvo malo unatrag, pa
                # tek onda rotacija u mjestu (umjesto direktne rotacije).
                "reverse_speed": 0.15,
                "reverse_duration_sec": 1.0,
                "rear_check_angle_deg": 60.0,
                "max_rotate_duration_sec": 4.0,
                # Javlja astar_path_planner_ob.py kad manevar završi,
                # da preplanira prema zadnjem goalu iz nove pozicije.
                "replan_request_topic": "/replan_request",
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

            # LOCAL OBSTACLE LAYER
            #
            # Treba mu samo /scan (bez /map, bez TF-a), pa vrijeme
            # pokretanja nije kritično - ovdje ostaje prije A*-a
            # radi konzistentnog redoslijeda pokretanja.
            #
            TimerAction(
                period=15.0,
                actions=[
                    dynamic_obstacle_layer
                ],
            ),

            # A*
            TimerAction(
                period=16.0,
                actions=[
                    astar
                ],
            ),

            # PURE PURSUIT
            TimerAction(
                period=18.0,
                actions=[
                    pure_pursuit
                ],
            ),

            # CMD VEL SAFETY GATE
            #
            # Nakon pure pursuita - sluša njegov /cmd_vel_nav izlaz.
            #
            TimerAction(
                period=19.0,
                actions=[
                    cmd_vel_safety_gate
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
