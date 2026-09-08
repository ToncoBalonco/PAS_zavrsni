import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # ============================================================
    # PARAMETERS
    # ============================================================

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true'
    )

    optitrack_bag_path = LaunchConfiguration(
        'optitrack_bag_path',
        default='/home/tona/astro_ws_pas/bags/bag_trajek_sporo_2_opt'
    )

    optitrack_topic = LaunchConfiguration(
        'optitrack_topic',
        default='/vrpn_mocap/astro_5_cuk/pose'
    )

    current_topic = LaunchConfiguration(
        'current_topic',
        default='/absoulte_current'
    )

    pkg_astro = get_package_share_directory(
        'astro'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    # ============================================================
    # GZ RESOURCE PATH
    # ============================================================

    pkg_parent_dir = os.path.dirname(
        pkg_astro
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent_dir]
    )

    # ============================================================
    # XACRO
    # ============================================================

    xacro_file = os.path.join(
        pkg_astro,
        'description',
        'astro.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        xacro_file
    ).toxml()

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ============================================================
    # GAZEBO
    # ============================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'gz_args':
                '-r empty.sdf'
        }.items()
    )

    # ============================================================
    # SPAWN ROBOT
    # ============================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-string',
            robot_description_config,

            '-name',
            'astro',

            '-z',
            '0.04',

            '-Y',
            '0.0'
        ],

        output='screen'
    )

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'joint_state_broadcaster'
        ],

        output='screen'
    )

    # ============================================================
    # DIFF DRIVE CONTROLLER  (with use_stamped_vel=true)
    # ============================================================

    diff_drive_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'diff_drive_base_controller'
            ],

        output='screen'
    )

    # ============================================================
    # LIDAR BRIDGE
    # ============================================================

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],

        output='screen',

        parameters=[
            {
                'override_frame_id': 'laser_frame'
            }
        ]
    )

    # ============================================================
    # IMU BRIDGE
    # ============================================================

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],

        output='screen'
    )

    # ============================================================
    # CLOCK BRIDGE
    # ============================================================

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],

        output='screen'
    )

    # ============================================================
    # CAMERA BRIDGE
    # ============================================================

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[

            '/camera/color/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/camera/color/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',

            '/camera/depth/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/camera/depth/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',

            '/camera/infra1/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/camera/infra1/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],

        output='screen'
    )

    # ============================================================
    # GAZEBO POSE BRIDGE
    # ============================================================

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/world/empty/dynamic_pose/info'
            '@tf2_msgs/msg/TFMessage'
            '[gz.msgs.Pose_V'
        ],

        output='screen'
    )

    # ============================================================
    # ROSBAG CMD_VEL REPLAYER  (publishes TwistStamped)
    # OptiTrack se sada čita iz zasebnog snimljenog baga (PoseStamped),
    # umjesto iz OptiTrack CSV exporta.
    # ============================================================

    cmd_vel_replayer = Node(
        package='astro',
        executable='simul_opt_bag.py',

        output='screen',

        parameters=[
            {
                'use_sim_time': True,

                'bag_path':
                    '/home/tona/astro_ws_pas/bags/'
                    'bag_trajek_sporo_2',

                'optitrack_bag_path':
                    optitrack_bag_path,

                'optitrack_topic':
                    optitrack_topic,

                'input_topic':
                    '/cmd_vel',

                # NOW PUBLISH ON THE STAMPED TOPIC
                'output_topic':
                    '/diff_drive_base_controller/cmd_vel',

                'gazebo_pose_topic':
                    '/world/empty/dynamic_pose/info',

                'robot_frame_id':
                    'astro',   # or 'astro' – choose your frame

                'current_topic':
                    current_topic,

                'rate':
                    1.0,

                'loop':
                    False,

                'output_dir':
                    '/home/tona/astro_ws_pas/results'
            }
        ]
    )

    # ============================================================
    # STARTUP SEQUENCE
    # ============================================================

    spawn_robot_delayed = TimerAction(
        period=1.0,
        actions=[spawn_robot]
    )

    controllers_delayed = TimerAction(
        period=10.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_broadcaster_spawner
        ]
    )

    bag_delayed = TimerAction(
        period=15.0,
        actions=[cmd_vel_replayer]
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'optitrack_bag_path',
            default_value='/home/tona/astro_ws_pas/bags/bag_trajek_sporo_2_opt',
            description='Path to bag with recorded OptiTrack pose (PoseStamped)'
        ),

        DeclareLaunchArgument(
            'optitrack_topic',
            default_value='/vrpn_mocap/astro_5_cuk/pose',
            description='Topic in the OptiTrack bag containing PoseStamped pose'
        ),

        DeclareLaunchArgument(
            'current_topic',
            default_value='/absoulte_current',
            description='Topic with recorded motor current in the real robot bag'
        ),

        set_gz_resource_path,
        gazebo,
        robot_state_publisher,

        # Bridges
        clock_bridge,
        pose_bridge,

        spawn_robot_delayed,
        controllers_delayed,
        bag_delayed
    ])
