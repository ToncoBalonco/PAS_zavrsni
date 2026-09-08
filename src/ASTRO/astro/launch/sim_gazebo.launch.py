import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# Dodan TimerAction u importe:
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_astro = get_package_share_directory('astro')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Putanja do roditeljskog direktorija paketa kako bi Gazebo mogao razriješiti "model://astro/..."
    pkg_parent_dir = os.path.dirname(pkg_astro)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent_dir]
    )

    xacro_file = os.path.join(pkg_astro, 'description', 'astro.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()



    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

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
                '-r "/home/tona/astro_ws_pas/worlds/test_crta.world"'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_config,
            '-name', 'astro',
            '-z', '0.1'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    # ---------------------------------------------------------
    # KREIRANJE ODGODE OD 5 SEKUNDI ZA ROBOTA I KONTROLERE
    # ---------------------------------------------------------
    delayed_spawners = TimerAction(
        period=10.0, # Vrijeme čekanja u sekundama
        actions=[
            spawn_robot, 
            joint_state_broadcaster_spawner, 
            diff_drive_broadcaster_spawner
        ]
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'override_frame_id': 'laser_frame'}],
    )
    
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            

            # =====================================================
            # DEPTH
            # =====================================================

            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',


            # =====================================================
            # INFRARED
            # =====================================================

            '/camera/infra1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            '/camera/infra1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        gazebo,
        robot_state_publisher,
        lidar_bridge,
        imu_bridge,
        clock_bridge,
        camera_bridge,
        # Ovdje pozivamo TimerAction umjesto originalnih Node-ova
        delayed_spawners 
    ])
