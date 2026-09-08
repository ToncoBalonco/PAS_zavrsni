from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('astro_slam')

    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg, 'config', 'astro_stage_mapper_online_async.yaml'),
        description='Path to slam_toolbox parameters file',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )
    
    rviz_config_file = os.path.join(pkg, 'config', 'rviz', 'astro_stage_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        declare_slam_params,
        slam_toolbox_node,
        rviz_node
    ])
