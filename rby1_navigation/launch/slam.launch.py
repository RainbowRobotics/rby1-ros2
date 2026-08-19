import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rby1_navigation')
    default_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for slam_toolbox'
    )

    simulated_lidar_node = Node(
        package='rby1_navigation',
        executable='simulated_lidar',
        name='simulated_lidar',
        output='screen'
    )

    lidar_merger_node = Node(
        package='rby1_navigation',
        executable='lidar_merger',
        name='lidar_merger',
        output='screen'
    )

    stream_manager_node = Node(
        package='rby1_navigation',
        executable='stream_manager',
        name='stream_manager',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        declare_params_file_cmd,
        simulated_lidar_node,
        lidar_merger_node,
        stream_manager_node,
        slam_toolbox_node,
        rviz_node
    ])
