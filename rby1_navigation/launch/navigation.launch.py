import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rby1_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_share, 'maps', 'rby1_sample_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_share, 'config', 'rby1_nav.rviz')

    map_arg = DeclareLaunchArgument('map', default_value=default_map, description='Full path to map file')
    params_arg = DeclareLaunchArgument('params_file', default_value=default_params, description='Full path to nav2 params file')
    sim_lidar_arg = DeclareLaunchArgument('use_sim_lidar', default_value='true', description='Enable virtual LiDAR simulation')

    use_sim_lidar = LaunchConfiguration('use_sim_lidar')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    simulated_lidar_node = Node(
        package='rby1_navigation',
        executable='simulated_lidar',
        name='simulated_lidar',
        output='screen',
        condition=IfCondition(use_sim_lidar)
    )

    lidar_merger_node = Node(
        package='rby1_navigation',
        executable='lidar_merger',
        name='lidar_merger',
        output='screen',
        condition=IfCondition(use_sim_lidar)
    )

    stream_manager_node = Node(
        package='rby1_navigation',
        executable='stream_manager',
        name='stream_manager',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        sim_lidar_arg,
        simulated_lidar_node,
        lidar_merger_node,
        stream_manager_node,
        nav2_bringup_launch,
        rviz_node
    ])
