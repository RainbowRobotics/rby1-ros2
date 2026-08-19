from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        output='screen',
        parameters=[{
            'front_topic': '/scan_front',
            'rear_topic': '/scan_rear',
            'output_topic': '/scan',
            'output_frame': 'base_footprint'
        }]
    )

    return LaunchDescription([
        simulated_lidar_node,
        lidar_merger_node
    ])
