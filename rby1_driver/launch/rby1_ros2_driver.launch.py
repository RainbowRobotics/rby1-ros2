import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('rby1_driver'),
        'config',
        'driver_parameters.yaml'
    )
    
    # Declare the namespace launch argument
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='Top-level namespace for the robot node, topics, and services'
    )
    
    namespace = LaunchConfiguration('namespace')

    rby1_ros2_driver = Node(
        package='rby1_driver',
        executable='rby1_ros2_driver',
        name='rby1_ros2_driver',
        namespace=namespace,
        parameters=[parameter_file],
    output='screen',
    # arguments=['--ros-args', '--log-level', 'DEBUG'] # ros2 의 디버깅 모드 설정
    )

    return LaunchDescription([
        declare_namespace_cmd,
        rby1_ros2_driver
    ]) 
