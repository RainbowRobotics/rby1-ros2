import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rby1_navigation')
    rby1_driver_share = get_package_share_directory('rby1_driver')
    rby1_desc_share = get_package_share_directory('rby1_description')

    model_arg = DeclareLaunchArgument('model', default_value='m', description='Robot model: a or m')
    version_arg = DeclareLaunchArgument('version', default_value='1_2', description='Model version: 1_0, 1_1, 1_2, etc.')

    rby1_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rby1_driver_share, 'launch', 'rby1_ros2_driver.launch.py'))
    )

    rby1_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rby1_desc_share, 'launch', 'rby1_state_publisher.launch.py')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'version': LaunchConfiguration('version')
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py'))
    )

    return LaunchDescription([
        model_arg,
        version_arg,
        rby1_driver_launch,
        rby1_desc_launch,
        navigation_launch
    ])
