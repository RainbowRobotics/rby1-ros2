"""A prepared set of obstacles into the MoveIt planning scene, once.

  ros2 launch rby1_moveit_objects scene.launch.py [config:=/path/to/scene.yaml]

Runs `scene load <config>` and ends; the obstacles stay in the scene. Needs a running
move_group (MoveIt or the cuMotion launch). The file format is that of
config/example_scene.yaml. Take them away with
ros2 run rby1_moveit_objects scene remove <name> ... or scene clear.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def loader(context):
    """`scene load`, with `config` as given or -- a bare file name -- from this package's config/."""
    share = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.isfile(config) and os.path.isfile(os.path.join(share, config)):
        config = os.path.join(share, config)
    return [Node(package='rby1_moveit_objects', executable='scene', output='screen',
                 arguments=['load', config])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config', 'example_scene.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='YAML file of obstacles: a path, or a file name in this package\'s config/'),
        OpaqueFunction(function=loader),
    ])
