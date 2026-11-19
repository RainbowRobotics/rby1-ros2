"""Modules on the robot and fixtures around it, into the MoveIt planning scene.

  ros2 launch rby1_moveit_objects objects.launch.py [config:=/path/to/objects.yaml]
  ros2 launch rby1_moveit_objects objects.launch.py config:=rby1_tools/model/rby1_gripper/gripper.yaml   # <package>/<file>

Needs a running move_group (MoveIt or the cuMotion launch). The objects stay while this
runs -- put back if move_group restarts -- and are taken away on Ctrl+C. The file
format is described at the top of config/objects.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def publisher(context):
    """The node, with `config` as given, a bare file name of this package's config/, or
    <package>/<path> in another package's share (rby1_tools/model/rby1_gripper/gripper.yaml)."""
    share = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.isfile(config):
        if os.path.isfile(os.path.join(share, config)):
            config = os.path.join(share, config)
        else:
            package, _, rest = config.partition('/')
            try:
                candidate = os.path.join(get_package_share_directory(package), rest)
            except Exception:
                candidate = ''
            if candidate and os.path.isfile(candidate):
                config = candidate
    return [Node(package='rby1_moveit_objects', executable='publish_objects', output='screen',
                 parameters=[{'config': config}])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config', 'objects.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='YAML file of objects: a path, a file name in this package\'s config/, or <package>/<path>'),
        OpaqueFunction(function=publisher),
    ])
