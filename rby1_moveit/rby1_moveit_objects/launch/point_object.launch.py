"""One obstacle in the MoveIt planning scene, put where a point topic says.

  ros2 launch rby1_moveit_objects point_object.launch.py [config:=/path/to/point_object.yaml]
  ros2 topic pub --once /rby1/scene/object_point geometry_msgs/msg/PointStamped \
      "{header: {frame_id: base}, point: {x: 0.5, y: -0.3, z: 1.0}}"

Needs a running move_group (MoveIt or the cuMotion launch). Every point on
/rby1/scene/object_point (an empty frame_id means base) moves the object there. The
object is described at the top of config/point_object.yaml. It stays in the scene
after Ctrl+C: ros2 run rby1_moveit_objects scene remove <name>.

  ros2 launch rby1_moveit_objects point_object.launch.py pose_topic:=/rby1/marker_7/pose

The object also goes where that pose is (a marker the camera sees), taken to base with
TF: the robot's TF and the camera's mounting TF must be up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def placer(context):
    """The node, with `config` as given or -- a bare file name -- from this package's config/."""
    share = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.isfile(config) and os.path.isfile(os.path.join(share, config)):
        config = os.path.join(share, config)
    # pose_topic left empty: the file's own value stands.
    pose_topic = LaunchConfiguration('pose_topic').perform(context)
    return [Node(package='rby1_moveit_objects', executable='object_at_point', output='screen',
                 parameters=[config] + ([{'pose_topic': pose_topic}] if pose_topic else []))]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config', 'point_object.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='Parameter file of the object: a path, or a file name in this '
                                          'package\'s config/'),
        DeclareLaunchArgument('pose_topic', default_value='',
                              description='A PoseStamped topic the object follows too, such as '
                                          '/rby1/marker_7/pose (default: pose_topic of the parameter file)'),
        OpaqueFunction(function=placer),
    ])
