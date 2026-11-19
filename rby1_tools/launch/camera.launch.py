"""A camera on the robot as ROS topics, for Isaac ROS AprilTag in the container.

  ros2 launch rby1_tools camera.launch.py                         # RealSense (config/camera/realsense.yaml)
  ros2 launch rby1_tools camera.launch.py config:=/path/my.yaml   # other settings (another device: serial in it)
  ros2 launch rby1_tools camera.launch.py camera_name:=desk mount:=none   # a second camera, on no link
  ros2 launch rby1_tools camera.launch.py rviz:=true              # with RViz: the image and the markers

Everything else is in the settings file: the device, size, frame rate,
exposure, which RealSense streams (use_rgb, use_depth, use_ir, use_point_cloud),
compressed, the camera model. Topics and frames follow camera_name:
/<camera_name>/rgb/raw (compressed: /<camera_name>/rgb/compressed) and
/<camera_name>/rgb/info, the same for depth, ir_left, ir_right, and
/<camera_name>/point_cloud; mount names an entry of config/camera/camera_mount.yaml,
which the camera node publishes as static TF to <camera_name>_link and
<camera_name>_optical_frame. Several cameras run side by side with different
camera_name (and serial, in their settings). Cameras stay on the host: no device
mounts or camera drivers in the container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    camera_name = LaunchConfiguration('camera_name')
    parameters = [LaunchConfiguration('config'), {'camera_name': camera_name, 'mount': LaunchConfiguration('mount')}]
    return LaunchDescription([
        # Which kind of camera, and so which publisher and settings file (config/camera/<camera>.yaml).
        # Only realsense for now; another kind adds its choice, its node below and its settings file.
        DeclareLaunchArgument('camera', default_value='realsense', choices=['realsense'],
                              description='the kind of camera: realsense (librealsense)'),
        DeclareLaunchArgument('camera_name', default_value='camera',
                              description='first part of the topics and frames (/<camera_name>/..., <camera_name>_link)'),
        DeclareLaunchArgument('config', description='settings YAML',
                              default_value=PathJoinSubstitution([FindPackageShare('rby1_tools'), 'config', 'camera',
                                                                  [camera, '.yaml']])),
        DeclareLaunchArgument('mount', default_value='head_default',
                              description='where the camera sits: a name in config/camera/camera_mount.yaml; none: '
                                          'on no link'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                              description='true: RViz with the image, the frames and the markers (config/camera/camera.rviz)'),

        Node(package='rby1_tools', executable='realsense_publisher', name=[camera_name, '_publisher'],
             parameters=parameters, output='screen', on_exit=Shutdown(),
             condition=LaunchConfigurationEquals('camera', 'realsense')),
        Node(package='rviz2', executable='rviz2', name=[camera_name, '_rviz'], output='log',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('rby1_tools'), 'config', 'camera', 'camera.rviz']),
                        '-f', [camera_name, '_link']],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
