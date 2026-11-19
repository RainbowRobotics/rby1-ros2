"""The RB-Y1's LakiBeam1 2D LiDARs as ROS topics: /lidar_front_left/scan and
/lidar_front_right/scan, and up to two more at the rear -- or, with merge: true in the settings,
all of them as one scan on the base, /lidar_merged/scan.

  ros2 launch rby1_tools lidar.launch.py                    # the front pair (type A)
  ros2 launch rby1_tools lidar.launch.py front_right:=false # the front left one only
  ros2 launch rby1_tools lidar.launch.py rear_left:=true rear_right:=true   # four (their sections uncommented)
  ros2 launch rby1_tools lidar.launch.py config:=/path/my_lidar.yaml

Run it on the PC the sensors send their packets to (their "host" in
curl http://<sensor_ip>/api/v1/sensor/overview). Settings, one section per sensor, in
config/lidar/lidar.yaml: its address and port, and where it sits (parent, xyz, rpy), which the
node publishes as static TF <parent> -> <lidar_name>_link. A sensor started without its section
stops with that reason. lidar_merger always starts, takes the sensors started here, and with
merge: false stops at once.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', description='settings YAML, a section per sensor (lidar_front_left, lidar_front_right, ...)',
                              default_value=PathJoinSubstitution([FindPackageShare('rby1_tools'), 'config', 'lidar',
                                                                  'lidar.yaml'])),
        DeclareLaunchArgument('front_left', default_value='true', choices=['true', 'false'], description='the front left LiDAR'),
        DeclareLaunchArgument('front_right', default_value='true', choices=['true', 'false'], description='the front right LiDAR'),
        DeclareLaunchArgument('rear_left', default_value='false', choices=['true', 'false'], description='the rear left LiDAR'),
        DeclareLaunchArgument('rear_right', default_value='false', choices=['true', 'false'], description='the rear right LiDAR'),

        Node(package='rby1_tools', executable='lidar_publisher', name='lidar_front_left', parameters=[config],
             output='screen', condition=IfCondition(LaunchConfiguration('front_left'))),
        Node(package='rby1_tools', executable='lidar_publisher', name='lidar_front_right', parameters=[config],
             output='screen', condition=IfCondition(LaunchConfiguration('front_right'))),
        Node(package='rby1_tools', executable='lidar_publisher', name='lidar_rear_left', parameters=[config],
             output='screen', condition=IfCondition(LaunchConfiguration('rear_left'))),
        Node(package='rby1_tools', executable='lidar_publisher', name='lidar_rear_right', parameters=[config],
             output='screen', condition=IfCondition(LaunchConfiguration('rear_right'))),
        Node(package='rby1_tools', executable='lidar_merger', name='lidar_merger', output='screen',
             parameters=[config, {
                 'front_left': ParameterValue(LaunchConfiguration('front_left'), value_type=bool),
                 'front_right': ParameterValue(LaunchConfiguration('front_right'), value_type=bool),
                 'rear_left': ParameterValue(LaunchConfiguration('rear_left'), value_type=bool),
                 'rear_right': ParameterValue(LaunchConfiguration('rear_right'), value_type=bool)}]),
    ])
