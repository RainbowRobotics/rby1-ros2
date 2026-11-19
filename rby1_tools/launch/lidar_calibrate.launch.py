"""Where the LiDARs sit against one of them, from what they see in common: starts them as
config/lidar/lidar.yaml says, runs lidar_calibrate once, and stops them all when it ends.

  ros2 launch rby1_tools lidar_calibrate.launch.py                              # the front pair
  ros2 launch rby1_tools lidar_calibrate.launch.py rear_left:=true rear_right:=true   # four

Stop lidar.launch.py first (the same ports). Keep the robot still where walls in several directions
are in view of the sensors. It prints each sensor's xyz and rpy for lidar.yaml against the
reference (lidar_calibrate's section there); copy them in yourself -- no file is changed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    calibrate = Node(package='rby1_tools', executable='lidar_calibrate', name='lidar_calibrate', output='screen',
                     parameters=[config, {
                         'front_left': ParameterValue(LaunchConfiguration('front_left'), value_type=bool),
                         'front_right': ParameterValue(LaunchConfiguration('front_right'), value_type=bool),
                         'rear_left': ParameterValue(LaunchConfiguration('rear_left'), value_type=bool),
                         'rear_right': ParameterValue(LaunchConfiguration('rear_right'), value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('config', description='settings YAML, as for lidar.launch.py',
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
        calibrate,
        RegisterEventHandler(OnProcessExit(target_action=calibrate,
                                           on_exit=[EmitEvent(event=Shutdown(reason='lidar_calibrate is done'))])),
    ])
