"""RB-Y1 driver against the rby1 simulator core (rby1-sim-isaac or the MuJoCo simulator image).

The simulator core serves the same gRPC API as the robot, by default on 127.0.0.1:50051. The
hardware enables power, servo and the control manager itself (auto_enable:=true).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    passthrough = {"model": "a", "version": "1.2", "robot_ip": "127.0.0.1:50051", "activate_controllers": ""}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in passthrough.items()]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("rby1_bringup"), "launch", "rby1_control.launch.py"])
                ),
                launch_arguments={
                    **{name: LaunchConfiguration(name) for name in passthrough},
                    "backend": "sdk",
                    "auto_enable": "true",
                    "gripper": "false",
                }.items(),
            )
        ]
    )
