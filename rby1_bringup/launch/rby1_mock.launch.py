"""RB-Y1 driver without a robot.

backend:=fake (default) runs the real RB-Y1 hardware interface against an in-memory robot, so the
command-mode switching and guards behave as on the robot. use_mock_hardware:=true uses
mock_components/GenericSystem instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    passthrough = ["model", "version", "use_mock_hardware", "activate_controllers", "gripper", "realtime"]
    defaults = {"model": "a", "version": "1.2", "use_mock_hardware": "false", "activate_controllers": "",
                "gripper": "false", "realtime": "false"}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=defaults[name]) for name in passthrough]
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("rby1_bringup"), "launch", "rby1_control.launch.py"])
                ),
                launch_arguments={
                    **{name: LaunchConfiguration(name) for name in passthrough},
                    "backend": "fake",
                    "gripper_backend": "fake",
                    "launch_robot_manager": "false",
                }.items(),
            )
        ]
    )
