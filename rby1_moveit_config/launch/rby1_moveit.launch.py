from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rby1", package_name="rby1_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("db", default_value="false"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    return ld
