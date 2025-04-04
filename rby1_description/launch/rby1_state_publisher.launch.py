import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory("rby1_description"), "urdf", "rby1.urdf"
    )

    # Read the URDF file
    with open(urdf_file_path, "r") as urdf_file:
        robot_description = urdf_file.read()

    # Parameters to pass to the robot_state_publisher
    params = {"robot_description": robot_description, "use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulated time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(get_package_share_directory("rby1_description"), "rviz", "robot.rviz")],
            ),
        ]
    )
