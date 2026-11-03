# Copyright 2026 Rainbow Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context).lower()
    if model not in ("a", "m", "ub"):
        raise RuntimeError("model must be 'a', 'm', or 'ub'")
    version = LaunchConfiguration("version").perform(context)

    robot_description_file = PathJoinSubstitution(
        [FindPackageShare("rby1_description"), "urdf", f"rby1{model}.urdf.xacro"]
    )
    xacro_cmd = [
        "xacro ",
        robot_description_file,
        " robot_ip:=",
        LaunchConfiguration("robot_ip"),
        " use_mock_hardware:=true",
    ]
    if model != "ub":
        xacro_cmd += [" version:=", version]
    robot_description = Command(xacro_cmd)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="a", description="RBY1 model: a, m, or ub"),
            DeclareLaunchArgument(
                "version",
                default_value="1.2",
                description="Model version (a: 1.0/1.1/1.2; m: 1.0/1.1/1.2/1.3; ignored for ub)",
            ),
            DeclareLaunchArgument("robot_ip", default_value="127.0.0.1:50051"),
            OpaqueFunction(function=launch_setup),
        ]
    )
