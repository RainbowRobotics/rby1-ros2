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

"""
RB-Y1 driver against the rby1 simulator core (rby1-sim-isaac or the MuJoCo simulator image).

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
