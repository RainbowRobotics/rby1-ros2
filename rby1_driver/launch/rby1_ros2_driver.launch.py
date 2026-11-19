import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
# For the optional launches commented out in setup():
from launch.actions import IncludeLaunchDescription  # noqa: F401
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: F401
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The driver reads config/driver_parameters.yaml. The robot's address and model can also be
# given here, for a robot other than the yaml's, without editing it:
#   ros2 launch rby1_driver rby1_ros2_driver.launch.py robot_ip:=192.168.30.1:50051 model:=a
ARGUMENTS = {
    'namespace': ('rby1', 'Top-level namespace for the robot node, topics, and services'),
    'params_file': ('', "Driver parameters YAML; empty: this package's config/driver_parameters.yaml"),
    'robot_ip': ('', "Robot IP address and gRPC port; empty: the yaml's robot_ip"),
    'model': ('', "Robot model, a or m; empty: the yaml's model"),
}


def setup(context):
    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    parameter_file = value['params_file'] or os.path.join(
        get_package_share_directory('rby1_driver'), 'config', 'driver_parameters.yaml')
    overrides = {name: value[name] for name in ('robot_ip', 'model') if value[name]}

    driver = Node(
        package='rby1_driver',
        executable='rby1_ros2_driver',
        name='rby1_ros2_driver',
        namespace=value['namespace'],
        parameters=[parameter_file] + ([overrides] if overrides else []),
        output='screen',
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return [driver]  # e.g. [driver, moveit_executor, camera]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGUMENTS.items()],
        OpaqueFunction(function=setup),
    ])
