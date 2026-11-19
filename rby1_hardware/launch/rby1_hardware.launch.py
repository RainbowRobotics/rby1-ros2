# ros2_control with the RB-Y1 hardware plugin (rby1_hardware/RBY1SystemHardware), without MoveIt.
#
# It starts the controller manager (ros2_control_node) with the robot's URDF, robot_state_publisher,
# the joint_state_broadcaster and the trajectory controllers listed below. The driver (rby1_driver)
# must already be running: the plugin takes the control right from it (<driver_namespace>/hardware_control)
# and then streams the joint positions to the robot itself.
#
#   ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2
#   ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2 robot_ip:=192.168.30.1:50051
#   ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2 use_fake_hardware:=true   # no robot, no driver
#
# model and version (required) pick rby1_moveit_<model>_<version>, whose URDF with its <ros2_control> tag
# and controllers are used (only its files; nothing of MoveIt runs). The plugin compares them with the
# connected robot and refuses to activate if they differ. The controllers started active are set here:
CONTROLLERS = ['right_arm_controller', 'left_arm_controller', 'torso_controller', 'head_controller']
# ^ from its config/ros2_controllers.yaml: right_arm_controller, left_arm_controller,
#   both_arms_controller, torso_controller, head_controller, body_controller (v1.3 also gripper_r/l_controller).
#   Two controllers cannot share a joint while active (both_arms_controller and right_arm_controller):
#   list one of them here and switch at run time with `ros2 control switch_controllers`.

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    value = {name: LaunchConfiguration(name).perform(context)
             for name in ('robot_ip', 'driver_namespace', 'use_fake_hardware', 'model', 'version')}
    model = value['model'].strip().lower()
    tag = value['version'].strip().replace('.', '_')   # 1.2 -> 1_2
    package = f'rby1_moveit_{model}_{tag}'
    try:
        config_dir = os.path.join(get_package_share_directory(package), 'config')
    except PackageNotFoundError:
        raise RuntimeError(f"model:={value['model']} version:={value['version']}: no package {package}; "
                           'it is rby1_moveit_{a,m}_{1_0,1_1,1_2,1_3}') from None
    robot = f'RBY1_{model.upper()}_v{tag}'

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', os.path.join(config_dir, f'{robot}.urdf.xacro'),
            ' use_fake_hardware:=', value['use_fake_hardware'],
            ' robot_ip:=', value['robot_ip'],
            ' model:=', model,
            ' driver_namespace:=', value['driver_namespace'],
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # the controller manager: loads the hardware plugin named in the URDF and runs the controllers.
    # Humble takes the URDF from the robot_description parameter below; Jazzy takes it from the
    # /robot_description topic of the robot_state_publisher above (the same URDF).
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    os.path.join(config_dir, 'ros2_controllers.yaml')],
        on_exit=Shutdown(),   # without it nothing else here is of use
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )
        for name in ['joint_state_broadcaster'] + CONTROLLERS
    ]

    return [
        robot_state_publisher,
        controller_manager,
        *spawners,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1:50051',
                              description='robot address and gRPC port (the driver connects to the same one)'),
        DeclareLaunchArgument('driver_namespace', default_value='rby1',
                              description='namespace of the running rby1_ros2_driver'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='true: mock_components/GenericSystem instead of the robot (no driver needed)'),
        DeclareLaunchArgument('model', description='Robot kind: a or m (required)'),
        DeclareLaunchArgument('version', description='Robot version such as 1.2 (required)'),
        OpaqueFunction(function=setup),
    ])
