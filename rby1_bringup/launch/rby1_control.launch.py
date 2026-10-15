"""Bring up the RB-Y1 ros2_control driver.

Starts robot_state_publisher, the controller manager with the RB-Y1 hardware interface (and the
gripper hardware if gripper:=true), joint_state_broadcaster, every RB-Y1 controller loaded
inactive, and the robot manager. Controllers listed in ``activate_controllers`` are activated.

Examples:
  ros2 launch rby1_bringup rby1_control.launch.py robot_ip:=192.168.30.1:50051 model:=a
  ros2 launch rby1_bringup rby1_control.launch.py backend:=fake model:=a     # no robot needed
  ros2 launch rby1_bringup rby1_control.launch.py activate_controllers:="right_arm_impedance_controller"
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

SUPPORTED_VERSIONS = {
    "a": ("1.0", "1.1", "1.2"),
    "m": ("1.0", "1.1", "1.2", "1.3"),
}
DEFAULT_VERSION = "1.2"
GRIPPER_CONTROLLERS = ("right_gripper_controller", "left_gripper_controller")


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _resolve_model_and_version(context, robot_ip, probe_allowed, connect_timeout_sec):
    model_arg = LaunchConfiguration("model").perform(context).strip().lower()
    version_arg = LaunchConfiguration("version").perform(context).strip().lower()
    if model_arg not in ("a", "m", "ub", "auto"):
        raise RuntimeError("model must be 'a', 'm', 'ub', or 'auto'")

    probed_model = probed_version = None
    if (model_arg == "auto" or version_arg == "auto") and probe_allowed:
        from rby1_bringup.model_probe import detect_model_and_version

        probed_model, probed_version = detect_model_and_version(robot_ip, connect_timeout_sec)

    model = (probed_model or "a") if model_arg == "auto" else model_arg
    if model == "ub":
        return model, ""
    if version_arg != "auto":
        return model, version_arg
    supported = SUPPORTED_VERSIONS.get(model, ())
    if probed_version in supported:
        return model, probed_version
    if probed_version is not None:
        print(
            f"[rby1_bringup] WARNING: probed version '{probed_version}' is not one of {supported} "
            f"for model '{model}'; using {DEFAULT_VERSION}."
        )
    return model, DEFAULT_VERSION


def _controller_names(*files):
    names = []
    for path in files:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        params = data.get("controller_manager", {}).get("ros__parameters", {})
        names += [k for k, v in params.items() if isinstance(v, dict) and "type" in v]
    return names


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_mock = _truthy(LaunchConfiguration("use_mock_hardware").perform(context))
    backend = LaunchConfiguration("backend").perform(context)
    gripper = _truthy(LaunchConfiguration("gripper").perform(context))
    try:
        connect_timeout = float(LaunchConfiguration("connect_timeout_sec").perform(context))
    except ValueError:
        connect_timeout = 3.0
    model, version = _resolve_model_and_version(
        context, robot_ip, probe_allowed=not use_mock and backend == "sdk", connect_timeout_sec=connect_timeout
    )

    xacro_file = PathJoinSubstitution([FindPackageShare("rby1_description"), "urdf", f"rby1{model}.urdf.xacro"])
    xacro_args = {
        "robot_ip": robot_ip,
        "use_mock_hardware": str(use_mock).lower(),
        "backend": backend,
        "auto_enable": LaunchConfiguration("auto_enable").perform(context),
        "command_rate": LaunchConfiguration("command_rate").perform(context),
        "state_rate": LaunchConfiguration("state_rate").perform(context),
        "impedance": LaunchConfiguration("impedance").perform(context),
        "task_space": LaunchConfiguration("task_space").perform(context),
        "gripper": str(gripper).lower(),
        "gripper_backend": LaunchConfiguration("gripper_backend").perform(context),
        "gripper_device": LaunchConfiguration("gripper_device").perform(context),
    }
    if model != "ub":
        xacro_args["version"] = version
    command = [FindExecutable(name="xacro"), " ", xacro_file]
    for key, value in xacro_args.items():
        command += [f" {key}:=", value]
    robot_description = {"robot_description": ParameterValue(Command(command), value_type=str)}

    share = get_package_share_directory("rby1_bringup")
    common_yaml = f"{share}/config/controllers_common.yaml"
    model_yaml = f"{share}/config/controllers_{model}.yaml"
    update_rate = {"update_rate": int(LaunchConfiguration("update_rate").perform(context))}

    names = [n for n in _controller_names(common_yaml, model_yaml) if n != "joint_state_broadcaster"]
    if not gripper:
        names = [n for n in names if n not in GRIPPER_CONTROLLERS]
    if not _truthy(LaunchConfiguration("task_space").perform(context)):
        names = [n for n in names if not any(k in n for k in ("cartesian", "pose_controller", "optimal_control"))]
    active = LaunchConfiguration("activate_controllers").perform(context).split()
    unknown = [n for n in active if n not in names]
    if unknown:
        raise RuntimeError(f"activate_controllers: unknown controllers {unknown}; available: {names}")
    inactive = [n for n in names if n not in active]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, common_yaml, model_yaml, update_rate],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]
    if inactive:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=inactive + ["--inactive", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )
    if active:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=active + ["--controller-manager", "/controller_manager"],
                output="screen",
            )
        )
    nodes.append(
        Node(
            package="rby1_robot_manager",
            executable="rby1_robot_manager_node",
            namespace=LaunchConfiguration("robot_namespace"),
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip,
                    "model": model,
                    "auto_reconnect": True,
                    "connect_timeout_sec": connect_timeout,
                    "status_period_sec": LaunchConfiguration("status_period_sec"),
                    "publish_status": LaunchConfiguration("publish_status"),
                    "publish_battery_state": LaunchConfiguration("publish_battery_state"),
                    "publish_tool_flange_state": LaunchConfiguration("publish_tool_flange_state"),
                }
            ],
            condition=IfCondition(
                "true"
                if _truthy(LaunchConfiguration("launch_robot_manager").perform(context))
                and not use_mock
                and backend == "sdk"
                else "false"
            ),
        )
    )
    return nodes


def generate_launch_description():
    args = [
        ("model", "auto", "RB-Y1 model: a, m, ub, or auto (detect via rby1-sdk)"),
        ("version", "auto", "Model version (a: 1.0/1.1/1.2, m: 1.0-1.3), or auto; ignored for ub"),
        ("robot_ip", "192.168.30.1:50051", "RB-Y1 gRPC address"),
        ("robot_namespace", "rby1", "Namespace of the robot manager"),
        ("use_mock_hardware", "false", "Use mock_components/GenericSystem instead of the RB-Y1 hardware"),
        ("backend", "sdk", "Hardware backend: sdk (robot or simulator core) or fake (in-memory robot)"),
        ("auto_enable", "false", "Power on, servo on and enable the control manager on activation"),
        ("update_rate", "100", "Controller manager update rate [Hz]"),
        ("command_rate", "100", "Rate of the SDK command stream [Hz]"),
        ("state_rate", "200", "Rate of the SDK state stream [Hz]"),
        ("impedance", "true", "Export stiffness/damping_ratio/torque_limit interfaces"),
        ("task_space", "true", "Export Cartesian/impedance/optimal-control gpios and load their controllers"),
        ("gripper", "false", "Load the gripper hardware (must run on the UPC for the RB gripper)"),
        ("gripper_backend", "rb_dynamixel", "Gripper backend: rb_dynamixel, robotiq_flange or fake"),
        ("gripper_device", "/dev/rby1_gripper", "Serial device of the RB gripper"),
        ("activate_controllers", "", "Space-separated controllers to activate at startup"),
        ("connect_timeout_sec", "3.0", "Connection timeout for model detection [s]"),
        ("launch_robot_manager", "true", "Start rby1_robot_manager (sdk backend only)"),
        ("status_period_sec", "0.02", "Robot manager status period [s]"),
        ("publish_status", "true", "Robot manager: publish robot_state"),
        ("publish_battery_state", "true", "Robot manager: publish battery_state"),
        ("publish_tool_flange_state", "true", "Robot manager: publish tool flange state"),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in args]
        + [OpaqueFunction(function=launch_setup)]
    )
