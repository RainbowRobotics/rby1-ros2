"""MoveIt (OMPL) plans, the RB-Y1 driver executes: move_group + target executor (+ RViz).

  ros2 launch rby1_moveit_executor moveit_executor.launch.py            # RViz included
  ros2 launch rby1_moveit_executor moveit_executor.launch.py rviz:=false

Start the RB-Y1 driver first. The robot kind and version come from the driver and
pick the matching rby1_moveit_<kind>_<version> configuration (model:=m version:=1.2
names it instead). move_group only plans -- no ros2_control -- and reads the
driver's joint states through a relay; the executor sends each planned path to
the driver's follow_joint_trajectory.

Targets: 4x4 tool poses on /rby1/right_arm/target_pose and /rby1/left_arm/target_pose,
results on /rby1/<arm>/target_status (/rby1: target_namespace:=) -- the same as the cuMotion launch (rby1_examples:
16_target_shuttle_publisher). Both arms are served. Obstacles: rby1_moveit_objects. Settings while running:
ros2 param set /rby1_target_executor duration 3.0

Only one planning stack per ROS domain: not together with the cuMotion launch or
an rby1_moveit_* demo.launch.py.

Moves the robot: on start a straight planning arm is bent to the ready pose, and
every target received is executed.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = {
    'driver_namespace': ('rby1', 'Namespace of the RB-Y1 driver'),
    'target_namespace': ('rby1', 'Namespace of the target topics: <it>/<arm>/target_pose, target_status'),
    'model': ('', 'Robot kind (m or a); empty reads it from the driver'),
    'version': ('', 'Robot version such as 1.2; empty reads it from the driver'),
    'rviz': ('true', 'Start RViz'),
    'velocity_scaling': ('0.5', 'MoveIt velocity scaling (0..1]'),
    'minimum_time': ('2.0', 'A move takes at least this many seconds'),
    'impedance': ('false', 'true: follow trajectories in joint impedance (compliant arm)'),
    'ready_pose': ('', 'File with the ready pose; empty: config/ready_pose.yaml of this package'),
}

# OMPL checks a path for collisions at states this fraction of the joint space apart.
# With its default, 0.01, a path through a 6 cm box passed the planner and was then
# refused by MoveIt's own check of the result (INVALID_MOTION_PLAN, 5 of 26 plans of
# one arm; none of 30 at 0.002). Both arms as one group span a space 1.4 times as wide,
# so its fraction is that much smaller.
CHECK_FRACTION = {'right_arm': 0.002, 'left_arm': 0.002, 'both_arms': 0.0014}


def keep_controller_plugin_loaded():
    """Environment that keeps move_group's controller plugin in memory until the process ends.

    MoveIt 2.5.9 (Humble) destroys the plugin loader before the node the plugin built
    (TrajectoryExecutionManager members), so the library is unmapped first and move_group
    segfaults on Ctrl+C in ~CallbackGroup (5 of 5 runs; preloaded: 0 of 5, clean in 0.2 s).
    """
    from ament_index_python.packages import get_package_prefix

    library = os.path.join(get_package_prefix('moveit_simple_controller_manager'), 'lib',
                           'libmoveit_simple_controller_manager.so')
    return {'LD_PRELOAD': ' '.join(filter(None, [os.environ.get('LD_PRELOAD', ''), library]))}


def config_package(kind, version):
    """('rby1_moveit_m_1_2', 'RBY1_M_v1_2') for an RB-Y1 'm' reporting version 1.2."""
    kind = kind.strip().lower()
    if kind not in ('m', 'a'):
        raise ValueError(f'Driver reports robot kind {kind!r}; expected m or a')
    if not version > 0:
        raise ValueError(f'Driver reports robot_version {version}. Drivers before the version-parsing fix '
                         'publish 0.0; rebuild the driver (colcon build --packages-select rby1_driver) and start '
                         'it again, or name the model: model:=m version:=1.2')
    tag = f'{version:.1f}'.replace('.', '_')
    return f'rby1_moveit_{kind}_{tag}', f'RBY1_{kind.upper()}_v{tag}'


def read_robot(namespace='rby1', timeout=15.0):
    """(kind, version) of the robot behind the driver, read in a context of its own.

    This runs before any node starts, so it must not touch the global rclpy context --
    nor read the launch's own arguments as ROS ones.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rcl_interfaces.srv import GetParameters
    from rby1_msgs.msg import RobotState

    context = rclpy.Context()
    rclpy.init(args=[], context=context)
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    node = rclpy.create_node('rby1_moveit_read_robot', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    seen = {}
    node.create_subscription(RobotState, f'{prefix}/robot_state',
                             lambda m: seen.setdefault('version', m.robot_version), 10)
    client = node.create_client(GetParameters, f'{prefix}/rby1_ros2_driver/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'No RB-Y1 driver answers under {prefix or "/"} -- start it '
                               '(ros2 launch rby1_driver rby1_ros2_driver.launch.py) on the same ROS_DOMAIN_ID')
        future = client.call_async(GetParameters.Request(names=['model']))
        deadline = time.monotonic() + timeout
        while not (future.done() and 'version' in seen):
            if time.monotonic() > deadline:
                raise TimeoutError(f'Driver under {prefix or "/"} did not publish robot_state in time')
            executor.spin_once(timeout_sec=0.05)
        return future.result().values[0].string_value, float(seen['version'])
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def other_move_group(timeout=2.0):
    """Name of a move_group node already on this ROS domain, or None."""
    import rclpy

    context = rclpy.Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('rby1_moveit_look_around', context=context)
    try:
        time.sleep(timeout)  # let discovery fill in the graph
        for name, namespace in node.get_node_names_and_namespaces():
            if name == 'move_group':
                return f'{namespace.rstrip("/")}/{name}'
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def setup(context):
    from moveit_configs_utils import MoveItConfigsBuilder

    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    try:
        other = other_move_group()
        if other:
            raise RuntimeError(f'{other} is already running on this ROS domain (the cuMotion launch, or an '
                               'rby1_moveit_* demo.launch.py). Stop it first: two planning stacks would '
                               "answer each other's requests")
        if value['model'] and value['version']:
            kind, version = value['model'], float(value['version'])
        else:
            kind, version = read_robot(value['driver_namespace'])
        package, robot = config_package(kind, version)
        get_package_share_directory(package)
    except Exception as error:
        return [LogInfo(msg=f'MOVEIT_EXECUTOR_FAILED: {error}'), Shutdown(reason=str(error))]

    config = (
        MoveItConfigsBuilder(robot, package_name=package)
        .robot_description(file_path=f'config/{robot}.urdf.xacro', mappings={
            # Only the <ros2_control> tag reads these, and no ros2_control runs here.
            'use_fake_hardware': 'true', 'model': kind, 'driver_namespace': value['driver_namespace']})
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    driver_states = f'/{value["driver_namespace"].strip("/")}/joint_overall_states'
    share = get_package_share_directory(package)
    ready_pose = value['ready_pose'] or os.path.join(
        get_package_share_directory('rby1_moveit_executor'), 'config', 'ready_pose.yaml')
    if not os.path.isfile(ready_pose):
        return [LogInfo(msg=f'MOVEIT_EXECUTOR_FAILED: no ready pose file {ready_pose}'),
                Shutdown(reason='no ready pose file')]
    return [
        LogInfo(msg=f'MoveIt configuration {package} for RB-Y1 {kind} {version}'),
        Node(package='rby1_moveit_executor', executable='joint_state_relay', output='screen',
             parameters=[{'source': driver_states, 'target': '/joint_states'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[config.robot_description]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             additional_env=keep_controller_plugin_loaded(),
             parameters=[config.to_dict(), {'allow_trajectory_execution': False,
                                            'publish_robot_description_semantic': True},
                         {f'ompl.{group}.longest_valid_segment_fraction': fraction
                          for group, fraction in CHECK_FRACTION.items()}],
             on_exit=Shutdown()),
        Node(package='rby1_moveit_executor', executable='moveit_executor', output='screen',
             parameters=[ready_pose,
                         {'driver_namespace': value['driver_namespace'],
                          'target_namespace': value['target_namespace'],
                          'robot_description': config.robot_description['robot_description'],
                          'velocity_scaling': float(value['velocity_scaling']),
                          'minimum_time': float(value['minimum_time']),
                          'impedance.enabled': value['impedance'] == 'true'}],
             on_exit=Shutdown()),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines,
                         config.joint_limits],
             condition=IfCondition(value['rviz'])),
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGUMENTS.items()],
        OpaqueFunction(function=setup),
    ])
