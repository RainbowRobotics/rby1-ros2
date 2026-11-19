"""Start the RB-Y1 driver and the parts you switch on below.

  ros2 launch rby1_bringup bringup.launch.py

Switch parts on and off with the True/False flags. What each part starts with is read from
CONFIG (config/bringup.yaml), one section per part. Each part below is that part's own launch
file, included with its arguments spelled out, so a block can be copied or changed as it is.
What each part is, is in the README under "Launch or Run".
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONFIG = os.path.join(get_package_share_directory('rby1_bringup'), 'config', 'bringup.yaml')

# What starts
DRIVER = True             # the RB-Y1 driver (rby1_driver)
MOVEIT_EXECUTOR = False   # the MoveIt planner that moves the arms through the driver; it takes the robot to the ready pose
MOVEIT_DEMO = True        # MoveIt with ros2_control instead of the planner (not both: one move_group per domain)
ROS2_CONTROL = False      # ros2_control with the hardware plugin, no MoveIt (not with MOVEIT_DEMO: one controller manager)
SCENE = False             # obstacles loaded into the planning scene (needs the planner or the demo)
OBJECTS = False           # modules on the robot or fixtures in the planning scene (needs the planner or the demo)
CAMERA = False            # the camera publisher (rby1_tools)
GRIPPER = True            # the gripper on the hands: model on TF, parts in the scene, its driver, virtual or real (rby1_tools)

# A launch evaluates every included launch file before it starts any process. The planner's
# launch looks the driver up while it is evaluated, so it is evaluated only after the driver has
# had this long to start; the scene and the objects load once move_group is up.
DRIVER_START_S = 3.0
MOVE_GROUP_START_S = 5.0


def launch_file(package, name):
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', name))


def config_dir(package):
    return os.path.join(get_package_share_directory(package), 'config')


def check(config):
    """Mistakes that are cheaper to find here than in the processes' logs: errors stop the
    launch with one line; warnings are printed and the launch goes on."""
    warnings = []
    for name in ('driver', 'moveit_executor', 'moveit_demo', 'ros2_control', 'scene', 'objects', 'camera', 'gripper'):
        if name not in config:
            raise RuntimeError(f"{CONFIG} has no '{name}:' section")
    driver = config['driver']
    if driver['model'] not in ('', 'a', 'm', 'A', 'M'):
        raise RuntimeError(f"driver.model must be a or m (or empty), not '{driver['model']}'")
    if driver['robot_ip'] and ':' not in driver['robot_ip']:
        raise RuntimeError(f"driver.robot_ip needs host:port, e.g. 192.168.30.1:50051, not '{driver['robot_ip']}'")
    if MOVEIT_EXECUTOR and MOVEIT_DEMO:
        raise RuntimeError('MOVEIT_EXECUTOR and MOVEIT_DEMO cannot run together: one move_group per ROS domain')
    if ROS2_CONTROL and MOVEIT_DEMO:
        raise RuntimeError('ROS2_CONTROL and MOVEIT_DEMO cannot run together: both start a controller manager '
                           'on the robot')
    if MOVEIT_DEMO:
        package = config['moveit_demo']['package']
        try:
            get_package_share_directory(package)
        except Exception:
            raise RuntimeError(f"moveit_demo.package '{package}' is not built or installed; "
                               'it is rby1_moveit_{a,m}_{1_0,1_1,1_2,1_3}') from None
        kind = package.split('_')[2] if package.startswith('rby1_moveit_') else ''
        if driver['model'] and kind and kind != driver['model'].lower():
            raise RuntimeError(f"moveit_demo.package '{package}' is for model {kind}, "
                               f"but driver.model is {driver['model']}")
    version = str(config['moveit_executor']['version'])
    if ROS2_CONTROL and not (driver['model'] and version):
        raise RuntimeError('ROS2_CONTROL needs the robot named: driver.model (a or m) and moveit_executor.version '
                           '(such as 1.2) in ' + CONFIG)
    has_gripper_in_model = version.startswith('1.3')   # RB-Y1 v1.3 carries its gripper in its URDF
    if has_gripper_in_model and OBJECTS and 'gripper' in str(config['objects']['config']):
        raise RuntimeError('RB-Y1 v1.3 has the gripper in its robot model: objects.config must not be '
                           'gripper.yaml (it would put a second gripper on the hands)')
    gripper = config['gripper']
    if has_gripper_in_model and GRIPPER and (gripper['model'] or gripper['scene']):
        raise RuntimeError("RB-Y1 v1.3 has the gripper in its robot model: set gripper.model and gripper.scene "
                           'false (its controller alone feeds the finger joints)')
    if has_gripper_in_model and not GRIPPER:
        warnings.append("RB-Y1 v1.3's finger joints get their joint states from the gripper controller: "
                        'switch GRIPPER on, or TF and MoveIt miss gripper_finger_*_joint')
    if str(gripper['virtual']).lower() not in ('', 'true', 'false'):
        raise RuntimeError(f"gripper.virtual is true or false (or \"\": its settings'), not '{gripper['virtual']}'")
    if (SCENE or OBJECTS or (GRIPPER and gripper['scene'])) and not (MOVEIT_EXECUTOR or MOVEIT_DEMO):
        warnings.append('SCENE/OBJECTS/gripper.scene need a move_group; none starts here, so one must '
                        'already be running on this ROS domain (they wait for it)')
    if OBJECTS and 'gripper' in str(config['objects']['config']) and not GRIPPER:
        warnings.append("objects.config is the gripper, but GRIPPER is off: its fingers stay where "
                        'config/gripper.yaml puts them')
    if not (DRIVER or MOVEIT_EXECUTOR or MOVEIT_DEMO or ROS2_CONTROL or SCENE or OBJECTS or CAMERA or GRIPPER):
        raise RuntimeError('nothing to start: every part is off')
    return warnings


def generate_launch_description():
    if not os.path.isfile(CONFIG):
        raise RuntimeError(f'no config file {CONFIG}')
    with open(CONFIG) as stream:
        config = yaml.safe_load(stream)
    parts = [LogInfo(msg=f'rby1_bringup: {warning}') for warning in check(config)]
    driver = config['driver']

    # ---- the driver --------------------------------------------------------------------
    # Launch configurations are global to a launch, so every argument is passed explicitly:
    # two included launch files that both take `config` would otherwise read each other's.
    if DRIVER:
        parts.append(IncludeLaunchDescription(
            launch_file('rby1_driver', 'rby1_ros2_driver.launch.py'),
            launch_arguments={
                'params_file': driver['params_file'],   # '': rby1_driver/config/driver_parameters.yaml
                'namespace': driver['namespace'],
                'robot_ip': driver['robot_ip'],         # '': the params file's
                'model': driver['model'],               # '': the params file's
            }.items()))

    # ---- MoveIt: the planner, or MoveIt with ros2_control ----------------------------------
    moveit = None
    if MOVEIT_EXECUTOR:
        executor = config['moveit_executor']
        moveit = IncludeLaunchDescription(
            launch_file('rby1_moveit_executor', 'moveit_executor.launch.py'),
            launch_arguments={
                'driver_namespace': driver['namespace'],
                'target_namespace': executor['target_namespace'],
                'model': driver['model'],               # '': read from the driver
                'version': executor['version'],         # '': read from the driver
                'rviz': str(executor['rviz']),
                'velocity_scaling': str(executor['velocity_scaling']),
                'minimum_time': str(executor['minimum_time']),
                'impedance': str(executor['impedance']),
                'ready_pose': executor['ready_pose'] or os.path.join(
                    config_dir('rby1_moveit_executor'), 'ready_pose.yaml'),
            }.items())
    if MOVEIT_DEMO:
        demo = config['moveit_demo']
        moveit = IncludeLaunchDescription(
            launch_file(demo['package'], 'demo.launch.py'),  # rby1_moveit_{a,m}_{1_0,...}
            launch_arguments={
                'use_fake_hardware': str(demo['use_fake_hardware']),
                'robot_ip': driver['robot_ip'] or '127.0.0.1:50051',
                'model': driver['model'] or 'm',
                'driver_namespace': driver['namespace'],
            }.items())
    if moveit is not None:
        if DRIVER:
            parts.append(TimerAction(period=DRIVER_START_S, actions=[moveit]))
        else:
            parts.append(moveit)   # the driver is already running elsewhere

    # ---- ros2_control without MoveIt --------------------------------------------------------
    # Which controllers start is set at the top of rby1_hardware/launch/rby1_hardware.launch.py.
    if ROS2_CONTROL:
        ros2_control = IncludeLaunchDescription(
            launch_file('rby1_hardware', 'rby1_hardware.launch.py'),
            launch_arguments={
                'use_fake_hardware': str(config['ros2_control']['use_fake_hardware']),
                'robot_ip': driver['robot_ip'] or '127.0.0.1:50051',
                'driver_namespace': driver['namespace'],
                'model': driver['model'],
                'version': str(config['moveit_executor']['version']),
            }.items())
        if DRIVER:
            # the hardware plugin asks the driver for the control right when it activates
            parts.append(TimerAction(period=DRIVER_START_S, actions=[ros2_control]))
        else:
            parts.append(ros2_control)   # the driver is already running elsewhere

    # ---- the planning scene: obstacles, and modules on the robot ----------------------------
    scene = []
    if SCENE:
        scene.append(IncludeLaunchDescription(
            launch_file('rby1_moveit_objects', 'scene.launch.py'),
            launch_arguments={
                'config': config['scene']['config'] or os.path.join(
                    config_dir('rby1_moveit_objects'), 'example_scene.yaml'),
            }.items()))
    if OBJECTS:
        scene.append(IncludeLaunchDescription(
            launch_file('rby1_moveit_objects', 'objects.launch.py'),
            launch_arguments={
                'config': config['objects']['config'] or os.path.join(
                    config_dir('rby1_moveit_objects'), 'objects.yaml'),
            }.items()))
    if scene and moveit is not None:
        # once move_group has started (and had MOVE_GROUP_START_S to come up)
        parts.append(RegisterEventHandler(OnProcessStart(
            target_action=lambda action: (getattr(action, 'process_details', None) or {})
            .get('name', '').startswith('move_group'),
            on_start=[TimerAction(period=MOVE_GROUP_START_S, actions=scene)])))
    elif scene:
        parts += scene   # a move_group is already running elsewhere

    # ---- tools: camera, virtual gripper ----------------------------------------------------
    if CAMERA:
        camera = config['camera']
        parts.append(IncludeLaunchDescription(
            launch_file('rby1_tools', 'camera.launch.py'),
            launch_arguments={
                'camera': camera['camera'],             # the kind of camera: realsense
                'camera_name': camera['camera_name'],
                'config': camera['config'] or os.path.join(
                    config_dir('rby1_tools'), 'camera', f"{camera['camera']}.yaml"),
                'mount': camera['mount'],               # a name in its config/camera/camera_mount.yaml; none: no link
            }.items()))
    if GRIPPER:
        gripper = config['gripper']
        version = str(config['moveit_executor']['version'])
        gripper_launch = IncludeLaunchDescription(
            launch_file('rby1_tools', 'gripper.launch.py'),
            launch_arguments={
                'model': str(gripper['model']),
                'scene': str(gripper['scene']),
                'virtual': str(gripper['virtual']).lower(),     # true, false; '': its settings'
                'hands': gripper['hands'],
                'mount': gripper['mount'],                      # '': its config/gripper/gripper_mount.yaml
                'config': gripper['config'],                    # '': the driver's default settings
                'driver_namespace': driver['namespace'],
                # v1.3: the robot URDF's finger joints take their state from the driver
                'joint_states_topic': '/joint_states' if version.startswith('1.3') else '',
            }.items())
        if gripper['scene'] and moveit is not None:
            # its parts go into the planning scene: the whole gripper launch waits for move_group
            parts.append(RegisterEventHandler(OnProcessStart(
                target_action=lambda action: (getattr(action, 'process_details', None) or {})
                .get('name', '').startswith('move_group'),
                on_start=[TimerAction(period=MOVE_GROUP_START_S, actions=[gripper_launch])])))
        else:
            parts.append(gripper_launch)

    return LaunchDescription(parts)
