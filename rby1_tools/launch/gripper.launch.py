"""The RB-Y1 gripper on the hands: its model on TF, its parts in the planning scene, and its
driver, virtual or real.

  ros2 launch rby1_tools gripper.launch.py                                 # model on TF, virtual driver
  ros2 launch rby1_tools gripper.launch.py scene:=true                     # and its parts in the planning scene
  ros2 launch rby1_tools gripper.launch.py virtual:=false                  # the gripper itself (on the UPC)
  ros2 launch rby1_tools gripper.launch.py hands:=right                    # one hand only
  ros2 launch rby1_tools gripper.launch.py model:=false joint_states_topic:=/joint_states   # RB-Y1 v1.3
  ros2 topic pub --once /rby1/right_gripper/grasp std_msgs/msg/Bool "{data: true}"          # close
  ros2 topic pub --once /rby1/right_gripper/command std_msgs/msg/Float64 "{data: 0.02}"    # each finger 2 cm open

What starts, by flag:
  model       a robot_state_publisher per hand with model/rby1_gripper/gripper.urdf.xacro, mounted
              on the robot's tool frame as config/gripper/gripper_mount.yaml says: the gripper's frames
              (gripper_<hand>, gripper_finger_<r|l>1, _2) on TF under ee_<hand>, and its meshes
              for RViz (RobotModel display, topic /gripper/<hand>/robot_description). Not for an
              RB-Y1 v1.3, whose URDF carries the gripper already.
  scene       the gripper's parts in the MoveIt planning scene, from model/rby1_gripper/gripper.yaml
              with the same mount applied (rby1_moveit_objects' objects.launch.py; needs a move_group).
Always: gripper_driver (config/gripper/gripper_driver.yaml), which takes /rby1/<hand>_gripper/grasp
(true closes) and /command (each finger's opening, m) and publishes /rby1/<hand>_gripper/state;
the finger joints of the model get their states on `joint_states_topic`. virtual (the
settings' `virtual`, or this argument): nothing moves, the fingers are reported going where they
are told; false: the gripper itself through the SDK's Dynamixel bus (built where the SDK is; the
fingers move at start, homing). To keep the gripper on the UPC, run this launch there.

The files: `urdf`, `mount`, `objects`, `config` (the driver's settings) are paths; empty
means the one of this package named above.
"""

import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = {
    'model': ('true', "the gripper's model on TF and for RViz (not for an RB-Y1 v1.3)"),
    'scene': ('false', "the gripper's parts in the MoveIt planning scene (needs a move_group)"),
    'virtual': ('', "true: nothing moves; false: the gripper itself; empty: the settings' virtual"),
    'hands': ('right,left', 'right, left, or both'),
    'joint_states_topic': ('', "where the finger joints' states go; empty: /gripper/joint_states when model is on "
                               '(RB-Y1 v1.3: /joint_states), else nowhere'),
    'urdf': ('', "the gripper's URDF xacro; empty: model/rby1_gripper/gripper.urdf.xacro"),
    'mount': ('', 'where each gripper sits on the robot; empty: config/gripper/gripper_mount.yaml'),
    'objects': ('', "the gripper's parts for the planning scene; empty: model/rby1_gripper/gripper.yaml"),
    'config': ('', "the driver's settings; empty: config/gripper/gripper_driver.yaml"),
    'driver_namespace': ('rby1', "namespace of the running RB-Y1 driver (virtual false: its tool_flange_power switches "
                                 "the motors' 12 V on)"),
}


def rotation(rpy):
    """The 3x3 rotation of roll, pitch, yaw (Rz Ry Rx), as rows."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rpy_of(m):
    """Roll, pitch, yaw of a rotation matrix (the inverse of rotation())."""
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    return [math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mounted_objects(objects_path, mount, hands):
    """The parts of `objects_path` for `hands`, each moved by its hand's mount; the path
    of the file holding them (the original when every mount is identity)."""
    with open(objects_path) as stream:
        data = yaml.safe_load(stream)
    parents = {mount[hand]['parent']: hand for hand in hands}
    kept = []
    moved = False
    for obj in data.get('objects', []):
        hand = parents.get(obj.get('frame'))
        if hand is None:
            continue
        m = mount[hand]
        if any(float(v) != 0.0 for v in list(m['xyz']) + list(m['rpy'])):
            moved = True
            rot = rotation([float(v) for v in m['rpy']])
            xyz = apply(rot, [float(v) for v in obj.get('xyz', [0.0, 0.0, 0.0])])
            obj['xyz'] = [xyz[i] + float(m['xyz'][i]) for i in range(3)]
            obj['rpy'] = rpy_of(multiply(rot, rotation([float(v) for v in obj.get('rpy', [0.0, 0.0, 0.0])])))
            if 'follows' in obj:
                obj['follows']['axis'] = apply(rot, [float(v) for v in obj['follows']['axis']])
        kept.append(obj)
    if not moved and len(kept) == len(data.get('objects', [])):
        return objects_path
    folder = os.path.dirname(os.path.abspath(objects_path))
    for obj in kept:   # the meshes are relative to the original file
        mesh = obj.get('mesh')
        if mesh and '://' not in mesh and not os.path.isabs(mesh):
            obj['mesh'] = os.path.join(folder, mesh)
    out = tempfile.NamedTemporaryFile('w', prefix='gripper_objects_', suffix='.yaml', delete=False)
    yaml.safe_dump({'objects': kept}, out)
    out.close()
    return out.name


def setup(context):
    share = get_package_share_directory('rby1_tools')
    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    on = {name: value[name].lower() in ('true', '1') for name in ('model', 'scene')}
    hands = [h.strip() for h in value['hands'].split(',') if h.strip()]
    for hand in hands:
        if hand not in ('right', 'left'):
            raise RuntimeError(f"hands are right and/or left, not '{hand}'")
    if value['virtual'].lower() not in ('', 'true', 'false'):
        raise RuntimeError(f"virtual is true or false (or empty: the settings'), not '{value['virtual']}'")
    urdf = value['urdf'] or os.path.join(share, 'model', 'rby1_gripper', 'gripper.urdf.xacro')
    mount_path = value['mount'] or os.path.join(share, 'config', 'gripper', 'gripper_mount.yaml')
    objects_path = value['objects'] or os.path.join(share, 'model', 'rby1_gripper', 'gripper.yaml')
    with open(mount_path) as stream:
        mount = yaml.safe_load(stream)
    for hand in hands:
        for key in ('parent', 'xyz', 'rpy'):
            if hand not in mount or key not in mount[hand]:
                raise RuntimeError(f'{mount_path} needs {hand}: {{parent, xyz, rpy}}')
    joint_states_topic = value['joint_states_topic'] or ('/gripper/joint_states' if on['model'] else '')

    nodes = []
    if on['model']:
        for hand in hands:
            m = mount[hand]
            description = xacro.process_file(urdf, mappings={
                'hand': hand, 'parent': str(m['parent']),
                'xyz': ' '.join(str(float(v)) for v in m['xyz']),
                'rpy': ' '.join(str(float(v)) for v in m['rpy'])}).toxml()
            nodes.append(Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                namespace=f'gripper/{hand}', output='log',
                parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
                remappings=[('joint_states', joint_states_topic)]))
    if on['scene']:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rby1_moveit_objects'), 'launch', 'objects.launch.py')),
            launch_arguments={'config': mounted_objects(objects_path, mount, hands)}.items()))
    overrides = {'hands': hands, 'joint_states_topic': joint_states_topic,
                 'driver_namespace': value['driver_namespace']}
    if value['virtual']:
        overrides['virtual'] = value['virtual'].lower() == 'true'
    nodes.append(Node(
        package='rby1_tools', executable='gripper_driver', name='gripper_driver', output='screen',
        parameters=[value['config'] or os.path.join(share, 'config', 'gripper', 'gripper_driver.yaml'), overrides]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGUMENTS.items()],
        OpaqueFunction(function=setup),
    ])
