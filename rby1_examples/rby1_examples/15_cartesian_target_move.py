#!/usr/bin/env python3
"""
Cartesian Target Move Example
=============================
The simplest target executor: the driver alone, no planner, nothing is checked for
collisions. It listens for hand targets on a topic and takes the hand straight to
each one with the driver's own Cartesian commands, until Ctrl+C.

A target is one message on `target_topic`: 16 values, the row-major 4x4 pose of
the arm's tool frame in `ref_link`. The answers go out on `status_topic`:

  READY                             once, when it is ready for targets
  EXECUTING mode=... distance=...   a target is started
  DONE                              the hand is within `tolerance` of the target
  FAILED: <reason and what to do>   the move failed, or the hand ended further away
  FAILED: rejected target: <why>    the message is not 16 finite values with a
                                    rotation and the bottom row 0 0 0 1

These are the topics of the target executors that plan around obstacles
(rby1_moveit_executor, and the cuMotion executor of the RB-Y1 Isaac ROS
repository), so whatever sends targets to them works with this example too. Only
one of them may serve an arm: if `target_topic` already has a subscriber when the
example starts, another executor is running and both would move the arm, so the
example refuses to run.

A target that arrives while the hand is moving waits for the move to end. Only
the latest one is kept.

How a target is carried out is chosen with `mode`:

  mode:=command  (default) One robot_cartesian action goal, as in 08_cartesian_command:
                 the robot's Cartesian controller takes the hand there in `duration` s.
  mode:=stream   The driver's stream_cartesian, a small step at a time at `rate` Hz.
                 The steps speed up to `max_speed`, slow down before the target, and
                 never run more than `lead` ahead of where the hand is measured: the
                 driver refuses a step that asks a joint for more than its
                 acceleration limit and holds the arm there, which is what a target
                 far from the hand does. Opens the arm's stream channel for a move
                 and closes it again after it, also on Ctrl+C.

The tool frame follows from the arm `target_topic` names: ee_right for
/rby1/right_arm/..., ee_left for /rby1/left_arm/...; `target_link` names another.

Sequence:
  1. Wait a moment for ROS discovery, check that nothing else listens on
     `target_topic`, read the hand from the driver: READY.
  2. For every target: EXECUTING. command: send it, wait for the action.
     stream: step toward it until the hand is there.
  3. Read the hand again: DONE within `tolerance` of the target, else FAILED.

Watch: READY, then for every target EXECUTING ... and DONE with "hand is 0.4 mm
from the target" under it, or FAILED: <reason>.

Try it with example 16, which publishes two poses near the right hand's ready pose
in turn, each in its own terminal:
  terminal 1:  ros2 launch rby1_driver rby1_ros2_driver.launch.py
  terminal 2:  ros2 run rby1_examples 15_cartesian_target_move
  terminal 3:  ros2 run rby1_examples 16_target_shuttle_publisher
  terminal 4:  ros2 topic echo /rby1/right_arm/target_status std_msgs/msg/String

Before: the RB-Y1 driver, the robot powered and its servos on, the arm bent
(ros2 run rby1_examples 06_zero_pose turns it on; a target executor's launch bends
the arm to the ready pose -- stop that launch before starting this example). To go
around obstacles use one of those executors instead of this example.

Run:
  ros2 run rby1_examples 15_cartesian_target_move
  ros2 run rby1_examples 15_cartesian_target_move --ros-args -p mode:=stream
  ros2 run rby1_examples 15_cartesian_target_move --ros-args -p duration:=5.0
  ros2 run rby1_examples 15_cartesian_target_move --ros-args \
      -p target_topic:=/rby1/left_arm/target_pose -p status_topic:=/rby1/left_arm/target_status

Parameters (defaults):
  mode             command               command | stream
  target_topic     /rby1/right_arm/target_pose     targets come in here
  status_topic     /rby1/right_arm/target_status   the answers go out here
  target_link      (from target_topic)   the tool frame: ee_right, or ee_left for the left arm's topic
  ref_link         base                  the frame of the targets
  arm_base_link    link_torso_5          the commands are sent in this frame, so only arm joints turn
  duration         3.0 s                 command: how long the move takes
  rate             20.0 Hz               stream: steps per second
  max_speed        0.10 m/s              stream: of the hand
  max_turn_speed   0.5 rad/s             stream: of the hand's orientation
  acceleration     0.2 m/s^2             stream: how fast the steps speed up and slow down
  lead             0.03 m                stream: how far a step may be ahead of the measured hand
  tolerance        0.005 m               how far from the target the hand may end
  timeout          30.0 s                for a move
Write every number as a float (5.0, not 5).

Topics subscribed: <target_topic>  (std_msgs/Float64MultiArray, 16 values, row-major 4x4 in ref_link)
Topics published:  <status_topic>  (std_msgs/String)
Actions used:  /rby1/robot_cartesian (Rby1CartesianCommand), /rby1/stream_cartesian (StreamCartesian)
Services used: /rby1/get_cartesian_pose (GetCartesianPose), /rby1/stream_control (StateOnOff, mode:=stream)
Moves the robot.

The driver answers get_cartesian_pose from its own model. On RB-Y1 v1.1 to v1.3
it matches the URDF exactly; on v1.0 the ee_* tool frames of the two differ by 46 mm.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rby1_msgs.action import Rby1CartesianCommand, StreamCartesian
from rby1_msgs.srv import GetCartesianPose, StateOnOff
from std_msgs.msg import Float64MultiArray, String

MODES = ('command', 'stream')
TOOL_FRAMES = {'right_arm': 'ee_right', 'left_arm': 'ee_left'}

# Values to change: edit these to adjust the example.
STEP_TIME = 0.2       # s the driver gives the arm for each stream step (its minimum_time)
DISCOVERY_TIME = 1.0  # s ROS discovery gets before the check for another executor on the topic
# Defaults of the parameters of the same name; the table above describes them.
MODE = 'command'                # one of MODES
TARGET_TOPIC = '/rby1/right_arm/target_pose'
STATUS_TOPIC = '/rby1/right_arm/target_status'
TARGET_LINK = ''                # empty: the tool frame of the arm TARGET_TOPIC names
REF_LINK = 'base'
ARM_BASE_LINK = 'link_torso_5'
DURATION = 3.0                  # s
RATE = 20.0                     # Hz
MAX_SPEED = 0.10                # m/s
MAX_TURN_SPEED = 0.5            # rad/s
ACCELERATION = 0.2              # m/s^2
LEAD = 0.03                     # m
TOLERANCE = 0.005               # m
TIMEOUT = 30.0                  # s


def transform_to_matrix(translation, rotation):
    """A geometry_msgs Transform's parts as a 4x4. rotation is (x, y, z, w)."""
    x, y, z, w = rotation
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError(f'Driver returned a non-unit quaternion (norm {norm:.4f}); '
                         'is the link name right? get_cartesian_pose answers with an empty '
                         'transform for links it does not know.')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = translation
    return matrix


def quaternion_of(rotation):
    """(x, y, z, w) of a 3x3 rotation."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
    q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return tuple(q)


def tool_frame(topic):
    """The tool frame of the arm a target topic names, or ValueError."""
    for arm, link in TOOL_FRAMES.items():
        if f'/{arm}/' in topic:
            return link
    raise ValueError(f'target_topic {topic} names neither right_arm nor left_arm: give target_link')


def check_matrix(values, tolerance=1e-3):
    """16 row-major values forming a rigid transform, or ValueError saying why."""
    matrix = np.asarray(values, dtype=float)
    if matrix.size != 16:
        raise ValueError(f'matrix needs 16 values (row-major 4x4), got {matrix.size}')
    matrix = matrix.reshape(4, 4)
    if not np.isfinite(matrix).all():
        raise ValueError('matrix contains non-finite values')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=tolerance):
        raise ValueError('matrix bottom row must be [0 0 0 1] -- is it transposed? '
                         'Values are read row-major.')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance) or np.linalg.det(rotation) < 0:
        raise ValueError('matrix rotation is not a proper rotation')
    return matrix


def turn_between(start, end):
    """(axis, angle) of the rotation that takes the 3x3 `start` to `end`."""
    x, y, z, w = quaternion_of(end @ start.T)
    angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))
    axis = np.array([x, y, z]) * (1.0 if w >= 0 else -1.0)
    norm = np.linalg.norm(axis)
    return (axis / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])), angle


def turned(start, axis, angle):
    """The 3x3 `start` turned by `angle` about `axis` (both in the reference frame)."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return (np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)) @ start


def next_progress(progress, speed, top_speed, acceleration, dt, reached, lead):
    """One step of how far along the path (0..1) to command, and how fast.

    Speeds up by `acceleration`, never above `top_speed`, slows so as to stop at 1,
    and stays within `lead` of `reached`, how far along the hand is measured: a
    command that runs ahead of the arm is what the driver refuses.
    """
    speed = min(speed + acceleration * dt, top_speed, math.sqrt(2.0 * acceleration * max(1.0 - progress, 0.0)))
    ahead = min(progress + speed * dt, 1.0, reached + lead)
    ahead = max(ahead, progress)  # never back
    return ahead, (ahead - progress) / dt


class CartesianTargetMove(Node):
    def __init__(self):
        super().__init__('cartesian_target_move_example', namespace='rby1')
        for name, default in {
            'mode': MODE, 'target_topic': TARGET_TOPIC, 'status_topic': STATUS_TOPIC,
            'ref_link': REF_LINK, 'target_link': TARGET_LINK, 'arm_base_link': ARM_BASE_LINK,
            'duration': DURATION, 'rate': RATE, 'max_speed': MAX_SPEED, 'max_turn_speed': MAX_TURN_SPEED,
            'acceleration': ACCELERATION, 'lead': LEAD, 'tolerance': TOLERANCE, 'timeout': TIMEOUT,
        }.items():
            self.declare_parameter(name, default)
        if self.param('mode') not in MODES:
            raise ValueError(f'mode must be one of {", ".join(MODES)}, got {self.param("mode")!r}')
        for name in ('duration', 'rate', 'max_speed', 'max_turn_speed', 'acceleration', 'lead', 'tolerance'):
            if not self.param(name) > 0.0:
                raise ValueError(f'{name} must be positive')
        if not self.param('target_link'):
            self.set_parameters([Parameter('target_link', value=tool_frame(self.param('target_topic')))])
        self.arm = 'left_arm' if 'left' in self.param('target_link') else 'right_arm'
        self.pose_client = self.create_client(GetCartesianPose, 'get_cartesian_pose')
        self.stream_client = self.create_client(StateOnOff, 'stream_control')
        self.command_action = ActionClient(self, Rby1CartesianCommand, 'robot_cartesian')
        self.stream_action = ActionClient(self, StreamCartesian, 'stream_cartesian')
        self.status = self.create_publisher(String, self.param('status_topic'), 10)
        self.pending = None  # the latest target not yet started, a 4x4 in ref_link
        self.busy = False
        self.move = 0        # counts stream moves: a step answered after its move ended is not this move's
        self.refused = None  # why the driver dropped a stream step

    def param(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def on_target(self, msg):
        try:
            target = check_matrix(msg.data)
        except ValueError as error:
            self.report(f'FAILED: rejected target: {error}')
            return
        if self.busy:
            self.get_logger().info('new target: it runs when the current move ends')
        self.pending = target

    def wait(self, condition, timeout, what):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError(f'Timed out waiting for {what}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self, ref_link, target_link):
        """`target_link` in `ref_link` now, as a 4x4, from the driver."""
        if not self.pose_client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError('get_cartesian_pose unavailable -- is the RB-Y1 driver running?')
        future = self.pose_client.call_async(GetCartesianPose.Request(ref_link=ref_link, target_link=target_link))
        self.wait(future.done, 10.0, 'get_cartesian_pose')
        transform = future.result().transform
        return transform_to_matrix(
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))

    def hand(self):
        """The tool frame in the arm's base frame now."""
        return self.pose(self.param('arm_base_link'), self.param('target_link'))

    def cartesian(self, command, matrix, minimum_time):
        """Fill a CartesianCommand: the tool frame at `matrix` in the arm's base frame."""
        command.ref_link, command.target_link = self.param('arm_base_link'), self.param('target_link')
        command.minimum_time = float(minimum_time)
        t = command.transform
        t.translation.x, t.translation.y, t.translation.z = (float(v) for v in matrix[:3, 3])
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = (float(v) for v in quaternion_of(matrix[:3, :3]))

    def by_command(self, target):
        if not self.command_action.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('robot_cartesian unavailable -- is the RB-Y1 driver running?')
        goal = Rby1CartesianCommand.Goal()
        self.cartesian(getattr(goal, self.arm), target, self.param('duration'))
        sent = self.command_action.send_goal_async(goal)
        self.wait(sent.done, 10.0, 'robot_cartesian to take the goal')
        if not sent.result().accepted:
            raise RuntimeError('the driver rejected the goal: a trajectory is running on the arm, or hardware '
                               'control is active (the driver log says which)')
        outcome = sent.result().get_result_async()
        self.wait(outcome.done, self.param('duration') + self.param('timeout'), 'robot_cartesian to finish')
        result = outcome.result().result
        if not result.success:
            raise RuntimeError(f'robot_cartesian ended with {result.finish_code}: see the driver log, and check '
                               'that the target is within the arm\'s reach')

    def switch_stream(self, on):
        if not self.stream_client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError('stream_control unavailable -- is the RB-Y1 driver running?')
        future = self.stream_client.call_async(StateOnOff.Request(state=on, parameters=self.arm))
        self.wait(future.done, 10.0, 'stream_control')
        if not future.result().success:
            raise RuntimeError(f'stream_control: {future.result().message}')
        self.get_logger().info(future.result().message)
        return future.result().message

    def send_step(self, matrix):
        goal = StreamCartesian.Goal()
        self.cartesian(getattr(goal.command, self.arm), matrix, STEP_TIME)
        move = self.move

        def accepted(future):
            handle = future.result()
            if not handle.accepted:
                if move == self.move:
                    self.refused = 'the driver rejected a step: hardware control is active'
                return
            handle.get_result_async().add_done_callback(finished)

        def finished(future):
            if move == self.move and not future.result().result.success:
                self.refused = ('the driver dropped a step: it asked a joint for more than its acceleration limit '
                                '(the driver then holds the arm where it is), a trajectory holds the arm, or the '
                                'stream closed -- the driver log says which')
        self.stream_action.send_goal_async(goal).add_done_callback(accepted)

    def by_stream(self, target):
        if not self.stream_action.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('stream_cartesian unavailable -- is the RB-Y1 driver running?')
        self.move += 1
        self.refused = None
        start = self.hand()
        distance = float(np.linalg.norm(target[:3, 3] - start[:3, 3]))
        axis, angle = turn_between(start[:3, :3], target[:3, :3])
        # Progress 0..1 along the straight line and the turn at once; the slower of the two sets the pace.
        scale = max(distance / self.param('max_speed'), angle / self.param('max_turn_speed'), 1e-6)
        top_speed = 1.0 / scale
        acceleration = self.param('acceleration') / max(distance, 1e-6) if distance > 1e-6 else top_speed
        lead = self.param('lead') / distance if distance > 1e-6 else 1.0
        dt = 1.0 / self.param('rate')

        def at(progress):
            matrix = np.eye(4)
            matrix[:3, 3] = start[:3, 3] + (target[:3, 3] - start[:3, 3]) * progress
            matrix[:3, :3] = turned(start[:3, :3], axis, angle * progress)
            return matrix

        opened = self.switch_stream(True).startswith('Stream channels opened')
        try:
            progress, speed, reached = 0.0, 0.0, 0.0
            pending = None
            deadline = time.monotonic() + self.param('timeout')
            tick = time.monotonic()
            while True:
                if self.refused:
                    raise RuntimeError(self.refused)
                if time.monotonic() > deadline:
                    raise TimeoutError(f'the hand is still {self.off(target) * 1000:.1f} mm from the target after '
                                       f'{self.param("timeout"):.0f} s: is the target within the arm\'s reach?')
                if pending is None:
                    pending = self.pose_client.call_async(GetCartesianPose.Request(
                        ref_link=self.param('arm_base_link'), target_link=self.param('target_link')))
                elif pending.done():
                    t = pending.result().transform.translation
                    measured = np.array([t.x, t.y, t.z])
                    pending = None
                    if distance > 1e-6:
                        along = (measured - start[:3, 3]) @ (target[:3, 3] - start[:3, 3]) / distance ** 2
                        reached = min(max(along, 0.0), 1.0)
                    else:
                        reached = progress
                    if progress >= 1.0 and np.linalg.norm(measured - target[:3, 3]) <= self.param('tolerance'):
                        return
                progress, speed = next_progress(progress, speed, top_speed, acceleration, dt, reached, lead)
                self.send_step(at(progress))
                tick += dt
                while time.monotonic() < tick:
                    rclpy.spin_once(self, timeout_sec=max(0.0, min(0.005, tick - time.monotonic())))
        finally:
            if opened:
                try:
                    self.switch_stream(False)
                except Exception as error:  # the driver may have closed it already
                    self.get_logger().warn(f'stream_control off: {error}')

    def off(self, target):
        """How far the hand is from the target's position now (m)."""
        return float(np.linalg.norm(self.hand()[:3, 3] - target[:3, 3]))

    def start(self):
        """Refuse to be a second executor on the topic, see that the driver answers, then listen."""
        topic = self.param('target_topic')
        settled = time.monotonic() + DISCOVERY_TIME
        while time.monotonic() < settled:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.count_subscribers(topic) > 0:
            raise RuntimeError(f'{topic} already has a subscriber: another target executor is running, and both '
                               'would move the arm. Stop one of them (a "ros2 topic echo" of that topic counts '
                               'too), then start this example again.')
        self.hand()  # fails here without the driver, or with a target_link it does not know
        self.create_subscription(Float64MultiArray, topic, self.on_target, 10)
        self.report('READY')

    def execute(self, target):
        """Take the hand to `target` (the tool frame in ref_link) and answer DONE or FAILED."""
        try:
            if self.param('ref_link') != self.param('arm_base_link'):
                target = self.pose(self.param('arm_base_link'), self.param('ref_link')) @ target
            distance = self.off(target)
            if self.param('mode') == 'command':
                self.report(f'EXECUTING mode=command distance={distance:.3f}m duration={self.param("duration"):.1f}s')
                self.by_command(target)
            else:
                self.report(f'EXECUTING mode=stream distance={distance:.3f}m '
                            f'max_speed={self.param("max_speed"):.2f}m/s')
                self.by_stream(target)
            time.sleep(0.5)  # the arm settles
            off = self.off(target)
            if off > self.param('tolerance'):
                raise RuntimeError(f'the hand ended {off * 1000:.1f} mm from the target '
                                   f'(tolerance {self.param("tolerance") * 1000:.0f} mm): see the driver log, and '
                                   'check that the target is within the arm\'s reach')
        except (RuntimeError, TimeoutError, ValueError) as error:
            self.report(f'FAILED: {error}')
            return
        self.report('DONE')
        self.get_logger().info(f'hand is {off * 1000:.1f} mm from the target')

    def serve(self):
        """Run the targets as they come, one at a time, until Ctrl+C."""
        while rclpy.ok():
            if self.pending is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            target, self.pending = self.pending, None
            self.busy = True
            self.execute(target)
            self.busy = False


def main(args=None):
    # Ctrl+C only raises KeyboardInterrupt here: with rclpy's own handler the context would be
    # shut down before a stream move could close its channel.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = CartesianTargetMove()
        node.start()
        node.serve()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        code = 1
        print(f'15_cartesian_target_move failed: {error}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == '__main__':
    main()
