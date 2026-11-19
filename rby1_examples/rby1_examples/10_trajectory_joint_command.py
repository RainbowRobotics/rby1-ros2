#!/usr/bin/env python3
"""
Trajectory Joint Command Example
===============================
Demonstrates multi-point whole-body trajectory streaming via the standard FollowJointTrajectory
action client over a persistent command stream.

Sequence (Phase 1 — Position Control):
  1. Ensure the robot is powered and enabled.
  2. Move whole body to Zero Pose.
  3. Enable a persistent command stream via '/stream_control': the channels of the parts
     the trajectory names (right_arm, left_arm, torso, head). A trajectory is refused when one is closed.
  4. Send a whole-body trajectory (Zero → Target) using FollowJointTrajectory.
  5. Disable stream, return to Zero Pose.

Sequence (Phase 2 — Impedance Control):
  6. Call '/set_trajectory_impedance' with state=True to switch to impedance mode.
  7. Enable a persistent command stream via '/stream_control'.
  8. Send the same whole-body trajectory again (Zero → Target) in impedance mode.
  9. Disable stream, disable impedance, return to Zero Pose.

NOTE: 'set_trajectory_impedance' applies only to follow_joint_trajectory.
      It does not affect robot_joint action commands.
      Head does not support impedance — the driver falls back to position control for head joints.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import RobotState, JointCommand
from rby1_msgs.srv import StateOnOff, SetTrajectoryImpedance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Values to change: edit these to adjust the example.
# The stream channels the trajectory needs: it names torso, arm and head joints. The base's (mobile) stays closed.
STREAM_CHANNELS = 'right_arm, left_arm, torso, head'
STREAM_HZ = 15.0                  # Hz, the stream rate; the trajectories have this many points per second
# Zero pose, where the robot is moved before and after the phases
ZERO_TORSO = [0.0] * 6            # rad, torso_0 .. torso_5
ZERO_ARM = [0.0] * 7              # rad, each arm
ZERO_HEAD = [0.0] * 2             # rad
ZERO_MINIMUM_TIME = 3.0           # s, the move takes at least this long
# The trajectory: from all joints at 0 to this target
JOINT_NAMES = [f'torso_{i}' for i in range(6)] + \
              [f'right_arm_{i}' for i in range(7)] + \
              [f'left_arm_{i}' for i in range(7)] + \
              [f'head_{i}' for i in range(2)]
TARGET_TORSO = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]             # rad
TARGET_RIGHT_ARM = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]  # rad
TARGET_LEFT_ARM = [0.0,  0.5, 0.0, -1.57, 0.0, 0.0, 0.0]   # rad
TARGET_HEAD = [0.0, 0.0]                                   # rad
TRAJECTORY_DURATION = 5.0                                  # s, in phases 1 and 2
# Phase 2, impedance
IMPEDANCE_PARTS = [False, True, True]  # impedance on for torso, right_arm, left_arm
TORSO_STIFFNESS = [200.0] * 6          # N·m/rad
RIGHT_ARM_STIFFNESS = [100.0] * 7      # N·m/rad
LEFT_ARM_STIFFNESS = [100.0] * 7       # N·m/rad
DAMPING_RATIO = 1.0
TORQUE_LIMIT = 10.0                    # N·m, per joint
# Phase 3, preemption and rejection
SLOW_TRAJECTORY_DURATION = 8.0    # s, trajectory 1
REJECT_TEST_AFTER = 1.0           # s after trajectory 1 starts, a joint command is sent that must be rejected
REJECT_TEST_TORSO = [0.0] * 6     # rad, that joint command
REJECT_TEST_MINIMUM_TIME = 2.0    # s, that joint command
PREEMPT_AFTER = 1.0               # s after the reject test, trajectory 2 is sent
PREEMPT_START_RATIO = 0.25        # trajectory 2 starts from this far along trajectory 1 (0..1)
PREEMPT_DURATION = 3.0            # s, trajectory 2, back to all joints at 0


class TrajectoryJointCommand(Node):
    def __init__(self):
        super().__init__('trajectory_joint_command', namespace='rby1')
        self.stream_hz =  STREAM_HZ
        self._stream_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')
        self._zero_pose_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        self.impedance_client = self.create_client(SetTrajectoryImpedance, 'set_trajectory_impedance')

        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state

    def wait_for_state(self, target_states, timeout=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.control_state in target_states:
                return True
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                return False
        return False

    def spin_sleep(self, duration: float):
        start_time = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= duration:
                break
            rclpy.spin_once(self, timeout_sec=0.01)

    def ensure_robot_ready(self):
        self.get_logger().info('Ensuring robot is powered on and servos are active...')

        # 1. Check current state first
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

        # 2. Power On
        req = StateOnOff.Request()
        req.state = True
        req.parameters = "all"
        self.get_logger().info('Sending Power ON request...')
        self.power_client.wait_for_service()
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to power on: {future.result().message}')
            return False

        self.spin_sleep(1.0)  # Wait for power to stabilize

        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False

        # 4. Wait for state to become 2 or 3
        if self.wait_for_state([2, 3], timeout=10.0):
            self.get_logger().info('Robot is ready.')
            self.spin_sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def toggle_stream(self, enable: bool, value: float = 0.0) -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            req.parameters = STREAM_CHANNELS
            req.value = value
            self.get_logger().info(f"Calling stream_control: state={enable}, channels '{STREAM_CHANNELS}'...")
            self.stream_control_client.wait_for_service(timeout_sec=1.0)
            future = self.stream_control_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                res = future.result()
                if res and res.success:
                    self.get_logger().info(f"Stream Control successfully {'enabled' if enable else 'disabled'}: "
                                           f"{res.message}")
                    return True
                else:
                    self.get_logger().error(f"Failed to toggle stream control: {res.message if res else 'No response'}")
            else:
                self.get_logger().error("Timeout toggling stream control.")
        except Exception as e:
            self.get_logger().error(f"Exception during stream control toggle: {e}")
        return False

    def set_impedance(self, enable_states: list,
                      torso_stiffness: list = None,
                      right_arm_stiffness: list = None,
                      left_arm_stiffness: list = None,
                      damping_ratio: float = 1.0,
                      torque_limit: float = 10.0) -> bool:
        """Enable or disable impedance mode for follow_joint_trajectory per part.

        Args:
            enable_states: List of booleans for [torso, right_arm, left_arm] to activate impedance control.
            torso_stiffness: Joint stiffness list for Torso.
            right_arm_stiffness: Joint stiffness list for Right Arm.
            left_arm_stiffness: Joint stiffness list for Left Arm.
            damping_ratio: Dimensionless damping ratio (default 1.0).
            torque_limit: Per-joint torque limit in N·m (default 10.0).
        """
        req = SetTrajectoryImpedance.Request()
        req.state = enable_states
        req.torso_stiffness = torso_stiffness if torso_stiffness else []
        req.right_arm_stiffness = right_arm_stiffness if right_arm_stiffness else []
        req.left_arm_stiffness = left_arm_stiffness if left_arm_stiffness else []
        req.damping_ratio = [damping_ratio]
        req.torque_limit = [torque_limit]

        self.get_logger().info(
            f"Calling set_trajectory_impedance: state={enable_states}, "
            f"torso_stiffness={'default(100.0)' if not torso_stiffness else f'{len(torso_stiffness)} values'}, "
            f"right_arm_stiffness={'default(100.0)' if not right_arm_stiffness else f'{len(right_arm_stiffness)} values'}, "
            f"left_arm_stiffness={'default(100.0)' if not left_arm_stiffness else f'{len(left_arm_stiffness)} values'}, "
            f"damping_ratio={damping_ratio}, torque_limit={torque_limit}"
        )
        self.impedance_client.wait_for_service(timeout_sec=2.0)
        future = self.impedance_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done():
            res = future.result()
            if res and res.success:
                self.get_logger().info(f"set_trajectory_impedance OK: {res.message}")
                return True
            else:
                self.get_logger().error(f"set_trajectory_impedance FAILED: {res.message if res else 'No response'}")
        else:
            self.get_logger().error("Timeout calling set_trajectory_impedance.")
        return False

    def go_to_zero_pose(self):
        self.get_logger().info('Moving Whole Body to Zero Pose...')
        goal_msg = Rby1JointCommand.Goal()

        for part in ['torso', 'right_arm', 'left_arm', 'head']:
            cmd = JointCommand()
            if part == 'torso':
                cmd.position = ZERO_TORSO
            elif part == 'head':
                cmd.position = ZERO_HEAD
            elif part in ['right_arm', 'left_arm']:
                cmd.position = ZERO_ARM
            cmd.minimum_time = ZERO_MINIMUM_TIME
            setattr(goal_msg, part, cmd)

        self._zero_pose_client.wait_for_server()
        future = self._zero_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result().result
            if result.success:
                self.get_logger().info('Zero Pose reached successfully.')
                return True
            else:
                self.get_logger().error(f'Zero Pose failed with code: {result.finish_code}')
                return False
        return True

    def send_stream_goal(self, trajectory):
        self.get_logger().info('Starting Trajectory Streaming...')
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._stream_client.wait_for_server()
        return self._stream_client.send_goal_async(goal_msg)


def build_trajectory(joint_names, full_start, full_target, num_points=10, total_sec=5.0):
    """Build a linear interpolation JointTrajectory from start to target."""
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    for i in range(1, num_points + 1):
        point = JointTrajectoryPoint()
        point.positions = [(s + (t - s) * i / num_points) for s, t in zip(full_start, full_target)]
        elapsed_ms = int(total_sec * 1000 * i / num_points)
        point.time_from_start.sec = elapsed_ms // 1000
        point.time_from_start.nanosec = (elapsed_ms % 1000) * 1_000_000
        trajectory.points.append(point)
    return trajectory


def run_trajectory_phase(action_client, label, trajectory):
    """Enable stream → send trajectory → wait for result → disable stream."""
    action_client.get_logger().info(f'[{label}] Enabling persistent stream...')
    if not action_client.toggle_stream(True, value=action_client.stream_hz):
        action_client.get_logger().error(f'[{label}] Failed to enable stream. Skipping phase.')
        return False

    action_client.spin_sleep(0.5)

    action_client.get_logger().info(f'[{label}] Sending trajectory...')
    future = action_client.send_stream_goal(trajectory)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        result = get_result_future.result().result
        action_client.get_logger().info(
            f'[{label}] Done: error_code={result.error_code}, msg="{result.error_string}"')

    action_client.spin_sleep(2.0)

    action_client.get_logger().info(f'[{label}] Disabling stream...')
    action_client.toggle_stream(False)
    action_client.spin_sleep(0.5)
    return True


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryJointCommand()
    stream_hz = action_client.stream_hz
    action_client.get_logger().info(f"Using stream_hz = {stream_hz} (trajectory waypoint density aligned to stream rate)")

    # --- Common trajectory setup ---
    joint_names = JOINT_NAMES

    target_torso = TARGET_TORSO
    target_right = TARGET_RIGHT_ARM
    target_left  = TARGET_LEFT_ARM
    target_head  = TARGET_HEAD

    full_target = target_torso + target_right + target_left + target_head
    full_start  = [0.0] * len(full_target)

    # 0. Ensure Robot is Power ON and Servo ON
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # =========================================================
    # Phase 1: Position Control  (Zero → Target → Zero)
    # =========================================================
    action_client.get_logger().info('=' * 50)
    action_client.get_logger().info('Phase 1: Position Control (Zero → Target → Zero)')
    action_client.get_logger().info('=' * 50)

    if not action_client.go_to_zero_pose():
        action_client.get_logger().error('Failed to establish zero pose.')
        return

    traj_pos = build_trajectory(joint_names, full_start, full_target, num_points=int(TRAJECTORY_DURATION * stream_hz), total_sec=TRAJECTORY_DURATION)
    run_trajectory_phase(action_client, 'PositionCtrl', traj_pos)

    action_client.get_logger().info('[Phase 1] Returning to Zero Pose...')
    action_client.go_to_zero_pose()
    action_client.spin_sleep(1.5)

    # =========================================================
    # Phase 2: Impedance Control  (Impedance ON → Target → Zero)
    # =========================================================
    action_client.get_logger().info('=' * 50)
    action_client.get_logger().info('Phase 2: Impedance Control (Impedance ON → Target → Zero)')
    action_client.get_logger().info('=' * 50)

    # Stiffness arrays per part
    torso_stiffness = TORSO_STIFFNESS
    right_arm_stiffness = RIGHT_ARM_STIFFNESS
    left_arm_stiffness = LEFT_ARM_STIFFNESS

    if not action_client.set_impedance(IMPEDANCE_PARTS,
                                      torso_stiffness=torso_stiffness,
                                      right_arm_stiffness=right_arm_stiffness,
                                      left_arm_stiffness=left_arm_stiffness,
                                      damping_ratio=DAMPING_RATIO,
                                      torque_limit=TORQUE_LIMIT):
        action_client.get_logger().error('[Phase 2] Failed to enable impedance mode. Skipping.')
    else:
        traj_imp = build_trajectory(joint_names, full_start, full_target, num_points=int(TRAJECTORY_DURATION * stream_hz), total_sec=TRAJECTORY_DURATION)
        run_trajectory_phase(action_client, 'ImpedanceCtrl', traj_imp)

        # Disable impedance before returning to zero (zero uses robot_joint, not follow_joint_trajectory,
        # but disabling here keeps the state clean)
        action_client.set_impedance([False, False, False])
        action_client.get_logger().info('[Phase 2] Returning to Zero Pose...')
        action_client.go_to_zero_pose()

    # =========================================================
    # Phase 3: Trajectory Preemption & Rejection Test
    # =========================================================
    action_client.get_logger().info('=' * 50)
    action_client.get_logger().info('Phase 3: Trajectory Preemption & Rejection Test')
    action_client.get_logger().info('=' * 50)

    action_client.get_logger().info('[Phase 3] Enabling persistent stream...')
    if not action_client.toggle_stream(True, value=stream_hz):
        action_client.get_logger().error('[Phase 3] Failed to enable stream. Skipping phase.')
    else:
        # Build Trajectory 1 (Slow motion, 8 seconds duration)
        traj_slow = build_trajectory(joint_names, full_start, full_target, num_points=int(SLOW_TRAJECTORY_DURATION * stream_hz), total_sec=SLOW_TRAJECTORY_DURATION)
        
        action_client.get_logger().info('[Phase 3] Sending Trajectory 1 (Slow Zero -> Target, 8s)...')
        future1 = action_client.send_stream_goal(traj_slow)
        rclpy.spin_until_future_complete(action_client, future1)
        goal_handle1 = future1.result()
        
        if not goal_handle1.accepted:
            action_client.get_logger().error('[Phase 3] Trajectory 1 was rejected.')
        else:
            # Sleep a bit
            action_client.spin_sleep(REJECT_TEST_AFTER)
            
            # TEST: Try sending a single joint command while the trajectory is running.
            # This should be REJECTED by the driver action server.
            action_client.get_logger().info('[Phase 3] TEST: Sending Rby1JointCommand during active trajectory (expecting REJECT)...')
            test_goal = Rby1JointCommand.Goal()
            test_goal.torso = JointCommand()
            test_goal.torso.position = REJECT_TEST_TORSO
            test_goal.torso.minimum_time = REJECT_TEST_MINIMUM_TIME
            
            action_client._zero_pose_client.wait_for_server()
            test_future = action_client._zero_pose_client.send_goal_async(test_goal)
            rclpy.spin_until_future_complete(action_client, test_future)
            test_goal_handle = test_future.result()
            
            if not test_goal_handle.accepted:
                action_client.get_logger().info('[Phase 3] SUCCESS: Rby1JointCommand was successfully REJECTED by the driver!')
            else:
                action_client.get_logger().error('[Phase 3] FAILURE: Rby1JointCommand was accepted when trajectory was running.')
            
            # Sleep the remaining time for the 2s interval
            action_client.spin_sleep(PREEMPT_AFTER)

            # Send Trajectory 2 to preempt it (moves back to zero in 3s)
            est_midpoint = [(s + (t - s) * PREEMPT_START_RATIO) for s, t in zip(full_start, full_target)]
            traj_preempt = build_trajectory(joint_names, est_midpoint, full_start, num_points=int(PREEMPT_DURATION * stream_hz), total_sec=PREEMPT_DURATION)
            
            action_client.get_logger().info('[Phase 3] Sending Trajectory 2 (Preempting Trajectory 1, moving back to Zero)...')
            future2 = action_client.send_stream_goal(traj_preempt)
            rclpy.spin_until_future_complete(action_client, future2)
            goal_handle2 = future2.result()
            
            if not goal_handle2.accepted:
                action_client.get_logger().error('[Phase 3] Trajectory 2 was rejected.')
            else:
                # Wait for both results
                res_future1 = goal_handle1.get_result_async()
                rclpy.spin_until_future_complete(action_client, res_future1)
                result1 = res_future1.result().result
                action_client.get_logger().info(
                    f'[Phase 3] Trajectory 1 finished: error_code={result1.error_code}, error_string="{result1.error_string}"')
                
                res_future2 = goal_handle2.get_result_async()
                rclpy.spin_until_future_complete(action_client, res_future2)
                result2 = res_future2.result().result
                action_client.get_logger().info(
                    f'[Phase 3] Trajectory 2 finished: error_code={result2.error_code}, error_string="{result2.error_string}"')
                
        # Clean up
        action_client.spin_sleep(1.0)
        action_client.get_logger().info('[Phase 3] Disabling stream...')
        action_client.toggle_stream(False)
        action_client.spin_sleep(0.5)

    action_client.get_logger().info('Example complete.')
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
