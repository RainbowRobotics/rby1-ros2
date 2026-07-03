#!/usr/bin/env python3
"""
Joint Command Example
=====================
Demonstrates joint position control of multiple body parts using the
Rby1JointCommand action. The example sends two sequential whole-body goals:
  1. Ready Pose   – preset joint positions for a natural standing posture.
  2. Zero Pose    – all joints returned to 0 rad.

Sequence:
  1. Ensure robot is powered on and servos are active.
  2. Send Ready Pose goal (torso + arms + head) and wait for completion.
  3. Send Zero Pose goal and wait for completion.

Run:
  ros2 run rby1_examples joint_command

Actions used:
  - robot_joint  (Rby1JointCommand)

Services used:
  - robot_power  (StateOnOff)
  - robot_servo  (StateOnOff)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff

class JointCommandExample(Node):
    def __init__(self):
        super().__init__('joint_command_example', namespace='rby1')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
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
        
        time.sleep(2.0)
            
        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        # 4. Wait for state
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def send_goal(self, use_zero_pose=False):
        goal_msg = Rby1JointCommand.Goal()
        
        if use_zero_pose:
            cmd_torso = JointCommand()
            cmd_torso.position = [0.0] * 6
            cmd_torso.minimum_time = 4.0
            goal_msg.torso = cmd_torso

            cmd_right = JointCommand()
            cmd_right.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            cmd_right.minimum_time = 4.0
            goal_msg.right_arm = cmd_right

            cmd_left = JointCommand()
            cmd_left.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            cmd_left.minimum_time = 4.0
            goal_msg.left_arm = cmd_left

            cmd_head = JointCommand()
            cmd_head.position = [0.0] * 2
            cmd_head.minimum_time = 4.0
            goal_msg.head = cmd_head
        else:
            # Example 1: Standard Joint Position Control for Head
            cmd_head = JointCommand()
            cmd_head.position = [0.2, 0.4]
            cmd_head.minimum_time = 5.0
            goal_msg.head = cmd_head

            # Example 2: Impedance Control for Right Arm
            cmd_right = JointCommand()
            cmd_right.position = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
            cmd_right.use_impedance = True
            cmd_right.stiffness = [100.0] * 7  # If empty, driver sets to 100.0 anyway!
            cmd_right.torque_limit = 100.0
            cmd_right.minimum_time = 5.0
            goal_msg.right_arm = cmd_right
            
            # Example 3: Joint Position Control for Left Arm
            cmd_left = JointCommand()
            cmd_left.position = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
            cmd_left.minimum_time = 5.0
            goal_msg.left_arm = cmd_left
            
            # Example 4: Joint Group Control for Torso
            cmd_torso = JointCommand()
            cmd_torso.position = [0.2, -0.4, 0.2]
            cmd_torso.use_group_joint = True
            cmd_torso.joint_names = ["torso_1", "torso_2", "torso_3"] # Must be explicitly set
            cmd_torso.minimum_time = 5.0
            goal_msg.torso = cmd_torso

        self._action_client.wait_for_server()
        if use_zero_pose:
            self.get_logger().info('Sending Zero Pose Goal...')
        else:
            self.get_logger().info('Sending Rby1 Joint Command Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = JointCommandExample()

    # 0. Ensure Robot is Ready
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Robot initialization failed. Exiting.')
        return

    # 1. Ready Pose First
    action_client.get_logger().info('Sending Ready Pose Goal...')
    future = action_client.send_goal(use_zero_pose=True)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        action_client.get_logger().error('Ready pose failed to accept')
        return
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)
    result = get_result_future.result().result
    if not result.success:
        action_client.get_logger().error(f'Ready pose failed with code: {result.finish_code}')
        return

    time.sleep(1.0)

    # 2. Send Main Task Goal
    future = action_client.send_goal(use_zero_pose=False)
    rclpy.spin_until_future_complete(action_client, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        action_client.get_logger().info('Goal rejected :(')
        return

    action_client.get_logger().info('Goal accepted :)')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)

    result = get_result_future.result().result
    action_client.get_logger().info(f'Result: {result.success}, Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
