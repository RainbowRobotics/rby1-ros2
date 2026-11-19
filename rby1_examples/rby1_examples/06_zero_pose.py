#!/usr/bin/env python3
"""
Zero Pose Example
=================
Moves all joints of the RBY1 robot to the zero position (0 rad) simultaneously.
This is a basic "home" position example and a good starting point for verifying
that all actuators respond correctly to joint position commands.

Sequence:
  1. Check robot state; power on and servo on if not already active.
  2. Send a whole-body JointPositionCommand with all positions set to 0.0 rad.
  3. Wait for the action to complete and report the result.

Run:
  ros2 run rby1_examples 06_zero_pose

Actions used:
  - robot_joint  (Rby1JointCommand)

Services used:
  - robot_power  (StateOnOff)
  - robot_servo  (StateOnOff)

Topics subscribed:
  - robot_state  (rby1_msgs/RobotState)
"""
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff

# Values to change: edit these to adjust the example.
ZERO_TORSO = [0.0] * 6        # rad, torso_0 .. torso_5
ZERO_RIGHT_ARM = [0.0] * 7    # rad, right_arm_0 .. right_arm_6
ZERO_LEFT_ARM = [0.0] * 7     # rad, left_arm_0 .. left_arm_6
ZERO_HEAD = [0.0] * 2         # rad, head_0 .. head_1
MINIMUM_TIME = 5.0            # s, the move takes at least this long
GOAL_PRIORITY = 10

class ZeroPoseExample(Node):
    def __init__(self):
        super().__init__('zero_pose_example', namespace='rby1')
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
        
        time.sleep(2.0) # Wait for power to stabilize
            
        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        # 4. Wait for state to become 2 or 3
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0) # One more second for control manager to settle
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def send_goal(self, torso_pos, right_pos, left_pos, head_pos, minimum_time):
        goal_msg = Rby1JointCommand.Goal()
        
        if torso_pos is not None:
            goal_msg.torso = JointCommand()
            goal_msg.torso.position = torso_pos
            goal_msg.torso.minimum_time = minimum_time
            
        if right_pos is not None:
            goal_msg.right_arm = JointCommand()
            goal_msg.right_arm.position = right_pos
            goal_msg.right_arm.minimum_time = minimum_time
            
        if left_pos is not None:
            goal_msg.left_arm = JointCommand()
            goal_msg.left_arm.position = left_pos
            goal_msg.left_arm.minimum_time = minimum_time
            
        if head_pos is not None:
            goal_msg.head = JointCommand()
            goal_msg.head.position = head_pos
            goal_msg.head.minimum_time = minimum_time

        goal_msg.priority = GOAL_PRIORITY

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Zero Pose Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = ZeroPoseExample()

    # 0. Ensure Robot is Ready
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Robot initialization failed. Exiting.')
        return

    # 1. Zero pose values
    torso_pos = ZERO_TORSO
    right_pos = ZERO_RIGHT_ARM
    left_pos = ZERO_LEFT_ARM
    head_pos = ZERO_HEAD
    min_time = MINIMUM_TIME
    
    future = action_client.send_goal(torso_pos, right_pos, left_pos, head_pos, min_time)
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
 
