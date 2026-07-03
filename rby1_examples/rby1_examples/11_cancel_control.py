#!/usr/bin/env python3
"""
Cancel Control Example
======================
Demonstrates two independent cancellation mechanisms in the RBY1 driver:

  Part 0: Move to Zero Pose (preparation, ensures a safe starting posture).
  Part 1: Action Cancellation  – Send a long-running right-arm goal, then
          cancel it mid-flight via the action client cancel call.
  Part 2: Service Cancellation – Send a left-arm goal, then cancel it by
          calling the 'cancel_control' Trigger service on the driver.

Sequence:
  0. Ensure robot is powered on and servos are active.
  1. Move whole body to zero pose.
  2. Send right-arm goal, wait 1 s, cancel via action protocol.
  3. Send left-arm goal, wait 1 s, cancel via Trigger service.

Run:
  ros2 run rby1_examples cancel_control

Actions used:
  - robot_joint        (Rby1JointCommand)

Services used:
  - robot_power        (StateOnOff)
  - robot_servo        (StateOnOff)
  - cancel_control     (std_srvs/Trigger)

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
from std_srvs.srv import Trigger

class CancelControlExample(Node):
    def __init__(self):
        super().__init__('cancel_control_example', namespace='rby1')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.cancel_control_client = self.create_client(Trigger, 'cancel_control')
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state

    def wait_for_state(self):
        while self.control_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.control_state

    def send_power_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_servo_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_cancel_service_request(self):
        req = Trigger.Request()
        future = self.cancel_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_right_arm_goal(self, position, minimum_time):
        goal_msg = Rby1JointCommand.Goal()
        cmd_right = JointCommand()
        cmd_right.position = position
        cmd_right.minimum_time = minimum_time
        goal_msg.right_arm = cmd_right

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Right Arm Joint Goal...')
        return self._action_client.send_goal_async(goal_msg)

    def send_left_arm_goal(self, position, minimum_time):
        goal_msg = Rby1JointCommand.Goal()
        cmd_left = JointCommand()
        cmd_left.position = position
        cmd_left.minimum_time = minimum_time
        goal_msg.left_arm = cmd_left

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Left Arm Joint Goal...')
        return self._action_client.send_goal_async(goal_msg)

    def send_zero_goal(self):
        goal_msg = Rby1JointCommand.Goal()
        
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

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Zero Pose Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    example = CancelControlExample()

    state = example.wait_for_state()
    if state not in [2, 3]:
        example.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        example.send_power_request(True, 'all')
        example.send_servo_request(True, 'all')
        while example.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(example, timeout_sec=0.1)

    # 0. Send zero pose goal
    example.get_logger().info('--- Part 0: Send Zero Pose Goal ---')
    future = example.send_zero_goal()
    rclpy.spin_until_future_complete(example, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        example.get_logger().error('Zero pose failed to accept')
        return
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(example, get_result_future)
    result = get_result_future.result().result
    if not result.success:
        example.get_logger().error(f'Zero pose failed with code: {result.finish_code}')
        return
    
    # 1. Action Cancellation (Right Arm)
    example.get_logger().info('--- Part 1: Action Cancellation ---')
    position = [0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    min_time = 10.0
    
    future = example.send_right_arm_goal(position, min_time)
    rclpy.spin_until_future_complete(example, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        example.get_logger().info('Goal accepted. Waiting 2 seconds before action-canceling...')
        time.sleep(2.0)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(example, cancel_future)
        example.get_logger().info('Action Cancel request sent.')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(example, get_result_future)
        result = get_result_future.result().result
        example.get_logger().info(f'Action Cancel Result: {result.success}, Code: {result.finish_code}')

    time.sleep(1.0)

    # 2. Service Cancellation (Left Arm)
    example.get_logger().info('--- Part 2: Service Cancellation (Global Stop) ---')
    left_position = [0.0, 1.0, 0.0, -1.57, 0.0, 0.0, 0.0] 
    min_time = 5.0
    
    future = example.send_left_arm_goal(left_position, min_time)
    rclpy.spin_until_future_complete(example, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        example.get_logger().info('Goal accepted. Waiting 2 seconds before service-canceling...')
        time.sleep(2.0)
        
        example.get_logger().info('Calling cancel_control service...')
        service_result = example.send_cancel_service_request()
        example.get_logger().info(f'Service Response: {service_result.success}, {service_result.message}')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(example, get_result_future)
        result = get_result_future.result().result
        example.get_logger().info(f'Action result after service cancel: {result.success}, Code: {result.finish_code}')

    example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
