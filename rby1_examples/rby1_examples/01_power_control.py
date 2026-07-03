#!/usr/bin/env python3
"""
Power Control Example
=====================
Demonstrates the full power lifecycle of the RBY1 robot:
  - Power ON  (48V motor bus and accessories)
  - Servo ON  (enable motor drive)
  - Servo OFF (disable motor drive, back-drivable)
  - Power OFF (cut motor bus power)

This is the recommended way to safely start up and shut down
the robot from a ROS2 node without using hardware buttons.

Sequence:
  1. Send Power ON request.
  2. Send Servo ON request.
  3. Wait for the robot to reach the enabled state.
  4. Send Servo OFF request.
  5. Send Power OFF request.

Run:
  ros2 run rby1_examples power_control

Services used:
  - robot_power  (StateOnOff)
  - robot_servo  (StateOnOff)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import StateOnOff

class PowerControlExample(Node):
    def __init__(self):
        super().__init__('power_control_example', namespace='rby1')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

        while not self.power_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('power service not available, waiting again...')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('servo service not available, waiting again...')

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

def main(args=None):
    rclpy.init(args=args)
    node = PowerControlExample()

    state = node.wait_for_state()
    node.get_logger().info(f'Current control state: {state}')

    if state not in [2, 3]:
        node.get_logger().info('Robot is not enabled (or in fault). Sending Power and Servo ON (all)...')
        node.send_power_request(True, 'all')
        node.send_servo_request(True, 'all')
        
        while node.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('Robot enabled.')
    else:
        node.get_logger().info('Robot is already enabled. Proceeding...')

    node.get_logger().info('Waiting for 3 seconds...')
    time.sleep(3.0)

    node.get_logger().info('Sending Power OFF (all)')
    res = node.send_power_request(False, 'all')
    node.get_logger().info(f'Power OFF Result: {res.success}, msg: {res.message}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
