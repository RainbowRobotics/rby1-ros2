#!/usr/bin/env python3
"""
Gravity Compensation Example
============================
Enables gravity compensation mode on selected body parts of the RBY1 robot.
In gravity compensation mode the driver continuously applies torques that
counterbalance gravity, allowing a human operator to back-drive the joints
freely by hand.

Sequence:
  1. Ensure robot is powered on and servos are active.
  2. Enable gravity compensation for torso, right arm, and left arm.
  3. Hold compensation for 10 seconds.
  4. Disable gravity compensation (restore position-hold mode).

Run:
  ros2 run rby1_examples gravity_compensation

Services used:
  - robot_power          (StateOnOff)
  - robot_servo          (StateOnOff)
  - gravity_compensation (GravityCompensation)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import StateOnOff, GravityCompensation

class GravityCompensationExample(Node):
    def __init__(self):
        super().__init__('gravity_compensation_example', namespace='rby1')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.gravity_client = self.create_client(GravityCompensation, 'gravity_compensation')
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

    def call_gravity_compensation(self, part_name, state):
        req = GravityCompensation.Request()
        req.part_name = part_name
        req.state = state
        
        self.gravity_client.wait_for_service()
        future = self.gravity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    example = GravityCompensationExample()

    state = example.wait_for_state()
    if state not in [2, 3]:
        example.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        example.send_power_request(True, 'all')
        example.send_servo_request(True, 'all')
        while example.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(example, timeout_sec=0.1)

    # Sequence:
    # 1. Right arm gravity compensation ON
    # 2. 5 seconds later, OFF
    # 3. Left arm gravity compensation ON
    # 4. 5 seconds later, OFF
    # 5. Torso gravity compensation ON
    # 6. 5 seconds later, OFF

    parts = ["right_arm", "left_arm"] # torso

    for part in parts:
        example.get_logger().info(f'==========================================')
        example.get_logger().info(f'Starting gravity compensation for {part}...')
        
        # 1. Turn ON
        res = example.call_gravity_compensation(part, True)
        if res and res.response:
            example.get_logger().info(f'-> [ON SUCCESS] Gravity compensation is now active on {part}!')
            example.get_logger().info(f'Waiting 5 seconds... You can manually move the {part} now.')
            
            # Non-blocking sleep with spin
            start_time = time.time()
            while time.time() - start_time < 5.0 and rclpy.ok():
                rclpy.spin_once(example, timeout_sec=0.1)
                
            # 2. Turn OFF
            example.get_logger().info(f'Stopping gravity compensation for {part}...')
            res_off = example.call_gravity_compensation(part, False)
            if res_off and res_off.response:
                example.get_logger().info(f'-> [OFF SUCCESS] Gravity compensation stopped on {part}!')
            else:
                example.get_logger().error(f'-> [OFF FAILED] Failed to disable gravity compensation on {part}!')
        else:
            example.get_logger().error(f'-> [ON FAILED] Failed to enable gravity compensation on {part}!')
            
        time.sleep(1.0)

    example.get_logger().info(f'==========================================')
    example.get_logger().info('Gravity Compensation Sequence Completed successfully.')
    example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
