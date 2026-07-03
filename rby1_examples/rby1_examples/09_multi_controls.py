#!/usr/bin/env python3
"""
Multi Controls Example
======================
Demonstrates how to interleave joint-space and Cartesian-space control
on the RBY1 robot. Joint control handles the torso and head while
Cartesian control independently moves each arm to a target SE3 pose.

Sequence:
  1. Ensure robot is powered on and servos are active.
  2. Move whole body to zero pose (safe starting posture).
  3. Send a simultaneous body command:
       - Torso  : joint position command
       - Right arm : Cartesian SE3 position command
       - Left arm  : Cartesian SE3 position command
       - Head   : joint position command
  4. Wait for the action to complete and report the result.

Run:
  ros2 run rby1_examples multi_controls

Actions used:
  - robot_joint      (Rby1JointCommand)
  - robot_cartesian  (Rby1CartesianCommand)

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
from rby1_msgs.action import Rby1JointCommand, Rby1CartesianCommand
from rby1_msgs.msg import JointCommand, CartesianCommand, RobotState
from rby1_msgs.srv import StateOnOff
from geometry_msgs.msg import Transform

class MultiControlsExample(Node):
    def __init__(self):
        super().__init__('multi_controls_example', namespace='rby1')
        self.joint_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.cartesian_client = ActionClient(self, Rby1CartesianCommand, 'robot_cartesian')
        
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
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

    def toggle_stream(self, enable: bool, value: float = 0.0) -> bool:
        req = StateOnOff.Request()
        req.state = enable
        req.parameters = ""
        req.value = value
        self.get_logger().info(f"Calling stream_control: state={enable}, value={value}...")
        self.stream_control_client.wait_for_service()
        future = self.stream_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f"Stream Control successfully {'enabled' if enable else 'disabled'}.")
            return True
        else:
            self.get_logger().error(f"Failed to toggle stream control: {res.message if res else 'No response'}")
            return False

    def send_ready_pose(self):
        goal_msg = Rby1JointCommand.Goal()
        
        for part in ['torso', 'right_arm', 'left_arm', 'head']:
            cmd = JointCommand()
            if part == 'torso':
                cmd.position = [0.0] * 6
            elif part == 'head':
                cmd.position = [0.0] * 2
            elif part == 'right_arm':
                cmd.position = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
            elif part == 'left_arm':
                cmd.position = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
            cmd.minimum_time = 4.0
            setattr(goal_msg, part, cmd)

        self.joint_client.wait_for_server()
        self.get_logger().info('Sending Ready Pose Goal via Joint Action...')
        future = self.joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Ready pose goal rejected!')
            return False
            
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        return result.success

    def send_joint_goal(self, torso_pos, left_arm_pos, minimum_time):
        goal_msg = Rby1JointCommand.Goal()
        
        if torso_pos is not None:
            cmd_torso = JointCommand()
            cmd_torso.position = torso_pos
            cmd_torso.minimum_time = minimum_time
            goal_msg.torso = cmd_torso
            
        if left_arm_pos is not None:
            cmd_left = JointCommand()
            cmd_left.position = left_arm_pos
            cmd_left.minimum_time = minimum_time
            goal_msg.left_arm = cmd_left

        self.joint_client.wait_for_server()
        self.get_logger().info('Sending Joint Goal (Torso + Left Arm)...')
        return self.joint_client.send_goal_async(goal_msg)

    def send_cartesian_goal(self, right_transform, minimum_time):
        goal_msg = Rby1CartesianCommand.Goal()
        
        cmd_right = CartesianCommand()
        cmd_right.ref_link = "link_torso_5"
        cmd_right.target_link = "link_right_arm_6"
        cmd_right.transform = right_transform
        cmd_right.minimum_time = minimum_time
        goal_msg.right_arm = cmd_right

        self.cartesian_client.wait_for_server()
        self.get_logger().info('Sending Cartesian Goal (Right Arm)...')
        return self.cartesian_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = MultiControlsExample()

    state = action_client.wait_for_state()
    if state not in [2, 3]:
        action_client.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    # Move to Ready Pose First
    action_client.get_logger().info('Sending Ready Pose Goal via Joint Action...')
    if not action_client.send_ready_pose():
        action_client.get_logger().error('Failed to move to Ready Pose. Exiting.')
        action_client.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(1.0)

    min_time = 5.0

    # 1. Prepare Joint Goal (Torso: Joint Position, Left Arm: Joint Position)
    torso_pos = [0.0,0.2,-0.4,0.2,0.0,0.0]
    left_arm_pos = [0.0, 0.5, 0.0, -1.5, 0.0, 0.0, 0.0]

    # 2. Prepare Cartesian Goal (Right Arm: Cartesian position)
    right_transform = Transform()
    right_transform.translation.x = 0.3
    right_transform.translation.y = -0.3
    right_transform.translation.z = -0.2
    right_transform.rotation.x = 0.0
    right_transform.rotation.y = 0.0
    right_transform.rotation.z = 0.0
    right_transform.rotation.w = 1.0

    action_client.get_logger().info('Moving all parts concurrently (Torso & Left Arm: Joint, Right Arm: Cartesian)...')
    
    # Send both goals asynchronously
    joint_future = action_client.send_joint_goal(torso_pos, left_arm_pos, min_time)
    cartesian_future = action_client.send_cartesian_goal(right_transform, min_time)

    # Wait for goal handles
    rclpy.spin_until_future_complete(action_client, joint_future)
    rclpy.spin_until_future_complete(action_client, cartesian_future)
    
    joint_handle = joint_future.result()
    cartesian_handle = cartesian_future.result()

    if joint_handle.accepted and cartesian_handle.accepted:
        action_client.get_logger().info('Both goals accepted. Waiting for execution results...')
        
        joint_result_future = joint_handle.get_result_async()
        cartesian_result_future = cartesian_handle.get_result_async()
        
        rclpy.spin_until_future_complete(action_client, joint_result_future)
        rclpy.spin_until_future_complete(action_client, cartesian_result_future)
        
        j_res = joint_result_future.result().result
        c_res = cartesian_result_future.result().result
        
        action_client.get_logger().info(f'Joint Execution - Result: {j_res.success}, Code: {j_res.finish_code}')
        action_client.get_logger().info(f'Cartesian Execution - Result: {c_res.success}, Code: {c_res.finish_code}')
    else:
        action_client.get_logger().error(f'Goal Rejection. Joint: {joint_handle.accepted}, Cartesian: {cartesian_handle.accepted}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
