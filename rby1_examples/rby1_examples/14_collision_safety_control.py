#!/usr/bin/env python3
"""
Collision Safety Control Example
================================
Demonstrates manually handling collision safety on the client side.
Subscribes to joint states to record the pose before commanding a movement.
If a collision is detected during motion:
  1. Cancels the active movement goal.
  2. Commands a recovery goal back to the pre-command pose.
  3. Cancels the recovery goal as soon as collision == False, stopping the robot safely.

Run:
  ros2 run rby1_examples 15_collision_safety_control
"""
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff
from sensor_msgs.msg import JointState

class CollisionSafetyExample(Node):
    def __init__(self):
        super().__init__('collision_safety_example', namespace='rby1')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None
        self.collision = False

        # Joint state cache for recovery
        self.torso_pos = None
        self.right_arm_pos = None
        self.left_arm_pos = None
        self.head_pos = None

        self.torso_sub = self.create_subscription(
            JointState, 'joint_states/torso', lambda msg: self.joint_state_callback(msg, 'torso'), 10)
        self.right_arm_sub = self.create_subscription(
            JointState, 'joint_states/right_arm', lambda msg: self.joint_state_callback(msg, 'right_arm'), 10)
        self.left_arm_sub = self.create_subscription(
            JointState, 'joint_states/left_arm', lambda msg: self.joint_state_callback(msg, 'left_arm'), 10)
        self.head_sub = self.create_subscription(
            JointState, 'joint_states/head', lambda msg: self.joint_state_callback(msg, 'head'), 10)

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state
        self.collision = msg.collision

    def joint_state_callback(self, msg, part_name):
        if part_name == 'torso':
            self.torso_pos = list(msg.position)
        elif part_name == 'right_arm':
            self.right_arm_pos = list(msg.position)
        elif part_name == 'left_arm':
            self.left_arm_pos = list(msg.position)
        elif part_name == 'head':
            self.head_pos = list(msg.position)

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
        self.get_logger().info('Checking robot power and servo status...')
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

        # Power On
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
            
        # Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False


    def send_joint_goal(self, torso_pos, right_pos, left_pos, head_pos, minimum_time):
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

        goal_msg.priority = 10
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionSafetyExample()

    # 1. Ensure Robot is Ready
    if not node.ensure_robot_ready():
        node.get_logger().error('Robot initialization failed. Exiting.')
        return


    # 3. Move to initial safe Zero Pose
    node.get_logger().info('Step 1: Moving to Zero Pose (initial safe pose)...')
    torso_pos = [0.0] * 6
    right_pos = [0.0] * 7
    left_pos = [0.0] * 7
    head_pos = [0.0] * 2
    min_time = 4.0
    
    future = node.send_joint_goal(torso_pos, right_pos, left_pos, head_pos, min_time)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Zero pose goal was rejected.')
        return
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)
    if not get_result_future.result().result.success:
        node.get_logger().error('Failed to reach Zero Pose.')
        return
    node.get_logger().info('Successfully reached Zero Pose.')
    time.sleep(1.0)

    # Wait until all joint state caches are populated
    node.get_logger().info('Waiting for joint states to be cached...')
    while rclpy.ok() and (node.torso_pos is None or node.right_arm_pos is None or node.left_arm_pos is None or node.head_pos is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    # 4. Record pre-command positions
    pre_cmd_torso = list(node.torso_pos)
    pre_cmd_right = list(node.right_arm_pos)
    pre_cmd_left = list(node.left_arm_pos)
    pre_cmd_head = list(node.head_pos)
    node.get_logger().info('Recorded pre-command joint positions as reference.')

    # 5. Command colliding pose to trigger safety retreat
    node.get_logger().info('Step 2: Sending a self-colliding command (arms crossing yaw joints)...')
    
    colliding_right_pos = [0.0, 0.0, 0.5, -1.57, 0.0, 0.0, 0.0]
    colliding_left_pos = [0.0, 0.0, -0.5, -1.57, 0.0, 0.0, 0.0]
    
    future = node.send_joint_goal(torso_pos, colliding_right_pos, colliding_left_pos, head_pos, 8.0)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().warn('Colliding pose goal rejected.')
        return
    
    node.get_logger().info('Colliding goal sent. Monitoring collision status...')
    
    collision_detected = False
    get_result_future = goal_handle.get_result_async()
    while rclpy.ok() and not get_result_future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.collision:
            node.get_logger().warn('\033[1;31mCollision detected! Cancelling colliding goal to stop the robot...\033[0m')
            collision_detected = True
            break

    if collision_detected:
        # Cancel current colliding goal
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future)
        node.get_logger().info('Colliding goal canceled.')

        # Wait a moment for safety
        time.sleep(0.5)

        # Send recovery goal to return to pre-command pose
        node.get_logger().info('Sending recovery goal to return to the pre-command pose...')
        rec_future = node.send_joint_goal(pre_cmd_torso, pre_cmd_right, pre_cmd_left, pre_cmd_head, 5.0)
        rclpy.spin_until_future_complete(node, rec_future)
        rec_goal_handle = rec_future.result()
        if not rec_goal_handle.accepted:
            node.get_logger().error('Recovery goal was rejected!')
            return
        node.get_logger().info('Recovery goal accepted. Moving back to previous pose...')

        # Monitor recovery progress
        rec_result_future = rec_goal_handle.get_result_async()
        recovered_safely = False
        while rclpy.ok() and not rec_result_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.collision:
                node.get_logger().info('\033[1;32mCollision resolved (collision == False). Canceling recovery action to stop robot safely.\033[0m')
                cancel_rec_future = rec_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_rec_future)
                node.get_logger().info('Recovery action canceled. Robot stopped safely.')
                recovered_safely = True
                break

        if not recovered_safely:
            rclpy.spin_until_future_complete(node, rec_result_future)
            node.get_logger().info('Recovery action completed.')
    else:
        rclpy.spin_until_future_complete(node, get_result_future)
        node.get_logger().info('Motion completed without collision.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
