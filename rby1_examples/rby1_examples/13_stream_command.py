#!/usr/bin/env python3
"""
Stream Command Example
======================
Demonstrates concurrent dual stream control. Activates stream control,
streams torso, arms, and head joint positions via StreamJoint action,
and concurrently publishes velocity commands to /cmd_vel to move the mobile base.

Run:
  ros2 run rby1_examples 13_stream_command
"""
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import Rby1JointCommand, StreamJoint
from rby1_msgs.msg import JointCommand, RobotState, StreamJointCommand
from rby1_msgs.srv import StateOnOff
from geometry_msgs.msg import Twist

class StreamCommand(Node):
    def __init__(self):
        super().__init__('stream_command', namespace='rby1')
        self.stream_hz = 30.0
        
        # ROS 2 publishers, clients and action clients
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.stream_joint_client = ActionClient(self, StreamJoint, 'stream_joint')
        
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None
        self.robot_stream_state = None
        self.mb_running = False
        self.mb_thread = None

        # Build joint ready/zero pose values
        self.zero_torso = [0.0] * 6
        self.zero_right = [0.0] * 7
        self.zero_left  = [0.0] * 7
        self.zero_head  = [0.0] * 2

        self.ready_torso = [0.0] * 6
        self.ready_right = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_left  = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_head  = [0.0, 0.0]

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state
        self.robot_stream_state = msg.robot_stream_state

    def start_mobile_base_thread(self):
        import threading
        self.mb_running = True
        self.mb_thread = threading.Thread(target=self.mobile_base_loop)
        self.mb_thread.daemon = True
        self.mb_thread.start()
        self.get_logger().info("Mobile base background thread started.")

    def stop_mobile_base_thread(self):
        self.mb_running = False
        if self.mb_thread is not None:
            self.mb_thread.join(timeout=1.0)
            self.mb_thread = None
        # Publish final stop command
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info("Mobile base background thread stopped.")

    def mobile_base_loop(self):
        # Move back and forth in 5-second cycles:
        # 0.0 - 1.5s: Forward (0.15 m/s)
        # 1.5 - 2.5s: Stop (0.0 m/s)
        # 2.5 - 4.0s: Backward (-0.15 m/s)
        # 4.0 - 5.0s: Stop (0.0 m/s)
        hz = 10.0
        dt = 1.0 / hz
        start_time = self.get_clock().now()
        while rclpy.ok() and self.mb_running:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            relative_t = elapsed % 5.0
            
            twist_msg = Twist()
            if relative_t < 1.5:
                twist_msg.linear.x = 0.15
            elif 1.5 <= relative_t < 2.5:
                twist_msg.linear.x = 0.0
            elif 2.5 <= relative_t < 4.0:
                twist_msg.linear.x = -0.15
            else:
                twist_msg.linear.x = 0.0
                
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(dt)

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

    def send_joint_stream(self, torso_pos, right_pos, left_pos, head_pos, dt):
        goal_msg = StreamJoint.Goal()
        cmd = StreamJointCommand()
        
        cmd.torso = JointCommand()
        cmd.torso.position = torso_pos
        cmd.torso.minimum_time = dt
        
        cmd.right_arm = JointCommand()
        cmd.right_arm.position = right_pos
        cmd.right_arm.minimum_time = dt
        
        cmd.left_arm = JointCommand()
        cmd.left_arm.position = left_pos
        cmd.left_arm.minimum_time = dt
        
        cmd.head = JointCommand()
        cmd.head.position = head_pos
        cmd.head.minimum_time = dt
        
        goal_msg.command = cmd
        self.stream_joint_client.send_goal_async(goal_msg)

    def ensure_robot_ready(self):
        self.get_logger().info('Ensuring robot is powered on and servos are active...')
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

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
        
        self.spin_sleep(2.0)
            
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            self.spin_sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable.')
            return False

    def go_to_zero_pose(self):
        self.get_logger().info('Moving Whole Body to Zero Pose...')
        goal_msg = Rby1JointCommand.Goal()

        for part in ['torso', 'right_arm', 'left_arm', 'head']:
            cmd = JointCommand()
            if part == 'torso':
                cmd.position = [0.0] * 6
            elif part == 'head':
                cmd.position = [0.0] * 2
            elif part in ['right_arm', 'left_arm']:
                cmd.position = [0.0] * 7
            cmd.minimum_time = 3.0
            setattr(goal_msg, part, cmd)

        self.joint_client.wait_for_server()
        future = self.joint_client.send_goal_async(goal_msg)
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

    def toggle_stream(self, enable: bool) -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            self.get_logger().info(f"Calling stream_control: state={enable}...")
            self.stream_control_client.wait_for_service(timeout_sec=1.0)
            future = self.stream_control_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                res = future.result()
                if res and res.success:
                    self.get_logger().info(f"Stream Control successfully {'enabled' if enable else 'disabled'}.")
                    return True
                else:
                    self.get_logger().error(f"Failed to toggle stream control: {res.message if res else 'No response'}")
            else:
                self.get_logger().error("Timeout toggling stream control.")
        except Exception as e:
            print(f"Exception during stream control toggle: {e}")
        return False

def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StreamCommand()

    # 1. Prepare Robot
    if not node.ensure_robot_ready():
        node.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # Go to zero pose first to make sure robot starts from [0.0]*22
    if not node.go_to_zero_pose():
        node.get_logger().error('Failed to establish zero pose.')
        return

    # 2. Enable Persistent Command Streams (both upper-body and mobility)
    if not node.toggle_stream(True):
        node.get_logger().error('Failed to enable streams. Exiting.')
        return

    # Wait a brief moment to flush any old state messages, then initialize the flag to True
    node.spin_sleep(0.2)
    node.robot_stream_state = True

    node.stream_joint_client.wait_for_server()

    try:
        # Start mobile base background thread
        node.start_mobile_base_thread()

        node.get_logger().info('Starting continuous whole-body stream execution loop.')
        dt = 1.0 / node.stream_hz
        cycle = 0

        while rclpy.ok():
            if node.robot_stream_state is False:
                node.get_logger().error("Stream has been deactivated by the driver. Exiting loop.")
                break

            cycle += 1
            if cycle > 3:
                break
            node.get_logger().info(f'\n==========================================')
            node.get_logger().info(f' Cycle {cycle} - Step 1: Zero Pose -> Ready Pose (3.0s)')
            node.get_logger().info(f'==========================================')

            # Zero -> Ready
            duration = 3.0
            num_points = int(duration * node.stream_hz)
            for i in range(1, num_points + 1):
                if node.robot_stream_state is False or not rclpy.ok():
                    break
                ratio = i / num_points
                curr_torso = [s + (t - s) * ratio for s, t in zip(node.zero_torso, node.ready_torso)]
                curr_right = [s + (t - s) * ratio for s, t in zip(node.zero_right, node.ready_right)]
                curr_left = [s + (t - s) * ratio for s, t in zip(node.zero_left, node.ready_left)]
                curr_head = [s + (t - s) * ratio for s, t in zip(node.zero_head, node.ready_head)]
                
                node.send_joint_stream(curr_torso, curr_right, curr_left, curr_head, dt)
                node.spin_sleep(dt)

            # Ready Pose Hold
            node.get_logger().info(f' Cycle {cycle} - Step 2: Holding Ready Pose (1.0s)')
            duration = 1.0
            num_points = int(duration * node.stream_hz)
            for i in range(1, num_points + 1):
                if node.robot_stream_state is False or not rclpy.ok():
                    break
                node.send_joint_stream(node.ready_torso, node.ready_right, node.ready_left, node.ready_head, dt)
                node.spin_sleep(dt)

            # Ready -> Zero
            node.get_logger().info(f' Cycle {cycle} - Step 3: Ready Pose -> Zero Pose (3.0s)')
            duration = 3.0
            num_points = int(duration * node.stream_hz)
            for i in range(1, num_points + 1):
                if node.robot_stream_state is False or not rclpy.ok():
                    break
                ratio = i / num_points
                curr_torso = [s + (t - s) * ratio for s, t in zip(node.ready_torso, node.zero_torso)]
                curr_right = [s + (t - s) * ratio for s, t in zip(node.ready_right, node.zero_right)]
                curr_left = [s + (t - s) * ratio for s, t in zip(node.ready_left, node.zero_left)]
                curr_head = [s + (t - s) * ratio for s, t in zip(node.ready_head, node.zero_head)]
                
                node.send_joint_stream(curr_torso, curr_right, curr_left, curr_head, dt)
                node.spin_sleep(dt)

            # Zero Pose Hold
            node.get_logger().info(f' Cycle {cycle} - Step 4: Holding Zero Pose (1.0s)')
            duration = 1.0
            num_points = int(duration * node.stream_hz)
            for i in range(1, num_points + 1):
                if node.robot_stream_state is False or not rclpy.ok():
                    break
                node.send_joint_stream(node.zero_torso, node.zero_right, node.zero_left, node.zero_head, dt)
                node.spin_sleep(dt)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        # Stop background thread
        node.stop_mobile_base_thread()
        # Deactivate stream on exit
        if rclpy.ok():
            node.get_logger().info('Cleaning up: Deactivating stream control...')
            node.toggle_stream(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
