#!/usr/bin/env python3
"""
Mobile Base Control Example
===========================
A demonstration node that publishes velocity commands to the 'cmd_vel' topic
to move the RBY1 robot's omnidirectional/wheeled mobile base while simultaneously
sending an upper-body (arm/head) joint position command.

NOTE: Stream control MUST be activated before publishing cmd_vel commands.
      The driver ignores cmd_vel topics if the stream is not open.

NOTE: cancel_control service also closes the stream.
      If you only want to cancel the command (keep stream open),
      use the action cancel instead (see Example 13).

Operational Sequence:
  1. Ensures the robot is powered and enabled.
  2. Activates stream control (mandatory for cmd_vel to be accepted by the driver).
  3. Sends an upper-body joint position goal (arms + head) with priority=1 simultaneously.
     Note: Mobile base default priority is 1, so upper-body commands must also use priority=1
     to allow simultaneous control.
  4. Publishes Twist commands to 'cmd_vel' to drive the base through a sequence of motions:
       - Forward at 0.15 m/s for 2.0 s
       - Backward at -0.15 m/s for 2.0 s
       - Rotate at 0.25 rad/s for 2.0 s
       - Lateral left at 0.15 m/s for 2.0 s  (mecanum base only)
       - Lateral right at -0.15 m/s for 2.0 s (mecanum base only)
  5. Stops the base and deactivates stream control on exit.

Run:
  ros2 run rby1_examples 12_mobile_base_control
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import StateOnOff

class MobileBaseControl(Node):
    def __init__(self):
        super().__init__('mobile_base_control', namespace='rby1')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_client = self.create_client(StateOnOff, 'stream_control')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        
        # Subscribe to flat robot_state topic
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
        
        time.sleep(1.0) # Wait for power to stabilize
            
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
            self.get_logger().info('Robot enabled.')
            time.sleep(1.0) # One more second for control manager to settle
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def activate_stream_control(self, state=True, value=0.0):
        self.get_logger().info(f'Setting stream control to {state} with value {value}...')
        self.stream_client.wait_for_service()
        req = StateOnOff.Request()
        req.state = state
        req.value = value
        future = self.stream_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None or not future.result().success:
            self.get_logger().error(f'Failed to change stream control to {state}: {future.result().message if future.result() else "No response"}')
            return False
        self.get_logger().info(f'Stream control is {"ON" if state else "OFF"}.')
        return True

    def send_velocity(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def send_pose_goal(self, torso_pos, right_pos, left_pos, head_pos, minimum_time):
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

        goal_msg.priority = 1

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Prepare Posture Goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result().result
            if result.success:
                self.get_logger().info('Prepare Posture Pose reached successfully.')
                return True
            else:
                self.get_logger().error(f'Prepare Posture Pose failed with code: {result.finish_code}')
                return False
        return False

def main(args=None):
    rclpy.init(args=args)
    controller = MobileBaseControl()

    velocity = 0.1
    angular_velocity = 0.25

    # Ensure Robot is Power ON and Servo ON
    if not controller.ensure_robot_ready():
        controller.get_logger().error('Failed to prepare robot. Exiting.')
        controller.destroy_node()
        rclpy.shutdown()
        return

    # 1. Send Prepare Posture Goal (Wait for completion while stream is inactive)
    controller.get_logger().info('Sending zero position (minimum_time = 5.0s)...')
    torso_pos = [0.0] * 6
    right_pos = [0.0] * 7
    left_pos = [0.0] * 7
    head_pos = [0.0] * 2
    
    if not controller.send_pose_goal(torso_pos, right_pos, left_pos, head_pos, 5.0):
        controller.get_logger().error('Failed to reach zero position. Exiting.')
        controller.destroy_node()
        rclpy.shutdown()
        return

    # 2. Activate Stream Control (Now safe because single command is done)
    if not controller.activate_stream_control(True):
        controller.get_logger().error('Failed to activate stream control. Exiting.')
        controller.destroy_node()
        rclpy.shutdown()
        return
    time.sleep(1.0)
    controller.get_logger().info('Stream activated. Starting mobile base control...')

    try:
        # Phase 1: Drive Forward
        controller.get_logger().info('Driving forward at {velocity} m/s for 2.0 seconds...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(velocity, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Stop
        controller.get_logger().info('Stopping base for 1.0 second...')
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Phase 2: Drive Backward
        controller.get_logger().info(f'Driving backward at {-velocity} m/s for 2.0 seconds...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(-velocity, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Stop
        controller.get_logger().info('Stopping base for 1.0 second...')
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Phase 3: Rotate
        controller.get_logger().info(f'Rotating base at {angular_velocity} rad/s for 2.0 seconds...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, angular_velocity)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Stop
        controller.get_logger().info('Stopping base for 1.0 second...')
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Phase 4: Rotate
        controller.get_logger().info(f'Rotating base at {angular_velocity} rad/s for 2.0 seconds...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, -angular_velocity)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Stop
        controller.get_logger().info('Stopping base for 1.0 second...')
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Phase 5: Drive Left (lateral translation)
        controller.get_logger().info(f'Driving left at {velocity} m/s for 2.0 seconds (mecanum base only)...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(0.0, velocity, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Stop
        controller.get_logger().info('Stopping base for 1.0 second...')
        start_time = time.time()
        while time.time() - start_time < 1.0 and rclpy.ok():
            controller.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.01)

        # Phase 6: Drive Right (lateral translation)
        controller.get_logger().info(f'Driving right at {-velocity} m/s for 2.0 seconds (mecanum base only)...')
        start_time = time.time()
        while time.time() - start_time < 2.0 and rclpy.ok():
            controller.send_velocity(0.0, -velocity, 0.0)
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.04)

        # Final Stop
        controller.get_logger().info('Stopping base movement.')
        controller.send_velocity(0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Turn stream OFF on exit
        try:
            if rclpy.ok():
                controller.activate_stream_control(False)
        except Exception as e:
            controller.get_logger().error(f"Error turning off stream control: {e}")
            
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
 
