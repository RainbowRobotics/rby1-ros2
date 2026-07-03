#!/usr/bin/env python3
"""
Cartesian Command Example
=========================
Demonstrates Cartesian-space (SE3 pose) control of the RBY1 robot arms using
the Rby1CartesianCommand action. The example moves one or both arms to target
end-effector poses expressed as homogeneous transforms (4x4 matrix flattened
to a 16-element row-major array in the CartesianCommand message).

Sequence:
  1. Ensure robot is powered on and servos are active.
  2. Move whole body to zero pose via JointCommand (safe starting posture).
  3. Query the current Cartesian pose of the right arm via GetCartesianPose.
  4. Send a Cartesian goal to the right arm (offset from current pose).
  5. Wait for the action to complete and report the result.

Run:
  ros2 run rby1_examples cartesian_command

Actions used:
  - robot_joint      (Rby1JointCommand)
  - robot_cartesian  (Rby1CartesianCommand)

Services used:
  - robot_power          (StateOnOff)
  - robot_servo          (StateOnOff)
  - get_cartesian_pose   (GetCartesianPose)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import Rby1CartesianCommand, Rby1JointCommand
from rby1_msgs.msg import CartesianCommand, JointCommand, RobotState
from rby1_msgs.srv import StateOnOff, GetCartesianPose
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform, Vector3, Quaternion

class CartesianCommandExample(Node):
    def __init__(self):
        super().__init__('cartesian_command_example', namespace='rby1')
        self._action_client = ActionClient(self, Rby1CartesianCommand, 'robot_cartesian')
        self._zero_pose_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.get_pose_client = self.create_client(GetCartesianPose, 'get_cartesian_pose')
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
        
        time.sleep(2.0)
            
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
            self.get_logger().error(f'Timed out waiting for robot to enable.')
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

        self._zero_pose_client.wait_for_server()
        self.get_logger().info('Sending Ready Pose Goal...')
        future = self._zero_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
            
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        if not result.success:
            self.get_logger().error(f'Ready pose failed with code: {result.finish_code}')
        return result.success

    def get_cartesian_pose(self, ref_link, target_link):
        req = GetCartesianPose.Request()
        req.ref_link = ref_link
        req.target_link = target_link
        self.get_pose_client.wait_for_service()
        future = self.get_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().transform

    def send_relative_z_goal(self):
        # 1. Get current pose for right arm
        right_transform = self.get_cartesian_pose("link_torso_5", "link_right_arm_6")
        self.get_logger().info(f"Current Right Arm Pose Translation: "
                               f"x={right_transform.translation.x:.3f}, "
                               f"y={right_transform.translation.y:.3f}, "
                               f"z={right_transform.translation.z:.3f}")

        # 2. Get current pose for left arm
        left_transform = self.get_cartesian_pose("link_torso_5", "link_left_arm_6")
        self.get_logger().info(f"Current Left Arm Pose Translation: "
                               f"x={left_transform.translation.x:.3f}, "
                               f"y={left_transform.translation.y:.3f}, "
                               f"z={left_transform.translation.z:.3f}")

        goal_msg = Rby1CartesianCommand.Goal()
        
        # Right Arm (z + 5cm)
        cmd_right = CartesianCommand()
        cmd_right.ref_link = "link_torso_5"
        cmd_right.target_link = "link_right_arm_6"
        cmd_right.transform = right_transform
        cmd_right.transform.translation.z += 0.05
        cmd_right.minimum_time = 5.0
        goal_msg.right_arm = cmd_right

        # Left Arm (z + 5cm) - Cartesian Impedance Control
        cmd_left = CartesianCommand()
        cmd_left.ref_link = "link_torso_5"
        cmd_left.target_link = "link_left_arm_6"
        cmd_left.use_impedance = True
        cmd_left.transform = left_transform
        cmd_left.transform.translation.z += 0.05
        cmd_left.translation_weight = [500.0, 500.0, 500.0]
        cmd_left.rotation_weight = [50.0, 50.0, 50.0]
        cmd_left.control_hold_time = 5.0
        cmd_left.minimum_time = 5.0
        goal_msg.left_arm = cmd_left

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Z+5cm Cartesian Command Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = CartesianCommandExample()

    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Robot initialization failed. Exiting.')
        return

    # 1. Ready Pose First
    action_client.get_logger().info('Sending Ready Pose Goal via Joint Action...')
    if not action_client.send_ready_pose():
        action_client.get_logger().error('Ready pose failed')
        return

    time.sleep(1.0)

    # 2. Query pose and send Cartesian Goal
    future = action_client.send_relative_z_goal()
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
 
