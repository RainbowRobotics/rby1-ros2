#!/usr/bin/env python3
"""
Joint State Monitoring Example
==============================
A diagnostic node that subscribes to the per-component JointState topics 
published by the RBY1 driver and renders a real-time console dashboard at 10 Hz.

Run:
  ros2 run rby1_examples 05_joint_state_monitoring

Topics subscribed:
  - joint_states/torso      (sensor_msgs/JointState)
  - joint_states/right_arm  (sensor_msgs/JointState)
  - joint_states/left_arm   (sensor_msgs/JointState)
  - joint_states/head       (sensor_msgs/JointState)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMonitoring(Node):
    def __init__(self):
        super().__init__('joint_state_monitoring', namespace='rby1')
        self.get_logger().info('Initializing Joint State Monitoring...')
        
        # State caches
        self.torso_msg = None
        self.right_arm_msg = None
        self.left_arm_msg = None
        self.head_msg = None

        # Subscribe to standard JointState topics for different components
        self.torso_sub = self.create_subscription(
            JointState, 'joint_states/torso', lambda msg: self.joint_state_callback(msg, 'torso'), 10)
        self.right_arm_sub = self.create_subscription(
            JointState, 'joint_states/right_arm', lambda msg: self.joint_state_callback(msg, 'right_arm'), 10)
        self.left_arm_sub = self.create_subscription(
            JointState, 'joint_states/left_arm', lambda msg: self.joint_state_callback(msg, 'left_arm'), 10)
        self.head_sub = self.create_subscription(
            JointState, 'joint_states/head', lambda msg: self.joint_state_callback(msg, 'head'), 10)

        # Timer for 10 Hz dynamic console updates
        self.timer = self.create_timer(0.1, self.render_dashboard)

    def joint_state_callback(self, msg, part_name):
        if part_name == 'torso':
            self.torso_msg = msg
        elif part_name == 'right_arm':
            self.right_arm_msg = msg
        elif part_name == 'left_arm':
            self.left_arm_msg = msg
        elif part_name == 'head':
            self.head_msg = msg

    def render_dashboard(self):
        # Clear screen and return cursor to top-left
        print("\033[H\033[J", end="", flush=True)
        print("=" * 65)
        print("                    RBY1 REAL-TIME JOINT MONITOR                 ")
        print("=" * 65)

        for name, msg in [("TORSO", self.torso_msg), 
                          ("RIGHT ARM", self.right_arm_msg), 
                          ("LEFT ARM", self.left_arm_msg), 
                          ("HEAD", self.head_msg)]:
            print(f"[{name}]")
            if msg:
                # Print table header
                print(f"  {'Joint Name':<20} | {'Position (rad)':<15} | {'Velocity (rad/s)':<15}")
                print(f"  {'-'*20} + {'-'*15} + {'-'*15}")
                for i in range(len(msg.name)):
                    j_name = msg.name[i]
                    pos = msg.position[i] if i < len(msg.position) else 0.0
                    vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    print(f"  {j_name:<20} | {pos:>14.4f} | {vel:>14.4f}")
            else:
                print("  \033[1;33mWaiting for topic update...\033[0m")
            print("-" * 65)
        print("=" * 65)

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitoring()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        # Clear screen on exit
        print("\033[H\033[J", end="", flush=True)
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 
