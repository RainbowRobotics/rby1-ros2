#!/usr/bin/env python3
"""
Joint State Monitoring Example
==============================
A diagnostic node that subscribes to the driver's per-part joint states
(`joint_states`) and renders a real-time console dashboard at 10 Hz.

Run:
  ros2 run rby1_examples 04_joint_state_monitoring

Topics subscribed:
  - joint_states  (rby1_msgs/PartJointStates: torso, right_arm, left_arm, head,
                   each a sensor_msgs/JointState)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rby1_msgs.msg import PartJointStates


class JointStateMonitoring(Node):
    def __init__(self):
        super().__init__('joint_state_monitoring', namespace='rby1')
        self.get_logger().info('Initializing Joint State Monitoring...')
        
        # State caches
        self.torso_msg = None
        self.right_arm_msg = None
        self.left_arm_msg = None
        self.head_msg = None

        self.part_state_sub = self.create_subscription(
            PartJointStates, 'joint_states', self.part_state_callback, 10)

        # Timer for 10 Hz dynamic console updates
        self.timer = self.create_timer(0.1, self.render_dashboard)

    def part_state_callback(self, msg):
        self.torso_msg = msg.torso
        self.right_arm_msg = msg.right_arm
        self.left_arm_msg = msg.left_arm
        self.head_msg = msg.head

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
    # Ctrl+C is a plain KeyboardInterrupt: rclpy's own handler can shut the context down in the
    # middle of a spin, which ends in an RCLError instead of a clean exit.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    monitor = JointStateMonitoring()
    
    try:
        rclpy.spin(monitor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Clear screen on exit
        print("\033[H\033[J", end="", flush=True)
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
 
