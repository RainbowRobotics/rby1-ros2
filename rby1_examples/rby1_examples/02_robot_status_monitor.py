#!/usr/bin/env python3
"""
Robot Status Monitor
====================
A comprehensive diagnostic node that subscribes to all major state
topics published by the RBY1 driver and renders a premium dynamic
dashboard to the terminal at 10 Hz.

Run:
  ros2 run rby1_examples 03_robot_status_monitor

Topics subscribed:
  - robot_state       (RobotState)
  - battery_state     (sensor_msgs/BatteryState)
  - tool_flange/left  (ToolFlangeState)
  - tool_flange/right (ToolFlangeState)
"""
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import RobotState, ToolFlangeState
from sensor_msgs.msg import BatteryState
import sys

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor', namespace='rby1')
        
        # State caches
        self.robot_state_msg = None
        self.battery_msg = None
        self.tf_left_msg = None
        self.tf_right_msg = None

        # Subscriptions to flat state topics
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        self.tf_left_sub = self.create_subscription(
            ToolFlangeState, 'tool_flange/left', lambda msg: self.tf_callback(msg, 'Left'), 10)
        self.tf_right_sub = self.create_subscription(
            ToolFlangeState, 'tool_flange/right', lambda msg: self.tf_callback(msg, 'Right'), 10)

        # Timer for 10 Hz dynamic console updates
        self.timer = self.create_timer(0.1, self.render_dashboard)

        # Verify robot state topic is active
        if not self.verify_topic_active('robot_state', timeout=1.5):
            self.get_logger().error(
                "Error: Topic 'robot_state' is not active!\n"
                "Please make sure the RBY1 ROS 2 driver is running."
            )
            sys.exit(1)

    def verify_topic_active(self, topic_name, timeout=1.5):
        import time
        start_time = time.time()
        resolved_topic = self.resolve_topic_name(topic_name)
        while rclpy.ok():
            try:
                pub_info = self.get_publishers_info_by_topic(resolved_topic)
                if len(pub_info) > 0:
                    return True
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                break
        return False

    def robot_state_callback(self, msg):
        self.robot_state_msg = msg

    def battery_callback(self, msg):
        self.battery_msg = msg

    def tf_callback(self, msg, side):
        if side == 'Left':
            self.tf_left_msg = msg
        else:
            self.tf_right_msg = msg

    def render_dashboard(self):
        # Clear screen and return cursor to top-left
        print("\033[H\033[J", end="", flush=True)
        print("=" * 65)
        print("                   RBY1 ROBOT DIAGNOSTIC MONITOR                 ")
        print("=" * 65)
        
        # 1. Control Manager State
        if self.robot_state_msg:
            msg = self.robot_state_msg
            state_names = {0: "NONE", 1: "IDLE", 2: "ENABLE", 3: "EXECUTING", 4: "MAJOR_FAULT", 5: "MINOR_FAULT"}
            state_str = state_names.get(msg.control_manager_state, f"UNKNOWN ({msg.control_manager_state})")
            
            state_color = "\033[1;32m"  # Green
            if "FAULT" in state_str:
                state_color = "\033[1;31m"  # Red
            elif state_str == "IDLE":
                state_color = "\033[1;33m"  # Yellow
                
            print(f"Control Manager State: {state_color}{state_str}\033[0m")
            emo_str = '\033[1;31mYES\033[0m' if msg.emo_state else '\033[1;32mNO\033[0m'
            print(f"EMO Pressed:           {emo_str}")
            print(f"Center of Mass [X,Y,Z]: {[round(x, 4) for x in msg.center_of_mass]} m")
            
            # Brakes
            print("-" * 65)
            print("Brakes Status (\033[1;31mENGAGED\033[0m / \033[1;32mRELEASED\033[0m):")
            
            def format_brakes(brakes):
                res = []
                for b in brakes:
                    if b:
                        res.append("\033[1;31mENG\033[0m")
                    else:
                        res.append("\033[1;32mREL\033[0m")
                return ", ".join(res)
                
            print(f"  Torso:     [{format_brakes(msg.brake_state.torso)}]")
            print(f"  Right Arm: [{format_brakes(msg.brake_state.right_arm)}]")
            print(f"  Left Arm:  [{format_brakes(msg.brake_state.left_arm)}]")
            print(f"  Head:      [{format_brakes(msg.brake_state.head)}]")
            
            if len(msg.tool_flange_state) >= 2:
                print(f"Tool Flanges Conn:     Left={msg.tool_flange_state[0]}, Right={msg.tool_flange_state[1]}")
        else:
            print("Control Manager State: \033[1;33mWaiting for robot_state topic...\033[0m")
            
        print("-" * 65)
        
        # 2. Battery Status
        if self.battery_msg:
            b = self.battery_msg
            percentage = b.percentage * 100.0
            pct_color = "\033[1;32m"
            if percentage < 20.0:
                pct_color = "\033[1;31m"
            elif percentage < 50.0:
                pct_color = "\033[1;33m"
            print(f"Battery Voltage:  {b.voltage:.2f} V")
            print(f"Battery Current:  {b.current:.2f} A")
            print(f"Battery Capacity: {pct_color}{percentage:.1f} %\033[0m")
        else:
            print("Battery Status:        \033[1;33mWaiting for battery_state topic...\033[0m")
            
        print("-" * 65)
        
        # 3. Tool Flanges
        for side, tf in [("Left", self.tf_left_msg), ("Right", self.tf_right_msg)]:
            if tf:
                print(f"Tool Flange [{side}]:")
                print(f"  FT Force:      {[round(x, 2) for x in tf.ft_force]} N")
                print(f"  FT Torque:     {[round(x, 2) for x in tf.ft_torque]} Nm")
                print(f"  IMU Gyro:      {[round(x, 3) for x in tf.gyro]} rad/s")
                print(f"  IMU Accel:     {[round(x, 3) for x in tf.acceleration]} m/s^2")
                print(f"  Output Volt:   {tf.output_voltage} mV | Switch A: {tf.switch_a}")
                print(f"  Digital I/O:   Inputs: [A={tf.digital_input_a}, B={tf.digital_input_b}] | Outputs: [A={tf.digital_output_a}, B={tf.digital_output_b}]")
            else:
                print(f"Tool Flange [{side}]:  \033[1;33mNo connection or waiting for topic...\033[0m")
        print("=" * 65)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clear screen on exit
        print("\033[H\033[J", end="", flush=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 
