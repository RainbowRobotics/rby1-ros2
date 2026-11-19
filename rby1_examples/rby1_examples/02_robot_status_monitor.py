#!/usr/bin/env python3
"""
Robot Status Monitor
====================
A comprehensive diagnostic node that subscribes to all major state
topics published by the RBY1 driver and renders a premium dynamic
dashboard to the terminal at 10 Hz.

Run:
  ros2 run rby1_examples 02_robot_status_monitor

Topics subscribed:
  - robot_state       (RobotState)
  - battery_state     (sensor_msgs/BatteryState)
  - tool_flange/left/{imu, wrench}
  - tool_flange/right/{imu, wrench}  (voltage, switch and connection are in robot_state)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rby1_msgs.msg import RobotState
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import WrenchStamped
import sys

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor', namespace='rby1')
        
        # State caches
        self.robot_state_msg = None
        self.battery_msg = None
        self.tf_left_imu = None
        self.tf_left_wrench = None
        self.tf_left_status = None
        self.tf_right_imu = None
        self.tf_right_wrench = None
        self.tf_right_status = None

        # Subscriptions to flat state topics
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        # Subscriptions to classified tool flange topics
        self.tf_left_imu_sub = self.create_subscription(
            Imu, 'tool_flange/left/imu', lambda msg: self.tf_imu_callback(msg, 'Left'), 10)
        self.tf_left_wrench_sub = self.create_subscription(
            WrenchStamped, 'tool_flange/left/wrench', lambda msg: self.tf_wrench_callback(msg, 'Left'), 10)

        self.tf_right_imu_sub = self.create_subscription(
            Imu, 'tool_flange/right/imu', lambda msg: self.tf_imu_callback(msg, 'Right'), 10)
        self.tf_right_wrench_sub = self.create_subscription(
            WrenchStamped, 'tool_flange/right/wrench', lambda msg: self.tf_wrench_callback(msg, 'Right'), 10)

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
        self.tf_left_status = msg.tool_flange_left
        self.tf_right_status = msg.tool_flange_right

    def battery_callback(self, msg):
        self.battery_msg = msg

    def tf_imu_callback(self, msg, side):
        if side == 'Left':
            self.tf_left_imu = msg
        else:
            self.tf_right_imu = msg

    def tf_wrench_callback(self, msg, side):
        if side == 'Left':
            self.tf_left_wrench = msg
        else:
            self.tf_right_wrench = msg


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
            
            print(f"Tool Flanges Conn:     Left={msg.tool_flange_left.connected}, Right={msg.tool_flange_right.connected}")
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
        for side, imu, wrench, status in [("Left", self.tf_left_imu, self.tf_left_wrench, self.tf_left_status),
                                          ("Right", self.tf_right_imu, self.tf_right_wrench, self.tf_right_status)]:
            if imu or wrench or status:
                print(f"Tool Flange [{side}]:")
                if wrench:
                    f = [round(wrench.wrench.force.x, 2), round(wrench.wrench.force.y, 2), round(wrench.wrench.force.z, 2)]
                    t = [round(wrench.wrench.torque.x, 2), round(wrench.wrench.torque.y, 2), round(wrench.wrench.torque.z, 2)]
                    print(f"  FT Force:      {f} N")
                    print(f"  FT Torque:     {t} Nm")
                if imu:
                    g = [round(imu.angular_velocity.x, 3), round(imu.angular_velocity.y, 3), round(imu.angular_velocity.z, 3)]
                    a = [round(imu.linear_acceleration.x, 3), round(imu.linear_acceleration.y, 3), round(imu.linear_acceleration.z, 3)]
                    print(f"  IMU Gyro:      {g} rad/s")
                    print(f"  IMU Accel:     {a} m/s^2")
                if status:
                    print(f"  Output Volt:   {status.output_voltage} V | Switch A: {status.switch_a}")
            else:
                print(f"Tool Flange [{side}]:  \033[1;33mNo connection or waiting for topic...\033[0m")
        print("=" * 65)

def main(args=None):
    # Ctrl+C is a plain KeyboardInterrupt: rclpy's own handler can shut the context down in the
    # middle of a spin, which ends in an RCLError instead of a clean exit.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Clear screen on exit
        print("\033[H\033[J", end="", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
 
