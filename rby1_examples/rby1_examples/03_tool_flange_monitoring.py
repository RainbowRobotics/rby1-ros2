#!/usr/bin/env python3
"""
Tool Flange Power Control & Monitoring Example
==============================================
A unified utility that demonstrates both tool flange power control and continuous 
state monitoring. Renders a dynamic real-time console dashboard at 10 Hz.

Run:
  ros2 run rby1_examples 04_tool_flange_monitoring

Services used:
  - tool_flange_power (StateOnOff)

Topics subscribed:
  - tool_flange/left  (ToolFlangeState)
  - tool_flange/right (ToolFlangeState)
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import ToolFlangeState, RobotState
from rby1_msgs.srv import StateOnOff, ControlManagerCommand

class ToolFlangeMonitoring(Node):
    def __init__(self):
        super().__init__('tool_flange_monitoring', namespace='rby1')
        self.get_logger().info('Initializing Tool Flange Power Control & Monitoring...')
        
        # State caches
        self.tf_left_msg = None
        self.tf_right_msg = None
        self.control_state = None
        self.tool_flange_connected = [False, False]

        # Service client for control manager
        self.control_manager_client = self.create_client(ControlManagerCommand, 'control_manager_command')

        # Service client for power control
        self.tool_flange_client = self.create_client(StateOnOff, 'tool_flange_power')

        # Subscribe to robot_state to check if enabled
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state', self.state_callback, 10)

        # Subscribe to separate left and right tool flange topics (flat namespace)
        self.left_sub = self.create_subscription(
            ToolFlangeState, 'tool_flange/left', lambda msg: self.tf_callback(msg, 'Left'), 10)
        self.right_sub = self.create_subscription(
            ToolFlangeState, 'tool_flange/right', lambda msg: self.tf_callback(msg, 'Right'), 10)

        # Verify tool flange topics are active
        if not self.verify_topic_active('tool_flange/left', timeout=5.0):
            self.get_logger().error(
                "Error: Topic 'tool_flange/left' is not active!\n"
                "Please make sure the RBY1 ROS 2 driver is running and 'publish_tool_flange_state: true' is set in driver_parameters.yaml."
            )
            sys.exit(1)

        # Wait for first robot state message to get the control state
        self.get_logger().info('Waiting for robot state...')
        start_time = time.time()
        while rclpy.ok() and self.control_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 5.0:
                self.get_logger().error("Timed out waiting for robot_state topic!")
                sys.exit(1)

        # If robot is enabled or executing, disable it first
        if self.control_state in [RobotState.STATE_ENABLE, RobotState.STATE_EXECUTING]:
            self.get_logger().info(f"Robot is in state {self.control_state}. Disabling robot before turning on tool flange power...")
            if not self.disable_robot():
                self.get_logger().error("Failed to disable robot. Exiting.")
                sys.exit(1)

        # Call service to turn on 12V tool flange power
        # self.send_tool_flange_request(True, '12')

        # Timer for 10 Hz dynamic console updates
        self.timer = self.create_timer(1, self.render_dashboard)

    def verify_topic_active(self, topic_name, timeout=1.5):
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

    def send_tool_flange_request(self, state: bool, parameters: str) -> bool:
        if not self.tool_flange_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'tool_flange_power' not available after 5 seconds.")
            return False

        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters

        op = 'ON' if state else 'OFF'
        self.get_logger().info(f"Calling tool_flange_power: state={state}, params='{parameters}'...")
        future = self.tool_flange_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("No response from tool_flange_power service.")
            return False

        if result.success:
            self.get_logger().info(f"tool_flange_power {op} OK: {result.message}")
        else:
            self.get_logger().error(f"tool_flange_power {op} FAILED: {result.message}")
        return result.success

    def disable_robot(self) -> bool:
        if not self.control_manager_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'control_manager_command' not available after 5 seconds.")
            return False

        req = ControlManagerCommand.Request()
        req.command = ControlManagerCommand.Request.CMD_DISABLE

        self.get_logger().info("Calling control_manager to disable robot...")
        future = self.control_manager_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("No response from control_manager service.")
            return False

        if result.success:
            self.get_logger().info("Robot successfully disabled.")
            # Wait for state to change to IDLE
            start_time = time.time()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.control_state == RobotState.STATE_IDLE:
                    break
                if time.time() - start_time > 5.0:
                    self.get_logger().warn("Timed out waiting for state to transition to IDLE.")
                    break
            return True
        else:
            self.get_logger().error(f"Failed to disable robot: {result.message}")
            return False

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state
        self.tool_flange_connected = msg.tool_flange_state

    def tf_callback(self, msg, side):
        if side == 'Left':
            self.tf_left_msg = msg
        else:
            self.tf_right_msg = msg

    def render_dashboard(self):
        # Clear screen and return cursor to top-left
        print("\033[H\033[J", end="", flush=True)
        print("=" * 65)
        print("                 RBY1 TOOL FLANGE REAL-TIME MONITOR              ")
        print("=" * 65)
        
        # Check if simulation/disconnected
        left_conn = self.tool_flange_connected[0] if len(self.tool_flange_connected) > 0 else False
        right_conn = self.tool_flange_connected[1] if len(self.tool_flange_connected) > 1 else False
        
        if not left_conn and not right_conn:
            print("\033[1;33m[NOTE] Simulation Mode or Hardware Disconnected detected.\033[0m")
            print("Tool flange data and output voltages will read as 0.0.")
            print("-" * 65)

        # Display each side's data side-by-side or beautifully stacked
        for side, tf, conn in [("Left", self.tf_left_msg, left_conn), ("Right", self.tf_right_msg, right_conn)]:
            if tf:
                voltage_v = tf.output_voltage / 1000.0  # mV -> V
                voltage_str = f"{voltage_v:.2f} V"
                if tf.output_voltage > 1000:
                    power_status = "\033[1;32mON (12V)\033[0m"
                else:
                    if not conn:
                        power_status = "\033[1;33mOFF / SIMULATED\033[0m"
                    else:
                        power_status = "\033[1;31mOFF\033[0m"
                
                print(f"Tool Flange [{side}]: Power Status = {power_status} ({voltage_str})")
                
                force_str = ", ".join([f"{f:.3f}" for f in tf.ft_force])
                torque_str = ", ".join([f"{t:.3f}" for t in tf.ft_torque])
                print(f"  FT Force:      [{force_str}] N")
                print(f"  FT Torque:     [{torque_str}] Nm")
                
                gyro_str = ", ".join([f"{g:.3f}" for g in tf.gyro])
                accel_str = ", ".join([f"{a:.3f}" for a in tf.acceleration])
                print(f"  IMU Gyro:      [{gyro_str}] rad/s")
                print(f"  IMU Accel:     [{accel_str}] m/s^2")
                
                print(f"  Switch A:      {tf.switch_a}")
                print(f"  Digital Input:  A={tf.digital_input_a}, B={tf.digital_input_b}")
                print(f"  Digital Output: A={tf.digital_output_a}, B={tf.digital_output_b}")
            else:
                print(f"Tool Flange [{side}]:  \033[1;33mWaiting for data/disconnected...\033[0m")
            print("-" * 65)
        print("=" * 65)

def main(args=None):
    rclpy.init(args=args)
    monitor = None
    try:
        monitor = ToolFlangeMonitoring()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        print("\033[H\033[J", end="", flush=True)
        if monitor is not None:
            try:
                if rclpy.ok():
                    print('Shutting down: turning off tool flange power...')
                    monitor.send_tool_flange_request(False, '')
            except Exception as e:
                print(f"Error during shutdown tool flange power off: {e}")
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
 
