#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rby1_msgs.srv import StateOnOff
from rby1_msgs.msg import RobotState

class StreamManager(Node):
    """
    Automatic lifecycle & stream manager for RBY1 Navigation.
    1. Monitors /rby1/stream_control service and activates it when needed.
    2. Sends 10Hz zero-velocity keepalive to /rby1/cmd_vel when idle to prevent stream timeout warnings.
    3. Relays external /cmd_vel commands from Nav2 to /rby1/cmd_vel.
    """
    def __init__(self):
        super().__init__('rby1_stream_manager')

        self.power_client = self.create_client(StateOnOff, '/rby1/robot_power')
        self.servo_client = self.create_client(StateOnOff, '/rby1/robot_servo')
        self.stream_client = self.create_client(StateOnOff, '/rby1/stream_control')

        # Cmd_vel Relay & Keepalive
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/rby1/cmd_vel', 10)

        # State Monitor
        self.state_sub = self.create_subscription(RobotState, '/rby1/robot_state', self.state_cb, 10)
        self.stream_active = False

        self.last_external_cmd_time = 0.0

        # Timer to maintain stream service (every 2s)
        self.service_timer = self.create_timer(2.0, self.check_and_enable_stream)
        # Keepalive timer at 10 Hz (0.1s)
        self.keepalive_timer = self.create_timer(0.1, self.publish_keepalive)

        self.get_logger().info('RBY1 Stream Manager initialized with 10Hz stream keep-alive.')

    def state_cb(self, msg):
        self.stream_active = msg.robot_stream_state

    def cmd_vel_cb(self, msg):
        self.last_external_cmd_time = time.time()
        self.cmd_vel_pub.publish(msg)

    def publish_keepalive(self):
        # If stream is active and no external cmd_vel was received in the last 0.3s, send zero twist
        if self.stream_active and (time.time() - self.last_external_cmd_time > 0.3):
            zero_twist = Twist()
            self.cmd_vel_pub.publish(zero_twist)

    def check_and_enable_stream(self):
        if not self.stream_active:
            if self.stream_client.service_is_ready():
                self.get_logger().info('Activating RBY1 persistent stream control...')
                req = StateOnOff.Request()
                req.state = True
                self.stream_client.call_async(req)
            else:
                self.get_logger().debug('Waiting for /rby1/stream_control service...')

def main(args=None):
    rclpy.init(args=args)
    node = StreamManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
