#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class SimulatedLidar(Node):
    """
    Acquires / simulates dual 2D LiDAR sensor scans (front and rear) for RBY1.
    Publishes /scan_front and /scan_rear along with static TFs (base_footprint -> front_laser / rear_laser).
    """
    def __init__(self):
        super().__init__('simulated_lidar')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('front_frame', 'front_laser')
        self.declare_parameter('rear_frame', 'rear_laser')
        self.declare_parameter('base_frame', 'base_footprint')

        rate = self.get_parameter('publish_rate').value
        self.front_frame = self.get_parameter('front_frame').value
        self.rear_frame = self.get_parameter('rear_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.front_pub = self.create_publisher(LaserScan, '/scan_front', 10)
        self.rear_pub = self.create_publisher(LaserScan, '/scan_rear', 10)

        # Broadcast static TFs for LiDAR sensors relative to robot base
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info('Simulated LiDAR node initialized (front & rear 270-deg scanners).')

    def publish_static_tfs(self):
        now = self.get_clock().now().to_msg()

        # Front LiDAR offset: +0.3m X
        tf_front = TransformStamped()
        tf_front.header.stamp = now
        tf_front.header.frame_id = self.base_frame
        tf_front.child_frame_id = self.front_frame
        tf_front.transform.translation.x = 0.3
        tf_front.transform.translation.y = 0.0
        tf_front.transform.translation.z = 0.2
        tf_front.transform.rotation.w = 1.0

        # Rear LiDAR offset: -0.3m X, rotated 180 deg (yaw = pi)
        tf_rear = TransformStamped()
        tf_rear.header.stamp = now
        tf_rear.header.frame_id = self.base_frame
        tf_rear.child_frame_id = self.rear_frame
        tf_rear.transform.translation.x = -0.3
        tf_rear.transform.translation.y = 0.0
        tf_rear.transform.translation.z = 0.2
        tf_rear.transform.rotation.z = 1.0  # sin(pi/2)
        tf_rear.transform.rotation.w = 0.0  # cos(pi/2)

        self.tf_broadcaster.sendTransform([tf_front, tf_rear])

    def create_scan_msg(self, frame_id, angle_offset=0.0):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id

        # 270 degree FOV (-135 to +135 deg)
        scan.angle_min = -math.radians(135)
        scan.angle_max = math.radians(135)
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [4.0] * num_readings
        scan.intensities = [100.0] * num_readings
        return scan

    def timer_callback(self):
        front_scan = self.create_scan_msg(self.front_frame, 0.0)
        rear_scan = self.create_scan_msg(self.rear_frame, math.pi)

        self.front_pub.publish(front_scan)
        self.rear_pub.publish(rear_scan)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedLidar()
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
