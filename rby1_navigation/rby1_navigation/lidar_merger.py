#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan

class LidarMerger(Node):
    """
    Merges multiple LaserScan topics (e.g. /scan_front and /scan_rear)
    into a unified 360-degree LaserScan topic (/scan) for SLAM and Navigation.
    """
    def __init__(self):
        super().__init__('lidar_merger')

        self.declare_parameter('front_topic', '/scan_front')
        self.declare_parameter('rear_topic', '/scan_rear')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_frame', 'base_footprint')

        front_topic = self.get_parameter('front_topic').value
        rear_topic = self.get_parameter('rear_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.front_scan = None
        self.rear_scan = None

        self.sub_front = self.create_subscription(LaserScan, front_topic, self.front_cb, 10)
        self.sub_rear = self.create_subscription(LaserScan, rear_topic, self.rear_cb, 10)
        self.pub_merged = self.create_publisher(LaserScan, output_topic, 10)

        # Publish merged scan at 10 Hz
        self.timer = self.create_timer(0.1, self.merge_and_publish)
        self.get_logger().info(f'LiDAR Merger initialized: {front_topic} + {rear_topic} -> {output_topic}')

    def front_cb(self, msg):
        self.front_scan = msg

    def rear_cb(self, msg):
        self.rear_scan = msg

    def merge_and_publish(self):
        now = self.get_clock().now().to_msg()

        # Build 360-degree scan (360 samples, 1 deg step)
        num_samples = 360
        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = 2.0 * math.pi / num_samples

        merged_scan = LaserScan()
        merged_scan.header.stamp = now
        merged_scan.header.frame_id = self.output_frame
        merged_scan.angle_min = angle_min
        merged_scan.angle_max = angle_max
        merged_scan.angle_increment = angle_inc
        merged_scan.time_increment = 0.0
        merged_scan.scan_time = 0.1
        merged_scan.range_min = 0.1
        merged_scan.range_max = 12.0

        ranges = [float('inf')] * num_samples
        intensities = [0.0] * num_samples

        # Map Front Scan
        if self.front_scan:
            self.map_scan_to_merged(self.front_scan, ranges, intensities, yaw_offset=0.0)

        # Map Rear Scan (yaw offset = pi)
        if self.rear_scan:
            self.map_scan_to_merged(self.rear_scan, ranges, intensities, yaw_offset=math.pi)

        # If no scans received yet, provide fallback clear ranges
        if not self.front_scan and not self.rear_scan:
            ranges = [5.0] * num_samples

        merged_scan.ranges = ranges
        merged_scan.intensities = intensities
        self.pub_merged.publish(merged_scan)

    def map_scan_to_merged(self, scan, ranges, intensities, yaw_offset):
        num_samples = len(ranges)
        for i, r in enumerate(scan.ranges):
            if scan.range_min <= r <= scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment + yaw_offset
                # Normalize angle to [-pi, pi]
                angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
                index = int((angle + math.pi) / (2.0 * math.pi) * num_samples) % num_samples
                if r < ranges[index]:
                    ranges[index] = r
                    if i < len(scan.intensities):
                        intensities[index] = scan.intensities[i]

def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
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
