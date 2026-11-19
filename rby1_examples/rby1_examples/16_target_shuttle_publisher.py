#!/usr/bin/env python3
"""
Target Shuttle Publisher Example
================================
Publishes two hand poses in turn on a target topic: `point_a`, `period` seconds
later `point_b`, and so on until Ctrl+C. It only publishes. Whatever
listens on the topic moves the arm:

  15_cartesian_target_move   the driver alone: no planner, nothing is checked for
                             collisions
  rby1_moveit_executor       MoveIt plans around the obstacles of its planning scene
  the cuMotion executor      (RB-Y1 Isaac ROS repository) plans around obstacles on
                             the GPU

A point is x, y, z (m, in base) and roll, pitch, yaw (rad; the rotation is
Rz(yaw) Ry(pitch) Rx(roll)) of the arm's tool frame: ee_right for the right arm's
topic, ee_left for the left arm's. It is sent as 16 values, the row-major 4x4.

Nothing here waits for the arm, so `period` must be longer than a move takes. A
target that arrives while the arm is still moving is the listener's to handle:
the cuMotion executor switches to it at once; rby1_moveit_executor and
15_cartesian_target_move run it when the current move ends.

Sequence:
  1. Wait until something listens on `target_topic` (up to WAIT_TIMEOUT s).
  2. Until Ctrl+C: `point_a`, `period` s later `point_b`, `period` s later `point_a` again.

Watch: "-> point_a ..." for every target sent and, after each, what the listener
answers on `status_topic` (EXECUTING ..., DONE, or FAILED: <reason>). The answers
are only logged: a FAILED does not stop the next target.

Before: the RB-Y1 driver, and one listener showing READY:
  ros2 run rby1_examples 15_cartesian_target_move
  ros2 launch rby1_moveit_executor moveit_executor.launch.py
  ros2 launch rby1_cumotion demo.launch.py        (RB-Y1 Isaac ROS repository, in its container)

Run:
  ros2 run rby1_examples 16_target_shuttle_publisher
  ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p period:=8.0
  ros2 run rby1_examples 16_target_shuttle_publisher --ros-args \
      -p "point_a:=[0.413,-0.367,1.034,-1.571,-1.071,1.571]" -p "point_b:=[0.413,-0.167,1.134,-1.571,-1.071,1.571]"
  The left arm: both topics, and points on the left side
  ros2 run rby1_examples 16_target_shuttle_publisher --ros-args \
      -p target_topic:=/rby1/left_arm/target_pose -p status_topic:=/rby1/left_arm/target_status \
      -p "point_a:=[0.413,0.367,1.034,1.571,-1.071,-1.571]" -p "point_b:=[0.413,0.367,1.334,1.571,-1.071,-1.571]"

Parameters (defaults):
  target_topic  /rby1/right_arm/target_pose    where the targets go
  status_topic  /rby1/right_arm/target_status  the listener's answers, logged
  point_a       [0.413, -0.367, 1.2, -1.571, -1.071, 1.571]    16.6 cm above point_b
  point_b       [0.413, -0.367, 1.04, -1.571, -1.071, 1.571]   6 mm above the right hand at the
                                                                ready pose (0.413, -0.367, 1.034).
                The planner takes farther ones, e.g.
                point_a:=[0.413,-0.2,0.918,...] point_b:=[0.413,-0.2,1.418,...] (50 cm apart)
  period        5.0 s                          from one target to the next: longer than a move takes
A point is always 6 values. Write every value as a float (1.0, not 1).

Topics published:  <target_topic>  (std_msgs/Float64MultiArray, 16 values, row-major 4x4 in base)
Topics subscribed: <status_topic>  (std_msgs/String)
Moves the robot, through whatever listens.
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# Values to change: edit these to adjust the example.
WAIT_TIMEOUT = 15.0  # s to wait for something to listen on the target topic
# Defaults of the parameters of the same name; the table above describes them.
TARGET_TOPIC = '/rby1/right_arm/target_pose'
STATUS_TOPIC = '/rby1/right_arm/target_status'
POINT_A = [0.413, -0.367, 1.2, -1.571, -1.071, 1.571]  # x, y, z (m), roll, pitch, yaw (rad)
POINT_B = [0.413, -0.367, 1.04, -1.571, -1.071, 1.571]  # x, y, z (m), roll, pitch, yaw (rad)
PERIOD = 5.0                                             # s


def pose_values(point):
    """x, y, z, roll, pitch, yaw as the 16 values of the row-major 4x4: Rz(yaw) Ry(pitch) Rx(roll)."""
    point = [float(v) for v in point]
    if len(point) != 6 or not all(math.isfinite(v) for v in point):
        raise ValueError('a point needs 6 finite values: x, y, z (m), roll, pitch, yaw (rad)')
    x, y, z, roll, pitch, yaw = point
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
            -sp, cp * sr, cp * cr, z,
            0.0, 0.0, 0.0, 1.0]


class TargetShuttlePublisher(Node):
    def __init__(self):
        super().__init__('target_shuttle_publisher', namespace='rby1')
        self.target_topic = self.declare_parameter('target_topic', TARGET_TOPIC).value
        self.status_topic = self.declare_parameter('status_topic', STATUS_TOPIC).value
        self.points = [(name, list(self.declare_parameter(name, default).value))
                       for name, default in (('point_a', POINT_A), ('point_b', POINT_B))]
        self.period = float(self.declare_parameter('period', PERIOD).value)
        if not self.period > 0.0:
            raise ValueError('period must be positive')
        self.targets = [pose_values(point) for _, point in self.points]
        self.publisher = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.create_subscription(String, self.status_topic, self.on_status, 10)

    def on_status(self, msg):
        self.get_logger().info(f'      {msg.data}')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.05, end - time.monotonic())))

    def wait_for_listener(self):
        deadline = time.monotonic() + WAIT_TIMEOUT
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                # Example 15 serves one arm, the one its topics name; the two launches serve both.
                topics = '' if self.target_topic == TARGET_TOPIC else (
                    f' --ros-args -p target_topic:={self.target_topic} -p status_topic:={self.status_topic}')
                raise RuntimeError(
                    f'nothing listens on {self.target_topic} after {WAIT_TIMEOUT:.0f} s. Start one of these and '
                    'wait for READY:\n'
                    f'  ros2 run rby1_examples 15_cartesian_target_move{topics}    (the driver alone, no planner)\n'
                    '  ros2 launch rby1_moveit_executor moveit_executor.launch.py    (MoveIt)\n'
                    '  ros2 launch rby1_cumotion demo.launch.py    (cuMotion: RB-Y1 Isaac ROS repository, in its '
                    'container)')
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.wait_for_listener()
        cycle = 0
        while True:
            cycle += 1
            for (name, point), target in zip(self.points, self.targets):
                self.get_logger().info(f'round {cycle}: -> {name} {point}')
                self.publisher.publish(Float64MultiArray(data=target))
                self.spin_for(self.period)


def main(args=None):
    # Ctrl+C is a plain KeyboardInterrupt: rclpy's own handler can shut the context down in the
    # middle of a spin, which ends in an RCLError instead of a clean exit.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = TargetShuttlePublisher()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        code = 1
        print(f'16_target_shuttle_publisher failed: {error}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == '__main__':
    main()
