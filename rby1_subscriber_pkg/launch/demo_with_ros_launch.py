import sys
sys.path.append('/home/ian/rby1_ws/rby1/rby1-sdk')
import rby1_sdk
import rclpy
import numpy as np
import time
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

ALL_POSITION_DATA = np.zeros(24)

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # 오른팔 subscriber
        self.right_arm_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/rby1_right_arm_controller/controller_state',
            self.right_arm_callback,
            10
        )

        # 왼팔 subscriber 추가
        self.left_arm_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/rby1_left_arm_controller/controller_state',
            self.left_arm_callback,
            10
        )

        # torso subscriber 추가
        self.torso_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/rby1_torso_controller/controller_state',
            self.torso_callback,
            10
        )

        # 데이터 수신 상태 확인을 위한 플래그
        self.right_ready = False
        self.left_arm_ready = False
        self.torso_ready = False

        # SDK 로봇 객체 생성
        self.robot = rby1_sdk.Robot()

    def right_arm_callback(self, msg):
        global ALL_POSITION_DATA
        ALL_POSITION_DATA[0:7] = np.rad2deg(msg.actual.positions)
        self.get_logger().info(f'Right arm updated: {ALL_POSITION_DATA[0:7]}')
        self.stream_positions()

    def left_arm_callback(self, msg):
        global ALL_POSITION_DATA
        ALL_POSITION_DATA[7:14] = np.rad2deg(msg.actual.positions)
        self.get_logger().info(f'Left Arm Positions: {ALL_POSITION_DATA[7:14]}')

    def torso_callback(self, msg):
        global ALL_POSITION_DATA
        ALL_POSITION_DATA[14:20] = np.rad2deg(msg.actual.positions)
        self.get_logger().info(f'Torso Positions: {ALL_POSITION_DATA[14:20]}')

        # torso까지 들어오면 데이터가 다 모였다고 가정하고 stream으로 전송
        self.stream_positions()

    def stream_positions_to_robot(self):
        command = rby1_sdk.RobotCommandBuilder().set_command(
            rby1_sdk.ComponentBasedCommandBuilder()
            .set_body_command(rby1_sdk.JointPositionCommandBuilder()
                              .set_positions(ALL_POSITION_DATA.tolist())  # 24개 포지션 리스트 전달
                              .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
                              .set_minimum_time(0.1))
        )

        self.robot.stream_command(command)
        self.get_logger().info(f'Sent positions via stream: {ALL_POSITION_DATA}')

    def start_streaming(self):
        while rclpy.ok():
            # 포지션 데이터를 로봇 SDK에 주기적으로 stream으로 전송
            self.right_arm_sub  # subscription 유지용
            self.left_arm_sub
            self.torso_sub

            self.get_logger().info(f'Streaming positions: {ALL_POSITION_DATA}')
            self.robot.send_command(
                rby1_sdk.RobotCommandBuilder().set_command(
                    rby1_sdk.ComponentBasedCommandBuilder()
                    .set_body_command(rby1_sdk.JointPositionCommandBuilder()
                                      .set_positions(ALL_POSITION_DATA.tolist())
                                      .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
                                      .set_minimum_time(0.1))
            ))

            time.sleep(0.05)  # 20Hz 스트리밍

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()

    # 10Hz로 ROS2 subscriber 노드 실행 및 SDK 명령 전송
    while rclpy.ok():
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        subscriber.start_stream()

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
