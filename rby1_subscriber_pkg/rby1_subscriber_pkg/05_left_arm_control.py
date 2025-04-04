import rclpy
import numpy as np
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import sys
import time
import message_filters

sys.path.append('/home/ian/rby1_ws/rby1/rby1-sdk')
import rby1_sdk

class GlobalVariable:
    ready = False
    torso_position_data = np.zeros(6)
    right_position_data = np.zeros(7)
    left_position_data = np.zeros(7)
    head_position_data = np.zeros(2)    
    

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/rby1_left_arm_controller/state',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        GlobalVariable.left_position_data = msg.actual.positions[0:7]
        GlobalVariable.ready = True

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()

    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(subscriber,))
    ros_thread.start()

    # robot = rby1_sdk.create_robot_a("192.168.100.47:50051") # for real rby1
    robot = rby1_sdk.create_robot_a("192.168.100.26:50051")
    robot.connect()

    print(robot.set_parameter("joint_position_command.cutoff_frequency", "5"))
    print(robot.set_parameter("default.acceleration_limit_scaling", "0.8"))

    robot.power_on(".*")
    robot.servo_on("right_arm_.*|left_arm_.*|torso_.*")
    robot.reset_fault_control_manager()
    robot.enable_control_manager()

    def initial_joint_position_command(robot):
        print("initial_joint_position_command")

        # Define joint positions
        q_joint_torso = np.zeros(6)
        q_joint_right_arm = np.zeros(7)
        q_joint_left_arm = np.zeros(7)

        # Combine joint positions
        q = np.concatenate([q_joint_torso, q_joint_right_arm, q_joint_left_arm])

        # Build command
        rc = rby1_sdk.RobotCommandBuilder().set_command(
            rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
                rby1_sdk.BodyCommandBuilder().set_command(
                    rby1_sdk.JointPositionCommandBuilder()
                    .set_position(q)
                    .set_minimum_time(10)
                )
            )
        )

        rv = robot.send_command(rc, 10).get()

        if rv.finish_code != rby1_sdk.RobotCommandFeedback.FinishCode.Ok:
            print("Error: Failed to conduct demo motion.")
            return 1

        return 0
    
    if not initial_joint_position_command(robot):
        print("finish motion")

    stream = robot.create_command_stream()

    try:
        while True:
            while not GlobalVariable.ready:
                time.sleep(0.001)

            rc = rby1_sdk.RobotCommandBuilder().set_command(
                rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
                    rby1_sdk.BodyComponentBasedCommandBuilder()
                    .set_torso_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
                        .set_minimum_time(0.35)
                        .set_position(GlobalVariable.torso_position_data)
                    )
                    .set_right_arm_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
                        .set_minimum_time(0.35)
                        .set_position(GlobalVariable.right_position_data)
                    )
                    .set_left_arm_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
                        .set_minimum_time(0.35)
                        .set_position(GlobalVariable.left_position_data)
                    )
                )
            )
            GlobalVariable.ready = False

            rv = stream.send_command(rc)

    except KeyboardInterrupt:
        print("Stopping robot control...")

    finally:
        subscriber.destroy_node()
        rclpy.shutdown()
        ros_thread.join()



if __name__ == "__main__":
    main()