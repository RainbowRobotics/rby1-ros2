"""Isaac ROS gear-assembly contract against the RB-Y1 driver (fake backend).

Reproduces what isaac_ros_manipulation_dnn_policy expects from a robot driver
(release-4.6, see docs/ISAAC_ROS.md):
  * observation_encoder_node reads sensor_msgs/JointState from /joint_states and drops
    samples whose header stamp is older than joint_state_age_threshold_ms (default 10 ms);
  * action_decoder_node (streaming mode) publishes sensor_msgs/JointState with the arm joint
    names and target = measured q + action_scale * a to /<controller_name>/joint_commands at 30 Hz;
  * the orchestrator swaps the trajectory and impedance controllers with
    /controller_manager/switch_controller.
"""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from controller_manager_msgs.srv import SwitchController
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from sensor_msgs.msg import JointState

ARM = [f"right_arm_{i}" for i in range(7)]
CONTROLLER = "right_arm_impedance_controller"
ACTION_SCALE = 0.025
# Policy action per joint, pointing into the joint range from the zero pose
# (right_arm_1 and right_arm_3 have their upper limit at +1 deg).
ACTION = [1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0]


@pytest.mark.launch_test
def generate_test_description():
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rby1_bringup"), "launch", "rby1_mock.launch.py"])
        ),
        launch_arguments={"model": "a", "activate_controllers": "right_arm_controller"}.items(),
    )
    return launch.LaunchDescription([driver, launch_testing.actions.ReadyToTest()])


class TestIsaacRosContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("isaac_ros_contract_test")
        cls.latest = None
        cls.ages = []

        def on_joint_states(msg):
            now = cls.node.get_clock().now().nanoseconds
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            cls.ages.append((now - stamp) * 1e-9)
            cls.latest = msg

        cls.node.create_subscription(JointState, "/joint_states", on_joint_states, 10)
        cls.pub = cls.node.create_publisher(JointState, f"/{CONTROLLER}/joint_commands", 1)
        cls.switch = cls.node.create_client(SwitchController, "/controller_manager/switch_controller")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def arm_positions(self):
        index = {n: i for i, n in enumerate(self.latest.name)}
        return [self.latest.position[index[n]] for n in ARM]

    def switch_controllers(self, activate, deactivate):
        self.assertTrue(self.switch.wait_for_service(timeout_sec=60.0))
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        future = self.switch.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        self.assertIsNotNone(future.result())
        return future.result().ok

    def test_contract(self):
        # wait until the driver is up and every controller is loaded
        deadline = time.time() + 60.0
        while self.latest is None and time.time() < deadline:
            self.spin_for(0.2)
        self.assertIsNotNone(self.latest, "no /joint_states")

        # observation_encoder_node: stamp age below its default 10 ms threshold
        self.ages.clear()
        self.spin_for(2.0)
        self.assertGreater(len(self.ages), 150, "joint_states slower than ~100 Hz")
        fresh = sum(1 for a in self.ages if a < 0.010) / len(self.ages)
        self.assertGreater(fresh, 0.95, f"only {fresh:.0%} of joint_states younger than 10 ms")

        # orchestrator: trajectory controller -> impedance controller (spawner may still be loading)
        ok = False
        deadline = time.time() + 60.0
        while not ok and time.time() < deadline:
            ok = self.switch_controllers([CONTROLLER], ["right_arm_controller"])
            if not ok:
                self.spin_for(1.0)
        self.assertTrue(ok, "could not switch to the impedance controller")

        # action_decoder_node: 30 Hz, target = measured + scale * a
        start = self.arm_positions()
        for _ in range(60):
            q = self.arm_positions()
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = ARM
            msg.position = [qi + ACTION_SCALE * a for qi, a in zip(q, ACTION)]
            self.pub.publish(msg)
            self.spin_for(1.0 / 30.0)
        moved = [(b - a) * sign for a, b, sign in zip(start, self.arm_positions(), ACTION)]
        for joint, delta in zip(ARM, moved):
            self.assertGreater(delta, 0.1, f"{joint} did not follow the streamed targets ({delta:.3f} rad)")

        # a message with a missing joint is dropped (action_decoder never sends one)
        before = self.arm_positions()
        msg = JointState()
        msg.name = ARM[:6]
        msg.position = [1.0] * 6
        self.pub.publish(msg)
        self.spin_for(0.5)
        for a, b in zip(before, self.arm_positions()):
            self.assertAlmostEqual(a, b, delta=0.03)

        # and back to the trajectory controller
        self.assertTrue(self.switch_controllers(["right_arm_controller"], [CONTROLLER]))
