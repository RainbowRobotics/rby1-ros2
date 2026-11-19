"""robot_state_publisher with the robot's URDF, included by demo.launch.py.

The URDF is rendered here with the same xacro arguments demo.launch.py uses (use_fake_hardware,
robot_ip, model, driver_namespace): an included launch reads the arguments the including one
declared. On ROS 2 Jazzy the ros2_control controller manager takes the URDF from the
/robot_description topic this node publishes, not from a parameter, so without these mappings
it got the xacro defaults -- use_fake_hardware true: mock hardware, whatever was asked for
(reported 2026-11-02). On Humble the controller manager gets the URDF as a parameter and this
node's copy only feeds TF and RViz; the mappings keep both the same.
"""

from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("RBY1_A_v1_0", package_name="rby1_moveit_a_1_0")
        .robot_description(
            file_path="config/RBY1_A_v1_0.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware", default="false"),
                "robot_ip": LaunchConfiguration("robot_ip", default="127.0.0.1:50051"),
                "model": LaunchConfiguration("model", default="a"),
                "driver_namespace": LaunchConfiguration("driver_namespace", default="rby1"),
            },
        )
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
