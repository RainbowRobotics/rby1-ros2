from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Start robot with fake hardware/simulation (mock_components/GenericSystem) or real SDK hardware."
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="127.0.0.1:50051",
        description="IP address of the RBY1 robot SDK server."
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="a",
        description="Robot model type (a or m)."
    )
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="rby1a",
        description="Folder name of the robot description model (rby1a or rby1m)."
    )
    model_version_arg = DeclareLaunchArgument(
        "model_version",
        default_value="1_0",
        description="Version string of the model (e.g., 1.0, 1_1, 1_2, 1_3)."
    )

    driver_namespace_arg = DeclareLaunchArgument(
        "driver_namespace",
        default_value="rby1",
        description="Top-level namespace of the robot driver node, topics, and services."
    )

    # Build MoveIt configuration with mappings
    moveit_config = (
        MoveItConfigsBuilder("RBY1_A_v1_0", package_name="rby1_moveit_a_1_0")
        .robot_description(
            file_path="config/RBY1_A_v1_0.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "model": LaunchConfiguration("model"),
                "model_name": LaunchConfiguration("model_name"),
                "model_version": LaunchConfiguration("model_version"),
                "driver_namespace": LaunchConfiguration("driver_namespace"),
            }
        )
        .to_moveit_configs()
    )
    
    # Generate the demo launch using moveit_configs_utils
    demo_launch = generate_demo_launch(moveit_config)
    
    # rqt_controller_manager
    # rqt_controller_manager = ExecuteProcess(
    #     cmd=['ros2', 'run', 'rqt_controller_manager', 'rqt_controller_manager', '--force-discover'],
    #     output='screen'
    # )
    
    return LaunchDescription([
        use_fake_hardware_arg,
        robot_ip_arg,
        model_arg,
        model_name_arg,
        model_version_arg,
        driver_namespace_arg,
        #rqt_controller_manager,
        *demo_launch.entities
    ])
