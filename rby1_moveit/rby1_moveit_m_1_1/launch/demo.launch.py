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
        default_value="m",
        description="Robot model type (a or m)."
    )

    driver_namespace_arg = DeclareLaunchArgument(
        "driver_namespace",
        default_value="rby1",
        description="Top-level namespace of the robot driver node, topics, and services."
    )

    # Build MoveIt configuration with mappings
    moveit_config = (
        MoveItConfigsBuilder("RBY1_M_v1_1", package_name="rby1_moveit_m_1_1")
        .robot_description(
            file_path="config/RBY1_M_v1_1.urdf.xacro",
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "model": LaunchConfiguration("model"),
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
        driver_namespace_arg,
        #rqt_controller_manager,
        *demo_launch.entities
    ])
