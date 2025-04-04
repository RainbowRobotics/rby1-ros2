import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rby1a',
        description='Robot model name (e.g. rby1a, rby1t5)'
    )

    model = LaunchConfiguration('model')

    xacro_file = [
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('rby1_description'),
            'urdf',
            model,
            TextSubstitution(text='')
        ]),
        model,
        TextSubstitution(text='.urdf.xacro')
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(xacro_file),
                value_type=str
            )
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rby1_description'),
        'rviz',
        'rby1.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])