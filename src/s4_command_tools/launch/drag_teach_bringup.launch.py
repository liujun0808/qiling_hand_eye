from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_joint_config = PathJoinSubstitution(
        [
            FindPackageShare("s4_command_tools"),
            "config",
            "drag_teach_joints.yaml",
        ]
    )
    controller_launch = PathJoinSubstitution(
        [
            FindPackageShare("s4_command_tools"),
            "launch",
            "drag_teach_controller.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_state_bridge", default_value="true"),
            DeclareLaunchArgument("enable_command_bridge", default_value="true"),
            DeclareLaunchArgument("enable_sdk_command", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument("joint_config_path", default_value=default_joint_config),
            DeclareLaunchArgument("gravity_scale", default_value="0.3"),
            Node(
                package="topic_convertor",
                executable="topic_converter_node",
                name="topic_converter_node",
                output="screen",
                parameters=[
                    {
                        "enable_state_bridge": ParameterValue(
                            LaunchConfiguration("enable_state_bridge"), value_type=bool
                        ),
                        "enable_command_bridge": ParameterValue(
                            LaunchConfiguration("enable_command_bridge"), value_type=bool
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch),
                launch_arguments={
                    "enable_sdk_command": LaunchConfiguration("enable_sdk_command"),
                    "control_enabled": LaunchConfiguration("control_enabled"),
                    "joint_config_path": LaunchConfiguration("joint_config_path"),
                    "gravity_scale": LaunchConfiguration("gravity_scale"),
                }.items(),
            ),
        ]
    )
