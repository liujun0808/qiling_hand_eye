from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("state_topic", default_value="/human_lower_state"),
            DeclareLaunchArgument("dryrun_topic", default_value="/s4/dryrun/drag_teach_command"),
            DeclareLaunchArgument("sdk_command_topic", default_value="/human_lower_command"),
            DeclareLaunchArgument("enable_sdk_command", default_value="false"),
            DeclareLaunchArgument("control_enabled", default_value="false"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("joint_config_path", default_value=""),
            DeclareLaunchArgument("gravity_compensation", default_value="true"),
            DeclareLaunchArgument("gravity_scale", default_value="0.0"),
            DeclareLaunchArgument("gravity_ramp_time_sec", default_value="2.0"),
            DeclareLaunchArgument("activation_hold_time_sec", default_value="1.0"),
            DeclareLaunchArgument("safety_hold_error_limit_rad", default_value="0.25"),
            DeclareLaunchArgument("teach_mode", default_value="drag_hold"),
            DeclareLaunchArgument("arm_kp", default_value="0.0"),
            DeclareLaunchArgument("arm_kd", default_value="0.35"),
            DeclareLaunchArgument("hold_arm_kp", default_value="5.0"),
            DeclareLaunchArgument("hold_arm_kd", default_value="0.6"),
            DeclareLaunchArgument("still_velocity_threshold_rad_s", default_value="0.03"),
            DeclareLaunchArgument("move_velocity_threshold_rad_s", default_value="0.08"),
            DeclareLaunchArgument("hold_position_error_threshold_rad", default_value="0.04"),
            DeclareLaunchArgument("still_time_sec", default_value="0.4"),
            DeclareLaunchArgument("arm_effort_limit", default_value="8.0"),
            DeclareLaunchArgument("leg_hold_kp", default_value="10.0"),
            DeclareLaunchArgument("leg_hold_kd", default_value="0.3"),
            DeclareLaunchArgument("publish_passive_on_fault", default_value="true"),
            Node(
                package="s4_command_tools",
                executable="drag_teach_controller",
                name="s4_drag_teach_controller",
                output="screen",
                parameters=[
                    {
                        "state_topic": LaunchConfiguration("state_topic"),
                        "dryrun_topic": LaunchConfiguration("dryrun_topic"),
                        "sdk_command_topic": LaunchConfiguration("sdk_command_topic"),
                        "enable_sdk_command": ParameterValue(
                            LaunchConfiguration("enable_sdk_command"), value_type=bool
                        ),
                        "control_enabled": ParameterValue(
                            LaunchConfiguration("control_enabled"), value_type=bool
                        ),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("publish_rate_hz"), value_type=float
                        ),
                        "joint_config_path": LaunchConfiguration("joint_config_path"),
                        "gravity_compensation": ParameterValue(
                            LaunchConfiguration("gravity_compensation"), value_type=bool
                        ),
                        "gravity_scale": ParameterValue(
                            LaunchConfiguration("gravity_scale"), value_type=float
                        ),
                        "gravity_ramp_time_sec": ParameterValue(
                            LaunchConfiguration("gravity_ramp_time_sec"), value_type=float
                        ),
                        "activation_hold_time_sec": ParameterValue(
                            LaunchConfiguration("activation_hold_time_sec"), value_type=float
                        ),
                        "safety_hold_error_limit_rad": ParameterValue(
                            LaunchConfiguration("safety_hold_error_limit_rad"), value_type=float
                        ),
                        "teach_mode": LaunchConfiguration("teach_mode"),
                        "arm_kp": ParameterValue(LaunchConfiguration("arm_kp"), value_type=float),
                        "arm_kd": ParameterValue(LaunchConfiguration("arm_kd"), value_type=float),
                        "hold_arm_kp": ParameterValue(
                            LaunchConfiguration("hold_arm_kp"), value_type=float
                        ),
                        "hold_arm_kd": ParameterValue(
                            LaunchConfiguration("hold_arm_kd"), value_type=float
                        ),
                        "still_velocity_threshold_rad_s": ParameterValue(
                            LaunchConfiguration("still_velocity_threshold_rad_s"), value_type=float
                        ),
                        "move_velocity_threshold_rad_s": ParameterValue(
                            LaunchConfiguration("move_velocity_threshold_rad_s"), value_type=float
                        ),
                        "hold_position_error_threshold_rad": ParameterValue(
                            LaunchConfiguration("hold_position_error_threshold_rad"), value_type=float
                        ),
                        "still_time_sec": ParameterValue(
                            LaunchConfiguration("still_time_sec"), value_type=float
                        ),
                        "arm_effort_limit": ParameterValue(
                            LaunchConfiguration("arm_effort_limit"), value_type=float
                        ),
                        "leg_hold_kp": ParameterValue(
                            LaunchConfiguration("leg_hold_kp"), value_type=float
                        ),
                        "leg_hold_kd": ParameterValue(
                            LaunchConfiguration("leg_hold_kd"), value_type=float
                        ),
                        "publish_passive_on_fault": ParameterValue(
                            LaunchConfiguration("publish_passive_on_fault"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
