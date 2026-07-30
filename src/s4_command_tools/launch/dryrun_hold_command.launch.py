from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="s4_command_tools",
                executable="hold_command_publisher",
                name="s4_hold_command_publisher",
                output="screen",
                parameters=[
                    {
                        "enable_sdk_command": False,
                        "publish_rate_hz": 50.0,
                        "state_topic": "/human_lower_state",
                        "dryrun_topic": "/s4/dryrun/human_lower_command",
                        "expected_motor_count": 26,
                    }
                ],
            )
        ]
    )
