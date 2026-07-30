from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="s4_handeye_calibration",
                executable="sample_recorder",
                name="s4_sample_recorder",
                output="screen",
                parameters=[
                    {
                        "state_topic": "/human_lower_state",
                        "output_file": "s4_handeye_samples.yaml",
                        "sample_rate_hz": 5.0,
                        "max_samples": 0,
                        "record_without_tags": True,
                        "tag_pose_topics": [],
                        "tag_pose_names": [],
                    }
                ],
            )
        ]
    )
