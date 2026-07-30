import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    serial_no = LaunchConfiguration("serial_no")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    tag_pose_topic = LaunchConfiguration("tag_pose_topic")
    tag_family = LaunchConfiguration("tag_family")
    tag_id = LaunchConfiguration("tag_id")
    tag_size = LaunchConfiguration("tag_size")

    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_namespace", default_value="handeye_camera"),
            DeclareLaunchArgument("serial_no", default_value="'135122070003'"),
            DeclareLaunchArgument(
                "image_topic", default_value="/handeye_camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/handeye_camera/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "tag_pose_topic", default_value="/handeye_camera/tag10_pose"
            ),
            DeclareLaunchArgument("tag_family", default_value="tag36h11"),
            DeclareLaunchArgument("tag_id", default_value="10"),
            DeclareLaunchArgument("tag_size", default_value="0.107"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    "camera_name": camera_name,
                    "camera_namespace": camera_namespace,
                    "serial_no": serial_no,
                    "enable_color": "true",
                    "enable_depth": "false",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "rgb_camera.color_profile": "640x480x30",
                }.items(),
            ),
            Node(
                package="s4_vision_bringup",
                executable="apriltag_pose_node",
                name="s4_apriltag_pose_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "output_topic": tag_pose_topic,
                        "tag_family": tag_family,
                        "tag_id": ParameterValue(tag_id, value_type=int),
                        "tag_size": ParameterValue(tag_size, value_type=float),
                    }
                ],
            ),
        ]
    )
