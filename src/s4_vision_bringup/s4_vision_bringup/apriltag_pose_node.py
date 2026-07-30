from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image


APRILTAG_DICTIONARIES = {
    "tag36h11": "DICT_APRILTAG_36H11",
    "tag36h10": "DICT_APRILTAG_36H10",
    "tag25h9": "DICT_APRILTAG_25H9",
    "tag16h5": "DICT_APRILTAG_16H5",
}


class AprilTagPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("s4_apriltag_pose_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_topic", "/camera/tag10_pose")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_id", 10)
        self.declare_parameter("tag_size", 0.075)

        self._image_topic = self.get_parameter("image_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._tag_family = self.get_parameter("tag_family").value
        self._tag_id = int(self.get_parameter("tag_id").value)
        self._tag_size = float(self.get_parameter("tag_size").value)

        if self._tag_size <= 0.0:
            raise ValueError("tag_size must be positive")

        self._bridge = CvBridge()
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_frame = ""

        self._dictionary = self._create_dictionary(self._tag_family)
        self._detector_params = cv2.aruco.DetectorParameters_create()
        if hasattr(self._detector_params, "cornerRefinementMethod"):
            self._detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

        self._pose_pub = self.create_publisher(PoseStamped, self._output_topic, 10)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_callback, 10
        )
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._image_callback, 10
        )

        self.get_logger().info(
            "AprilTag pose node running: "
            f"family={self._tag_family}, tag_id={self._tag_id}, "
            f"tag_size={self._tag_size}, image_topic={self._image_topic}, "
            f"camera_info_topic={self._camera_info_topic}, output_topic={self._output_topic}"
        )

    def _create_dictionary(self, family: str):
        key = family.lower()
        if key not in APRILTAG_DICTIONARIES:
            raise ValueError(f"Unsupported AprilTag family: {family}")
        dictionary_id = getattr(cv2.aruco, APRILTAG_DICTIONARIES[key])
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.asarray(msg.d, dtype=np.float64)
        self._camera_frame = msg.header.frame_id

    def _detect_markers(self, gray):
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self._dictionary, self._detector_params)
            return detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._detector_params
        )

    def _image_callback(self, msg: Image) -> None:
        if self._camera_matrix is None or self._dist_coeffs is None:
            self.get_logger().warn("Waiting for CameraInfo", throttle_duration_sec=2.0)
            return

        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)
        if ids is None:
            return

        ids = ids.flatten()
        matched = np.where(ids == self._tag_id)[0]
        if len(matched) == 0:
            return

        marker_index = int(matched[0])
        marker_corners = [corners[marker_index]]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, self._tag_size, self._camera_matrix, self._dist_coeffs
        )

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self._camera_frame or msg.header.frame_id
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = float(quat_xyzw[0])
        pose.pose.orientation.y = float(quat_xyzw[1])
        pose.pose.orientation.z = float(quat_xyzw[2])
        pose.pose.orientation.w = float(quat_xyzw[3])
        self._pose_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
