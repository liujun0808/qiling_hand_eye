from __future__ import annotations

import os
import tempfile
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mit_msgs.msg import MITLowState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from .pose_math import pose_msg_to_transform, transform_to_dict
from .s4_model import ARM_JOINT_NAMES, BODY_26_MOTOR_ORDER, DEFAULT_FRAMES, S4Kinematics


def _create_session_directory(root_dir: str, requested_name: str) -> Tuple[str, str]:
    root = os.path.abspath(os.path.expanduser(root_dir))
    os.makedirs(root, exist_ok=True)
    base_name = requested_name.strip() or datetime.now().strftime("%Y%m%d_%H%M%S")

    for suffix in range(1000):
        session_name = base_name if suffix == 0 else f"{base_name}_{suffix:02d}"
        session_dir = os.path.join(root, session_name)
        try:
            os.makedirs(session_dir)
            return session_name, session_dir
        except FileExistsError:
            continue
    raise RuntimeError(f"unable to allocate a unique session directory below {root}")


class InteractiveSampleRecorder(Node):
    def __init__(self) -> None:
        super().__init__("s4_interactive_sample_recorder")

        self.declare_parameter("state_topic", "/human_lower_state")
        self.declare_parameter("session_root_dir", "samples")
        self.declare_parameter("session_name", "")
        self.declare_parameter("output_file", "")
        self.declare_parameter("tag_pose_topics", ["/handeye_camera/tag10_pose"])
        self.declare_parameter("tag_pose_names", ["tag10"])
        self.declare_parameter("tag_pose_msg_type", "PoseStamped")
        self.declare_parameter("tracked_frames", DEFAULT_FRAMES)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("max_tag_age_sec", 0.5)
        self.declare_parameter("require_tags", True)
        self.declare_parameter("image_topic", "/handeye_camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/handeye_camera/camera/color/camera_info")
        self.declare_parameter("save_images", True)
        self.declare_parameter("image_output_dir", "")
        self.declare_parameter("image_extension", "png")
        self.declare_parameter("max_image_age_sec", 1.0)
        self.declare_parameter("draw_tag_overlay", True)
        self.declare_parameter("tag_size", 0.107)
        self.declare_parameter("max_samples", 17)
        self.declare_parameter("require_stationary", True)
        self.declare_parameter("stationary_velocity_threshold_rad_s", 0.08)
        self.declare_parameter("stationary_duration_sec", 0.5)

        self.state_topic = self.get_parameter("state_topic").value
        self.session_name, self.session_dir = _create_session_directory(
            self.get_parameter("session_root_dir").value,
            self.get_parameter("session_name").value,
        )
        configured_output_file = self.get_parameter("output_file").value
        self.output_file = configured_output_file or os.path.join(
            self.session_dir, "samples.yaml"
        )
        self.tag_pose_topics = list(self.get_parameter("tag_pose_topics").value)
        self.tag_pose_names = list(self.get_parameter("tag_pose_names").value)
        self.tag_pose_msg_type = self.get_parameter("tag_pose_msg_type").value
        self.tracked_frames = list(self.get_parameter("tracked_frames").value)
        self.max_tag_age_sec = float(self.get_parameter("max_tag_age_sec").value)
        self.require_tags = bool(self.get_parameter("require_tags").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.save_images = bool(self.get_parameter("save_images").value)
        configured_image_dir = self.get_parameter("image_output_dir").value
        self.image_output_dir = configured_image_dir or os.path.join(
            self.session_dir, "images"
        )
        self.image_extension = self.get_parameter("image_extension").value.strip(".").lower()
        self.max_image_age_sec = float(self.get_parameter("max_image_age_sec").value)
        self.draw_tag_overlay = bool(self.get_parameter("draw_tag_overlay").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.require_stationary = bool(self.get_parameter("require_stationary").value)
        self.stationary_velocity_threshold = float(
            self.get_parameter("stationary_velocity_threshold_rad_s").value
        )
        self.stationary_duration_sec = float(
            self.get_parameter("stationary_duration_sec").value
        )

        if self.tag_pose_topics and not self.tag_pose_names:
            self.tag_pose_names = [f"tag_{i}" for i in range(len(self.tag_pose_topics))]
        if len(self.tag_pose_topics) != len(self.tag_pose_names):
            raise ValueError("tag_pose_names must match tag_pose_topics")
        if self.stationary_velocity_threshold < 0.0:
            raise ValueError("stationary_velocity_threshold_rad_s must be non-negative")
        if self.stationary_duration_sec < 0.0:
            raise ValueError("stationary_duration_sec must be non-negative")

        self.kinematics = S4Kinematics(self.get_parameter("urdf_path").value)
        self.bridge = CvBridge()
        self.latest_state: Optional[MITLowState] = None
        self.latest_image: Optional[object] = None
        self.latest_image_stamp: Optional[Tuple[int, int]] = None
        self.latest_image_frame = ""
        self.latest_image_received_time_sec = 0.0
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.latest_tags: Dict[str, Dict[str, object]] = {}
        self.samples: List[Dict[str, object]] = []
        self.latest_arm_max_velocity = float("inf")
        self.stationary_since_time_sec: Optional[float] = None
        self.lock = threading.Lock()

        self.state_sub = self.create_subscription(
            MITLowState, self.state_topic, self._state_callback, 10
        )
        self.image_sub = None
        if self.save_images:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._image_callback, 10
            )
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self._camera_info_callback, 10
            )

        self.tag_subs = []
        msg_type = (
            PoseWithCovarianceStamped
            if self.tag_pose_msg_type == "PoseWithCovarianceStamped"
            else PoseStamped
        )
        for name, topic in zip(self.tag_pose_names, self.tag_pose_topics):
            self.tag_subs.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, tag_name=name, tag_topic=topic: self._tag_callback(
                        tag_name, tag_topic, msg
                    ),
                    10,
                )
            )

    def _state_callback(self, msg: MITLowState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        velocities = [float(value) for value in msg.joint_states.velocity]
        arm_velocities = velocities[12:26] if len(velocities) == 26 else velocities
        max_velocity = (
            max(abs(value) for value in arm_velocities)
            if len(arm_velocities) == 14
            else float("inf")
        )
        with self.lock:
            self.latest_state = msg
            self.latest_arm_max_velocity = max_velocity
            if max_velocity <= self.stationary_velocity_threshold:
                if self.stationary_since_time_sec is None:
                    self.stationary_since_time_sec = now
            else:
                self.stationary_since_time_sec = None

    def _image_callback(self, msg: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.latest_image = image.copy()
            self.latest_image_stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
            self.latest_image_frame = msg.header.frame_id
            self.latest_image_received_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self.lock:
            self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def _tag_callback(self, name: str, topic: str, msg) -> None:
        with self.lock:
            self.latest_tags[name] = {
                "topic": topic,
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "frame_id": msg.header.frame_id,
                "transform": pose_msg_to_transform(msg),
                "received_time_sec": self.get_clock().now().nanoseconds * 1e-9,
            }

    def _fresh_tags_locked(self) -> Dict[str, Dict[str, object]]:
        now = self.get_clock().now().nanoseconds * 1e-9
        return {
            name: tag
            for name, tag in self.latest_tags.items()
            if now - float(tag["received_time_sec"]) <= self.max_tag_age_sec
        }

    def status_text(self) -> str:
        with self.lock:
            state_ok = self.latest_state is not None and len(self.latest_state.joint_states.position) in (14, 26)
            stationary_ok = self._stationary_ready_locked()
            image_ok = (not self.save_images) or self._fresh_image_locked()
            camera_info_ok = (not self.save_images) or (not self.draw_tag_overlay) or self.camera_matrix is not None
            fresh_tags = self._fresh_tags_locked()
            tag_parts = []
            for name in self.tag_pose_names:
                if name in fresh_tags:
                    tag_parts.append(f"{name}:VISIBLE")
                else:
                    tag_parts.append(f"{name}:MISSING")
            if not tag_parts:
                tag_parts.append("no tag topics configured")
            return (
                f"state={'OK' if state_ok else 'WAIT'} | "
                f"robot={'STABLE' if stationary_ok else 'MOVING'}"
                f"({self.latest_arm_max_velocity:.3f}rad/s) | "
                f"tags={', '.join(tag_parts)} | "
                f"image={'OK' if image_ok else 'WAIT'} | "
                f"camera_info={'OK' if camera_info_ok else 'WAIT'} | "
                f"samples={len(self.samples)}/{self.max_samples if self.max_samples > 0 else 'inf'}"
            )

    def wait_for_required_tags(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and rclpy.ok():
            with self.lock:
                fresh_tags = self._fresh_tags_locked()
                if all(name in fresh_tags for name in self.tag_pose_names):
                    return True
            time.sleep(0.1)
        return False

    def _fresh_image_locked(self) -> bool:
        if self.latest_image is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - self.latest_image_received_time_sec <= self.max_image_age_sec

    def _stationary_ready_locked(self) -> bool:
        if not self.require_stationary:
            return True
        if self.stationary_since_time_sec is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - self.stationary_since_time_sec >= self.stationary_duration_sec

    def _draw_tag_overlay_locked(self, image, tags: Dict[str, Dict[str, object]]) -> None:
        if self.camera_matrix is None:
            return

        dist_coeffs = self.dist_coeffs if self.dist_coeffs is not None else np.zeros(5)
        half_size = self.tag_size * 0.5
        object_corners = np.asarray(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )

        for name, tag in tags.items():
            transform = tag["transform"]
            rvec = Rotation.from_matrix(transform[:3, :3]).as_rotvec().reshape(3, 1)
            tvec = np.asarray(transform[:3, 3], dtype=np.float64).reshape(3, 1)
            image_points, _ = cv2.projectPoints(
                object_corners, rvec, tvec, self.camera_matrix, dist_coeffs
            )
            points = np.round(image_points.reshape(-1, 2)).astype(np.int32)
            cv2.polylines(image, [points], True, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(
                image,
                str(name),
                tuple(points[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.drawFrameAxes(
                image,
                self.camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                self.tag_size * 0.5,
            )

    def capture(self) -> Tuple[bool, str]:
        with self.lock:
            if self.max_samples > 0 and len(self.samples) >= self.max_samples:
                return False, f"max_samples={self.max_samples} already reached"
            if self.latest_state is None:
                return False, "no /human_lower_state has been received"
            if not self._stationary_ready_locked():
                return False, (
                    "robot is not stationary: "
                    f"max arm velocity={self.latest_arm_max_velocity:.3f} rad/s; "
                    f"require <= {self.stationary_velocity_threshold:.3f} rad/s "
                    f"for {self.stationary_duration_sec:.2f} s"
                )
            if self.save_images and not self._fresh_image_locked():
                return False, f"no fresh image has been received on {self.image_topic}"
            if self.save_images and self.draw_tag_overlay and self.camera_matrix is None:
                return False, f"no camera info has been received on {self.camera_info_topic}"

            positions = list(self.latest_state.joint_states.position)
            velocities = list(self.latest_state.joint_states.velocity)
            efforts = list(self.latest_state.joint_states.effort)
            if len(positions) not in (14, 26):
                return False, f"state has {len(positions)} positions; expected 14 or 26"
            joint_names = (
                BODY_26_MOTOR_ORDER if len(positions) == 26 else ARM_JOINT_NAMES
            )

            fresh_tags = self._fresh_tags_locked()
            missing = [name for name in self.tag_pose_names if name not in fresh_tags]
            if self.require_tags and missing:
                return False, f"tag not visible or stale: {missing}"

            transforms = self.kinematics.frame_transforms(positions, self.tracked_frames)
            sample_index = len(self.samples) + 1
            image_file = ""
            image_stamp = None
            image_frame = ""
            if self.save_images:
                os.makedirs(self.image_output_dir, exist_ok=True)
                width = max(2, len(str(self.max_samples if self.max_samples > 0 else sample_index)))
                image_file = os.path.join(
                    self.image_output_dir, f"{sample_index:0{width}d}.{self.image_extension}"
                )
                image_to_save = self.latest_image.copy()
                if self.draw_tag_overlay:
                    self._draw_tag_overlay_locked(image_to_save, fresh_tags)
                ok = cv2.imwrite(image_file, image_to_save)
                if not ok:
                    return False, f"failed to write image {image_file}"
                if self.latest_image_stamp is not None:
                    image_stamp = {
                        "sec": self.latest_image_stamp[0],
                        "nanosec": self.latest_image_stamp[1],
                    }
                image_frame = self.latest_image_frame

            sample = {
                "t": self.get_clock().now().nanoseconds * 1e-9,
                "state_stamp": {
                    "sec": int(self.latest_state.stamp.sec),
                    "nanosec": int(self.latest_state.stamp.nanosec),
                },
                "q": [float(v) for v in positions],
                "q14": self.kinematics.q14_from_state_positions(positions),
                "settled_joint_angles": {
                    "unit": "rad",
                    "names": list(joint_names),
                    "positions": [float(v) for v in positions],
                    "arm_joint_names": list(ARM_JOINT_NAMES),
                    "arm_positions": self.kinematics.q14_from_state_positions(positions),
                    "max_abs_arm_velocity_rad_s": self.latest_arm_max_velocity,
                    "required_stationary_duration_sec": self.stationary_duration_sec,
                },
                "dq": [float(v) for v in velocities],
                "tau_est": [float(v) for v in efforts],
                "frames": {
                    name: transform_to_dict(transform) for name, transform in transforms.items()
                },
                "tag_poses": {
                    name: {
                        "topic": str(tag["topic"]),
                        "frame_id": str(tag["frame_id"]),
                        "stamp": {
                            "sec": int(tag["stamp_sec"]),
                            "nanosec": int(tag["stamp_nanosec"]),
                        },
                        **transform_to_dict(tag["transform"]),
                    }
                    for name, tag in fresh_tags.items()
                },
            }
            if self.save_images:
                sample["image"] = {
                    "file": image_file,
                    "topic": self.image_topic,
                    "frame_id": image_frame,
                    "stamp": image_stamp,
                }
            self.samples.append(sample)

            frame_names = ", ".join(sample["frames"].keys())
            tag_names = ", ".join(sample["tag_poses"].keys()) or "none"
            image_text = f" | image={image_file}" if image_file else ""
            return True, (
                f"captured sample #{len(self.samples)} | frames=[{frame_names}] | "
                f"tags=[{tag_names}]" + image_text
            )

    def discard_last(self) -> Tuple[bool, str]:
        with self.lock:
            if not self.samples:
                return False, "no sample to discard"
            sample = self.samples.pop()
            sample_number = len(self.samples) + 1
            image_file = str(sample.get("image", {}).get("file", ""))

        if image_file and os.path.isfile(image_file):
            os.remove(image_file)
        return True, f"discarded sample #{sample_number}"

    def save(self) -> None:
        directory = os.path.dirname(os.path.abspath(self.output_file))
        os.makedirs(directory, exist_ok=True)
        data = {
            "metadata": {
                "format": "s4_handeye_samples_v1",
                "mode": "interactive",
                "session_name": self.session_name,
                "session_dir": self.session_dir,
                "urdf": self.kinematics.urdf_path,
                "state_topic": self.state_topic,
                "tracked_frames": self.tracked_frames,
                "tag_pose_topics": self.tag_pose_topics,
                "tag_pose_names": self.tag_pose_names,
                "max_tag_age_sec": self.max_tag_age_sec,
                "image_topic": self.image_topic if self.save_images else "",
                "camera_info_topic": self.camera_info_topic if self.save_images else "",
                "image_output_dir": self.image_output_dir if self.save_images else "",
                "max_image_age_sec": self.max_image_age_sec,
                "draw_tag_overlay": self.draw_tag_overlay,
                "tag_size": self.tag_size,
                "max_samples": self.max_samples,
                "require_stationary": self.require_stationary,
                "stationary_velocity_threshold_rad_s": self.stationary_velocity_threshold,
                "stationary_duration_sec": self.stationary_duration_sec,
            },
            "samples": self.samples,
        }
        fd, tmp_path = tempfile.mkstemp(prefix=".interactive_", suffix=".yaml", dir=directory)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        os.replace(tmp_path, self.output_file)
        print(f"saved {len(self.samples)} samples to {self.output_file}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveSampleRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("")
    print("S4 interactive hand-eye sampler")
    print("Move the arm, release the clutch, wait for STABLE, then verify the tag.")
    print("Commands: c=capture candidate, s=status, w=wait for tag, q=save and quit")
    print("Each candidate requires n=accept or r=discard before the next pose.")
    print(f"Target samples: {node.max_samples if node.max_samples > 0 else 'unlimited'}")
    print(f"Session directory: {node.session_dir}")
    print(f"Samples file: {node.output_file}")
    if node.save_images:
        print(f"Images will be saved to: {node.image_output_dir}")
        if node.draw_tag_overlay:
            print("Saved images will include projected AprilTag outline and axes.")
    print("")

    pending_sample = False
    try:
        while rclpy.ok():
            print(node.status_text())
            command = input("[c/s/w/q] > ").strip().lower()
            if command == "q":
                break
            if command == "s":
                continue
            if command == "w":
                ok = node.wait_for_required_tags()
                print("tag visible" if ok else "tag wait timeout")
                continue
            if command != "c":
                print("Use c to capture after state=OK, robot=STABLE, and tag10:VISIBLE.")
                continue
            ok, message = node.capture()
            print(("OK: " if ok else "SKIP: ") + message)
            if not ok:
                continue

            pending_sample = True
            while rclpy.ok() and pending_sample:
                confirmation = input("[n=accept/r=discard] > ").strip().lower()
                if confirmation == "n":
                    pending_sample = False
                    node.save()
                    print(f"Accepted sample #{len(node.samples)}.")
                elif confirmation == "r":
                    discarded, discard_message = node.discard_last()
                    pending_sample = False
                    print(("OK: " if discarded else "SKIP: ") + discard_message)
                else:
                    print("Press n to accept this sample or r to discard it.")

            if node.max_samples > 0 and len(node.samples) >= node.max_samples:
                print(f"Reached max_samples={node.max_samples}; saving and exiting.")
                break
    except KeyboardInterrupt:
        if pending_sample:
            node.discard_last()
    finally:
        node.save()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
