from __future__ import annotations

import os
import tempfile
from typing import Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mit_msgs.msg import MITLowState
from rclpy.node import Node

from .pose_math import pose_msg_to_transform, transform_to_dict
from .s4_model import DEFAULT_FRAMES, S4Kinematics


class SampleRecorder(Node):
    def __init__(self) -> None:
        super().__init__("s4_sample_recorder")

        self.declare_parameter("state_topic", "/human_lower_state")
        self.declare_parameter("output_file", "s4_handeye_samples.yaml")
        self.declare_parameter("sample_rate_hz", 5.0)
        self.declare_parameter("max_samples", 0)
        self.declare_parameter("record_without_tags", True)
        self.declare_parameter("tag_pose_topics", [])
        self.declare_parameter("tag_pose_names", [])
        self.declare_parameter("tag_pose_msg_type", "PoseStamped")
        self.declare_parameter("tracked_frames", DEFAULT_FRAMES)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("max_tag_age_sec", 0.5)

        self._state_topic = self.get_parameter("state_topic").value
        self._output_file = self.get_parameter("output_file").value
        self._sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        self._max_samples = int(self.get_parameter("max_samples").value)
        self._record_without_tags = bool(self.get_parameter("record_without_tags").value)
        self._tag_pose_topics = list(self.get_parameter("tag_pose_topics").value)
        self._tag_pose_names = list(self.get_parameter("tag_pose_names").value)
        self._tag_pose_msg_type = self.get_parameter("tag_pose_msg_type").value
        self._tracked_frames = list(self.get_parameter("tracked_frames").value)
        self._max_tag_age_sec = float(self.get_parameter("max_tag_age_sec").value)

        if self._sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")

        if self._tag_pose_topics and not self._tag_pose_names:
            self._tag_pose_names = [f"tag_{i}" for i in range(len(self._tag_pose_topics))]
        if len(self._tag_pose_names) != len(self._tag_pose_topics):
            raise ValueError("tag_pose_names must be empty or have the same length as tag_pose_topics")

        self._kinematics = S4Kinematics(self.get_parameter("urdf_path").value)
        self._latest_state: Optional[MITLowState] = None
        self._latest_tags: Dict[str, Dict[str, object]] = {}
        self._samples: List[Dict[str, object]] = []

        self._state_sub = self.create_subscription(
            MITLowState, self._state_topic, self._state_callback, 10
        )
        self._tag_subs = []
        for name, topic in zip(self._tag_pose_names, self._tag_pose_topics):
            msg_type = PoseWithCovarianceStamped if self._tag_pose_msg_type == "PoseWithCovarianceStamped" else PoseStamped
            self._tag_subs.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, tag_name=name, tag_topic=topic: self._tag_callback(
                        tag_name, tag_topic, msg
                    ),
                    10,
                )
            )

        self._timer = self.create_timer(1.0 / self._sample_rate_hz, self._timer_callback)
        self.get_logger().info(
            "Recording samples: "
            f"state_topic={self._state_topic}, output_file={self._output_file}, "
            f"sample_rate_hz={self._sample_rate_hz}, tag_topics={self._tag_pose_topics}"
        )

    def _state_callback(self, msg: MITLowState) -> None:
        self._latest_state = msg

    def _tag_callback(self, name: str, topic: str, msg) -> None:
        self._latest_tags[name] = {
            "topic": topic,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "transform": pose_msg_to_transform(msg),
            "received_time_sec": self.get_clock().now().nanoseconds * 1e-9,
        }

    def _fresh_tags(self) -> Dict[str, Dict[str, object]]:
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh: Dict[str, Dict[str, object]] = {}
        for name, tag in self._latest_tags.items():
            if now - float(tag["received_time_sec"]) <= self._max_tag_age_sec:
                fresh[name] = tag
        return fresh

    def _timer_callback(self) -> None:
        if self._max_samples > 0 and len(self._samples) >= self._max_samples:
            return
        if self._latest_state is None:
            return

        positions = list(self._latest_state.joint_states.position)
        velocities = list(self._latest_state.joint_states.velocity)
        efforts = list(self._latest_state.joint_states.effort)
        if len(positions) not in (14, 26):
            self.get_logger().warn(
                f"Skipping state with {len(positions)} positions; expected 14 or 26",
                throttle_duration_sec=2.0,
            )
            return

        fresh_tags = self._fresh_tags()
        if self._tag_pose_topics and not self._record_without_tags:
            missing = [name for name in self._tag_pose_names if name not in fresh_tags]
            if missing:
                self.get_logger().warn(
                    f"Waiting for fresh tag poses: {missing}",
                    throttle_duration_sec=2.0,
                )
                return

        transforms = self._kinematics.frame_transforms(positions, self._tracked_frames)
        sample = {
            "t": self.get_clock().now().nanoseconds * 1e-9,
            "state_stamp": {
                "sec": int(self._latest_state.stamp.sec),
                "nanosec": int(self._latest_state.stamp.nanosec),
            },
            "q": [float(v) for v in positions],
            "q14": self._kinematics.q14_from_state_positions(positions),
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
        self._samples.append(sample)

        if len(self._samples) % max(1, int(self._sample_rate_hz)) == 0:
            self.get_logger().info(f"Recorded {len(self._samples)} samples")

    def save(self) -> None:
        directory = os.path.dirname(os.path.abspath(self._output_file))
        if directory:
            os.makedirs(directory, exist_ok=True)

        data = {
            "metadata": {
                "format": "s4_handeye_samples_v1",
                "urdf": self._kinematics.urdf_path,
                "state_topic": self._state_topic,
                "tracked_frames": self._tracked_frames,
                "tag_pose_topics": self._tag_pose_topics,
                "tag_pose_names": self._tag_pose_names,
                "sample_rate_hz": self._sample_rate_hz,
            },
            "samples": self._samples,
        }

        fd, tmp_path = tempfile.mkstemp(prefix=".samples_", suffix=".yaml", dir=directory or ".")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
        os.replace(tmp_path, self._output_file)
        self.get_logger().info(f"Saved {len(self._samples)} samples to {self._output_file}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SampleRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
