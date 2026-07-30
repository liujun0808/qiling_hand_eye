import argparse

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from .pose_math import transform_from_dict, transform_to_xyz_quat


def _auto_key(data):
    if data.get("mode") == "eye_in_hand":
        return "T_tool_camera"
    if data.get("mode") == "eye_to_hand":
        return "T_base_camera"
    if "T_tool_camera" in data:
        return "T_tool_camera"
    if "T_base_camera" in data:
        return "T_base_camera"
    raise ValueError("Cannot infer transform key from calibration file")


def _default_parent(data, key, explicit_parent):
    if explicit_parent:
        return explicit_parent
    if key == "T_tool_camera":
        return data.get("tool_frame", "tool_frame")
    return "base_link"


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Publish a hand-eye calibration as static TF.")
    parser.add_argument("--calibration", required=True, help="Calibration YAML from handeye_calibrate")
    parser.add_argument("--transform-key", default="auto", help="Transform key, or auto")
    parser.add_argument("--parent-frame", default="", help="Override parent frame")
    parser.add_argument("--child-frame", required=True, help="Child frame for the calibrated camera/tag")
    parsed, ros_args = parser.parse_known_args(args)

    with open(parsed.calibration, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    key = _auto_key(data) if parsed.transform_key == "auto" else parsed.transform_key
    transform = transform_from_dict(data[key])
    pose = transform_to_xyz_quat(transform)
    parent_frame = _default_parent(data, key, parsed.parent_frame)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("s4_calibration_static_tf_publisher")
    broadcaster = StaticTransformBroadcaster(node)

    msg = TransformStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = parsed.child_frame
    msg.transform.translation.x = float(pose["translation"][0])
    msg.transform.translation.y = float(pose["translation"][1])
    msg.transform.translation.z = float(pose["translation"][2])
    msg.transform.rotation.x = float(pose["quaternion_xyzw"][0])
    msg.transform.rotation.y = float(pose["quaternion_xyzw"][1])
    msg.transform.rotation.z = float(pose["quaternion_xyzw"][2])
    msg.transform.rotation.w = float(pose["quaternion_xyzw"][3])
    broadcaster.sendTransform(msg)

    print(f"publishing static TF from {parsed.calibration}", flush=True)
    print(f"  key: {key}", flush=True)
    print(f"  parent_frame: {parent_frame}", flush=True)
    print(f"  child_frame: {parsed.child_frame}", flush=True)
    print(f"  translation: {pose['translation']}", flush=True)
    print(f"  quaternion_xyzw: {pose['quaternion_xyzw']}", flush=True)
    print("Press Ctrl-C to stop publishing this static TF.", flush=True)

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
