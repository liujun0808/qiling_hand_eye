from __future__ import annotations

from typing import Dict, Iterable, List

import numpy as np
from scipy.spatial.transform import Rotation


def transform_from_xyz_quat(xyz: Iterable[float], quat_xyzw: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(list(quat_xyzw)).as_matrix()
    transform[:3, 3] = np.asarray(list(xyz), dtype=float)
    return transform


def transform_to_xyz_quat(transform: np.ndarray) -> Dict[str, List[float]]:
    rotation = Rotation.from_matrix(transform[:3, :3])
    return {
        "translation": transform[:3, 3].astype(float).tolist(),
        "quaternion_xyzw": rotation.as_quat().astype(float).tolist(),
    }


def pose_msg_to_transform(msg) -> np.ndarray:
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    return transform_from_xyz_quat(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def transform_to_dict(transform: np.ndarray) -> Dict[str, object]:
    pose = transform_to_xyz_quat(transform)
    pose["matrix"] = transform.astype(float).reshape(-1).tolist()
    return pose


def transform_from_dict(data: Dict[str, object]) -> np.ndarray:
    if "matrix" in data:
        return np.asarray(data["matrix"], dtype=float).reshape(4, 4)
    return transform_from_xyz_quat(data["translation"], data["quaternion_xyzw"])


def se3_from_vector(vector: Iterable[float]) -> np.ndarray:
    values = np.asarray(list(vector), dtype=float)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_rotvec(values[3:6]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def vector_from_se3(transform: np.ndarray) -> np.ndarray:
    values = np.zeros(6, dtype=float)
    values[:3] = transform[:3, 3]
    values[3:6] = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    return values


def relative_error_vector(expected: np.ndarray, actual: np.ndarray) -> np.ndarray:
    delta = np.linalg.inv(expected) @ actual
    return vector_from_se3(delta)


def mean_transform(transforms: List[np.ndarray]) -> np.ndarray:
    if not transforms:
        return np.eye(4, dtype=float)

    out = np.eye(4, dtype=float)
    out[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    rotations = Rotation.from_matrix([t[:3, :3] for t in transforms])
    out[:3, :3] = rotations.mean().as_matrix()
    return out


def residual_stats(residuals: np.ndarray) -> Dict[str, float]:
    residuals = residuals.reshape(-1, 6)
    trans_norm = np.linalg.norm(residuals[:, :3], axis=1)
    rot_norm = np.linalg.norm(residuals[:, 3:], axis=1)
    return {
        "translation_rmse_m": float(np.sqrt(np.mean(trans_norm ** 2))),
        "translation_max_m": float(np.max(trans_norm)),
        "rotation_rmse_rad": float(np.sqrt(np.mean(rot_norm ** 2))),
        "rotation_max_rad": float(np.max(rot_norm)),
        "sample_count": int(residuals.shape[0]),
    }
