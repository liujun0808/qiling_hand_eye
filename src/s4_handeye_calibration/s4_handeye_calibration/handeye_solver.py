from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
from scipy.optimize import least_squares

from .pose_math import (
    mean_transform,
    relative_error_vector,
    residual_stats,
    se3_from_vector,
    transform_from_dict,
    transform_to_dict,
    vector_from_se3,
)


def _sample_pairs(
    data: Dict[str, object], tool_frame: str, tag_name: str
) -> List[Tuple[np.ndarray, np.ndarray]]:
    pairs: List[Tuple[np.ndarray, np.ndarray]] = []
    for sample in data.get("samples", []):
        frames = sample.get("frames", {})
        tag_poses = sample.get("tag_poses", {})
        if tool_frame not in frames or tag_name not in tag_poses:
            continue
        pairs.append(
            (
                transform_from_dict(frames[tool_frame]),
                transform_from_dict(tag_poses[tag_name]),
            )
        )
    return pairs


def _pack(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.concatenate([vector_from_se3(a), vector_from_se3(b)])


def _unpack(x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return se3_from_vector(x[:6]), se3_from_vector(x[6:12])


def solve_eye_in_hand(
    data: Dict[str, object],
    tool_frame: str,
    tag_name: str,
    rotation_weight: float = 1.0,
) -> Dict[str, object]:
    """Solve T_tool_camera and T_base_tag from samples.

    The residual enforces:
        T_base_tool_i * T_tool_camera * T_camera_tag_i == T_base_tag
    """

    pairs = _sample_pairs(data, tool_frame, tag_name)
    if len(pairs) < 3:
        raise ValueError("At least 3 valid samples are required for eye-in-hand calibration")

    initial_tool_camera = np.eye(4)
    initial_base_tag = mean_transform(
        [base_tool @ initial_tool_camera @ camera_tag for base_tool, camera_tag in pairs]
    )

    def residual(x: np.ndarray) -> np.ndarray:
        tool_camera, base_tag = _unpack(x)
        values = []
        for base_tool, camera_tag in pairs:
            predicted_base_tag = base_tool @ tool_camera @ camera_tag
            error = relative_error_vector(base_tag, predicted_base_tag)
            error[3:6] *= rotation_weight
            values.append(error)
        return np.concatenate(values)

    result = least_squares(
        residual,
        _pack(initial_tool_camera, initial_base_tag),
        loss="soft_l1",
        f_scale=0.05,
        max_nfev=500,
    )
    tool_camera, base_tag = _unpack(result.x)
    residuals = residual(result.x).reshape(-1, 6)
    residuals[:, 3:6] /= rotation_weight

    return {
        "mode": "eye_in_hand",
        "tool_frame": tool_frame,
        "tag_name": tag_name,
        "sample_count": len(pairs),
        "success": bool(result.success),
        "message": result.message,
        "cost": float(result.cost),
        "T_tool_camera": transform_to_dict(tool_camera),
        "T_base_tag": transform_to_dict(base_tag),
        "residuals": residual_stats(residuals),
    }


def solve_eye_to_hand(
    data: Dict[str, object],
    tool_frame: str,
    tag_name: str,
    rotation_weight: float = 1.0,
) -> Dict[str, object]:
    """Solve T_base_camera and T_tool_tag from samples.

    The residual enforces:
        T_base_tool_i * T_tool_tag == T_base_camera * T_camera_tag_i
    """

    pairs = _sample_pairs(data, tool_frame, tag_name)
    if len(pairs) < 3:
        raise ValueError("At least 3 valid samples are required for eye-to-hand calibration")

    initial_tool_tag = np.eye(4)
    initial_base_camera = mean_transform(
        [base_tool @ initial_tool_tag @ np.linalg.inv(camera_tag) for base_tool, camera_tag in pairs]
    )

    def residual(x: np.ndarray) -> np.ndarray:
        base_camera, tool_tag = _unpack(x)
        values = []
        for base_tool, camera_tag in pairs:
            from_robot = base_tool @ tool_tag
            from_camera = base_camera @ camera_tag
            error = relative_error_vector(from_robot, from_camera)
            error[3:6] *= rotation_weight
            values.append(error)
        return np.concatenate(values)

    result = least_squares(
        residual,
        _pack(initial_base_camera, initial_tool_tag),
        loss="soft_l1",
        f_scale=0.05,
        max_nfev=500,
    )
    base_camera, tool_tag = _unpack(result.x)
    residuals = residual(result.x).reshape(-1, 6)
    residuals[:, 3:6] /= rotation_weight

    return {
        "mode": "eye_to_hand",
        "tool_frame": tool_frame,
        "tag_name": tag_name,
        "sample_count": len(pairs),
        "success": bool(result.success),
        "message": result.message,
        "cost": float(result.cost),
        "T_base_camera": transform_to_dict(base_camera),
        "T_tool_tag": transform_to_dict(tool_tag),
        "residuals": residual_stats(residuals),
    }
