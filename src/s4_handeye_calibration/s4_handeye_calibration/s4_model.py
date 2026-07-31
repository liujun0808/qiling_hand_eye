from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional

import numpy as np
import pinocchio as pin
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

BODY_26_MOTOR_ORDER = [
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_foot_pitch_joint",
    "left_foot_roll_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_foot_pitch_joint",
    "right_foot_roll_joint",
    *ARM_JOINT_NAMES,
]

DEFAULT_FRAMES = [
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "LH_hand_base_link",
    "RH_hand_base_link",
]


def default_urdf_path() -> str:
    try:
        share_dir = get_package_share_directory("qi_robot_description")
        return os.path.join(share_dir, "urdf", "s4_dual_arm.urdf")
    except PackageNotFoundError:
        cwd_path = os.path.join(
            os.getcwd(), "src", "qi_robot_description", "urdf", "s4_dual_arm.urdf"
        )
        return cwd_path


class S4Kinematics:
    def __init__(self, urdf_path: str = "") -> None:
        self.urdf_path = urdf_path or default_urdf_path()
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.arm_joint_names = list(ARM_JOINT_NAMES)
        self.body_26_motor_order = list(BODY_26_MOTOR_ORDER)
        self._joint_q_index = self._build_joint_q_index()
        self._joint_v_index = self._build_joint_v_index()

    def _build_joint_q_index(self) -> Dict[str, int]:
        mapping: Dict[str, int] = {}
        for name in self.arm_joint_names:
            if not self.model.existJointName(name):
                raise ValueError(f"URDF model does not contain movable joint {name}")
            joint_id = self.model.getJointId(name)
            mapping[name] = self.model.joints[joint_id].idx_q
        return mapping

    def _build_joint_v_index(self) -> Dict[str, int]:
        mapping: Dict[str, int] = {}
        for name in self.arm_joint_names:
            joint_id = self.model.getJointId(name)
            mapping[name] = self.model.joints[joint_id].idx_v
        return mapping

    def q_from_state_positions(
        self,
        positions: Iterable[float],
        joint_signs: Optional[Dict[str, float]] = None,
    ) -> np.ndarray:
        values = list(positions)
        q = pin.neutral(self.model)
        signs = joint_signs or {}

        if len(values) == 26:
            source_names = self.body_26_motor_order
        elif len(values) == 14:
            source_names = self.arm_joint_names
        else:
            raise ValueError(f"Expected 14 or 26 joint positions, got {len(values)}")

        for index, name in enumerate(source_names):
            q_index = self._joint_q_index.get(name)
            if q_index is not None:
                q[q_index] = values[index] * float(signs.get(name, 1.0))
        return q

    def q14_from_state_positions(self, positions: Iterable[float]) -> List[float]:
        values = list(positions)
        if len(values) == 14:
            return [float(v) for v in values]
        if len(values) == 26:
            return [float(values[index]) for index in range(12, 26)]
        raise ValueError(f"Expected 14 or 26 joint positions, got {len(values)}")

    def arm_gravity_torques(self, positions: Iterable[float]) -> List[float]:
        q = self.q_from_state_positions(positions)
        tau = pin.computeGeneralizedGravity(self.model, self.data, q)
        return [float(tau[self._joint_v_index[name]]) for name in self.arm_joint_names]

    def frame_transform(self, q: np.ndarray, frame_name: str) -> np.ndarray:
        if not self.model.existFrame(frame_name):
            raise ValueError(f"URDF model does not contain frame {frame_name}")
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        frame_id = self.model.getFrameId(frame_name)
        placement = self.data.oMf[frame_id]
        transform = np.eye(4, dtype=float)
        transform[:3, :3] = placement.rotation
        transform[:3, 3] = placement.translation
        return transform

    def frame_transforms(
        self,
        positions: Iterable[float],
        frame_names: List[str],
        joint_signs: Optional[Dict[str, float]] = None,
    ) -> Dict[str, np.ndarray]:
        q = self.q_from_state_positions(positions, joint_signs=joint_signs)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        transforms: Dict[str, np.ndarray] = {}
        for frame_name in frame_names:
            if not self.model.existFrame(frame_name):
                continue
            placement = self.data.oMf[self.model.getFrameId(frame_name)]
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = placement.rotation
            transform[:3, 3] = placement.translation
            transforms[frame_name] = transform
        return transforms
