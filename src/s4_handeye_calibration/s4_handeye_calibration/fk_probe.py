import argparse
from typing import List

from .pose_math import transform_to_dict
from .s4_model import DEFAULT_FRAMES, S4Kinematics


def _parse_positions(raw: str) -> List[float]:
    if not raw:
        return [0.0] * 14
    return [float(item.strip()) for item in raw.split(",") if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute S4 FK for 14 or 26 joint positions.")
    parser.add_argument("--urdf", default="", help="Path to s4_dual_arm.urdf")
    parser.add_argument(
        "--positions",
        default="",
        help="Comma-separated 14-arm or 26-body joint positions. Defaults to 14 zeros.",
    )
    parser.add_argument(
        "--frames",
        default=",".join(DEFAULT_FRAMES),
        help="Comma-separated frame names to print.",
    )
    args = parser.parse_args()

    kinematics = S4Kinematics(args.urdf)
    positions = _parse_positions(args.positions)
    frames = [item.strip() for item in args.frames.split(",") if item.strip()]
    transforms = kinematics.frame_transforms(positions, frames)

    print(f"urdf: {kinematics.urdf_path}")
    print(f"input_positions: {len(positions)}")
    for frame_name, transform in transforms.items():
        pose = transform_to_dict(transform)
        print(frame_name)
        print(f"  translation: {pose['translation']}")
        print(f"  quaternion_xyzw: {pose['quaternion_xyzw']}")


if __name__ == "__main__":
    main()
