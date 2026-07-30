import argparse
import os

import yaml

from .handeye_solver import solve_eye_in_hand, solve_eye_to_hand
from .pose_math import transform_from_dict, transform_to_xyz_quat


def _result_transform_key(result):
    if result["mode"] == "eye_in_hand":
        return "T_tool_camera"
    return "T_base_camera"


def _print_transform(label, data):
    transform = transform_from_dict(data)
    pose = transform_to_xyz_quat(transform)
    print(f"{label}:")
    print(f"  translation: {pose['translation']}")
    print(f"  quaternion_xyzw: {pose['quaternion_xyzw']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Solve S4 hand-eye calibration from YAML samples.")
    parser.add_argument("--samples", required=True, help="Input sample YAML from sample_recorder")
    parser.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    parser.add_argument("--tool-frame", required=True, help="Robot tool frame in recorded samples")
    parser.add_argument("--tag-name", required=True, help="Tag pose stream name in recorded samples")
    parser.add_argument("--output", required=True, help="Output calibration YAML")
    parser.add_argument(
        "--rotation-weight",
        type=float,
        default=1.0,
        help="Scale rotational residuals during optimization.",
    )
    args = parser.parse_args()

    with open(args.samples, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if args.mode == "eye_in_hand":
        result = solve_eye_in_hand(
            data, args.tool_frame, args.tag_name, rotation_weight=args.rotation_weight
        )
    else:
        result = solve_eye_to_hand(
            data, args.tool_frame, args.tag_name, rotation_weight=args.rotation_weight
        )

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        yaml.safe_dump(result, handle, sort_keys=False)

    print(f"mode: {result['mode']}")
    print(f"success: {result['success']}")
    print(f"sample_count: {result['sample_count']}")
    print(f"translation_rmse_m: {result['residuals']['translation_rmse_m']:.6f}")
    print(f"rotation_rmse_rad: {result['residuals']['rotation_rmse_rad']:.6f}")
    transform_key = _result_transform_key(result)
    _print_transform(transform_key, result[transform_key])
    if result["mode"] == "eye_in_hand":
        _print_transform("T_base_tag", result["T_base_tag"])
    else:
        _print_transform("T_tool_tag", result["T_tool_tag"])
    print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
