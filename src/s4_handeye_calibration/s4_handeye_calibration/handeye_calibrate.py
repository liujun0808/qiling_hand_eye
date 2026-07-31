import argparse
import copy
import os

import yaml

from .handeye_solver import solve_eye_in_hand, solve_eye_to_hand
from .pose_math import transform_from_dict, transform_to_dict, transform_to_xyz_quat


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


def _parse_sample_indices(raw):
    if not raw.strip():
        return []
    indices = []
    for item in raw.split(","):
        value = int(item.strip())
        if value < 1:
            raise ValueError("Excluded sample indices are 1-based and must be positive")
        indices.append(value)
    return sorted(set(indices))


def _parse_joint_sign_overrides(raw):
    overrides = {}
    if not raw.strip():
        return overrides
    for item in raw.split(","):
        name, separator, value = item.strip().partition("=")
        if not separator or not name:
            raise ValueError(
                "Joint sign overrides must use joint_name=sign, separated by commas"
            )
        sign = float(value)
        if sign not in (-1.0, 1.0):
            raise ValueError(f"Joint sign for {name} must be +1 or -1")
        overrides[name] = sign
    return overrides


def prepare_samples(
    data,
    tool_frame,
    exclude_samples="",
    recompute_fk=False,
    urdf="",
    joint_sign_overrides="",
):
    prepared = copy.deepcopy(data)
    source_samples = list(prepared.get("samples", []))
    excluded = _parse_sample_indices(exclude_samples)
    invalid = [index for index in excluded if index > len(source_samples)]
    if invalid:
        raise ValueError(
            f"Excluded sample indices exceed sample count {len(source_samples)}: {invalid}"
        )

    excluded_set = set(excluded)
    prepared["samples"] = [
        sample
        for index, sample in enumerate(source_samples, start=1)
        if index not in excluded_set
    ]

    signs = _parse_joint_sign_overrides(joint_sign_overrides)
    urdf_path = ""
    if recompute_fk:
        from .s4_model import ARM_JOINT_NAMES, S4Kinematics

        unknown = sorted(set(signs) - set(ARM_JOINT_NAMES))
        if unknown:
            raise ValueError(f"Unknown arm joints in sign overrides: {unknown}")
        urdf_path = urdf or str(prepared.get("metadata", {}).get("urdf", ""))
        kinematics = S4Kinematics(urdf_path)
        urdf_path = kinematics.urdf_path
        for sample in prepared["samples"]:
            positions = sample.get("q", [])
            transforms = kinematics.frame_transforms(
                positions,
                [tool_frame],
                joint_signs=signs,
            )
            if tool_frame not in transforms:
                raise ValueError(f"URDF does not contain tool frame {tool_frame}")
            sample.setdefault("frames", {})[tool_frame] = transform_to_dict(
                transforms[tool_frame]
            )
    elif signs:
        raise ValueError("--joint-sign-overrides requires --recompute-fk")

    return prepared, excluded, signs, urdf_path, len(source_samples)


def main() -> None:
    parser = argparse.ArgumentParser(description="Solve S4 hand-eye calibration from YAML samples.")
    parser.add_argument("--samples", required=True, help="Input sample YAML from sample_recorder")
    parser.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    parser.add_argument("--tool-frame", required=True, help="Robot tool frame in recorded samples")
    parser.add_argument("--tag-name", required=True, help="Tag pose stream name in recorded samples")
    parser.add_argument("--output", required=True, help="Output calibration YAML")
    parser.add_argument(
        "--exclude-samples",
        default="",
        help="Comma-separated 1-based sample indices to exclude, for example 3,10.",
    )
    parser.add_argument(
        "--recompute-fk",
        action="store_true",
        help="Recompute the tool pose from each sample's raw q values.",
    )
    parser.add_argument(
        "--urdf",
        default="",
        help="URDF used with --recompute-fk. Defaults to metadata.urdf.",
    )
    parser.add_argument(
        "--joint-sign-overrides",
        default="",
        help="Comma-separated URDF sign overrides, for example joint_name=-1.",
    )
    parser.add_argument(
        "--rotation-weight",
        type=float,
        default=1.0,
        help="Scale rotational residuals during optimization.",
    )
    args = parser.parse_args()

    with open(args.samples, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    data, excluded, signs, urdf_path, input_sample_count = prepare_samples(
        data,
        tool_frame=args.tool_frame,
        exclude_samples=args.exclude_samples,
        recompute_fk=args.recompute_fk,
        urdf=args.urdf,
        joint_sign_overrides=args.joint_sign_overrides,
    )

    if args.mode == "eye_in_hand":
        result = solve_eye_in_hand(
            data, args.tool_frame, args.tag_name, rotation_weight=args.rotation_weight
        )
    else:
        result = solve_eye_to_hand(
            data, args.tool_frame, args.tag_name, rotation_weight=args.rotation_weight
        )

    result["input_sample_count"] = input_sample_count
    result["excluded_sample_indices"] = excluded
    result["fk_recomputed"] = bool(args.recompute_fk)
    result["joint_sign_overrides"] = signs
    if urdf_path:
        result["urdf"] = urdf_path

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        yaml.safe_dump(result, handle, sort_keys=False)

    print(f"mode: {result['mode']}")
    print(f"success: {result['success']}")
    print(f"input_sample_count: {result['input_sample_count']}")
    print(f"sample_count: {result['sample_count']}")
    print(f"excluded_sample_indices: {result['excluded_sample_indices']}")
    print(f"fk_recomputed: {result['fk_recomputed']}")
    print(f"joint_sign_overrides: {result['joint_sign_overrides']}")
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
