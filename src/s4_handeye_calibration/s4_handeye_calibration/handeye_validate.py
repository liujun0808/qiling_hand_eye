import argparse
import math
import os

import numpy as np
import yaml

from .handeye_calibrate import prepare_samples
from .pose_math import (
    mean_transform,
    relative_error_vector,
    residual_stats,
    transform_from_dict,
    transform_to_dict,
)


def _sample_base_tag(sample, tool_frame, tag_name, tool_camera):
    frames = sample.get("frames", {})
    tag_poses = sample.get("tag_poses", {})
    if tool_frame not in frames:
        raise ValueError(f"Sample does not contain tool frame {tool_frame}")
    if tag_name not in tag_poses:
        raise ValueError(f"Sample does not contain tag pose {tag_name}")
    base_tool = transform_from_dict(frames[tool_frame])
    camera_tag = transform_from_dict(tag_poses[tag_name])
    return base_tool @ tool_camera @ camera_tag


def _sample_base_camera(sample, tool_frame, tag_name, tool_tag):
    frames = sample.get("frames", {})
    tag_poses = sample.get("tag_poses", {})
    if tool_frame not in frames:
        raise ValueError(f"Sample does not contain tool frame {tool_frame}")
    if tag_name not in tag_poses:
        raise ValueError(f"Sample does not contain tag pose {tag_name}")
    base_tool = transform_from_dict(frames[tool_frame])
    camera_tag = transform_from_dict(tag_poses[tag_name])
    return base_tool @ tool_tag @ np.linalg.inv(camera_tag)


def _errors(reference, transforms):
    return np.asarray(
        [relative_error_vector(reference, transform) for transform in transforms],
        dtype=float,
    )


def _metrics(errors):
    metrics = residual_stats(errors)
    metrics["translation_rmse_mm"] = metrics["translation_rmse_m"] * 1000.0
    metrics["translation_max_mm"] = metrics["translation_max_m"] * 1000.0
    metrics["rotation_rmse_deg"] = math.degrees(metrics["rotation_rmse_rad"])
    metrics["rotation_max_deg"] = math.degrees(metrics["rotation_max_rad"])
    return metrics


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Validate an eye-in-hand or eye-to-hand calibration using independent samples."
        )
    )
    parser.add_argument("--samples", required=True, help="Independent validation sample YAML")
    parser.add_argument("--calibration", required=True, help="Hand-eye calibration YAML")
    parser.add_argument(
        "--tool-frame",
        default="",
        help="Tool frame. Defaults to the frame stored in the calibration.",
    )
    parser.add_argument(
        "--tag-name",
        default="",
        help="Tag pose name. Defaults to the name stored in the calibration.",
    )
    parser.add_argument(
        "--reference-count",
        type=int,
        default=5,
        help="Number of leading samples used only to estimate the tag's new fixed pose.",
    )
    parser.add_argument(
        "--exclude-samples",
        default="",
        help="Comma-separated 1-based validation sample indices to exclude.",
    )
    parser.add_argument(
        "--recompute-fk",
        action="store_true",
        help="Recompute tool FK from each validation sample's raw q values.",
    )
    parser.add_argument(
        "--urdf",
        default="",
        help="URDF used with --recompute-fk. Defaults to validation metadata.urdf.",
    )
    parser.add_argument(
        "--joint-sign-overrides",
        default="",
        help="Comma-separated URDF sign overrides, for example joint_name=-1.",
    )
    parser.add_argument("--output", required=True, help="Output validation report YAML")
    args = parser.parse_args()

    with open(args.calibration, "r", encoding="utf-8") as handle:
        calibration = yaml.safe_load(handle)
    calibration_mode = str(calibration.get("mode", ""))
    if calibration_mode == "eye_in_hand":
        if "T_tool_camera" not in calibration:
            raise ValueError("Eye-in-hand calibration must contain T_tool_camera")
    elif calibration_mode == "eye_to_hand":
        if "T_base_camera" not in calibration or "T_tool_tag" not in calibration:
            raise ValueError(
                "Eye-to-hand calibration must contain T_base_camera and T_tool_tag"
            )
    else:
        raise ValueError("Calibration mode must be eye_in_hand or eye_to_hand")

    tool_frame = args.tool_frame or str(calibration.get("tool_frame", ""))
    tag_name = args.tag_name or str(calibration.get("tag_name", ""))
    if not tool_frame or not tag_name:
        raise ValueError("tool frame and tag name must be provided or stored in calibration")

    with open(args.samples, "r", encoding="utf-8") as handle:
        sample_data = yaml.safe_load(handle)
    prepared, excluded, signs, urdf_path, input_count = prepare_samples(
        sample_data,
        tool_frame=tool_frame,
        exclude_samples=args.exclude_samples,
        recompute_fk=args.recompute_fk,
        urdf=args.urdf,
        joint_sign_overrides=args.joint_sign_overrides,
    )
    samples = prepared.get("samples", [])
    excluded_set = set(excluded)
    source_indices = [
        index for index in range(1, input_count + 1) if index not in excluded_set
    ]
    common_report = {
        "samples_file": os.path.abspath(args.samples),
        "calibration_file": os.path.abspath(args.calibration),
        "tool_frame": tool_frame,
        "tag_name": tag_name,
        "input_sample_count": input_count,
        "used_sample_count": len(samples),
        "excluded_sample_indices": excluded,
        "fk_recomputed": bool(args.recompute_fk),
        "joint_sign_overrides": signs,
        "urdf": urdf_path,
    }

    if calibration_mode == "eye_in_hand":
        if args.reference_count < 1:
            raise ValueError("--reference-count must be at least 1")
        if args.reference_count >= len(samples):
            raise ValueError(
                "--reference-count must leave at least one independent evaluation sample"
            )

        tool_camera = transform_from_dict(calibration["T_tool_camera"])
        base_tags = [
            _sample_base_tag(sample, tool_frame, tag_name, tool_camera)
            for sample in samples
        ]
        reference = mean_transform(base_tags[: args.reference_count])
        evaluation_errors = _errors(reference, base_tags[args.reference_count :])
        all_mean = mean_transform(base_tags)
        all_errors = _errors(all_mean, base_tags)

        per_sample = []
        for position, (source_index, base_tag) in enumerate(
            zip(source_indices, base_tags)
        ):
            error = relative_error_vector(reference, base_tag)
            per_sample.append(
                {
                    "sample_index": source_index,
                    "role": (
                        "reference"
                        if position < args.reference_count
                        else "evaluation"
                    ),
                    "T_base_tag": transform_to_dict(base_tag),
                    "translation_error_m": float(np.linalg.norm(error[:3])),
                    "translation_error_mm": float(np.linalg.norm(error[:3]) * 1000.0),
                    "rotation_error_rad": float(np.linalg.norm(error[3:])),
                    "rotation_error_deg": float(
                        math.degrees(np.linalg.norm(error[3:]))
                    ),
                }
            )

        report = {
            "mode": "eye_in_hand_fixed_tag_consistency_validation",
            **common_report,
            "reference_sample_indices": source_indices[: args.reference_count],
            "evaluation_sample_indices": source_indices[args.reference_count :],
            "T_base_tag_reference": transform_to_dict(reference),
            "evaluation_metrics": _metrics(evaluation_errors),
            "all_sample_dispersion": _metrics(all_errors),
            "samples": per_sample,
        }
    else:
        base_camera_reference = transform_from_dict(calibration["T_base_camera"])
        tool_tag = transform_from_dict(calibration["T_tool_tag"])
        base_cameras = [
            _sample_base_camera(sample, tool_frame, tag_name, tool_tag)
            for sample in samples
        ]
        evaluation_errors = _errors(base_camera_reference, base_cameras)
        all_mean = mean_transform(base_cameras)
        all_errors = _errors(all_mean, base_cameras)

        per_sample = []
        for source_index, base_camera in zip(source_indices, base_cameras):
            error = relative_error_vector(base_camera_reference, base_camera)
            per_sample.append(
                {
                    "sample_index": source_index,
                    "role": "evaluation",
                    "T_base_camera": transform_to_dict(base_camera),
                    "translation_error_m": float(np.linalg.norm(error[:3])),
                    "translation_error_mm": float(np.linalg.norm(error[:3]) * 1000.0),
                    "rotation_error_rad": float(np.linalg.norm(error[3:])),
                    "rotation_error_deg": float(
                        math.degrees(np.linalg.norm(error[3:]))
                    ),
                }
            )

        report = {
            "mode": "eye_to_hand_fixed_camera_consistency_validation",
            **common_report,
            "evaluation_sample_indices": source_indices,
            "T_base_camera_reference": transform_to_dict(base_camera_reference),
            "T_base_camera_mean": transform_to_dict(all_mean),
            "evaluation_metrics": _metrics(evaluation_errors),
            "all_sample_dispersion": _metrics(all_errors),
            "samples": per_sample,
        }

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        yaml.safe_dump(report, handle, sort_keys=False)

    metrics = report["evaluation_metrics"]
    print(f"mode: {report['mode']}")
    print(f"input_sample_count: {input_count}")
    print(f"used_sample_count: {len(samples)}")
    if "reference_sample_indices" in report:
        print(f"reference_sample_indices: {report['reference_sample_indices']}")
    print(f"evaluation_sample_indices: {report['evaluation_sample_indices']}")
    print(f"excluded_sample_indices: {excluded}")
    print(f"translation_rmse_mm: {metrics['translation_rmse_mm']:.3f}")
    print(f"translation_max_mm: {metrics['translation_max_mm']:.3f}")
    print(f"rotation_rmse_deg: {metrics['rotation_rmse_deg']:.3f}")
    print(f"rotation_max_deg: {metrics['rotation_max_deg']:.3f}")
    print(f"wrote: {args.output}")


if __name__ == "__main__":
    main()
