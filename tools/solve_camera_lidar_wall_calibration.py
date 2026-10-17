#!/usr/bin/env python3
"""Solve camera/2D-lidar extrinsics from a checkerboard fixed to a flat wall.

The camera observes the checkerboard front plane while a low 2D lidar observes
the wall behind it.  ``board_thickness_mm`` therefore shifts the visual plane
back to the lidar plane.  The script deliberately writes a candidate report;
it never edits URDF or runtime configuration.

Only NumPy is required.  The checkerboard corners are read from the compact
capture produced by ``capture_camera_lidar_calibration.py``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np


def quaternion_matrix(xyzw):
    x, y, z, w = (float(v) for v in xyzw)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-15:
        raise ValueError("zero-length quaternion")
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.asarray(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


def transform_matrix(translation, rotation_xyzw):
    result = np.eye(4, dtype=float)
    result[:3, :3] = quaternion_matrix(rotation_xyzw)
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def rotation_vector_matrix(vector):
    vector = np.asarray(vector, dtype=float)
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        x, y, z = vector
        skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        return np.eye(3) + skew
    axis = vector / theta
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(theta) * skew + (1.0 - math.cos(theta)) * (skew @ skew)


def matrix_rpy(rotation):
    """Return fixed-axis roll, pitch, yaw, matching URDF rpy."""
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.asarray([roll, pitch, yaw])


def undistort_points(points_px, camera_info):
    k = np.asarray(camera_info["k"], dtype=float).reshape(3, 3)
    d = list(camera_info.get("d", [])) + [0.0] * 8
    k1, k2, p1, p2, k3, k4, k5, k6 = d[:8]
    xd = (points_px[:, 0] - k[0, 2]) / k[0, 0]
    yd = (points_px[:, 1] - k[1, 2]) / k[1, 1]
    x, y = xd.copy(), yd.copy()
    for _ in range(12):
        r2 = x * x + y * y
        numerator = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3
        denominator = 1.0 + k4 * r2 + k5 * r2**2 + k6 * r2**3
        radial = numerator / np.maximum(denominator, 1e-12)
        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - delta_x) / radial
        y = (yd - delta_y) / radial
    return np.column_stack((x, y)), k


def normalized_dlt(object_xy, image_xy):
    def normalize(values):
        center = values.mean(axis=0)
        shifted = values - center
        scale = math.sqrt(2.0) / max(float(np.sqrt((shifted**2).sum(axis=1)).mean()), 1e-12)
        matrix = np.asarray(
            [[scale, 0.0, -scale * center[0]], [0.0, scale, -scale * center[1]], [0.0, 0.0, 1.0]]
        )
        homogeneous = np.column_stack((values, np.ones(len(values))))
        return (matrix @ homogeneous.T).T[:, :2], matrix

    source, source_t = normalize(object_xy)
    target, target_t = normalize(image_xy)
    rows = []
    for (x, y), (u, v) in zip(source, target):
        rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.asarray(rows, dtype=float), full_matrices=False)
    normalized_h = vt[-1].reshape(3, 3)
    result = np.linalg.inv(target_t) @ normalized_h @ source_t
    return result / result[2, 2]


def checkerboard_plane(corners_px, camera_info, cols, rows, square_size_m):
    normalized, k = undistort_points(corners_px, camera_info)
    x_values = (np.arange(cols, dtype=float) - (cols - 1.0) / 2.0) * square_size_m
    y_values = (np.arange(rows, dtype=float) - (rows - 1.0) / 2.0) * square_size_m
    object_xy = np.asarray([(x, y) for y in y_values for x in x_values])
    homography = normalized_dlt(object_xy, normalized)
    if homography[2, 2] < 0.0:
        homography *= -1.0
    scale = 0.5 * (np.linalg.norm(homography[:, 0]) + np.linalg.norm(homography[:, 1]))
    first = homography[:, 0] / scale
    second = homography[:, 1] / scale
    third = np.cross(first, second)
    raw_rotation = np.column_stack((first, second, third))
    u, _, vt = np.linalg.svd(raw_rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    translation = homography[:, 2] / scale
    normal = rotation[:, 2]
    if float(normal @ translation) < 0.0:
        normal *= -1.0

    projected_h = (homography @ np.column_stack((object_xy, np.ones(len(object_xy)))).T).T
    projected = projected_h[:, :2] / projected_h[:, 2:3]
    errors_px = np.linalg.norm((projected - normalized) * np.asarray([k[0, 0], k[1, 1]]), axis=1)
    return {
        "center": translation,
        "normal": normal,
        "rotation": rotation,
        "reprojection_rms_px": float(np.sqrt(np.mean(errors_px**2))),
        "reprojection_p95_px": float(np.percentile(errors_px, 95)),
    }


def tf_chain(metadata, root, target):
    transforms = {}
    for item in metadata["tf_static"]:
        transforms[item["child"]] = (
            item["parent"],
            transform_matrix(item["translation"], item["rotation_xyzw"]),
        )

    def to_root(frame, seen=None):
        if frame == root:
            return np.eye(4)
        seen = set() if seen is None else seen
        if frame in seen or frame not in transforms:
            raise ValueError(f"cannot build static TF chain {root} -> {target}; stopped at {frame}")
        seen.add(frame)
        parent, parent_from_child = transforms[frame]
        return to_root(parent, seen) @ parent_from_child

    return to_root(target)


def scan_points(scan_path):
    scan = np.load(scan_path)
    ranges = np.asarray(scan["ranges"], dtype=float)
    angles = float(scan["angle_min"]) + np.arange(len(ranges)) * float(scan["angle_increment"])
    valid = np.isfinite(ranges) & (ranges >= float(scan["range_min"])) & (ranges <= float(scan["range_max"]))
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    return points, angles[valid]


def fit_local_wall(points_laser, angles, predicted_angle, initial_camera_from_laser, wall_normal, wall_point, rng):
    camera_points = (initial_camera_from_laser[:3, :3] @ points_laser.T).T + initial_camera_from_laser[:3, 3]
    plane_error = np.abs((camera_points - wall_point) @ wall_normal)
    angle_error = np.abs(np.arctan2(np.sin(angles - predicted_angle), np.cos(angles - predicted_angle)))
    candidate_mask = (plane_error < 0.055) & (angle_error < math.radians(38.0))
    candidates = points_laser[candidate_mask]
    if len(candidates) < 18:
        raise ValueError(f"only {len(candidates)} lidar candidates near the visual wall")

    xy = candidates[:, :2]
    best_inliers = None
    best_span = -1.0
    for _ in range(400):
        indices = rng.choice(len(xy), size=2, replace=False)
        direction = xy[indices[1]] - xy[indices[0]]
        length = float(np.linalg.norm(direction))
        if length < 0.08:
            continue
        direction /= length
        normal = np.asarray([-direction[1], direction[0]])
        residual = np.abs((xy - xy[indices[0]]) @ normal)
        inliers = residual < 0.007
        if inliers.sum() < 16:
            continue
        projected = xy[inliers] @ direction
        span = float(np.percentile(projected, 95) - np.percentile(projected, 5))
        score = int(inliers.sum()) + min(span, 1.0) * 15.0
        if best_inliers is None or score > int(best_inliers.sum()) + min(best_span, 1.0) * 15.0:
            best_inliers, best_span = inliers, span
    if best_inliers is None or best_span < 0.18:
        raise ValueError("no sufficiently long straight lidar wall segment")

    wall_xy = xy[best_inliers]
    center = wall_xy.mean(axis=0)
    _, _, vt = np.linalg.svd(wall_xy - center, full_matrices=False)
    direction = vt[0]
    line_normal = np.asarray([-direction[1], direction[0]])
    residual = np.abs((wall_xy - center) @ line_normal)
    keep = residual <= max(0.006, float(np.percentile(residual, 90)))
    wall_points = np.column_stack((wall_xy[keep], np.zeros(keep.sum())))
    if len(wall_points) > 60:
        ordered = np.argsort(wall_points[:, :2] @ direction)
        wall_points = wall_points[ordered[np.linspace(0, len(ordered) - 1, 60).astype(int)]]
    return wall_points, float(np.sqrt(np.mean(residual[keep] ** 2))), best_span, len(candidates)


def camera_from_laser(base_from_camera, base_from_laser):
    return np.linalg.inv(base_from_camera) @ base_from_laser


def candidate_camera_pose(initial_base_from_camera, delta):
    result = initial_base_from_camera.copy()
    result[:3, :3] = rotation_vector_matrix(delta[3:]) @ initial_base_from_camera[:3, :3]
    result[:3, 3] = initial_base_from_camera[:3, 3] + delta[:3]
    return result


def data_residuals(delta, initial_base_from_camera, base_from_laser, samples):
    candidate = candidate_camera_pose(initial_base_from_camera, delta)
    camera_laser = camera_from_laser(candidate, base_from_laser)
    values = []
    labels = []
    for sample in samples:
        transformed = (camera_laser[:3, :3] @ sample["laser_points"].T).T + camera_laser[:3, 3]
        pose_residuals = (transformed - sample["wall_point"]) @ sample["wall_normal"]
        values.extend(pose_residuals)
        labels.extend([sample["name"]] * len(pose_residuals))
    return np.asarray(values), np.asarray(labels)


def robust_scale(values):
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    return max(1.4826 * mad, 0.0015)


def solve(initial_base_from_camera, base_from_laser, samples, active_parameters, prior_sigma, iterations=30):
    delta = np.zeros(6, dtype=float)
    active = np.asarray(active_parameters, dtype=int)
    damping = 1e-4
    for _ in range(iterations):
        data, _ = data_residuals(delta, initial_base_from_camera, base_from_laser, samples)
        scale = robust_scale(data)
        cutoff = 1.5 * scale
        weights = np.ones_like(data)
        outside = np.abs(data) > cutoff
        weights[outside] = cutoff / np.maximum(np.abs(data[outside]), 1e-12)

        jacobian = np.empty((len(data), len(active)), dtype=float)
        for column, parameter in enumerate(active):
            step = 1e-5 if parameter < 3 else 1e-5
            moved = delta.copy()
            moved[parameter] += step
            jacobian[:, column] = (
                data_residuals(moved, initial_base_from_camera, base_from_laser, samples)[0] - data
            ) / step

        weighted_j = jacobian * np.sqrt(weights)[:, None]
        weighted_r = data * np.sqrt(weights)
        prior_rows, prior_values = [], []
        for column, parameter in enumerate(active):
            sigma = prior_sigma[parameter]
            if math.isfinite(sigma) and sigma > 0.0:
                row = np.zeros(len(active))
                row[column] = 1.0 / sigma
                prior_rows.append(row)
                prior_values.append(delta[parameter] / sigma)
        if prior_rows:
            weighted_j = np.vstack((weighted_j, np.asarray(prior_rows)))
            weighted_r = np.concatenate((weighted_r, np.asarray(prior_values)))
        lhs = weighted_j.T @ weighted_j + damping * np.eye(len(active))
        step_active = -np.linalg.solve(lhs, weighted_j.T @ weighted_r)
        trial = delta.copy()
        trial[active] += step_active
        trial_data, _ = data_residuals(trial, initial_base_from_camera, base_from_laser, samples)
        old_cost = np.mean(np.minimum(data**2, 2.0 * cutoff * np.abs(data) - cutoff**2))
        new_cost = np.mean(np.minimum(trial_data**2, 2.0 * cutoff * np.abs(trial_data) - cutoff**2))
        if new_cost <= old_cost:
            delta = trial
            damping = max(damping / 3.0, 1e-9)
            if float(np.linalg.norm(step_active)) < 1e-8:
                break
        else:
            damping *= 10.0

    data, _ = data_residuals(delta, initial_base_from_camera, base_from_laser, samples)
    jacobian = np.empty((len(data), len(active)), dtype=float)
    for column, parameter in enumerate(active):
        step = 1e-5
        moved = delta.copy()
        moved[parameter] += step
        jacobian[:, column] = (
            data_residuals(moved, initial_base_from_camera, base_from_laser, samples)[0] - data
        ) / step
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return delta, singular_values


def residual_statistics(delta, initial_base_from_camera, base_from_laser, samples):
    residuals, labels = data_residuals(delta, initial_base_from_camera, base_from_laser, samples)
    result = {}
    for name in sorted(set(labels)):
        values = residuals[labels == name]
        result[name] = {
            "count": int(len(values)),
            "mean_mm": float(values.mean() * 1000.0),
            "rms_mm": float(np.sqrt(np.mean(values**2)) * 1000.0),
            "p95_abs_mm": float(np.percentile(np.abs(values), 95) * 1000.0),
        }
    result["ALL"] = {
        "count": int(len(residuals)),
        "mean_mm": float(residuals.mean() * 1000.0),
        "rms_mm": float(np.sqrt(np.mean(residuals**2)) * 1000.0),
        "p95_abs_mm": float(np.percentile(np.abs(residuals), 95) * 1000.0),
    }
    return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="extracted camera_lidar_calib_* directory")
    parser.add_argument("--output", type=Path, help="candidate JSON report path")
    parser.add_argument("--validation-poses", type=int, default=2, help="hold out last N poses")
    parser.add_argument(
        "--board-thickness-mm",
        type=float,
        help=(
            "legacy alias: override checkerboard-front-to-wall offset; "
            "includes plate plus adhesive/air gap"
        ),
    )
    parser.add_argument(
        "--wall-offset-mm",
        type=float,
        help="checkerboard printed front surface to wall surface distance",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        help="override measured checkerboard square size without recapturing",
    )
    parser.add_argument("--seed", type=int, default=921)
    return parser.parse_args()


def main():
    args = parse_args()
    dataset = args.dataset.expanduser().resolve()
    summary_path = dataset / "capture_summary.json"
    if not summary_path.exists():
        raise SystemExit(f"not a calibration dataset: {dataset}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    target = summary["target"]
    pose_dirs = sorted(path for path in dataset.glob("pose_*") if path.is_dir())
    if len(pose_dirs) < 6:
        raise SystemExit("at least 6 complete poses are required")
    validation_count = max(0, min(args.validation_poses, len(pose_dirs) - 4))
    train_names = {path.name for path in pose_dirs[:-validation_count]} if validation_count else {p.name for p in pose_dirs}
    rng = np.random.default_rng(args.seed)

    first_metadata = json.loads((pose_dirs[0] / "metadata.json").read_text(encoding="utf-8"))
    base_laser = tf_chain(first_metadata, "base_link", "laser")
    base_camera = tf_chain(first_metadata, "base_link", "camera_color_optical_frame")
    initial_camera_laser = camera_from_laser(base_camera, base_laser)
    if args.wall_offset_mm is not None and args.board_thickness_mm is not None:
        raise SystemExit("use only one of --wall-offset-mm and --board-thickness-mm")
    thickness_mm = float(target["board_thickness_mm"])
    if args.board_thickness_mm is not None:
        thickness_mm = float(args.board_thickness_mm)
    if args.wall_offset_mm is not None:
        thickness_mm = float(args.wall_offset_mm)
    if thickness_mm < 0.0:
        raise SystemExit("--board-thickness-mm cannot be negative")
    thickness_m = thickness_mm / 1000.0
    cols = int(target["checkerboard_inner_corners_cols"])
    rows = int(target["checkerboard_inner_corners_rows"])
    square_mm = (
        float(args.square_size_mm)
        if args.square_size_mm is not None
        else float(target["square_size_mm"])
    )
    if square_mm <= 0.0:
        raise SystemExit("--square-size-mm must be positive")
    square_m = square_mm / 1000.0

    samples, quality = [], []
    for pose_dir in pose_dirs:
        metadata = json.loads((pose_dir / "metadata.json").read_text(encoding="utf-8"))
        corner_data = np.load(pose_dir / "checkerboard_corners.npz")
        corners = np.asarray(corner_data["corners_px"], dtype=float)
        plane = checkerboard_plane(corners, metadata["color_camera_info"], cols, rows, square_m)
        wall_point = plane["center"] + thickness_m * plane["normal"]
        lidar_points, lidar_angles = scan_points(pose_dir / "scan_raw.npz")
        predicted_laser = np.linalg.inv(initial_camera_laser) @ np.r_[wall_point, 1.0]
        predicted_angle = math.atan2(predicted_laser[1], predicted_laser[0])
        wall_points, line_rms, span, candidates = fit_local_wall(
            lidar_points,
            lidar_angles,
            predicted_angle,
            initial_camera_laser,
            plane["normal"],
            wall_point,
            rng,
        )
        samples.append(
            {
                "name": pose_dir.name,
                "wall_normal": plane["normal"],
                "wall_point": wall_point,
                "laser_points": wall_points,
            }
        )
        quality.append(
            {
                "pose": pose_dir.name,
                "timestamp_spread_ms": float(metadata["timestamp_spread_ms"]),
                "square_pixels": float(corner_data["square_pixel_median"]),
                "checkerboard_distance_m": float(np.linalg.norm(plane["center"])),
                "reprojection_rms_px": plane["reprojection_rms_px"],
                "reprojection_p95_px": plane["reprojection_p95_px"],
                "lidar_line_rms_mm": line_rms * 1000.0,
                "lidar_line_span_m": span,
                "lidar_candidates": candidates,
                "lidar_inliers_used": len(wall_points),
            }
        )

    train = [sample for sample in samples if sample["name"] in train_names]
    validation = [sample for sample in samples if sample["name"] not in train_names]
    # A vertical wall cannot independently determine base-frame camera height.
    # Keep Z at its CAD/URDF value, while estimating X/Y and all mounting angles.
    active = [0, 1, 3, 4, 5]
    prior_sigma = np.asarray([0.10, 0.10, 0.001, math.radians(15), math.radians(15), math.radians(15)])
    delta, singular_values = solve(base_camera, base_laser, train, active, prior_sigma)
    candidate_base_camera = candidate_camera_pose(base_camera, delta)
    base_screw = tf_chain(first_metadata, "base_link", "camera_color_screw_frame")
    screw_from_camera = np.linalg.inv(base_screw) @ base_camera
    candidate_base_screw = candidate_base_camera @ np.linalg.inv(screw_from_camera)

    train_initial = residual_statistics(np.zeros(6), base_camera, base_laser, train)
    train_candidate = residual_statistics(delta, base_camera, base_laser, train)
    validation_initial = residual_statistics(np.zeros(6), base_camera, base_laser, validation) if validation else None
    validation_candidate = residual_statistics(delta, base_camera, base_laser, validation) if validation else None

    leave_one_out = []
    if len(train) >= 6:
        for omitted in train:
            subset = [sample for sample in train if sample["name"] != omitted["name"]]
            subset_delta, _ = solve(base_camera, base_laser, subset, active, prior_sigma)
            omitted_stats = residual_statistics(subset_delta, base_camera, base_laser, [omitted])["ALL"]
            leave_one_out.append(
                {
                    "omitted_pose": omitted["name"],
                    "delta_xyz_mm": (subset_delta[:3] * 1000.0).tolist(),
                    "delta_rotation_vector_deg": np.degrees(subset_delta[3:]).tolist(),
                    "omitted_pose_rms_mm": omitted_stats["rms_mm"],
                }
            )
    loo_vectors = np.asarray(
        [item["delta_xyz_mm"] + item["delta_rotation_vector_deg"] for item in leave_one_out],
        dtype=float,
    )
    leave_one_out_stability = None
    if len(loo_vectors):
        leave_one_out_stability = {
            "columns": ["dx_mm", "dy_mm", "dz_mm", "rx_deg", "ry_deg", "rz_deg"],
            "std": loo_vectors.std(axis=0).tolist(),
            "min": loo_vectors.min(axis=0).tolist(),
            "max": loo_vectors.max(axis=0).tolist(),
            "max_held_out_rms_mm": max(item["omitted_pose_rms_mm"] for item in leave_one_out),
        }

    report = {
        "format_version": 1,
        "status": "CANDIDATE_ONLY_DO_NOT_WRITE_URDF",
        "dataset": str(dataset),
        "model": "checkerboard printed front plane shifted by wall offset to flat wall plane",
        "square_size_mm_used": square_mm,
        "wall_offset_mm_used": thickness_mm,
        "board_bottom_mm_role": "metadata/consistency check only; not used in plane solve",
        "train_poses": [sample["name"] for sample in train],
        "validation_poses": [sample["name"] for sample in validation],
        "fixed_parameter": "base-frame camera Z translation (vertical wall does not observe it)",
        "initial_base_to_camera_color_optical": {
            "xyz_m": base_camera[:3, 3].tolist(),
            "rpy_rad": matrix_rpy(base_camera[:3, :3]).tolist(),
        },
        "candidate_base_to_camera_color_optical": {
            "xyz_m": candidate_base_camera[:3, 3].tolist(),
            "rpy_rad": matrix_rpy(candidate_base_camera[:3, :3]).tolist(),
        },
        "initial_base_to_camera_color_screw_frame": {
            "xyz_m": base_screw[:3, 3].tolist(),
            "rpy_rad": matrix_rpy(base_screw[:3, :3]).tolist(),
        },
        "candidate_base_to_camera_color_screw_frame": {
            "xyz_m": candidate_base_screw[:3, 3].tolist(),
            "rpy_rad": matrix_rpy(candidate_base_screw[:3, :3]).tolist(),
        },
        "candidate_delta_base_frame": {
            "xyz_m": delta[:3].tolist(),
            "rotation_vector_rad": delta[3:].tolist(),
            "rotation_vector_deg": np.degrees(delta[3:]).tolist(),
        },
        "jacobian_singular_values": singular_values.tolist(),
        "train_initial": train_initial,
        "train_candidate": train_candidate,
        "validation_initial": validation_initial,
        "validation_candidate": validation_candidate,
        "leave_one_out": leave_one_out,
        "leave_one_out_stability": leave_one_out_stability,
        "quality": quality,
    }
    output = args.output or dataset / "wall_calibration_candidate.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    quality_csv = output.with_suffix(".quality.csv")
    with quality_csv.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(quality[0]))
        writer.writeheader()
        writer.writerows(quality)

    print("===== Camera/Lidar wall calibration =====")
    print(f"dataset: {dataset}")
    print(f"poses: {len(samples)} (train {len(train)}, validation {len(validation)})")
    print("board_bottom_mm is not used by this wall-plane solver")
    print(f"board thickness compensation: {thickness_m * 1000.0:.1f} mm")
    print("delta xyz [mm]:", np.round(delta[:3] * 1000.0, 3).tolist())
    print("delta rotation vector [deg]:", np.round(np.degrees(delta[3:]), 4).tolist())
    print("train RMS initial/candidate [mm]:", round(train_initial["ALL"]["rms_mm"], 3), "/", round(train_candidate["ALL"]["rms_mm"], 3))
    if validation:
        print("validation RMS initial/candidate [mm]:", round(validation_initial["ALL"]["rms_mm"], 3), "/", round(validation_candidate["ALL"]["rms_mm"], 3))
    print("Jacobian singular values:", np.round(singular_values, 5).tolist())
    if leave_one_out_stability:
        print("leave-one-out std [mm, mm, mm, deg, deg, deg]:", np.round(loo_vectors.std(axis=0), 4).tolist())
        print("leave-one-out worst held-out RMS [mm]:", round(leave_one_out_stability["max_held_out_rms_mm"], 3))
    print(f"candidate report: {output}")
    print(f"quality table: {quality_csv}")
    print("No URDF or runtime configuration was modified.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, np.linalg.LinAlgError) as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
