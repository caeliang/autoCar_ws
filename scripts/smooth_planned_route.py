#!/usr/bin/env python3
"""Smooth a planned route CSV without modifying the original route."""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class TurnDebug:
    vertex: np.ndarray
    center: np.ndarray
    radius: float
    start_angle: float
    end_angle: float
    ccw: bool


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def read_route(path):
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"No waypoints found in {path}")

    points = np.array([[float(r["x"]), float(r["y"])] for r in rows], dtype=float)
    z_values = [float(r.get("z", 0.5) or 0.5) for r in rows]
    return rows, fieldnames, points, float(np.median(z_values))


def remove_duplicate_points(points, min_distance=1e-4):
    kept = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - kept[-1]) >= min_distance:
            kept.append(point)
    return np.array(kept)


def simplify_by_heading(points, angle_threshold_deg):
    if len(points) <= 2:
        return points.copy()

    threshold = math.radians(angle_threshold_deg)
    simplified = [points[0]]

    for i in range(1, len(points) - 1):
        prev_point = points[i - 1]
        point = points[i]
        next_point = points[i + 1]
        v1 = point - prev_point
        v2 = next_point - point
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue

        h1 = math.atan2(v1[1], v1[0])
        h2 = math.atan2(v2[1], v2[0])
        if abs(wrap_pi(h2 - h1)) >= threshold:
            simplified.append(point)

    simplified.append(points[-1])
    return np.array(simplified)


def local_turn_cost(points):
    if len(points) < 3:
        return 0.0
    cost = 0.0
    for i in range(1, len(points) - 1):
        a = points[i] - points[i - 1]
        b = points[i + 1] - points[i]
        if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
            continue
        ha = math.atan2(a[1], a[0])
        hb = math.atan2(b[1], b[0])
        cost += abs(wrap_pi(hb - ha))
    return cost


def clean_short_keypoint_segments(keypoints, min_segment_length):
    cleaned = [p.copy() for p in keypoints]
    changed = True
    while changed and len(cleaned) > 3:
        changed = False
        for i in range(len(cleaned) - 1):
            seg_len = float(np.linalg.norm(cleaned[i + 1] - cleaned[i]))
            if seg_len >= min_segment_length:
                continue

            candidates = []
            if 0 < i < len(cleaned) - 1:
                candidate = cleaned[:i] + cleaned[i + 1:]
                start = max(0, i - 2)
                end = min(len(candidate), i + 3)
                candidates.append((local_turn_cost(np.array(candidate[start:end])), i, candidate))
            if 0 < i + 1 < len(cleaned) - 1:
                candidate = cleaned[:i + 1] + cleaned[i + 2:]
                start = max(0, i - 2)
                end = min(len(candidate), i + 3)
                candidates.append((local_turn_cost(np.array(candidate[start:end])), i + 1, candidate))

            if candidates:
                _, _, cleaned = min(candidates, key=lambda item: item[0])
                changed = True
                break

    return np.array(cleaned)


def line_intersection(p, r, q, s):
    denom = r[0] * s[1] - r[1] * s[0]
    if abs(denom) < 1e-9:
        return None
    qp = q - p
    t = (qp[0] * s[1] - qp[1] * s[0]) / denom
    return p + t * r


def arc_points(center, radius, start_angle, end_angle, ccw, spacing):
    if ccw:
        delta = (end_angle - start_angle) % (2.0 * math.pi)
    else:
        delta = -((start_angle - end_angle) % (2.0 * math.pi))

    arc_len = abs(delta) * radius
    count = max(4, int(math.ceil(arc_len / spacing)) + 1)
    angles = np.linspace(start_angle, start_angle + delta, count)
    return np.column_stack((
        center[0] + radius * np.cos(angles),
        center[1] + radius * np.sin(angles),
    ))


def append_without_duplicate(out, segment):
    if len(segment) == 0:
        return
    if out and np.linalg.norm(segment[0] - out[-1]) < 1e-6:
        out.extend(segment[1:])
    else:
        out.extend(segment)


def smooth_corners(keypoints, minimum_turn_radius, corner_threshold_deg, spacing):
    if len(keypoints) <= 2:
        return keypoints.copy(), []

    corner_threshold = math.radians(corner_threshold_deg)
    smoothed = [keypoints[0]]
    debug_turns = []

    for i in range(1, len(keypoints) - 1):
        prev_point = keypoints[i - 1]
        vertex = keypoints[i]
        next_point = keypoints[i + 1]

        incoming = vertex - prev_point
        outgoing = next_point - vertex
        len_in = np.linalg.norm(incoming)
        len_out = np.linalg.norm(outgoing)
        if len_in < 1e-6 or len_out < 1e-6:
            append_without_duplicate(smoothed, [vertex])
            continue

        u_in = incoming / len_in
        u_out = outgoing / len_out
        heading_in = math.atan2(u_in[1], u_in[0])
        heading_out = math.atan2(u_out[1], u_out[0])
        turn_angle = abs(wrap_pi(heading_out - heading_in))

        if turn_angle < corner_threshold or turn_angle > math.radians(165.0):
            append_without_duplicate(smoothed, [vertex])
            continue

        tangent_distance = minimum_turn_radius * math.tan(turn_angle / 2.0)
        max_tangent = min(len_in, len_out) * 0.48
        tangent_distance = min(tangent_distance, max_tangent)
        if tangent_distance < spacing:
            append_without_duplicate(smoothed, [vertex])
            continue

        start = vertex - u_in * tangent_distance
        end = vertex + u_out * tangent_distance
        normal_in = np.array([-u_in[1], u_in[0]])
        normal_out = np.array([-u_out[1], u_out[0]])
        center = line_intersection(start, normal_in, end, normal_out)
        if center is None:
            append_without_duplicate(smoothed, [vertex])
            continue

        actual_radius = float(np.linalg.norm(start - center))
        if actual_radius < 1e-3:
            append_without_duplicate(smoothed, [vertex])
            continue

        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        end_angle = math.atan2(end[1] - center[1], end[0] - center[0])
        cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
        ccw = cross > 0.0
        arc = arc_points(center, actual_radius, start_angle, end_angle, ccw, spacing)
        append_without_duplicate(smoothed, arc)

        debug_turns.append(TurnDebug(
            vertex=vertex,
            center=center,
            radius=actual_radius,
            start_angle=start_angle,
            end_angle=end_angle,
            ccw=ccw,
        ))

    append_without_duplicate(smoothed, [keypoints[-1]])
    return np.array(smoothed), debug_turns


def resample_polyline(points, spacing):
    if len(points) <= 1:
        return points.copy()

    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(distances)))
    total = cumulative[-1]
    if total < 1e-6:
        return points[:1].copy()

    sample_s = np.arange(0.0, total, spacing)
    if sample_s.size == 0 or abs(sample_s[-1] - total) > 1e-6:
        sample_s = np.append(sample_s, total)

    xs = np.interp(sample_s, cumulative, points[:, 0])
    ys = np.interp(sample_s, cumulative, points[:, 1])
    return np.column_stack((xs, ys))


def compute_unwrapped_yaw(points):
    yaws = []
    for i in range(len(points)):
        if i < len(points) - 1:
            delta = points[i + 1] - points[i]
        else:
            delta = points[i] - points[i - 1]
        yaws.append(math.atan2(delta[1], delta[0]))
    return np.unwrap(np.array(yaws))


def nearest_source_indices(points, source_points):
    indices = []
    for point in points:
        d2 = np.sum((source_points - point) ** 2, axis=1)
        indices.append(int(np.argmin(d2)))
    return indices


def save_route(path, points, z, yaws, source_rows=None, source_points=None, source_fieldnames=None):
    metadata_fields = []
    if source_fieldnames:
        metadata_fields = [
            field for field in source_fieldnames
            if field not in {"step", "x", "y", "z", "yaw", "connect_next"}
        ]
    fieldnames = ["step", "x", "y", "z", "yaw"] + metadata_fields
    if source_fieldnames and "connect_next" in source_fieldnames:
        fieldnames.append("connect_next")

    source_indices = []
    if source_rows is not None and source_points is not None and len(source_rows) == len(source_points):
        source_indices = nearest_source_indices(points, source_points)

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i, (point, yaw) in enumerate(zip(points, yaws), start=1):
            row = {}
            if source_indices:
                row.update(source_rows[source_indices[i - 1]])
            row.update({
                "step": i,
                "x": f"{point[0]:.3f}",
                "y": f"{point[1]:.3f}",
                "z": f"{z:.3f}",
                "yaw": f"{math.degrees(yaw):.3f}",
            })
            if "connect_next" in fieldnames:
                row["connect_next"] = 1 if i < len(points) else 0
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def draw_debug_matplotlib(path, original, smoothed, yaws, turns, minimum_turn_radius):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.plot(original[:, 0], original[:, 1], "r.-", linewidth=1.2, markersize=2.5, label="original planned_route.csv")
    ax.plot(smoothed[:, 0], smoothed[:, 1], "g-", linewidth=2.0, label="smoothed route")

    arrow_step = max(1, len(smoothed) // 45)
    arrow_points = smoothed[::arrow_step]
    arrow_yaws = yaws[::arrow_step]
    ax.quiver(
        arrow_points[:, 0],
        arrow_points[:, 1],
        np.cos(arrow_yaws),
        np.sin(arrow_yaws),
        color="black",
        width=0.003,
        scale=32,
        label="waypoint yaw",
    )

    for idx, turn in enumerate(turns):
        circle = plt.Circle(
            turn.center,
            turn.radius,
            fill=False,
            color="#2f80ed",
            linestyle="--",
            linewidth=1.0,
            alpha=0.8,
        )
        ax.add_patch(circle)
        ax.plot(turn.center[0], turn.center[1], "b+", markersize=7)
        ax.text(
            turn.vertex[0],
            turn.vertex[1],
            f"R={turn.radius:.1f}m",
            color="#1f5fbf",
            fontsize=8,
        )
        if idx == 0:
            circle.set_label(f"turn radius target >= {minimum_turn_radius:.1f} m")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Route smoothing debug")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def draw_polyline(draw, points, project, fill, width):
    if len(points) < 2:
        return
    draw.line([project(p) for p in points], fill=fill, width=width, joint="curve")


def draw_debug_pillow(path, original, smoothed, yaws, turns, minimum_turn_radius):
    from PIL import Image, ImageDraw, ImageFont

    width, height = 1600, 1200
    margin = 90
    all_points = np.vstack((original, smoothed))
    min_xy = all_points.min(axis=0)
    max_xy = all_points.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    scale = min((width - 2 * margin) / span[0], (height - 2 * margin) / span[1])

    def project(point):
        x = margin + (point[0] - min_xy[0]) * scale
        y = height - margin - (point[1] - min_xy[1]) * scale
        return (float(x), float(y))

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    for gx in np.linspace(min_xy[0], max_xy[0], 10):
        x, _ = project(np.array([gx, min_xy[1]]))
        draw.line([(x, margin), (x, height - margin)], fill=(225, 225, 225), width=1)
    for gy in np.linspace(min_xy[1], max_xy[1], 10):
        _, y = project(np.array([min_xy[0], gy]))
        draw.line([(margin, y), (width - margin, y)], fill=(225, 225, 225), width=1)

    draw_polyline(draw, original, project, (210, 40, 40), 3)
    draw_polyline(draw, smoothed, project, (30, 150, 50), 5)

    arrow_step = max(1, len(smoothed) // 45)
    arrow_len = 18
    for point, yaw in zip(smoothed[::arrow_step], yaws[::arrow_step]):
        x, y = project(point)
        x2 = x + arrow_len * math.cos(yaw)
        y2 = y - arrow_len * math.sin(yaw)
        draw.line([(x, y), (x2, y2)], fill=(20, 20, 20), width=2)
        left = yaw + math.radians(150)
        right = yaw - math.radians(150)
        draw.line([(x2, y2), (x2 + 7 * math.cos(left), y2 - 7 * math.sin(left))], fill=(20, 20, 20), width=2)
        draw.line([(x2, y2), (x2 + 7 * math.cos(right), y2 - 7 * math.sin(right))], fill=(20, 20, 20), width=2)

    for turn in turns:
        cx, cy = project(turn.center)
        r = turn.radius * scale
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(40, 105, 210), width=2)
        vx, vy = project(turn.vertex)
        draw.text((vx + 5, vy + 5), f"R={turn.radius:.1f}m", fill=(30, 80, 170), font=font)

    draw.text((margin, 22), "Route smoothing debug", fill=(0, 0, 0), font=font)
    legend_y = 50
    legend = [
        ((210, 40, 40), "original planned_route.csv"),
        ((30, 150, 50), "smoothed route"),
        ((20, 20, 20), "waypoint yaw arrows"),
        ((40, 105, 210), f"turn radius target >= {minimum_turn_radius:.1f} m"),
    ]
    for color, label in legend:
        draw.line([(margin, legend_y + 5), (margin + 45, legend_y + 5)], fill=color, width=4)
        draw.text((margin + 55, legend_y), label, fill=(0, 0, 0), font=font)
        legend_y += 22

    image.save(path)


def draw_debug(path, original, smoothed, yaws, turns, minimum_turn_radius, prefer_matplotlib=False):
    if prefer_matplotlib:
        try:
            draw_debug_matplotlib(path, original, smoothed, yaws, turns, minimum_turn_radius)
            return
        except Exception as exc:
            print(f"matplotlib_debug_failed={exc}")
    draw_debug_pillow(path, original, smoothed, yaws, turns, minimum_turn_radius)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="waypoints/planned_route.csv")
    parser.add_argument("--output", default="waypoints/planned_route_smoothed.csv")
    parser.add_argument("--debug-output", default="waypoints/route_smoothing_debug.png")
    parser.add_argument("--minimum-turn-radius", type=float, default=4.0)
    parser.add_argument("--spacing", type=float, default=0.5)
    parser.add_argument("--corner-threshold-deg", type=float, default=20.0)
    parser.add_argument("--simplify-threshold-deg", type=float, default=12.0)
    parser.add_argument("--min-key-segment", type=float, default=3.0)
    parser.add_argument("--force-corner-smoothing", action="store_true")
    parser.add_argument("--prefer-matplotlib", action="store_true")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)
    debug_path = Path(args.debug_output)

    source_rows, source_fieldnames, original_raw, z = read_route(input_path)
    original = remove_duplicate_points(original_raw)
    has_lane_metadata = {"lane_id", "direction_group"}.issubset(set(source_fieldnames))
    if has_lane_metadata and not args.force_corner_smoothing:
        keypoints = original
        rounded = original
        turns = []
    else:
        keypoints = simplify_by_heading(original, args.simplify_threshold_deg)
        keypoints = clean_short_keypoint_segments(keypoints, args.min_key_segment)
        rounded, turns = smooth_corners(
            keypoints,
            args.minimum_turn_radius,
            args.corner_threshold_deg,
            args.spacing,
        )
    smoothed = resample_polyline(rounded, args.spacing)
    yaws = compute_unwrapped_yaw(smoothed)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    debug_path.parent.mkdir(parents=True, exist_ok=True)
    save_route(output_path, smoothed, z, yaws, source_rows, original_raw, source_fieldnames)
    draw_debug(
        debug_path,
        original,
        smoothed,
        yaws,
        turns,
        args.minimum_turn_radius,
        args.prefer_matplotlib,
    )

    print(f"original_points={len(original)}")
    print(f"keypoints={len(keypoints)}")
    print(f"smoothed_points={len(smoothed)}")
    print(f"smoothed_route={output_path}")
    print(f"debug_image={debug_path}")
    if turns:
        min_radius = min(t.radius for t in turns)
        print(f"turns_smoothed={len(turns)}")
        print(f"minimum_actual_radius={min_radius:.3f}")
    else:
        print("turns_smoothed=0")
    print(f"lane_geometry_preserved={has_lane_metadata and not args.force_corner_smoothing}")


if __name__ == "__main__":
    main()
