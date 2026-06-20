#!/usr/bin/env python3
import argparse
import contextlib
import csv
import io
import math
import os
from collections import OrderedDict
from typing import Dict, List, Optional, Sequence, Tuple

try:
    with contextlib.redirect_stderr(io.StringIO()):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

    MATPLOTLIB_IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - environment fallback
    plt = None
    MATPLOTLIB_IMPORT_ERROR = str(exc)

from PIL import Image, ImageDraw


Point = Tuple[float, float]
ROUNDABOUT_CENTER = (-18.0, -4.0)


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def canonical_segment_id(segment_id: str) -> Tuple[str, str]:
    if segment_id.endswith("_fwd"):
        return segment_id[:-4], "forward"
    if segment_id.endswith("_rev"):
        return segment_id[:-4], "reverse"
    return segment_id, "forward"


def normalize_yaw(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw <= -180.0:
        yaw += 360.0
    return yaw


def compute_yaws(points: Sequence[Point]) -> List[float]:
    if not points:
        return []
    yaws: List[float] = []
    last = 0.0
    for i in range(len(points) - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        if math.hypot(dx, dy) > 1e-9:
            last = math.degrees(math.atan2(dy, dx))
        yaws.append(normalize_yaw(last))
    yaws.append(normalize_yaw(last))
    return yaws


def polyline_length(points: Sequence[Point]) -> float:
    return sum(dist(points[i - 1], points[i]) for i in range(1, len(points)))


def unit_vector(a: Point, b: Point) -> Point:
    length = dist(a, b)
    if length < 1e-9:
        return (1.0, 0.0)
    return ((b[0] - a[0]) / length, (b[1] - a[1]) / length)


def tangent_at_start(points: Sequence[Point]) -> Point:
    if len(points) < 2:
        return (1.0, 0.0)
    p0 = points[0]
    for p in points[1:]:
        if dist(p0, p) > 1e-6:
            return unit_vector(p0, p)
    return (1.0, 0.0)


def tangent_at_end(points: Sequence[Point]) -> Point:
    if len(points) < 2:
        return (1.0, 0.0)
    p1 = points[-1]
    for p0 in reversed(points[:-1]):
        if dist(p0, p1) > 1e-6:
            return unit_vector(p0, p1)
    return (1.0, 0.0)


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def angle_between(a: Point, b: Point) -> float:
    value = max(-1.0, min(1.0, dot(a, b)))
    return math.degrees(math.acos(value))


def bezier_point(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - t
    return (
        u * u * u * p0[0] + 3.0 * u * u * t * p1[0] + 3.0 * u * t * t * p2[0] + t * t * t * p3[0],
        u * u * u * p0[1] + 3.0 * u * u * t * p1[1] + 3.0 * u * t * t * p2[1] + t * t * t * p3[1],
    )


def resample_polyline(points: Sequence[Point], spacing: float) -> List[Point]:
    if len(points) < 2:
        return list(points)
    result = [points[0]]
    carry = spacing
    prev = points[0]
    for current in points[1:]:
        seg_len = dist(prev, current)
        while seg_len >= carry and seg_len > 1e-9:
            ratio = carry / seg_len
            new_pt = (prev[0] + (current[0] - prev[0]) * ratio, prev[1] + (current[1] - prev[1]) * ratio)
            result.append(new_pt)
            prev = new_pt
            seg_len = dist(prev, current)
            carry = spacing
        carry -= seg_len
        prev = current
    if dist(result[-1], points[-1]) > 1e-6:
        result.append(points[-1])
    return result


def signed_angle_delta(a0: float, a1: float) -> float:
    delta = a1 - a0
    while delta <= -math.pi:
        delta += 2.0 * math.pi
    while delta > math.pi:
        delta -= 2.0 * math.pi
    return delta


def classify_segment(points: Sequence[Point], segment_type: str) -> str:
    if segment_type == "roundabout_arc":
        return "roundabout"
    if len(points) < 2:
        return "unknown"
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    if dx >= dy * 2.5:
        return "horizontal"
    if dy >= dx * 2.5:
        return "vertical"
    return "connector_curve"


def normalize_centerline_direction(points: Sequence[Point], source: Optional[str], target: Optional[str], segment_type: str) -> Tuple[List[Point], Optional[str], Optional[str], str, str]:
    """Normalize parent centerline order to the project's right-hand traffic convention."""
    clean = list(points)
    if len(clean) < 2:
        return clean, source, target, "unknown", "kept"

    kind = classify_segment(clean, segment_type)
    start = clean[0]
    end = clean[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    mx = sum(p[0] for p in clean) / len(clean)
    my = sum(p[1] for p in clean) / len(clean)
    reverse = False
    rule = "preserve"

    if kind == "horizontal":
        if my >= 40.0:
            reverse = dx < 0.0
            rule = "top_horizontal_west_to_east"
        elif my <= -25.0:
            reverse = dx > 0.0
            rule = "bottom_horizontal_east_to_west"
        else:
            reverse = dx < 0.0
            rule = "inner_horizontal_eastbound_forward"
    elif kind == "vertical":
        if mx <= -30.0:
            reverse = dy > 0.0
            rule = "left_vertical_north_to_south"
        elif mx >= 30.0:
            reverse = dy < 0.0
            rule = "right_vertical_south_to_north"
        else:
            reverse = dy < 0.0
            rule = "inner_vertical_northbound_forward"
    elif kind == "roundabout":
        cx, cy = ROUNDABOUT_CENTER
        a0 = math.atan2(start[1] - cy, start[0] - cx)
        a1 = math.atan2(end[1] - cy, end[0] - cx)
        reverse = signed_angle_delta(a0, a1) < 0.0
        rule = "roundabout_counter_clockwise"
    else:
        # For short connectors and curves, keep the source graph direction.
        # Lane offset is still recomputed from the driving direction later.
        reverse = False
        rule = "connector_preserve_graph_direction"

    if reverse:
        return list(reversed(clean)), target, source, kind, rule + "_reversed"
    return clean, source, target, kind, rule


def offset_polyline(points: Sequence[Point], lane_offset: float, side: str) -> Tuple[List[Point], List[float]]:
    """Offset points relative to the actual driving direction of this point order."""
    yaws = compute_yaws(points)
    shifted: List[Point] = []
    for point, yaw in zip(points, yaws):
        rad = math.radians(yaw)
        tx = math.cos(rad)
        ty = math.sin(rad)
        if side == "left":
            nx, ny = -ty, tx
        elif side == "right":
            nx, ny = ty, -tx
        else:
            raise ValueError(f"Invalid lane side: {side}")
        shifted.append((point[0] + nx * lane_offset, point[1] + ny * lane_offset))
    return shifted, yaws


def make_lane_record(
    parent_id: str,
    data: dict,
    direction: str,
    side: str,
    lane_offset: float,
) -> dict:
    center_points = data["points"] if direction == "forward" else list(reversed(data["points"]))
    effective_offset = float(data.get("lane_offset_override", lane_offset))
    shifted, yaws = offset_polyline(center_points, effective_offset, side)
    source = data.get("source")
    target = data.get("target")
    if direction == "reverse":
        source, target = target, source
    direction_group = f"{direction}_{side}"
    return {
        "lane_id": f"{parent_id}_{direction}_{side}",
        "parent_segment_id": parent_id,
        "segment_type": data.get("segment_type", "unknown"),
        "lane_side": side,
        "direction": direction,
        "direction_group": direction_group,
        "points": shifted,
        "yaws": yaws,
        "z": float(data.get("z", 0.5)),
        "source": source,
        "target": target,
    }


def read_parent_segments(input_csv: str) -> "OrderedDict[str, dict]":
    by_segment: "OrderedDict[str, List[dict]]" = OrderedDict()
    with open(input_csv, newline="") as f:
        reader = csv.DictReader(f)
        required = {"segment_id", "x", "y", "z", "connect_next"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"Missing columns in {input_csv}: {sorted(missing)}")
        for row in reader:
            by_segment.setdefault(row["segment_id"], []).append(row)

    parents: "OrderedDict[str, dict]" = OrderedDict()
    for segment_id, rows in by_segment.items():
        base_id, source_direction = canonical_segment_id(segment_id)
        if base_id in parents:
            continue

        rows = sorted(rows, key=lambda r: int(float(r.get("step", "0"))))
        points = [(float(r["x"]), float(r["y"])) for r in rows]
        source = rows[0].get("source")
        target = rows[0].get("target")
        if source_direction == "reverse":
            points = list(reversed(points))
            source, target = target, source

        # Drop accidental duplicate consecutive points.
        clean_points: List[Point] = []
        for p in points:
            if not clean_points or dist(clean_points[-1], p) > 1e-6:
                clean_points.append(p)
        if len(clean_points) < 2:
            continue
        clean_points, source, target, geometry_class, normalization_rule = normalize_centerline_direction(
            clean_points,
            source,
            target,
            rows[0].get("segment_type", "unknown"),
        )

        parents[base_id] = {
            "points": clean_points,
            "z": float(rows[0].get("z", 0.5)),
            "segment_type": rows[0].get("segment_type", "unknown"),
            "source": source,
            "target": target,
            "geometry_class": geometry_class,
            "normalization_rule": normalization_rule,
        }
    return parents


def build_lane_records(parents: "OrderedDict[str, dict]", lane_offset: float) -> List[dict]:
    lanes: List[dict] = []
    for parent_id, data in parents.items():
        is_roundabout = data.get("segment_type") == "roundabout_arc"
        if data.get("segment_type") == "entry_road":
            xs = [p[0] for p in data["points"]]
            mx = sum(xs) / len(xs)
            data = dict(data)
            data["lane_offset_override"] = 0.0
            # The bottom entry already consists of two lane centerlines.
            # x=6 is the northbound right lane; x=2 is the southbound right lane.
            variants = [("forward", "right")] if mx >= 4.0 else [("reverse", "right")]
        else:
            variants = [
                ("forward", "left"),
                ("forward", "right"),
            ]
        # Roundabout center loop is intentionally one-way. The rule-based
        # graph stores it in counter-clockwise order, so only forward lanes
        # are emitted for the loop itself.
        if not is_roundabout and data.get("segment_type") != "entry_road":
            variants.extend(
                [
                    ("reverse", "left"),
                    ("reverse", "right"),
                ]
            )
        for direction, side in variants:
            lanes.append(make_lane_record(parent_id, data, direction, side, lane_offset))
    return lanes


def write_lane_level_csv(output_csv: str, lanes: Sequence[dict]) -> int:
    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
    fields = [
        "step",
        "x",
        "y",
        "z",
        "yaw",
        "lane_id",
        "segment_id",
        "parent_segment_id",
        "lane_side",
        "direction",
        "direction_group",
        "connect_next",
    ]
    step = 1
    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for lane in lanes:
            for i, (point, yaw) in enumerate(zip(lane["points"], lane["yaws"])):
                writer.writerow(
                    {
                        "step": step,
                        "x": f"{point[0]:.3f}",
                        "y": f"{point[1]:.3f}",
                        "z": f"{float(lane['z']):.3f}",
                        "yaw": f"{yaw:.3f}",
                        "lane_id": lane["lane_id"],
                        "segment_id": lane["parent_segment_id"],
                        "parent_segment_id": lane["parent_segment_id"],
                        "lane_side": lane["lane_side"],
                        "direction": lane["direction"],
                        "direction_group": lane.get("direction_group", f"{lane['direction']}_{lane['lane_side']}"),
                        "connect_next": 1 if i < len(lane["points"]) - 1 else 0,
                    }
                )
                step += 1
    return len(lanes)


def build_lane_lookup(lanes: Sequence[dict]) -> Dict[str, List[dict]]:
    by_node: Dict[str, List[dict]] = {}
    for lane in lanes:
        for key in ("source", "target"):
            node = lane.get(key)
            if node:
                by_node.setdefault(node, []).append(lane)
    return by_node


def endpoint_for(lane: dict, key: str) -> Optional[Point]:
    points = lane["points"]
    if not points:
        return None
    if key == "source":
        return points[0]
    if key == "target":
        return points[-1]
    raise ValueError(key)


def yaw_at(lane: dict, key: str) -> float:
    if key == "source":
        return float(lane["yaws"][0])
    if key == "target":
        return float(lane["yaws"][-1])
    raise ValueError(key)


def min_lane_distance(a: Point, lanes: Sequence[dict], endpoint_key: str) -> Tuple[float, Optional[dict]]:
    best = (float("inf"), None)
    for lane in lanes:
        p = endpoint_for(lane, endpoint_key)
        if p is None:
            continue
        d = dist(a, p)
        if d < best[0]:
            best = (d, lane)
    return best


def validate_lane_continuity(lanes: Sequence[dict], lane_offset: float) -> List[dict]:
    incoming_by_node: Dict[str, List[dict]] = {}
    outgoing_by_node: Dict[str, List[dict]] = {}
    faults: List[dict] = []

    for lane in lanes:
        source = lane.get("source")
        target = lane.get("target")
        if source:
            outgoing_by_node.setdefault(source, []).append(lane)
        if target:
            incoming_by_node.setdefault(target, []).append(lane)
        if lane.get("segment_type") == "roundabout_arc" and lane.get("direction") != "forward":
            faults.append({"type": "ROUNDABOUT_REVERSE", "lane": lane, "point": endpoint_for(lane, "source")})

    max_expected_gap = max(2.0, lane_offset * 3.5)
    nodes = set(incoming_by_node) | set(outgoing_by_node)
    for node in nodes:
        incoming_right = [l for l in incoming_by_node.get(node, []) if l["lane_side"] == "right"]
        outgoing_right = [l for l in outgoing_by_node.get(node, []) if l["lane_side"] == "right"]
        outgoing_left = [l for l in outgoing_by_node.get(node, []) if l["lane_side"] == "left"]
        for inc in incoming_right:
            end_pt = endpoint_for(inc, "target")
            if end_pt is None:
                continue
            best_right_gap, best_right = min_lane_distance(end_pt, outgoing_right, "source")
            best_left_gap, best_left = min_lane_distance(end_pt, outgoing_left, "source")
            if best_right is None and outgoing_by_node.get(node):
                faults.append({"type": "NO_RIGHT_OUT", "lane": inc, "point": end_pt, "node": node})
                continue
            if best_right is not None and best_right_gap > max_expected_gap:
                faults.append(
                    {
                        "type": "RIGHT_CONTINUITY_GAP",
                        "lane": inc,
                        "other": best_right,
                        "point": end_pt,
                        "other_point": endpoint_for(best_right, "source"),
                        "node": node,
                        "gap": best_right_gap,
                    }
                )
            if best_left is not None and best_right_gap > max_expected_gap and best_left_gap + 0.25 < best_right_gap:
                faults.append(
                    {
                        "type": "RIGHT_TO_LEFT_RISK",
                        "lane": inc,
                        "other": best_left,
                        "point": end_pt,
                        "other_point": endpoint_for(best_left, "source"),
                        "node": node,
                        "gap": best_left_gap,
                    }
                )
    return faults


def make_bridge_record(incoming: dict, outgoing: dict, node: str, spacing: float, bridge_index: int) -> Optional[dict]:
    start = endpoint_for(incoming, "target")
    end = endpoint_for(outgoing, "source")
    if start is None or end is None:
        return None
    gap = dist(start, end)
    if gap < 0.15:
        return None
    t_in = tangent_at_end(incoming["points"])
    t_out = tangent_at_start(outgoing["points"])
    handle = min(3.0, max(0.8, gap * 0.55))
    p1 = (start[0] + t_in[0] * handle, start[1] + t_in[1] * handle)
    p2 = (end[0] - t_out[0] * handle, end[1] - t_out[1] * handle)
    samples = [bezier_point(start, p1, p2, end, i / 24.0) for i in range(25)]
    points = resample_polyline(samples, spacing)
    if len(points) < 2:
        return None
    yaws = compute_yaws(points)
    direction = incoming["direction"]
    direction_group = incoming.get("direction_group", f"{direction}_right")
    return {
        "lane_id": f"bridge_{bridge_index:04d}_{direction}_right",
        "parent_segment_id": f"{incoming['parent_segment_id']}_to_{outgoing['parent_segment_id']}",
        "segment_type": "lane_bridge",
        "lane_side": "right",
        "direction": direction,
        "direction_group": direction_group,
        "points": points,
        "yaws": yaws,
        "z": float(incoming.get("z", 0.5)),
        "source": incoming.get("target"),
        "target": outgoing.get("source"),
        "incoming_lane_id": incoming["lane_id"],
        "outgoing_lane_id": outgoing["lane_id"],
        "junction_node": node,
    }


def build_cleaned_lane_network(
    lanes: Sequence[dict],
    spacing: float,
    max_connection_distance: float = 7.5,
    max_turn_angle: float = 135.0,
) -> Tuple[List[dict], List[dict], List[dict], List[dict]]:
    # Continuous lane geometry mode:
    # Junctions are not lane endpoints that need a bridge. The generated offset
    # lane polylines are allowed to pass through the intersection geometry as-is.
    # This intentionally disables bridge generation, endpoint termination, and
    # dangling-stub pruning for the cleaned output.
    right_lanes = [lane for lane in lanes if lane["lane_side"] == "right"]
    return right_lanes, [], [], []


def build_bridge_based_lane_network(
    lanes: Sequence[dict],
    spacing: float,
    max_connection_distance: float = 7.5,
    max_turn_angle: float = 135.0,
) -> Tuple[List[dict], List[dict], List[dict], List[dict]]:
    right_lanes = [lane for lane in lanes if lane["lane_side"] == "right"]
    incoming_by_node: Dict[str, List[dict]] = {}
    outgoing_by_node: Dict[str, List[dict]] = {}
    for lane in right_lanes:
        if lane.get("target"):
            incoming_by_node.setdefault(lane["target"], []).append(lane)
        if lane.get("source"):
            outgoing_by_node.setdefault(lane["source"], []).append(lane)

    accepted: List[dict] = []
    rejected: List[dict] = []
    used_incoming = set()
    used_outgoing = set()
    bridge_index = 1

    direction_groups = ("forward_right", "reverse_right")

    for node in sorted(set(incoming_by_node) | set(outgoing_by_node)):
        incoming_all = incoming_by_node.get(node, [])
        outgoing_all = outgoing_by_node.get(node, [])

        for inc in incoming_all:
            for out in outgoing_all:
                if inc["lane_id"] == out["lane_id"]:
                    continue
                if inc.get("direction_group") != out.get("direction_group"):
                    rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "direction_group_mismatch"})

        for group in direction_groups:
            candidates = []
            incoming = [lane for lane in incoming_all if lane.get("direction_group") == group]
            outgoing = [lane for lane in outgoing_all if lane.get("direction_group") == group]
            for inc in incoming:
                p = endpoint_for(inc, "target")
                if p is None:
                    continue
                for out in outgoing:
                    q = endpoint_for(out, "source")
                    if q is None or inc["lane_id"] == out["lane_id"]:
                        continue
                    gap = dist(p, q)
                    if gap > max_connection_distance:
                        rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "too_far", "distance": gap})
                        continue
                    t_in = tangent_at_end(inc["points"])
                    t_out = tangent_at_start(out["points"])
                    angle = angle_between(t_in, t_out)
                    if angle > max_turn_angle:
                        rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "u_turn_or_reverse", "angle": angle})
                        continue
                    score = gap + angle * 0.035
                    candidates.append((score, gap, angle, inc, out))

            candidates.sort(key=lambda item: item[0])
            accepted_for_group = 0
            for score, gap, angle, inc, out in candidates:
                if inc["lane_id"] in used_incoming or out["lane_id"] in used_outgoing:
                    rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "endpoint_already_used", "distance": gap, "angle": angle})
                    continue
                if inc.get("direction_group") != out.get("direction_group"):
                    rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "direction_group_mismatch"})
                    continue
                bridge = make_bridge_record(inc, out, node, spacing, bridge_index)
                if bridge is None:
                    rejected.append({"node": node, "incoming": inc, "outgoing": out, "reason": "zero_length_bridge", "distance": gap, "angle": angle})
                    continue
                bridge["distance"] = gap
                bridge["turn_angle"] = angle
                accepted.append(bridge)
                used_incoming.add(inc["lane_id"])
                used_outgoing.add(out["lane_id"])
                bridge_index += 1
                accepted_for_group += 1
                if accepted_for_group >= 1:
                    # One local bridge per color group per junction keeps the
                    # automatic layer from creating spider-web junctions.
                    break

    connected_lane_ids = set()
    for bridge in accepted:
        connected_lane_ids.add(bridge["incoming_lane_id"])
        connected_lane_ids.add(bridge["outgoing_lane_id"])

    cleaned_lanes: List[dict] = []
    removed_stubs: List[dict] = []
    for lane in right_lanes:
        length = polyline_length(lane["points"])
        is_tiny = 0.5 <= length <= 3.0
        protected = lane.get("segment_type") in {"roundabout_arc", "outer_corner_arc"}
        if is_tiny and lane["lane_id"] not in connected_lane_ids and not protected:
            removed_stubs.append({"lane": lane, "reason": "short_dangling_stub", "length": length})
            continue
        cleaned_lanes.append(lane)

    return cleaned_lanes + accepted, accepted, rejected, removed_stubs


def draw_pil_lines(
    path: str,
    line_layers: Sequence[Tuple[Sequence[Point], Tuple[int, int, int], int]],
    text_layers: Sequence[Tuple[Point, str, Tuple[int, int, int]]] = (),
    point_layers: Sequence[Tuple[Point, Tuple[int, int, int], int]] = (),
    title: str = "",
) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    all_points: List[Point] = []
    for pts, _, _ in line_layers:
        all_points.extend(pts)
    for point, _, _ in point_layers:
        all_points.append(point)
    for point, _, _ in text_layers:
        all_points.append(point)
    if not all_points:
        all_points = [(0.0, 0.0), (1.0, 1.0)]

    width, height = 1700, 1400
    margin = 70
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def to_px(p: Point) -> Tuple[int, int]:
        return (
            int(margin + (p[0] - min_x) * scale),
            int(height - margin - (p[1] - min_y) * scale),
        )

    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    draw.text((margin, 20), title, fill=(20, 20, 20))

    # Light grid.
    for gx in range(int(math.floor(min_x / 10) * 10), int(math.ceil(max_x / 10) * 10) + 1, 10):
        x, _ = to_px((gx, min_y))
        draw.line([(x, margin), (x, height - margin)], fill=(235, 235, 235), width=1)
    for gy in range(int(math.floor(min_y / 10) * 10), int(math.ceil(max_y / 10) * 10) + 1, 10):
        _, y = to_px((min_x, gy))
        draw.line([(margin, y), (width - margin, y)], fill=(235, 235, 235), width=1)

    for pts, color, line_width in line_layers:
        if len(pts) < 2:
            continue
        draw.line([to_px(p) for p in pts], fill=color, width=line_width)
    for point, color, radius in point_layers:
        x, y = to_px(point)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
    for point, text, color in text_layers:
        draw.text(to_px(point), text, fill=color)
    img.save(path)


def draw_cleaned_debug(path: str, cleaned_lanes: Sequence[dict], accepted: Sequence[dict], removed_stubs: Sequence[dict], title: str) -> None:
    if plt is None:
        lines = []
        for lane in cleaned_lanes:
            if lane["direction_group"] == "forward_right":
                color = (21, 101, 192)
            else:
                color = (239, 108, 0)
            width = 4 if lane.get("segment_type") == "lane_bridge" else 3
            lines.append((lane["points"], color, width))
        for item in removed_stubs:
            lines.append((item["lane"]["points"], (211, 47, 47), 4))
        draw_pil_lines(path, lines, title=title)
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(14, 12))
    ax.set_title(title)
    labels = set()
    for lane in cleaned_lanes:
        pts = lane["points"]
        if lane["direction_group"] == "forward_right":
            color = "#1565c0"
            label = "blue forward_right bridge" if lane.get("segment_type") == "lane_bridge" else "blue forward_right"
            width = 2.4 if lane.get("segment_type") == "lane_bridge" else 1.5
            alpha = 0.9 if lane.get("segment_type") == "lane_bridge" else 0.85
        else:
            color = "#ef6c00"
            label = "orange reverse_right bridge" if lane.get("segment_type") == "lane_bridge" else "orange reverse_right"
            width = 2.4 if lane.get("segment_type") == "lane_bridge" else 1.5
            alpha = 0.9 if lane.get("segment_type") == "lane_bridge" else 0.85
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=width, alpha=alpha, label=label if label not in labels else None)
        labels.add(label)
    for item in removed_stubs:
        lane = item["lane"]
        pts = lane["points"]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#d32f2f", linewidth=2.0, alpha=0.9, linestyle=":", label="removed stub" if "removed stub" not in labels else None)
        labels.add("removed stub")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="best")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def draw_connection_decisions(path: str, lanes: Sequence[dict], decisions: Sequence[dict], accepted_mode: bool) -> None:
    if plt is None:
        lines = []
        texts = []
        for lane in lanes:
            if lane["lane_side"] != "right":
                continue
            color = (144, 202, 249) if lane["direction"] == "forward" else (255, 204, 128)
            lines.append((lane["points"], color, 2))
        for item in decisions:
            if accepted_mode:
                pts = item["points"]
                color = (21, 101, 192) if item.get("direction_group") == "forward_right" else (239, 108, 0)
                lines.append((pts, color, 4))
                mid = pts[len(pts) // 2]
                texts.append((mid, f"{item['direction_group']} {item.get('turn_angle', 0):.0f}deg", color))
            else:
                p = endpoint_for(item["incoming"], "target")
                q = endpoint_for(item["outgoing"], "source")
                if p and q:
                    lines.append(([p, q], (198, 40, 40), 2))
                    texts.append((((p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5), item.get("reason", "rejected"), (180, 0, 0)))
        draw_pil_lines(path, lines, texts, title="Accepted lane connections" if accepted_mode else "Rejected lane connections")
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(14, 12))
    ax.set_title("Accepted lane connections" if accepted_mode else "Rejected lane connections")
    for lane in lanes:
        if lane["lane_side"] != "right":
            continue
        pts = lane["points"]
        color = "#90caf9" if lane["direction"] == "forward" else "#ffcc80"
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=0.8, alpha=0.35)
    for item in decisions:
        if accepted_mode:
            pts = item["points"]
            color = "#1565c0" if item.get("direction_group") == "forward_right" else "#ef6c00"
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=2.3, alpha=0.95)
            mid = pts[len(pts) // 2]
            txt = f"{item['direction_group']} {item.get('turn_angle', 0):.0f}deg"
        else:
            inc = item["incoming"]
            out = item["outgoing"]
            p = endpoint_for(inc, "target")
            q = endpoint_for(out, "source")
            if not p or not q:
                continue
            ax.plot([p[0], q[0]], [p[1], q[1]], color="#c62828", linewidth=1.1, alpha=0.35)
            mid = ((p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5)
            txt = item.get("reason", "rejected")
        ax.text(mid[0], mid[1], txt, fontsize=5, color=color if accepted_mode else "#b71c1c", alpha=0.75)
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def draw_debug(path: str, parents: "OrderedDict[str, dict]", lanes: Sequence[dict], arrow_stride: int) -> None:
    if plt is None:
        lines = []
        for data in parents.values():
            lines.append((data["points"], (170, 170, 170), 2))
        for lane in lanes:
            if lane["lane_side"] != "right":
                continue
            color = (21, 101, 192) if lane["direction"] == "forward" else (239, 108, 0)
            lines.append((lane["points"], color, 3))
        draw_pil_lines(path, lines, title="Lane-level graph from generated centerlines")
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 12))
    ax.set_title("Lane-level graph from generated centerlines")
    first_center = True
    first_fwd = True
    first_rev = True

    for parent_id, data in parents.items():
        center = data["points"]
        ax.plot(
            [p[0] for p in center],
            [p[1] for p in center],
            color="#9e9e9e",
            linewidth=1.0,
            alpha=0.45,
            label="centerline" if first_center else None,
        )
        first_center = False

    for lane in lanes:
        if lane["lane_side"] != "right":
            continue
        pts = lane["points"]
        yaws = lane["yaws"]
        color = "#1f77b4" if lane["direction"] == "forward" else "#ff7f0e"
        label = None
        if lane["direction"] == "forward" and first_fwd:
            label = "forward right lane"
            first_fwd = False
        elif lane["direction"] == "reverse" and first_rev:
            label = "reverse right lane"
            first_rev = False

        ax.plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            color=color,
            linewidth=1.7,
            alpha=0.85,
            label=label,
        )

        for idx in range(0, len(pts), max(1, arrow_stride)):
            yaw = math.radians(yaws[idx])
            ax.arrow(
                pts[idx][0],
                pts[idx][1],
                0.55 * math.cos(yaw),
                0.55 * math.sin(yaw),
                head_width=0.22,
                head_length=0.32,
                fc=color,
                ec=color,
                alpha=0.75,
                length_includes_head=True,
            )

    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="best")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def draw_validation(
    path: str,
    parents: "OrderedDict[str, dict]",
    lanes: Sequence[dict],
    faults: Sequence[dict],
    arrow_stride: int,
) -> None:
    if plt is None:
        lines = []
        texts = []
        points = []
        for parent_id, data in parents.items():
            pts = data["points"]
            lines.append((pts, (190, 190, 190), 2))
            if pts:
                points.append((pts[0], (46, 125, 50), 4))
                points.append((pts[-1], (38, 50, 56), 4))
                mid = pts[len(pts) // 2]
                texts.append((mid, f"{parent_id} {data.get('normalization_rule')}", (80, 80, 80)))
        for lane in lanes:
            if lane["lane_side"] != "right":
                continue
            color = (21, 101, 192) if lane["direction"] == "forward" else (239, 108, 0)
            lines.append((lane["points"], color, 3))
        for fault in faults:
            p = fault.get("point")
            q = fault.get("other_point")
            if p and q:
                lines.append(([p, q], (198, 40, 40), 4))
                texts.append((((p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5), fault["type"], (180, 0, 0)))
        draw_pil_lines(path, lines, texts, points, title="Lane direction validation")
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(15, 13))
    ax.set_title("Lane direction validation: right-lane continuity and junction directions")

    for data in parents.values():
        pts = data["points"]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#bdbdbd", linewidth=0.8, alpha=0.35)
        if pts:
            ax.scatter([pts[0][0]], [pts[0][1]], color="#2e7d32", s=10, zorder=4)
            ax.scatter([pts[-1][0]], [pts[-1][1]], color="#263238", s=10, zorder=4)

    for idx, (parent_id, data) in enumerate(parents.items()):
        # Label every few segments, plus all roundabout arcs/connectors, to keep the
        # validation image useful instead of turning it into a wall of text.
        if idx % 4 != 0 and data.get("segment_type") not in {"roundabout_arc", "roundabout_connector"}:
            continue
        pts = data["points"]
        if not pts:
            continue
        mid = pts[len(pts) // 2]
        yaws = compute_yaws(pts)
        direction_label = f"{parent_id}\n{data.get('geometry_class')} {yaws[len(yaws)//2]:.0f}deg\n{data.get('normalization_rule')}"
        ax.text(
            mid[0],
            mid[1],
            direction_label,
            fontsize=3.8,
            color="#424242",
            alpha=0.55,
            ha="center",
            va="center",
        )

    label_seen = set()
    for lane in lanes:
        if lane["lane_side"] != "right":
            continue
        pts = lane["points"]
        yaws = lane["yaws"]
        if lane["direction"] == "forward":
            color = "#1565c0"
            label = "forward right"
        else:
            color = "#ef6c00"
            label = "reverse right"
        ax.plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            color=color,
            linewidth=1.4,
            alpha=0.8,
            label=label if label not in label_seen else None,
        )
        label_seen.add(label)
        for idx in range(0, len(pts), max(1, arrow_stride)):
            yaw = math.radians(yaws[idx])
            ax.arrow(
                pts[idx][0],
                pts[idx][1],
                0.45 * math.cos(yaw),
                0.45 * math.sin(yaw),
                head_width=0.18,
                head_length=0.28,
                fc=color,
                ec=color,
                alpha=0.75,
                length_includes_head=True,
            )

        if pts:
            ax.text(
                pts[len(pts) // 2][0],
                pts[len(pts) // 2][1],
                lane["lane_id"].replace("_forward_right", "_F_R").replace("_reverse_right", "_R_R"),
                fontsize=4.0,
                color=color,
                alpha=0.55,
            )

    for fault in faults:
        p = fault.get("point")
        q = fault.get("other_point")
        if p and q:
            ax.plot([p[0], q[0]], [p[1], q[1]], color="red", linewidth=1.8, alpha=0.9, linestyle="--")
            tx, ty = (p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5
        elif p:
            ax.scatter([p[0]], [p[1]], color="red", s=30, zorder=5)
            tx, ty = p
        else:
            continue
        ax.text(tx, ty, fault["type"], fontsize=6, color="red", weight="bold")

    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="best")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate lane-level left/right graph from generated centerline graph CSV")
    parser.add_argument("--input", default="/home/ranim/autoCar_ws/waypoints/generated_lane_graph.csv")
    parser.add_argument("--output", default="/home/ranim/autoCar_ws/waypoints/lane_level_graph.csv")
    parser.add_argument("--cleaned-output", default="/home/ranim/autoCar_ws/waypoints/lane_level_graph_cleaned.csv")
    parser.add_argument("--lane-offset", type=float, default=1.5)
    parser.add_argument("--spacing", type=float, default=0.5)
    parser.add_argument("--debug", default="/home/ranim/autoCar_ws/waypoints/lane_level_graph_debug.png")
    parser.add_argument("--validation-debug", default="/home/ranim/autoCar_ws/waypoints/lane_direction_validation.png")
    parser.add_argument("--cleaned-debug", default="/home/ranim/autoCar_ws/waypoints/lane_level_graph_cleaned_debug.png")
    parser.add_argument("--connections-debug", default="/home/ranim/autoCar_ws/waypoints/lane_level_connections_debug.png")
    parser.add_argument("--rejected-debug", default="/home/ranim/autoCar_ws/waypoints/rejected_lane_connections_debug.png")
    parser.add_argument("--arrow-stride", type=int, default=12)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    parents = read_parent_segments(args.input)
    if not parents:
        raise RuntimeError(f"No parent segments found in {args.input}")
    lanes = build_lane_records(parents, args.lane_offset)
    faults = validate_lane_continuity(lanes, args.lane_offset)
    lane_count = write_lane_level_csv(args.output, lanes)
    draw_debug(args.debug, parents, lanes, args.arrow_stride)
    draw_validation(args.validation_debug, parents, lanes, faults, args.arrow_stride)
    cleaned_lanes, accepted_connections, rejected_connections, removed_stubs = build_cleaned_lane_network(lanes, args.spacing)
    cleaned_count = write_lane_level_csv(args.cleaned_output, cleaned_lanes)
    draw_cleaned_debug(
        args.cleaned_debug,
        cleaned_lanes,
        accepted_connections,
        removed_stubs,
        "Continuous lane-level graph: right lanes pass through junction geometry",
    )
    draw_connection_decisions(args.connections_debug, lanes, accepted_connections, accepted_mode=True)
    draw_connection_decisions(args.rejected_debug, lanes, rejected_connections, accepted_mode=False)
    print("=" * 72)
    print("Lane-level graph generated")
    print(f"Input centerline CSV : {args.input}")
    print(f"Parent segments      : {len(parents)}")
    print(f"Lane count           : {lane_count}")
    print(f"Cleaned lane count   : {cleaned_count}")
    print(f"Generated bridges    : {len(accepted_connections)}")
    print(f"Rejected bridges     : {len(rejected_connections)}")
    print(f"Removed stubs        : {len(removed_stubs)}")
    print(f"Lane offset          : {args.lane_offset:.3f} m")
    print(f"Validation faults    : {len(faults)}")
    print(f"Output CSV           : {args.output}")
    print(f"Cleaned CSV          : {args.cleaned_output}")
    print(f"Debug image          : {args.debug}")
    print(f"Validation image     : {args.validation_debug}")
    print(f"Cleaned debug        : {args.cleaned_debug}")
    print(f"Connections debug    : {args.connections_debug}")
    print(f"Rejected debug       : {args.rejected_debug}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
