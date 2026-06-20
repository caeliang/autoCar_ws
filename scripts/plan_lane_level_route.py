#!/usr/bin/env python3
"""Plan a lane-level route from lane_level_graph_cleaned.csv."""

import argparse
import csv
import heapq
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont


Point = Tuple[float, float]


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_diff_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def signed_yaw_diff_deg(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def tangent(yaw_deg: float) -> Point:
    rad = math.radians(yaw_deg)
    return (math.cos(rad), math.sin(rad))


def heading_name(yaw_deg: float) -> str:
    yaw = yaw_deg % 360.0
    if yaw < 45.0 or yaw >= 315.0:
        return "east"
    if yaw < 135.0:
        return "north"
    if yaw < 225.0:
        return "west"
    return "south"


def turn_type(incoming_yaw: float, outgoing_yaw: float) -> str:
    incoming = heading_name(incoming_yaw)
    outgoing = heading_name(outgoing_yaw)
    if incoming == outgoing:
        return f"{incoming}_straight"
    return f"{incoming}_to_{outgoing}"


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def read_csv_rows(path: Path) -> Tuple[List[dict], List[str]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    return rows, fieldnames


def read_lane_rows(path: Path) -> Tuple[List[dict], List[str]]:
    rows, fieldnames = read_csv_rows(path)
    required = {"x", "y", "yaw", "lane_id", "direction_group", "connect_next"}
    missing = required - set(fieldnames)
    if missing:
        raise RuntimeError(f"Missing required columns in {path}: {sorted(missing)}")

    if "segment_id" not in fieldnames:
        fieldnames = fieldnames + ["segment_id"]

    for idx, row in enumerate(rows):
        row["idx"] = idx
        row["x_f"] = float(row["x"])
        row["y_f"] = float(row["y"])
        row["z_f"] = float(row.get("z", 0.5) or 0.5)
        row["yaw_f"] = float(row.get("yaw", 0.0) or 0.0)
        row["connect_next_i"] = int(float(row.get("connect_next", 1) or 1))
        row["segment_id"] = row.get("segment_id") or row.get("parent_segment_id") or row.get("lane_id", "")
    if rows:
        rows[-1]["connect_next_i"] = 0
    return rows, fieldnames


def xy(row: dict) -> Point:
    return (row["x_f"], row["y_f"])


def read_reference_start_goal(route_path: Path) -> Tuple[Point, float, Point, float]:
    rows, _ = read_csv_rows(route_path)
    if len(rows) < 2:
        raise RuntimeError(f"Cannot infer start/goal from {route_path}")
    first = rows[0]
    last = rows[-1]
    return (
        (float(first["x"]), float(first["y"])),
        float(first.get("yaw", 0.0) or 0.0),
        (float(last["x"]), float(last["y"])),
        float(last.get("yaw", 0.0) or 0.0),
    )


def candidate_nodes(
    rows: Sequence[dict],
    point: Point,
    yaw_deg: float,
    max_yaw_diff: float,
    limit: int,
    max_extra_distance: float,
) -> List[int]:
    scored = []
    for row in rows:
        d = dist(xy(row), point)
        yd = yaw_diff_deg(row["yaw_f"], yaw_deg)
        if yd > max_yaw_diff:
            continue
        scored.append((d + yd * 0.05, d, yd, row["idx"]))
    if not scored:
        raise RuntimeError(f"No lane-level candidate near {point} with yaw {yaw_deg:.1f} deg")
    scored.sort()
    best_d = scored[0][1]
    return [idx for _, d, _, idx in scored if d <= best_d + max_extra_distance][:limit]


def build_runs(rows: Sequence[dict]) -> List[List[int]]:
    runs: List[List[int]] = []
    current: List[int] = []
    for i, row in enumerate(rows):
        current.append(i)
        if row["connect_next_i"] == 0 or i == len(rows) - 1:
            if current:
                runs.append(current)
            current = []
    return runs


def first_indices_by_run(rows: Sequence[dict]) -> List[int]:
    return [run[0] for run in build_runs(rows) if run]


def is_valid_transition(a: dict, b: dict, max_distance: float, max_yaw_diff: float) -> Tuple[bool, float]:
    if a["lane_id"] == b["lane_id"]:
        return False, math.inf
    d = dist(xy(a), xy(b))
    if d > max_distance:
        return False, math.inf

    yd = yaw_diff_deg(a["yaw_f"], b["yaw_f"])
    if yd >= max_yaw_diff:
        return False, math.inf

    gap = (b["x_f"] - a["x_f"], b["y_f"] - a["y_f"])
    if d > 1e-6 and dot(gap, tangent(a["yaw_f"])) < -0.35 * d:
        return False, math.inf

    direction_penalty = 0.0 if a.get("direction_group") == b.get("direction_group") else 1.5
    same_parent_penalty = 0.0 if a.get("parent_segment_id") == b.get("parent_segment_id") else 0.4
    return True, d + yd * 0.04 + direction_penalty + same_parent_penalty


def build_graph(rows: Sequence[dict], max_transition_distance: float, max_transition_yaw: float) -> Dict[int, List[Tuple[float, int]]]:
    graph: Dict[int, List[Tuple[float, int]]] = {i: [] for i in range(len(rows))}

    for i in range(len(rows) - 1):
        a = rows[i]
        b = rows[i + 1]
        if a["connect_next_i"] != 1:
            continue
        if a.get("lane_id") != b.get("lane_id"):
            continue
        if a.get("direction_group") != b.get("direction_group"):
            continue
        graph[i].append((dist(xy(a), xy(b)), i + 1))

    run_starts = first_indices_by_run(rows)
    run_ends = [run[-1] for run in build_runs(rows) if run]
    for end_idx in run_ends:
        a = rows[end_idx]
        for start_idx in run_starts:
            b = rows[start_idx]
            ok, cost = is_valid_transition(a, b, max_transition_distance, max_transition_yaw)
            if ok:
                graph[end_idx].append((cost, start_idx))

    return graph


def bezier_point(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - t
    return (
        u**3 * p0[0] + 3.0 * u**2 * t * p1[0] + 3.0 * u * t**2 * p2[0] + t**3 * p3[0],
        u**3 * p0[1] + 3.0 * u**2 * t * p1[1] + 3.0 * u * t**2 * p2[1] + t**3 * p3[1],
    )


def resample_polyline(points: Sequence[Point], spacing: float) -> List[Point]:
    if len(points) < 2:
        return list(points)
    result = [points[0]]
    carry = 0.0
    current = points[0]
    for target in points[1:]:
        seg_len = dist(current, target)
        while carry + seg_len >= spacing and seg_len > 1e-9:
            t = (spacing - carry) / seg_len
            new_point = (
                current[0] + (target[0] - current[0]) * t,
                current[1] + (target[1] - current[1]) * t,
            )
            result.append(new_point)
            current = new_point
            seg_len = dist(current, target)
            carry = 0.0
        carry += seg_len
        current = target
    if dist(result[-1], points[-1]) > spacing * 0.35:
        result.append(points[-1])
    return result


def compute_yaws_for_points(points: Sequence[Point]) -> List[float]:
    yaws: List[float] = []
    last = 0.0
    for i in range(len(points)):
        if i < len(points) - 1:
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
        elif i > 0:
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
        else:
            dx = dy = 0.0
        if math.hypot(dx, dy) > 1e-9:
            last = math.degrees(math.atan2(dy, dx))
        yaws.append(last)
    return yaws


def transition_bezier_points(
    start_row: dict,
    end_row: dict,
    spacing: float,
    north_to_west_offset_correction: float,
) -> Tuple[List[Point], str]:
    p0 = xy(start_row)
    p3 = xy(end_row)
    gap = dist(p0, p3)
    if gap < 1e-6:
        return [p0, p3], turn_type(start_row["yaw_f"], end_row["yaw_f"])

    t0 = tangent(start_row["yaw_f"])
    t1 = tangent(end_row["yaw_f"])
    handle = max(0.8, min(4.0, gap * 0.75))
    turn = turn_type(start_row["yaw_f"], end_row["yaw_f"])
    p1 = (p0[0] + t0[0] * handle, p0[1] + t0[1] * handle)
    p2 = (p3[0] - t1[0] * handle, p3[1] - t1[1] * handle)

    samples = [bezier_point(p0, p1, p2, p3, i / 32.0) for i in range(33)]
    if turn == "north_to_west" and north_to_west_offset_correction > 0.0:
        # Last-resort nudge along the expected right-lane side for a north->west
        # maneuver: east/north quadrant relative to the centerline corner.
        corrected = []
        for i, point in enumerate(samples):
            t = i / max(1, len(samples) - 1)
            weight = math.sin(math.pi * t)
            corrected.append((
                point[0] + north_to_west_offset_correction * weight,
                point[1] + north_to_west_offset_correction * weight,
            ))
        samples = corrected

    return resample_polyline(samples, spacing), turn


def astar(
    rows: Sequence[dict],
    graph: Dict[int, List[Tuple[float, int]]],
    starts: Sequence[int],
    goals: Sequence[int],
    start_point: Point,
    start_yaw: float,
) -> List[int]:
    goal_set = set(goals)

    def h(idx: int) -> float:
        p = xy(rows[idx])
        return min(dist(p, xy(rows[g])) for g in goals)

    heap = []
    best: Dict[int, float] = {}
    prev: Dict[int, int] = {}
    for start in starts:
        initial_cost = dist(xy(rows[start]), start_point) * 12.0 + yaw_diff_deg(rows[start]["yaw_f"], start_yaw) * 0.08
        best[start] = initial_cost
        heapq.heappush(heap, (initial_cost + h(start), initial_cost, start))

    final = None
    while heap:
        _, cost, cur = heapq.heappop(heap)
        if cost > best.get(cur, math.inf):
            continue
        if cur in goal_set:
            final = cur
            break
        for edge_cost, nxt in graph.get(cur, []):
            new_cost = cost + edge_cost
            if new_cost < best.get(nxt, math.inf):
                best[nxt] = new_cost
                prev[nxt] = cur
                heapq.heappush(heap, (new_cost + h(nxt), new_cost, nxt))

    if final is None:
        return []
    route = [final]
    while route[-1] in prev:
        route.append(prev[route[-1]])
    route.reverse()
    return route


def route_to_rows(
    rows: Sequence[dict],
    route: Sequence[int],
    fieldnames: Sequence[str],
    spacing: float,
    north_to_west_offset_correction: float,
) -> Tuple[List[dict], List[str], List[dict]]:
    output_fields = ["step", "x", "y", "z", "yaw", "lane_id", "direction_group", "segment_id", "connect_next"]
    for field in fieldnames:
        if field not in output_fields and not field.endswith("_f") and field not in {"idx", "connect_next_i"}:
            output_fields.append(field)

    out = []
    turn_logs: List[dict] = []

    def append_row(src_idx: int, x: float, y: float, yaw: float, connect_next: str = "1") -> None:
        src = dict(rows[src_idx])
        src["step"] = str(len(out) + 1)
        src["x"] = f"{x:.3f}"
        src["y"] = f"{y:.3f}"
        src["z"] = f"{rows[src_idx]['z_f']:.3f}"
        src["yaw"] = f"{yaw:.3f}"
        src["segment_id"] = rows[src_idx].get("segment_id", "")
        src["connect_next"] = connect_next
        out.append({field: src.get(field, "") for field in output_fields})

    for pos, idx in enumerate(route):
        if pos == 0:
            append_row(idx, rows[idx]["x_f"], rows[idx]["y_f"], rows[idx]["yaw_f"])
            continue

        prev_idx = route[pos - 1]
        prev_row = rows[prev_idx]
        row = rows[idx]
        same_lane_step = (
            idx == prev_idx + 1
            and prev_row.get("lane_id") == row.get("lane_id")
            and prev_row.get("direction_group") == row.get("direction_group")
        )
        if same_lane_step:
            append_row(idx, row["x_f"], row["y_f"], row["yaw_f"])
            continue

        connector, turn = transition_bezier_points(prev_row, row, spacing, north_to_west_offset_correction)
        connector_yaws = compute_yaws_for_points(connector)
        turn_logs.append({
            "turn_type": turn,
            "incoming_yaw": prev_row["yaw_f"],
            "outgoing_yaw": row["yaw_f"],
            "start": xy(prev_row),
            "end": xy(row),
            "incoming_lane_id": prev_row.get("lane_id", ""),
            "outgoing_lane_id": row.get("lane_id", ""),
        })
        for point, yaw in zip(connector[1:-1], connector_yaws[1:-1]):
            append_row(prev_idx, point[0], point[1], yaw)
        append_row(idx, row["x_f"], row["y_f"], row["yaw_f"])

    if out:
        out[-1]["connect_next"] = "0"
    return out, output_fields, turn_logs


def write_rows(path: Path, rows: Sequence[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_points(path: Path) -> List[dict]:
    rows, _ = read_csv_rows(path)
    out = []
    for row in rows:
        out.append({"x": float(row["x"]), "y": float(row["y"]), **row})
    return out


def mean_centerline_offset(route_rows: Sequence[dict], center_rows: Sequence[dict]) -> float:
    if not route_rows or not center_rows:
        return math.inf
    total = 0.0
    for row in route_rows:
        p = (float(row["x"]), float(row["y"]))
        total += min(dist(p, (float(c["x"]), float(c["y"]))) for c in center_rows)
    return total / len(route_rows)


def project_factory(groups: Sequence[Sequence[Point]], width: int, height: int, margin: int):
    points = [p for group in groups for p in group]
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def project(p: Point):
        x = margin + (p[0] - min_x) * scale
        y = height - margin - (p[1] - min_y) * scale
        return (x, y)

    return project


def draw_runs(draw: ImageDraw.ImageDraw, rows: Sequence[dict], project, color_for_row, width: int) -> None:
    for run in build_runs(rows):
        if len(run) < 2:
            continue
        pts = [project(xy(rows[i])) for i in run]
        draw.line(pts, fill=color_for_row(rows[run[0]]), width=width, joint="curve")


def draw_overlay(centerline: Path, lane_rows: Sequence[dict], route_rows: Sequence[dict], output: Path) -> None:
    center_rows, _ = read_csv_rows(centerline)
    width, height, margin = 1600, 1200, 85
    groups = [
        [(float(r["x"]), float(r["y"])) for r in center_rows],
        [xy(r) for r in lane_rows],
        [(float(r["x"]), float(r["y"])) for r in route_rows],
    ]
    project = project_factory(groups, width, height, margin)
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    for row_a, row_b in zip(center_rows, center_rows[1:]):
        if int(float(row_a.get("connect_next", 1) or 1)) == 1:
            draw.line([project((float(row_a["x"]), float(row_a["y"]))), project((float(row_b["x"]), float(row_b["y"])))], fill=(170, 170, 170), width=2)

    def lane_color(row):
        return (21, 101, 192) if row.get("direction_group") == "forward_right" else (239, 108, 0)

    draw_runs(draw, lane_rows, project, lane_color, 2)
    route_points = [project((float(r["x"]), float(r["y"]))) for r in route_rows]
    if len(route_points) >= 2:
        draw.line(route_points, fill=(220, 30, 30), width=6, joint="curve")

    legend = [
        ((170, 170, 170), "centerline generated_lane_graph.csv"),
        ((21, 101, 192), "lane_level_graph_cleaned.csv forward_right"),
        ((239, 108, 0), "lane_level_graph_cleaned.csv reverse_right"),
        ((220, 30, 30), "planned_route_lane_smoothed.csv"),
    ]
    y = 25
    for color, label in legend:
        draw.line([(margin, y + 6), (margin + 55, y + 6)], fill=color, width=5)
        draw.text((margin + 65, y), label, fill=(0, 0, 0), font=font)
        y += 24
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="waypoints/lane_level_graph_cleaned.csv")
    parser.add_argument("--centerline", default="waypoints/generated_lane_graph.csv")
    parser.add_argument("--reference-route", default="waypoints/planned_route.csv")
    parser.add_argument("--output", default="waypoints/planned_route_lane.csv")
    parser.add_argument("--overlay-output", default="waypoints/lane_route_overlay_debug.png")
    parser.add_argument("--route-for-overlay", default=None)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--start-yaw", type=float)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-yaw", type=float)
    parser.add_argument("--candidate-yaw-diff", type=float, default=75.0)
    parser.add_argument("--transition-yaw-diff", type=float, default=135.0)
    parser.add_argument("--transition-distance", type=float, default=6.0)
    parser.add_argument("--candidate-limit", type=int, default=14)
    parser.add_argument("--candidate-extra-distance", type=float, default=4.0)
    parser.add_argument("--min-centerline-offset", type=float, default=0.8)
    parser.add_argument("--spacing", type=float, default=0.5)
    parser.add_argument("--north-to-west-offset-correction", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    centerline_path = Path(args.centerline)
    output_path = Path(args.output)
    overlay_path = Path(args.overlay_output)

    ref_start, ref_start_yaw, ref_goal, ref_goal_yaw = read_reference_start_goal(Path(args.reference_route))
    start = (args.start_x if args.start_x is not None else ref_start[0], args.start_y if args.start_y is not None else ref_start[1])
    goal = (args.goal_x if args.goal_x is not None else ref_goal[0], args.goal_y if args.goal_y is not None else ref_goal[1])
    start_yaw = args.start_yaw if args.start_yaw is not None else ref_start_yaw
    goal_yaw = args.goal_yaw if args.goal_yaw is not None else ref_goal_yaw

    rows, fieldnames = read_lane_rows(input_path)
    starts = candidate_nodes(rows, start, start_yaw, args.candidate_yaw_diff, args.candidate_limit, args.candidate_extra_distance)
    goals = candidate_nodes(rows, goal, goal_yaw, args.candidate_yaw_diff, args.candidate_limit, args.candidate_extra_distance)
    graph = build_graph(rows, args.transition_distance, args.transition_yaw_diff)
    route = astar(rows, graph, starts, goals, start, start_yaw)
    if not route:
        raise RuntimeError(
            "Lane-level route not found. Start/goal candidates are not connected through valid directed lane links."
        )

    route_rows, output_fields, turn_logs = route_to_rows(
        rows,
        route,
        fieldnames,
        args.spacing,
        args.north_to_west_offset_correction,
    )
    center_rows, _ = read_csv_rows(centerline_path)
    offset = mean_centerline_offset(route_rows, center_rows)
    if offset < args.min_centerline_offset:
        raise RuntimeError(f"Route is still centerline, not lane-level. mean_offset={offset:.3f} m")

    write_rows(output_path, route_rows, output_fields)
    overlay_route_rows = route_rows
    if args.route_for_overlay and Path(args.route_for_overlay).exists():
        overlay_route_rows, _ = read_csv_rows(Path(args.route_for_overlay))
    draw_overlay(centerline_path, rows, overlay_route_rows, overlay_path)

    print("=" * 72)
    print("Lane-level route planned")
    print(f"Input lane graph        : {input_path}")
    print(f"Output route            : {output_path}")
    print(f"Overlay debug           : {overlay_path}")
    print(f"Lane graph points       : {len(rows)}")
    print(f"Graph edges             : {sum(len(v) for v in graph.values())}")
    print(f"Start candidates        : {len(starts)} first={starts[0]} lane={rows[starts[0]]['lane_id']} yaw={rows[starts[0]]['yaw_f']:.1f}")
    print(f"Goal candidates         : {len(goals)} first={goals[0]} lane={rows[goals[0]]['lane_id']} yaw={rows[goals[0]]['yaw_f']:.1f}")
    print(f"Route points            : {len(route_rows)}")
    print(f"Mean centerline offset  : {offset:.3f} m")
    print(f"Turn transitions        : {len(turn_logs)}")
    for item in turn_logs:
        print(
            "  {turn_type}: incoming_yaw={incoming_yaw:.1f} outgoing_yaw={outgoing_yaw:.1f} "
            "from={incoming_lane_id} to={outgoing_lane_id} start=({sx:.2f},{sy:.2f}) end=({ex:.2f},{ey:.2f})".format(
                sx=item["start"][0],
                sy=item["start"][1],
                ex=item["end"][0],
                ey=item["end"][1],
                **item,
            )
        )
    print("=" * 72)


if __name__ == "__main__":
    main()
