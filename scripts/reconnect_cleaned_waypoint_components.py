#!/usr/bin/env python3
import argparse
import csv
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import networkx as nx


Point = Tuple[float, float]


def distance(a: dict, b: dict) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def yaw_diff_deg(a: float, b: float) -> float:
    diff = (a - b + 180.0) % 360.0 - 180.0
    return abs(diff)


def read_rows(path: str) -> Tuple[List[str], List[dict]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise RuntimeError(f"CSV has no header: {path}")
        missing = {"x", "y"} - set(reader.fieldnames)
        if missing:
            raise RuntimeError(f"Missing required columns in {path}: {sorted(missing)}")
        return list(reader.fieldnames), list(reader)


def save_rows(path: str, fieldnames: Sequence[str], rows: Sequence[dict]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    out_fields = list(fieldnames)
    if "connect_next" not in out_fields:
        out_fields.append("connect_next")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=out_fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in out_fields})


def nearest_waypoint(rows: Sequence[dict], target: Point) -> int:
    return min(range(len(rows)), key=lambda i: math.hypot(float(rows[i]["x"]) - target[0], float(rows[i]["y"]) - target[1]))


def build_graph(rows: Sequence[dict]) -> nx.Graph:
    graph = nx.Graph()
    for idx in range(len(rows)):
        graph.add_node(idx)
    for idx in range(len(rows) - 1):
        if int(float(rows[idx].get("connect_next", 1))) == 1:
            graph.add_edge(idx, idx + 1)
    return graph


def component_lookup(components: Sequence[set]) -> Dict[int, int]:
    lookup = {}
    for component_id, component in enumerate(components):
        for node in component:
            lookup[node] = component_id
    return lookup


def connected_runs(rows: Sequence[dict]) -> List[List[dict]]:
    runs = []
    current = []
    for idx, row in enumerate(rows):
        current.append(row)
        if int(float(row.get("connect_next", 1))) == 0 or idx == len(rows) - 1:
            if len(current) >= 2:
                runs.append(current)
            current = []
    return runs


def collect_breaks(rows: Sequence[dict]) -> List[dict]:
    breaks = []
    for idx in range(len(rows) - 1):
        if int(float(rows[idx].get("connect_next", 1))) != 0:
            continue
        a = rows[idx]
        b = rows[idx + 1]
        breaks.append(
            {
                "idx": idx,
                "a": a,
                "b": b,
                "distance": distance(a, b),
                "yaw_diff": yaw_diff_deg(float(a.get("yaw", 0.0) or 0.0), float(b.get("yaw", 0.0) or 0.0)),
            }
        )
    return breaks


def should_reconnect(candidate: dict, threshold: float) -> Tuple[bool, str]:
    dist = candidate["distance"]
    yaw_diff = candidate["yaw_diff"]
    if dist > threshold:
        return False, "too_far_for_threshold"
    if dist > 8.0:
        return False, "over_absolute_limit"
    if dist <= 4.0 and yaw_diff <= 90.0:
        return True, "short_arc"
    if yaw_diff <= 45.0:
        return True, "same_road"
    return False, "yaw_mismatch"


def reconnect_for_threshold(rows: Sequence[dict], threshold: float) -> Tuple[List[dict], List[dict], List[dict]]:
    trial_rows = [dict(row) for row in rows]
    accepted = []
    rejected = []
    for candidate in collect_breaks(trial_rows):
        ok, reason = should_reconnect(candidate, threshold)
        candidate["reason"] = reason
        if ok:
            trial_rows[candidate["idx"]]["connect_next"] = "1"
            accepted.append(candidate)
        else:
            trial_rows[candidate["idx"]]["connect_next"] = "0"
            rejected.append(candidate)
    if trial_rows:
        trial_rows[-1]["connect_next"] = "0"
    return trial_rows, accepted, rejected


def analyze(rows: Sequence[dict], start_xy: Point, goal_xy: Point) -> dict:
    graph = build_graph(rows)
    components = list(nx.connected_components(graph))
    lookup = component_lookup(components)
    start_idx = nearest_waypoint(rows, start_xy)
    goal_idx = nearest_waypoint(rows, goal_xy)
    return {
        "graph": graph,
        "components": components,
        "lookup": lookup,
        "start_idx": start_idx,
        "goal_idx": goal_idx,
        "start_component": lookup[start_idx],
        "goal_component": lookup[goal_idx],
        "same_component": lookup[start_idx] == lookup[goal_idx],
    }


def plot_candidates(output: str, rows: Sequence[dict], accepted: Sequence[dict], rejected: Sequence[dict]) -> None:
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 11))
    ax.set_title("Reconnect candidates from existing consecutive CSV breaks")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.scatter([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], s=5, color="#bdbdbd", alpha=0.35, label="waypoints")

    first = True
    for candidate in rejected:
        a = candidate["a"]
        b = candidate["b"]
        ax.plot([float(a["x"]), float(b["x"])], [float(a["y"]), float(b["y"])], color="#d62728", linewidth=1.0, alpha=0.55, label="kept disconnected" if first else None)
        first = False

    first = True
    for candidate in accepted:
        a = candidate["a"]
        b = candidate["b"]
        ax.plot([float(a["x"]), float(b["x"])], [float(a["y"]), float(b["y"])], color="#2ca02c", linewidth=1.7, alpha=0.9, label="reconnected" if first else None)
        ax.text(
            0.5 * (float(a["x"]) + float(b["x"])),
            0.5 * (float(a["y"]) + float(b["y"])),
            f"{candidate['distance']:.1f}m/{candidate['yaw_diff']:.0f}",
            fontsize=6,
            color="#145a14",
        )
        first = False

    ax.legend(loc="best")
    fig.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(fig)


def plot_components(output: str, rows: Sequence[dict], components: Sequence[set], start_xy: Point, goal_xy: Point, start_idx: int, goal_idx: int) -> None:
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 11))
    ax.set_title("Components after conservative reconnect")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    cmap = plt.get_cmap("tab20")
    sorted_components = sorted(enumerate(components), key=lambda item: len(item[1]), reverse=True)
    for draw_idx, (component_id, component) in enumerate(sorted_components):
        color = cmap(draw_idx % 20)
        points = [rows[idx] for idx in sorted(component)]
        ax.scatter([float(p["x"]) for p in points], [float(p["y"]) for p in points], s=6, color=color, alpha=0.8, label=f"C{component_id} ({len(component)})" if draw_idx < 12 else None)
        component_set = set(component)
        run = []
        for idx, row in enumerate(rows):
            if idx in component_set:
                run.append(row)
                if int(float(row.get("connect_next", 1))) == 0 or idx == len(rows) - 1:
                    if len(run) >= 2:
                        ax.plot([float(r["x"]) for r in run], [float(r["y"]) for r in run], color=color, linewidth=0.8, alpha=0.7)
                    run = []
            elif run:
                if len(run) >= 2:
                    ax.plot([float(r["x"]) for r in run], [float(r["y"]) for r in run], color=color, linewidth=0.8, alpha=0.7)
                run = []

    ax.scatter([start_xy[0]], [start_xy[1]], s=120, color="#2ca02c", marker="o", label="start request", zorder=5)
    ax.scatter([goal_xy[0]], [goal_xy[1]], s=150, color="#e31a1c", marker="*", label="goal request", zorder=5)
    ax.scatter([float(rows[start_idx]["x"])], [float(rows[start_idx]["y"])], s=90, color="#006400", marker="x", label="nearest start wp", zorder=6)
    ax.scatter([float(rows[goal_idx]["x"])], [float(rows[goal_idx]["y"])], s=100, color="#8b0000", marker="x", label="nearest goal wp", zorder=6)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Conservatively reconnect only existing consecutive CSV breaks in cleaned waypoint map")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--thresholds", default="5.0,6.0,7.0,8.0")
    parser.add_argument("--debug-dir", default="/home/ranim/autoCar_ws/waypoints")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    fieldnames, rows = read_rows(args.input)
    if "connect_next" not in fieldnames:
        fieldnames.append("connect_next")
        for row in rows:
            row["connect_next"] = "1"
    if rows:
        rows[-1]["connect_next"] = "0"

    start_xy = (args.start_x, args.start_y)
    goal_xy = (args.goal_x, args.goal_y)
    thresholds = [float(value.strip()) for value in args.thresholds.split(",") if value.strip()]
    thresholds = [value for value in thresholds if value <= 8.0]
    if not thresholds:
        raise RuntimeError("No valid threshold <= 8.0 was provided")

    initial = analyze(rows, start_xy, goal_xy)
    selected_rows: Optional[List[dict]] = None
    selected_threshold: Optional[float] = None
    selected_accepted: List[dict] = []
    selected_rejected: List[dict] = []
    selected_analysis = initial

    print("=" * 72)
    print("Conservative waypoint reconnect")
    print(f"Input              : {args.input}")
    print(f"Initial components : {len(initial['components'])}")
    print(f"Initial start comp : {initial['start_component']}")
    print(f"Initial goal comp  : {initial['goal_component']}")

    for threshold in thresholds:
        trial_rows, accepted, rejected = reconnect_for_threshold(rows, threshold)
        trial_analysis = analyze(trial_rows, start_xy, goal_xy)
        print("-" * 72)
        print(f"Threshold {threshold:.1f} m")
        print(f"Accepted reconnects : {len(accepted)}")
        print(f"Rejected breaks     : {len(rejected)}")
        print(f"Components          : {len(trial_analysis['components'])}")
        print(f"Start component     : {trial_analysis['start_component']}")
        print(f"Goal component      : {trial_analysis['goal_component']}")
        selected_rows = trial_rows
        selected_threshold = threshold
        selected_accepted = accepted
        selected_rejected = rejected
        selected_analysis = trial_analysis
        if trial_analysis["same_component"]:
            print("Start and goal are now in the same component")
            break

    if selected_rows is None:
        raise RuntimeError("Reconnect did not run")

    save_rows(args.output, fieldnames, selected_rows)
    plot_candidates(os.path.join(args.debug_dir, "reconnect_candidates_debug.png"), selected_rows, selected_accepted, selected_rejected)
    plot_components(
        os.path.join(args.debug_dir, "reconnected_components_debug.png"),
        selected_rows,
        selected_analysis["components"],
        start_xy,
        goal_xy,
        selected_analysis["start_idx"],
        selected_analysis["goal_idx"],
    )

    print("-" * 72)
    print(f"Selected threshold : {selected_threshold:.1f} m")
    print(f"Output             : {args.output}")
    print(f"Final components   : {len(selected_analysis['components'])}")
    print(f"Final start comp   : {selected_analysis['start_component']}")
    print(f"Final goal comp    : {selected_analysis['goal_component']}")
    print(f"Same component     : {selected_analysis['same_component']}")
    print(f"Debug candidates   : {os.path.join(args.debug_dir, 'reconnect_candidates_debug.png')}")
    print(f"Debug components   : {os.path.join(args.debug_dir, 'reconnected_components_debug.png')}")
    print("=" * 72)


if __name__ == "__main__":
    main()
