#!/usr/bin/env python3
"""
Connected CSV shortest-route planner - FIXED endpoint detection.

Why this version exists:
The previous version added --undirected-base by also increasing indeg/outdeg.
That made every segment endpoint look "not open", so topology connectors were
not generated. This fixed version keeps two graphs separate:

1) raw directed CSV degrees:
   Used only to find real open endpoints.

2) traversal graph:
   Used for Dijkstra, optionally with undirected base traversal.

So --undirected-base no longer disables topology connector creation.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import os
import sys
from collections import defaultdict, deque
from typing import Dict, Iterable, List, Sequence, Tuple


TRUE_VALUES = {"1", "1.0", "true", "True", "TRUE", "yes", "YES"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Shortest route over CSV waypoints with topology stitching."
    )
    parser.add_argument("--input", required=True, help="Input waypoint CSV.")
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--output", required=True, help="Output route CSV.")

    parser.add_argument("--overlay-output", default="", help="Optional route overlay PNG.")
    parser.add_argument("--no-overlay", action="store_true")

    parser.add_argument(
        "--undirected-base",
        action="store_true",
        help="Allow original connect_next edges to be traversed in both directions.",
    )
    parser.add_argument(
        "--connector-distance",
        type=float,
        default=0.75,
        help="Max distance for graph-only same-color topology connector edges.",
    )
    parser.add_argument(
        "--connector-penalty",
        type=float,
        default=0.03,
        help="Small extra cost added to topology connector edges.",
    )
    parser.add_argument(
        "--snap-direction-group",
        default="",
        choices=["", "forward_right", "reverse_right"],
        help="Optional restriction for start/goal snapping.",
    )
    parser.add_argument(
        "--connector-direction-group",
        default="same",
        choices=["same", "any"],
        help="Use same direction_group connectors or allow any direction_group.",
    )
    parser.add_argument(
        "--max-snap-distance",
        type=float,
        default=0.0,
        help="Fail if start/goal snap farther than this many meters. 0 disables.",
    )
    parser.add_argument(
        "--preserve-yaw",
        action="store_true",
        help="Keep source yaw values instead of recomputing route yaw.",
    )
    parser.add_argument("--debug-components", action="store_true")
    return parser.parse_args()


def read_waypoints(path: str) -> Tuple[List[dict], List[str]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    required = {"x", "y", "yaw", "connect_next"}
    missing = required.difference(fieldnames)
    if missing:
        raise ValueError(f"CSV eksik kolonlar: {', '.join(sorted(missing))}")

    for i, row in enumerate(rows):
        row["_idx"] = i
        row["_x"] = float(row["x"])
        row["_y"] = float(row["y"])
        row["_yaw"] = float(row.get("yaw", 0.0) or 0.0)

    return rows, fieldnames


def connect_next_is_true(row: dict) -> bool:
    return str(row.get("connect_next", "")).strip() in TRUE_VALUES


def dist(a: dict, b: dict) -> float:
    return math.hypot(b["_x"] - a["_x"], b["_y"] - a["_y"])


def add_edge(graph, edge_set, u: int, v: int, cost: float, kind: str) -> bool:
    if u == v:
        return False
    key = (u, v, kind)
    if key in edge_set:
        return False
    edge_set.add(key)
    graph[u].append((v, cost, kind))
    return True


def compute_raw_directed_degrees(rows: Sequence[dict]) -> Tuple[List[int], List[int], int]:
    """
    Degrees from original CSV connect_next direction only.
    These degrees are used to detect open segment endpoints.
    IMPORTANT: --undirected-base must NOT change these.
    """
    indeg = [0] * len(rows)
    outdeg = [0] * len(rows)
    count = 0
    for i in range(len(rows) - 1):
        if connect_next_is_true(rows[i]):
            outdeg[i] += 1
            indeg[i + 1] += 1
            count += 1
    return indeg, outdeg, count


def build_traversal_graph(
    rows: Sequence[dict],
    undirected_base: bool,
) -> Tuple[List[List[Tuple[int, float, str]]], set, int]:
    graph: List[List[Tuple[int, float, str]]] = [[] for _ in rows]
    edge_set = set()
    edge_count = 0

    for i in range(len(rows) - 1):
        if not connect_next_is_true(rows[i]):
            continue

        cost = dist(rows[i], rows[i + 1])
        if add_edge(graph, edge_set, i, i + 1, cost, "csv"):
            edge_count += 1

        if undirected_base:
            if add_edge(graph, edge_set, i + 1, i, cost, "csv_reverse"):
                edge_count += 1

    return graph, edge_set, edge_count


def nearest_node(rows, x, y, direction_group="") -> Tuple[int, float]:
    best_i = -1
    best_d = float("inf")
    for i, row in enumerate(rows):
        if direction_group and row.get("direction_group") != direction_group:
            continue
        d = math.hypot(row["_x"] - x, row["_y"] - y)
        if d < best_d:
            best_d = d
            best_i = i
    if best_i < 0:
        raise ValueError("Snap icin uygun waypoint bulunamadi.")
    return best_i, best_d


def is_endpoint(i: int, raw_indeg: Sequence[int], raw_outdeg: Sequence[int]) -> bool:
    return raw_indeg[i] == 0 or raw_outdeg[i] == 0


def same_group_ok(a: dict, b: dict, mode: str) -> bool:
    if mode == "any":
        return True
    return a.get("direction_group") == b.get("direction_group")


def build_buckets(rows, cell_size: float):
    buckets = defaultdict(list)
    for i, row in enumerate(rows):
        buckets[(math.floor(row["_x"] / cell_size), math.floor(row["_y"] / cell_size))].append(i)
    return buckets


def nearby(buckets, row, cell_size: float):
    bx = math.floor(row["_x"] / cell_size)
    by = math.floor(row["_y"] / cell_size)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            yield from buckets.get((bx + dx, by + dy), [])


def add_topology_connectors(
    rows: Sequence[dict],
    graph,
    edge_set,
    raw_indeg,
    raw_outdeg,
    max_dist: float,
    penalty: float,
    direction_group_mode: str,
) -> int:
    """
    Add graph-only stitching edges between nearby same-color points.

    Endpoint candidates are detected from raw directed CSV degrees, not from
    the traversal graph. This is the key fix.
    """
    if max_dist <= 0:
        return 0

    buckets = build_buckets(rows, max_dist)
    endpoint_indices = [i for i in range(len(rows)) if is_endpoint(i, raw_indeg, raw_outdeg)]

    added = 0
    for i in endpoint_indices:
        a = rows[i]
        for j in nearby(buckets, a, max_dist):
            if i == j:
                continue

            b = rows[j]
            if not same_group_ok(a, b, direction_group_mode):
                continue

            d = dist(a, b)
            if d > max_dist:
                continue

            # Do not duplicate an original consecutive CSV edge.
            if abs(i - j) == 1 and (
                connect_next_is_true(rows[min(i, j)])
                or connect_next_is_true(rows[max(i, j)])
            ):
                continue

            # Avoid self-links inside exactly the same lane segment.
            if (
                a.get("lane_id") == b.get("lane_id")
                and a.get("parent_segment_id") == b.get("parent_segment_id")
            ):
                continue

            cost = d + penalty

            if add_edge(graph, edge_set, i, j, cost, "topology_connector"):
                added += 1
            if add_edge(graph, edge_set, j, i, cost, "topology_connector"):
                added += 1

    return added


def dijkstra(graph, start_idx: int, goal_idx: int):
    distances = [float("inf")] * len(graph)
    previous = [-1] * len(graph)
    distances[start_idx] = 0.0
    heap = [(0.0, start_idx)]

    while heap:
        cur_d, u = heapq.heappop(heap)
        if cur_d != distances[u]:
            continue
        if u == goal_idx:
            break

        for v, cost, _kind in graph[u]:
            nd = cur_d + cost
            if nd < distances[v]:
                distances[v] = nd
                previous[v] = u
                heapq.heappush(heap, (nd, v))

    if math.isinf(distances[goal_idx]):
        return None, float("inf")

    path = []
    node = goal_idx
    while node != -1:
        path.append(node)
        node = previous[node]
    path.reverse()
    return path, distances[goal_idx]


def component_sizes(graph):
    und = [[] for _ in graph]
    for i, edges in enumerate(graph):
        for j, _cost, _kind in edges:
            und[i].append(j)
            und[j].append(i)

    seen = [False] * len(graph)
    sizes = []
    for i in range(len(graph)):
        if seen[i]:
            continue
        q = deque([i])
        seen[i] = True
        size = 0
        while q:
            u = q.popleft()
            size += 1
            for v in und[u]:
                if not seen[v]:
                    seen[v] = True
                    q.append(v)
        sizes.append(size)

    sizes.sort(reverse=True)
    return sizes


def route_yaw(rows, route_indices, pos: int) -> float:
    if pos < len(route_indices) - 1:
        a = rows[route_indices[pos]]
        b = rows[route_indices[pos + 1]]
        return math.degrees(math.atan2(b["_y"] - a["_y"], b["_x"] - a["_x"]))
    if pos > 0:
        return route_yaw(rows, route_indices, pos - 1)
    return rows[route_indices[pos]]["_yaw"]


def clean_row(row: dict) -> dict:
    return {k: v for k, v in row.items() if not k.startswith("_")}


def save_route(output_path, rows, route_indices, fieldnames, preserve_yaw: bool):
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for out_step, idx in enumerate(route_indices, start=1):
            src = rows[idx]
            out = clean_row(src)
            if "step" in out:
                out["step"] = out_step
            if "connect_next" in out:
                out["connect_next"] = 1 if out_step < len(route_indices) else 0
            if "yaw" in out and not preserve_yaw:
                out["yaw"] = f"{route_yaw(rows, route_indices, out_step - 1):.3f}"
            writer.writerow(out)


def max_route_segment_length(rows, route_indices) -> float:
    if len(route_indices) < 2:
        return 0.0
    return max(dist(rows[a], rows[b]) for a, b in zip(route_indices, route_indices[1:]))


def topology_jumps(rows, route_indices):
    jumps = []
    for a, b in zip(route_indices, route_indices[1:]):
        if abs(a - b) != 1:
            jumps.append((a, b, dist(rows[a], rows[b])))
    return jumps


def print_node(label, rows, idx, snap_dist):
    r = rows[idx]
    print(
        f"{label}: idx={idx}, step={r.get('step')}, "
        f"x={r['_x']:.3f}, y={r['_y']:.3f}, snap_dist={snap_dist:.3f}, "
        f"lane_id={r.get('lane_id')}, parent={r.get('parent_segment_id')}, "
        f"direction_group={r.get('direction_group')}, connect_next={r.get('connect_next')}"
    )


def overlay_path(output_csv, overlay_output):
    if overlay_output:
        return overlay_output
    base, _ = os.path.splitext(output_csv)
    return f"{base}_overlay.png"


def plot_route(rows, route_indices, output_png):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"Overlay atlandi: matplotlib yok ({exc}).")
        return False

    os.makedirs(os.path.dirname(output_png) or ".", exist_ok=True)

    colors = {
        "forward_right": "#1565c0",
        "reverse_right": "#ef6c00",
    }

    fig, ax = plt.subplots(figsize=(12, 10))

    # draw original CSV edges
    for i in range(len(rows) - 1):
        if not connect_next_is_true(rows[i]):
            continue
        color = colors.get(rows[i].get("direction_group"), "#9e9e9e")
        ax.plot(
            [rows[i]["_x"], rows[i + 1]["_x"]],
            [rows[i]["_y"], rows[i + 1]["_y"]],
            color=color,
            linewidth=0.8,
            alpha=0.45,
        )

    rx = [rows[i]["_x"] for i in route_indices]
    ry = [rows[i]["_y"] for i in route_indices]
    ax.plot(rx, ry, color="#d7191c", linewidth=3.0, label="planned route")
    ax.scatter([rx[0]], [ry[0]], s=70, color="#2e7d32", label="start")
    ax.scatter([rx[-1]], [ry[-1]], s=70, color="#6a1b9a", label="goal")

    # dotted topology stitches used by route
    jump_x, jump_y = [], []
    for a, b, _d in topology_jumps(rows, route_indices):
        jump_x += [rows[a]["_x"], rows[b]["_x"], None]
        jump_y += [rows[a]["_y"], rows[b]["_y"], None]
    if jump_x:
        ax.plot(jump_x, jump_y, color="#7b1fa2", linewidth=1.3, linestyle="--", label="topology stitch")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_png, dpi=170)
    plt.close(fig)
    return True


def main() -> int:
    args = parse_args()

    rows, fieldnames = read_waypoints(args.input)

    # Raw degrees are always directed, even when traversal is undirected.
    raw_indeg, raw_outdeg, raw_csv_edges = compute_raw_directed_degrees(rows)

    graph, edge_set, traversal_csv_edges = build_traversal_graph(
        rows,
        undirected_base=args.undirected_base,
    )

    topology_edges = add_topology_connectors(
        rows=rows,
        graph=graph,
        edge_set=edge_set,
        raw_indeg=raw_indeg,
        raw_outdeg=raw_outdeg,
        max_dist=args.connector_distance,
        penalty=args.connector_penalty,
        direction_group_mode=args.connector_direction_group,
    )

    start_idx, start_snap = nearest_node(
        rows, args.start_x, args.start_y, args.snap_direction_group
    )
    goal_idx, goal_snap = nearest_node(
        rows, args.goal_x, args.goal_y, args.snap_direction_group
    )

    print("=" * 72)
    print("CONNECTED CSV SHORTEST ROUTE - FIXED")
    print("=" * 72)
    print(f"input: {args.input}")
    print(f"nodes: {len(rows)}")
    print(f"raw directed connect_next edges: {raw_csv_edges}")
    print(f"traversal connect_next edges: {traversal_csv_edges}")
    print(f"topology connector edges added: {topology_edges}")
    print(f"connector distance: {args.connector_distance}")
    print(f"undirected base traversal: {args.undirected_base}")

    print_node("Start snap", rows, start_idx, start_snap)
    print_node("Goal snap", rows, goal_idx, goal_snap)

    if args.max_snap_distance > 0:
        if start_snap > args.max_snap_distance or goal_snap > args.max_snap_distance:
            print("Snap mesafesi limit disinda; rota uretilmedi.")
            return 2

    if args.debug_components:
        sizes = component_sizes(graph)
        print(f"component count after topology connectors: {len(sizes)}")
        print(f"largest components: {sizes[:10]}")

    path, total_len = dijkstra(graph, start_idx, goal_idx)
    if path is None:
        sizes = component_sizes(graph)
        print()
        print("Rota bulunamadi.")
        print("Start ve goal hala bagli degil.")
        print("Dene: --connector-distance 1.0 veya 1.25")
        print(f"component count: {len(sizes)}")
        print(f"largest components: {sizes[:10]}")
        return 1

    save_route(args.output, rows, path, fieldnames, preserve_yaw=args.preserve_yaw)

    jumps = topology_jumps(rows, path)

    print(f"route nodes: {len(path)}")
    print(f"route edges: {max(0, len(path) - 1)}")
    print(f"total route length: {total_len:.3f}")
    print(f"max route segment length: {max_route_segment_length(rows, path):.3f}")
    print(f"topology stitches used in route: {len(jumps)}")
    if jumps:
        print("first topology stitches:")
        for a, b, d in jumps[:12]:
            print(
                f"  {a}->{b}, d={d:.3f}, "
                f"{rows[a].get('parent_segment_id')} -> {rows[b].get('parent_segment_id')}"
            )
    print(f"output: {args.output}")

    if not args.no_overlay:
        png = overlay_path(args.output, args.overlay_output)
        if plot_route(rows, path, png):
            print(f"overlay: {png}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
