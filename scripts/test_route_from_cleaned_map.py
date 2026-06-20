#!/usr/bin/env python3
import argparse
import csv
import heapq
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import networkx as nx


Point = Tuple[float, float]


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def read_waypoints(path: str) -> List[dict]:
    points = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"Missing required columns in {path}: {sorted(missing)}")
        for idx, row in enumerate(reader):
            points.append(
                {
                    "idx": idx,
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row.get("z", 0.5) or 0.5),
                    "yaw": float(row.get("yaw", 0.0) or 0.0),
                    "connect_next": int(float(row.get("connect_next", 1))) if row.get("connect_next", "") != "" else 1,
                }
            )
    if points:
        points[-1]["connect_next"] = 0
    return points


def nearest_waypoint(points: Sequence[dict], target: Point) -> int:
    return min(range(len(points)), key=lambda i: dist((points[i]["x"], points[i]["y"]), target))


def build_graph(points: Sequence[dict]) -> Dict[int, List[Tuple[float, int]]]:
    graph: Dict[int, List[Tuple[float, int]]] = {point["idx"]: [] for point in points}
    for i in range(len(points) - 1):
        if int(points[i].get("connect_next", 1)) != 1:
            continue
        cost = dist((points[i]["x"], points[i]["y"]), (points[i + 1]["x"], points[i + 1]["y"]))
        graph[i].append((cost, i + 1))
    return graph


def build_networkx_graph(points: Sequence[dict]) -> nx.Graph:
    graph = nx.Graph()
    for point in points:
        graph.add_node(point["idx"])
    for i in range(len(points) - 1):
        if int(points[i].get("connect_next", 1)) != 1:
            continue
        graph.add_edge(i, i + 1)
    return graph


def component_id_by_node(components: Sequence[set]) -> Dict[int, int]:
    ids = {}
    for component_id, component in enumerate(components):
        for node in component:
            ids[node] = component_id
    return ids


def connected_components_directed_runs(points: Sequence[dict]) -> Dict[int, int]:
    component_by_idx: Dict[int, int] = {}
    component_id = 0
    for i, point in enumerate(points):
        if i not in component_by_idx:
            component_id += 1
        component_by_idx[i] = component_id
        if int(point.get("connect_next", 1)) == 0:
            component_id += 1
    return component_by_idx


def dijkstra(graph: Dict[int, List[Tuple[float, int]]], start_idx: int, goal_idx: int) -> List[int]:
    heap = [(0.0, start_idx)]
    cost_so_far = {start_idx: 0.0}
    previous: Dict[int, int] = {}
    while heap:
        cost, current = heapq.heappop(heap)
        if current == goal_idx:
            break
        if cost > cost_so_far.get(current, math.inf):
            continue
        for edge_cost, nxt in graph.get(current, []):
            new_cost = cost + edge_cost
            if new_cost < cost_so_far.get(nxt, math.inf):
                cost_so_far[nxt] = new_cost
                previous[nxt] = current
                heapq.heappush(heap, (new_cost, nxt))

    if goal_idx not in cost_so_far:
        return []

    route = [goal_idx]
    while route[-1] != start_idx:
        route.append(previous[route[-1]])
    route.reverse()
    return route


def save_route(points: Sequence[dict], route_indices: Sequence[int], output: str) -> None:
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["step", "x", "y", "z", "yaw"])
        writer.writeheader()
        for step, idx in enumerate(route_indices, start=1):
            point = points[idx]
            writer.writerow(
                {
                    "step": step,
                    "x": f"{point['x']:.3f}",
                    "y": f"{point['y']:.3f}",
                    "z": f"{point['z']:.3f}",
                    "yaw": f"{point['yaw']:.2f}",
                }
            )


def connected_runs(points: Sequence[dict]) -> List[List[dict]]:
    runs = []
    current = []
    for i, point in enumerate(points):
        current.append(point)
        if int(point.get("connect_next", 1)) == 0 or i == len(points) - 1:
            if len(current) >= 2:
                runs.append(current)
            current = []
    return runs


def save_debug(
    points: Sequence[dict],
    route_indices: Sequence[int],
    start_xy: Point,
    goal_xy: Point,
    start_idx: int,
    goal_idx: int,
    output: str,
) -> None:
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 11))
    ax.set_title("Route test from cleaned waypoint map")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")

    ax.scatter([p["x"] for p in points], [p["y"] for p in points], s=5, color="#bdbdbd", alpha=0.5, label="map waypoints")
    for run in connected_runs(points):
        ax.plot([p["x"] for p in run], [p["y"] for p in run], color="#d0d0d0", linewidth=0.7, alpha=0.5)

    if route_indices:
        route_points = [points[idx] for idx in route_indices]
        ax.plot([p["x"] for p in route_points], [p["y"] for p in route_points], color="#e31a1c", linewidth=2.2, label="planned route")
        ax.scatter([p["x"] for p in route_points], [p["y"] for p in route_points], s=8, color="#e31a1c", alpha=0.8)

    ax.scatter([start_xy[0]], [start_xy[1]], s=120, color="#2ca02c", marker="o", label="start request", zorder=5)
    ax.scatter([goal_xy[0]], [goal_xy[1]], s=150, color="#e31a1c", marker="*", label="goal request", zorder=5)
    ax.scatter([points[start_idx]["x"]], [points[start_idx]["y"]], s=80, color="#006400", marker="x", label="nearest start wp", zorder=6)
    ax.scatter([points[goal_idx]["x"]], [points[goal_idx]["y"]], s=90, color="#8b0000", marker="x", label="nearest goal wp", zorder=6)
    ax.legend(loc="best")
    fig.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(fig)


def save_components_debug(
    points: Sequence[dict],
    components: Sequence[set],
    start_xy: Point,
    goal_xy: Point,
    start_idx: int,
    goal_idx: int,
    output: str,
) -> None:
    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 11))
    ax.set_title("Connected components from cleaned waypoint graph")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")

    cmap = plt.get_cmap("tab20")
    sorted_components = sorted(enumerate(components), key=lambda item: len(item[1]), reverse=True)
    for draw_idx, (component_id, component) in enumerate(sorted_components):
        component_points = [points[idx] for idx in sorted(component)]
        color = cmap(draw_idx % 20)
        ax.scatter(
            [p["x"] for p in component_points],
            [p["y"] for p in component_points],
            s=7,
            color=color,
            alpha=0.85,
            label=f"C{component_id} ({len(component)})" if draw_idx < 12 else None,
        )
        for run in connected_runs(component_points):
            ax.plot([p["x"] for p in run], [p["y"] for p in run], color=color, linewidth=0.8, alpha=0.7)

    ax.scatter([start_xy[0]], [start_xy[1]], s=120, color="#2ca02c", marker="o", label="start request", zorder=5)
    ax.scatter([goal_xy[0]], [goal_xy[1]], s=150, color="#e31a1c", marker="*", label="goal request", zorder=5)
    ax.scatter([points[start_idx]["x"]], [points[start_idx]["y"]], s=90, color="#006400", marker="x", label="nearest start wp", zorder=6)
    ax.scatter([points[goal_idx]["x"]], [points[goal_idx]["y"]], s=100, color="#8b0000", marker="x", label="nearest goal wp", zorder=6)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(fig)


def nearby_breaks(points: Sequence[dict], idx: int, window: int = 8) -> List[int]:
    breaks = []
    start = max(0, idx - window)
    end = min(len(points) - 1, idx + window)
    for i in range(start, end):
        if int(points[i].get("connect_next", 1)) == 0:
            breaks.append(i)
    return breaks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test a start-goal route using only connect_next links from cleaned waypoint CSV")
    parser.add_argument("--input", required=True, help="Cleaned waypoint CSV")
    parser.add_argument("--output", required=True, help="planned_route.csv output")
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--debug-output", default="/home/ranim/autoCar_ws/waypoints/route_test_debug.png")
    parser.add_argument("--components-debug-output", default="/home/ranim/autoCar_ws/waypoints/graph_components_debug.png")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    points = read_waypoints(args.input)
    if not points:
        raise RuntimeError(f"No waypoints found: {args.input}")

    start_xy = (args.start_x, args.start_y)
    goal_xy = (args.goal_x, args.goal_y)
    start_idx = nearest_waypoint(points, start_xy)
    goal_idx = nearest_waypoint(points, goal_xy)
    graph = build_graph(points)
    nx_graph = build_networkx_graph(points)
    components = list(nx.connected_components(nx_graph))
    component_lookup = component_id_by_node(components)
    route = dijkstra(graph, start_idx, goal_idx)
    save_route(points, route, args.output)
    save_debug(points, route, start_xy, goal_xy, start_idx, goal_idx, args.debug_output)
    save_components_debug(points, components, start_xy, goal_xy, start_idx, goal_idx, args.components_debug_output)

    print("=" * 72)
    print("Cleaned map route test complete")
    print(f"Input             : {args.input}")
    print(f"Output            : {args.output}")
    print(f"Debug             : {args.debug_output}")
    print(f"Components debug  : {args.components_debug_output}")
    print(f"Total nodes       : {nx_graph.number_of_nodes()}")
    print(f"Total edges       : {nx_graph.number_of_edges()}")
    print(f"Connected components: {len(components)}")
    for component_id, component in enumerate(components):
        print(f"Component {component_id}: {len(component)} nodes")
    print(f"Waypoints         : {len(points)}")
    print(f"Graph edges       : {sum(len(v) for v in graph.values())}")
    print(f"Start waypoint    : idx={start_idx}, x={points[start_idx]['x']:.3f}, y={points[start_idx]['y']:.3f}")
    print(f"Goal waypoint     : idx={goal_idx}, x={points[goal_idx]['x']:.3f}, y={points[goal_idx]['y']:.3f}")
    print(f"Start node component: {component_lookup[start_idx]}")
    print(f"Goal node component : {component_lookup[goal_idx]}")
    print(f"Route waypoints   : {len(route)}")
    if not route:
        print("ROUTE NOT FOUND")
        if component_lookup[start_idx] != component_lookup[goal_idx]:
            print("PATH IMPOSSIBLE - GRAPH DISCONNECTED")
        print(f"Breaks near start idx : {nearby_breaks(points, start_idx)}")
        print(f"Breaks near goal idx  : {nearby_breaks(points, goal_idx)}")
        print("Reason: nearest start and goal are not connected through connect_next=1 directed links.")
    print("=" * 72)


if __name__ == "__main__":
    main()
