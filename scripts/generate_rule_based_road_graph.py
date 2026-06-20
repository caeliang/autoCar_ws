#!/usr/bin/env python3
import argparse
import csv
import heapq
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
import networkx as nx

matplotlib.use("Agg")
import matplotlib.pyplot as plt


Point = Tuple[float, float]


VERTICAL_ROADS_X = [-40.0, -20.0, 20.0, 40.0]
HORIZONTAL_ROADS_Y = [48.0, 26.0, -3.0, -34.0]
BOTTOM_ENTRY_X = [2.0, 6.0]
BOTTOM_ENTRY_Y_RANGE = [-52.0, -34.0]
ROUNDABOUT_CENTER = (-18.0, -4.0)
ROUNDABOUT_RADIUS = 8.0


@dataclass
class EdgeGeom:
    edge_id: str
    source: str
    target: str
    segment_type: str
    points: List[Point]
    length: float


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point]) -> float:
    return sum(dist(points[i], points[i + 1]) for i in range(len(points) - 1))


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
        yaws.append(last)
    yaws.append(last)
    return yaws


def resample_polyline(points: Sequence[Point], spacing: float) -> List[Point]:
    if len(points) < 2:
        return list(points)
    output = [points[0]]
    carry = spacing
    current = points[0]
    for target in points[1:]:
        seg_len = dist(current, target)
        while seg_len >= carry and seg_len > 1e-9:
            t = carry / seg_len
            new_point = (current[0] + (target[0] - current[0]) * t, current[1] + (target[1] - current[1]) * t)
            output.append(new_point)
            current = new_point
            seg_len = dist(current, target)
            carry = spacing
        carry -= seg_len
        current = target
    if dist(output[-1], points[-1]) > spacing * 0.35:
        output.append(points[-1])
    return output


def arc_points(center: Point, radius: float, start_deg: float, end_deg: float, spacing: float) -> List[Point]:
    delta = end_deg - start_deg
    arc_len = abs(math.radians(delta)) * radius
    steps = max(4, int(math.ceil(arc_len / max(spacing, 0.05))) + 1)
    pts = []
    for i in range(steps):
        t = i / (steps - 1)
        angle = math.radians(start_deg + delta * t)
        pts.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)))
    return resample_polyline(pts, spacing)


def reverse_points(points: Sequence[Point]) -> List[Point]:
    return list(reversed(points))


def axis_aligned_connector(a: Point, b: Point, spacing: float, prefer: str = "hv") -> List[Point]:
    if abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6:
        return resample_polyline([a, b], spacing)
    mid = (b[0], a[1]) if prefer == "hv" else (a[0], b[1])
    return resample_polyline([a, mid, b], spacing)


def node_id(prefix: str, x: float, y: float) -> str:
    return f"{prefix}_{x:.2f}_{y:.2f}".replace("-", "m").replace(".", "p")


def add_node(graph: nx.DiGraph, node: str, point: Point, node_type: str = "junction") -> None:
    if node not in graph:
        graph.add_node(node, x=point[0], y=point[1], node_type=node_type)


def add_directed_edge(graph: nx.DiGraph, source: str, target: str, points: Sequence[Point], segment_type: str, edge_id: str) -> None:
    pts = list(points)
    graph.add_edge(
        source,
        target,
        edge_id=edge_id,
        segment_type=segment_type,
        points=pts,
        length=polyline_length(pts),
    )


def add_bidirectional_edge(graph: nx.DiGraph, source: str, target: str, points: Sequence[Point], segment_type: str, edge_id: str) -> None:
    add_directed_edge(graph, source, target, points, segment_type, f"{edge_id}_fwd")
    add_directed_edge(graph, target, source, reverse_points(points), segment_type, f"{edge_id}_rev")


def nearest_node(graph: nx.DiGraph, point: Point, exclude: Optional[set] = None) -> str:
    exclude = exclude or set()
    return min(
        [n for n in graph.nodes if n not in exclude],
        key=lambda n: math.hypot(float(graph.nodes[n]["x"]) - point[0], float(graph.nodes[n]["y"]) - point[1]),
    )


def build_rule_based_graph(spacing: float) -> nx.DiGraph:
    graph = nx.DiGraph()
    xs = sorted(VERTICAL_ROADS_X)
    ys = sorted(HORIZONTAL_ROADS_Y, reverse=True)

    grid_nodes: Dict[Tuple[float, float], str] = {}
    for x in xs:
        for y in ys:
            nid = node_id("J", x, y)
            grid_nodes[(x, y)] = nid
            add_node(graph, nid, (x, y), "junction")

    # Straight horizontal roads between adjacent vertical roads.
    for y in ys:
        for a, b in zip(xs[:-1], xs[1:]):
            src = grid_nodes[(a, y)]
            dst = grid_nodes[(b, y)]
            points = resample_polyline([(a, y), (b, y)], spacing)
            add_bidirectional_edge(graph, src, dst, points, "straight", f"H_y{y:.1f}_x{a:.1f}_{b:.1f}")

    # Straight vertical roads between adjacent horizontal roads.
    ys_asc = sorted(HORIZONTAL_ROADS_Y)
    for x in xs:
        for a, b in zip(ys_asc[:-1], ys_asc[1:]):
            src = grid_nodes[(x, a)]
            dst = grid_nodes[(x, b)]
            points = resample_polyline([(x, a), (x, b)], spacing)
            add_bidirectional_edge(graph, src, dst, points, "straight", f"V_x{x:.1f}_y{a:.1f}_{b:.1f}")

    # Bottom entry roads.
    bottom_y, top_y = BOTTOM_ENTRY_Y_RANGE
    for x in BOTTOM_ENTRY_X:
        entry_bottom = node_id("ENTRY", x, bottom_y)
        entry_top = node_id("ENTRY", x, top_y)
        add_node(graph, entry_bottom, (x, bottom_y), "entry")
        add_node(graph, entry_top, (x, top_y), "entry_connector")
        points = resample_polyline([(x, bottom_y), (x, top_y)], spacing)
        add_bidirectional_edge(graph, entry_bottom, entry_top, points, "entry_road", f"ENTRY_x{x:.1f}")

        # Connect to nearest bottom horizontal grid node with a short straight connector.
        nearest_bottom = nearest_node(graph, (x, top_y), exclude={entry_bottom, entry_top})
        connector = resample_polyline([(x, top_y), (float(graph.nodes[nearest_bottom]["x"]), float(graph.nodes[nearest_bottom]["y"]))], spacing)
        add_bidirectional_edge(graph, entry_top, nearest_bottom, connector, "entry_connector", f"ENTRY_CONN_x{x:.1f}")

    # Outer corner quarter-circle turn shortcuts. These are additional valid turn
    # maneuvers; straight grid edges remain the main road graph.
    r = 4.0
    corner_defs = [
        ("outer_top_left", (-40.0 + r, 48.0 - r), 180.0, 90.0, (-40.0, 48.0 - r), (-40.0 + r, 48.0)),
        ("outer_top_right", (40.0 - r, 48.0 - r), 90.0, 0.0, (40.0 - r, 48.0), (40.0, 48.0 - r)),
        ("outer_bottom_right", (40.0 - r, -34.0 + r), 0.0, -90.0, (40.0, -34.0 + r), (40.0 - r, -34.0)),
        ("outer_bottom_left", (-40.0 + r, -34.0 + r), -90.0, -180.0, (-40.0 + r, -34.0), (-40.0, -34.0 + r)),
    ]
    for name, center, a0, a1, p0, p1 in corner_defs:
        n0 = node_id("ARC", *p0)
        n1 = node_id("ARC", *p1)
        add_node(graph, n0, p0, "turn_tangent")
        add_node(graph, n1, p1, "turn_tangent")
        arc = arc_points(center, r, a0, a1, spacing)
        if dist(arc[0], p0) > dist(arc[-1], p0):
            arc = reverse_points(arc)
        add_bidirectional_edge(graph, n0, n1, arc, "outer_corner_arc", name)
        # Tie tangent nodes to nearest real junction so graph remains connected.
        j0 = nearest_node(graph, p0, exclude={n0, n1})
        j1 = nearest_node(graph, p1, exclude={n0, n1})
        add_bidirectional_edge(graph, n0, j0, resample_polyline([p0, (graph.nodes[j0]["x"], graph.nodes[j0]["y"])], spacing), "arc_connector", f"{name}_conn0")
        add_bidirectional_edge(graph, n1, j1, resample_polyline([p1, (graph.nodes[j1]["x"], graph.nodes[j1]["y"])], spacing), "arc_connector", f"{name}_conn1")

    # Roundabout loop.
    cx, cy = ROUNDABOUT_CENTER
    rr = ROUNDABOUT_RADIUS
    ring_nodes = []
    ring_count = 32
    for i in range(ring_count):
        angle = 2.0 * math.pi * i / ring_count
        p = (cx + rr * math.cos(angle), cy + rr * math.sin(angle))
        nid = f"R_{i:02d}"
        ring_nodes.append(nid)
        add_node(graph, nid, p, "roundabout")
    for i in range(ring_count):
        a = ring_nodes[i]
        b = ring_nodes[(i + 1) % ring_count]
        pa = (float(graph.nodes[a]["x"]), float(graph.nodes[a]["y"]))
        pb = (float(graph.nodes[b]["x"]), float(graph.nodes[b]["y"]))
        # Use short circular arc between neighboring ring nodes. Bidirectional by requirement.
        start_deg = math.degrees(math.atan2(pa[1] - cy, pa[0] - cx))
        end_deg = math.degrees(math.atan2(pb[1] - cy, pb[0] - cx))
        if end_deg < start_deg:
            end_deg += 360.0
        pts = arc_points((cx, cy), rr, start_deg, end_deg, spacing)
        add_bidirectional_edge(graph, a, b, pts, "roundabout_arc", f"RING_{i:02d}")

    # Connect roundabout to nearest grid roads at cardinal points.
    roundabout_connectors = [
        ("R_N", (cx, cy + rr), (-20.0, -3.0)),
        ("R_S", (cx, cy - rr), (-20.0, -34.0)),
        ("R_E", (cx + rr, cy), (20.0, -3.0)),
        ("R_W", (cx - rr, cy), (-40.0, -3.0)),
    ]
    for label, ring_point, grid_point in roundabout_connectors:
        ring = nearest_node(graph, ring_point)
        target = nearest_node(graph, grid_point, exclude=set(ring_nodes))
        ring_xy = (float(graph.nodes[ring]["x"]), float(graph.nodes[ring]["y"]))
        target_xy = (float(graph.nodes[target]["x"]), float(graph.nodes[target]["y"]))
        pts = axis_aligned_connector(ring_xy, target_xy, spacing, prefer="hv")
        add_bidirectional_edge(graph, ring, target, pts, "roundabout_connector", label)

    return graph


def astar_path(graph: nx.DiGraph, start_xy: Point, goal_xy: Point) -> List[str]:
    start = nearest_node(graph, start_xy)
    goal = nearest_node(graph, goal_xy)

    def heuristic(a: str, b: str) -> float:
        ax, ay = float(graph.nodes[a]["x"]), float(graph.nodes[a]["y"])
        bx, by = float(graph.nodes[b]["x"]), float(graph.nodes[b]["y"])
        return math.hypot(ax - bx, ay - by)

    return nx.astar_path(graph, start, goal, heuristic=heuristic, weight="length")


def path_to_points(graph: nx.DiGraph, path: Sequence[str]) -> List[Point]:
    out: List[Point] = []
    for a, b in zip(path[:-1], path[1:]):
        pts = list(graph.edges[a, b]["points"])
        if out and pts and dist(out[-1], pts[0]) < 1e-6:
            out.extend(pts[1:])
        else:
            out.extend(pts)
    return out


def save_generated_graph_csv(path: str, graph: nx.DiGraph) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = ["step", "x", "y", "z", "yaw", "lane_id", "segment_id", "segment_type", "direction", "source", "target", "connect_next"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        step = 1
        for source, target, data in graph.edges(data=True):
            points = list(data["points"])
            yaws = compute_yaws(points)
            edge_id = str(data["edge_id"])
            direction = "bidirectional"
            for i, (p, yaw) in enumerate(zip(points, yaws)):
                writer.writerow(
                    {
                        "step": step,
                        "x": f"{p[0]:.3f}",
                        "y": f"{p[1]:.3f}",
                        "z": "0.500",
                        "yaw": f"{yaw:.3f}",
                        "lane_id": edge_id,
                        "segment_id": edge_id,
                        "segment_type": data["segment_type"],
                        "direction": direction,
                        "source": source,
                        "target": target,
                        "connect_next": 1 if i < len(points) - 1 else 0,
                    }
                )
                step += 1


def save_route_csv(path: str, points: Sequence[Point], spacing: float) -> None:
    sampled = resample_polyline(points, spacing)
    yaws = compute_yaws(sampled)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["step", "x", "y", "z", "yaw"])
        writer.writeheader()
        for step, (p, yaw) in enumerate(zip(sampled, yaws), start=1):
            writer.writerow({"step": step, "x": f"{p[0]:.3f}", "y": f"{p[1]:.3f}", "z": "0.500", "yaw": f"{yaw:.3f}"})


def draw_graph(path: str, graph: nx.DiGraph) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 11))
    ax.set_title("Rule-based lane graph")
    for u, v, data in graph.edges(data=True):
        color = {
            "roundabout_arc": "#ff7f0e",
            "roundabout_connector": "#2ca02c",
            "outer_corner_arc": "#9467bd",
            "entry_road": "#8c564b",
            "entry_connector": "#8c564b",
        }.get(data["segment_type"], "#1f77b4")
        # Draw every directed edge lightly; overlapping reverse edges are harmless here.
        pts = data["points"]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=1.8, alpha=0.75)
    xs = [float(graph.nodes[n]["x"]) for n in graph.nodes]
    ys = [float(graph.nodes[n]["y"]) for n in graph.nodes]
    ax.scatter(xs, ys, s=16, color="#111111", alpha=0.7, label="graph nodes")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="best")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def draw_route(path: str, graph: nx.DiGraph, route: Sequence[Point], start_xy: Point, goal_xy: Point) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 11))
    ax.set_title("Rule-based A* route")
    for _, _, data in graph.edges(data=True):
        pts = data["points"]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#bdbdbd", linewidth=1.2, alpha=0.45)
    if route:
        ax.plot([p[0] for p in route], [p[1] for p in route], color="#d62728", linewidth=2.8, label="A* route")
    ax.scatter([start_xy[0]], [start_xy[1]], s=130, color="#2ca02c", marker="o", label="start")
    ax.scatter([goal_xy[0]], [goal_xy[1]], s=160, color="#1f77b4", marker="*", label="goal")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="best")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate clean rule-based city-grid road graph")
    parser.add_argument("--spacing", type=float, default=0.5)
    parser.add_argument("--start-x", type=float, default=-40.0)
    parser.add_argument("--start-y", type=float, default=48.0)
    parser.add_argument("--goal-x", type=float, default=40.0)
    parser.add_argument("--goal-y", type=float, default=-34.0)
    parser.add_argument("--output-graph", default="/home/ranim/autoCar_ws/waypoints/generated_lane_graph.csv")
    parser.add_argument("--output-route", default="/home/ranim/autoCar_ws/waypoints/planned_route.csv")
    parser.add_argument("--graph-debug", default="/home/ranim/autoCar_ws/waypoints/graph_debug.png")
    parser.add_argument("--route-debug", default="/home/ranim/autoCar_ws/waypoints/route_debug.png")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    graph = build_rule_based_graph(args.spacing)
    components = list(nx.weakly_connected_components(graph))
    if len(components) != 1:
        raise RuntimeError(f"Rule-based graph is disconnected: components={len(components)}")

    start_xy = (args.start_x, args.start_y)
    goal_xy = (args.goal_x, args.goal_y)
    path_nodes = astar_path(graph, start_xy, goal_xy)
    route_points = path_to_points(graph, path_nodes)
    if not route_points:
        raise RuntimeError("A* route is empty")

    save_generated_graph_csv(args.output_graph, graph)
    save_route_csv(args.output_route, route_points, args.spacing)
    draw_graph(args.graph_debug, graph)
    draw_route(args.route_debug, graph, route_points, start_xy, goal_xy)

    print("=" * 72)
    print("Rule-based road graph generated")
    print(f"Nodes              : {graph.number_of_nodes()}")
    print(f"Directed edges     : {graph.number_of_edges()}")
    print(f"Weak components    : {len(components)}")
    print(f"A* route nodes     : {len(path_nodes)}")
    print(f"Route points       : {len(resample_polyline(route_points, args.spacing))}")
    print(f"Graph CSV          : {args.output_graph}")
    print(f"Planned route CSV  : {args.output_route}")
    print(f"Graph debug        : {args.graph_debug}")
    print(f"Route debug        : {args.route_debug}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
