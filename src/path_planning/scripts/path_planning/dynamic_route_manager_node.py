#!/usr/bin/env python3
from __future__ import annotations

import csv
import heapq
import math
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler


DEFAULT_CSV = "/home/ranim/autoCar_ws/waypoints/lane_points_left_outer_vertical_fixed_like_right.csv"
TRUE_VALUES = {"1", "1.0", "true", "True", "TRUE", "yes", "YES"}


def connect_next_is_true(row: dict) -> bool:
    return str(row.get("connect_next", "")).strip() in TRUE_VALUES


def row_parent(row: dict) -> str:
    return row.get("parent_segment_id") or row.get("segment_id") or row.get("lane_id") or ""


def xy_dist(a: dict, b: dict) -> float:
    return math.hypot(b["_x"] - a["_x"], b["_y"] - a["_y"])


class WaypointGraph:
    def __init__(
        self,
        csv_path: str,
        connector_distance: float,
        connector_penalty: float,
        undirected_base: bool,
        connector_direction_group: str,
    ) -> None:
        self.csv_path = csv_path
        self.connector_distance = connector_distance
        self.connector_penalty = connector_penalty
        self.undirected_base = undirected_base
        self.connector_direction_group = connector_direction_group
        self.blocked_parents: set[str] = set()

        self.rows, self.fieldnames = self._read_waypoints(csv_path)
        self.raw_indeg, self.raw_outdeg, self.raw_edge_count = self._raw_directed_degrees()
        self.graph, self.base_edge_count = self._build_traversal_graph()
        self.topology_edge_count = self._add_topology_connectors()

    def _read_waypoints(self, path: str) -> Tuple[List[dict], List[str]]:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            fieldnames = reader.fieldnames or []

        missing = {"x", "y", "connect_next"}.difference(fieldnames)
        if missing:
            raise ValueError(f"CSV eksik kolonlar: {', '.join(sorted(missing))}")

        for i, row in enumerate(rows):
            row["_idx"] = i
            row["_x"] = float(row["x"])
            row["_y"] = float(row["y"])
            row["_yaw"] = float(row.get("yaw", 0.0) or 0.0)
        return rows, fieldnames

    def _raw_directed_degrees(self) -> Tuple[List[int], List[int], int]:
        indeg = [0] * len(self.rows)
        outdeg = [0] * len(self.rows)
        edge_count = 0
        for i in range(len(self.rows) - 1):
            if not connect_next_is_true(self.rows[i]):
                continue
            outdeg[i] += 1
            indeg[i + 1] += 1
            edge_count += 1
        return indeg, outdeg, edge_count

    def _build_traversal_graph(self) -> Tuple[List[List[Tuple[int, float, str]]], int]:
        graph: List[List[Tuple[int, float, str]]] = [[] for _ in self.rows]
        edge_set: set[Tuple[int, int, str]] = set()
        edge_count = 0
        for i in range(len(self.rows) - 1):
            if not connect_next_is_true(self.rows[i]):
                continue
            cost = xy_dist(self.rows[i], self.rows[i + 1])
            if self._add_edge(graph, edge_set, i, i + 1, cost, "csv"):
                edge_count += 1
            if self.undirected_base:
                if self._add_edge(graph, edge_set, i + 1, i, cost, "csv_reverse"):
                    edge_count += 1
        self._edge_set = edge_set
        return graph, edge_count

    @staticmethod
    def _add_edge(graph, edge_set, u: int, v: int, cost: float, kind: str) -> bool:
        if u == v:
            return False
        key = (u, v, kind)
        if key in edge_set:
            return False
        edge_set.add(key)
        graph[u].append((v, cost, kind))
        return True

    def _is_endpoint(self, idx: int) -> bool:
        return self.raw_indeg[idx] == 0 or self.raw_outdeg[idx] == 0

    def _same_group_ok(self, a: dict, b: dict) -> bool:
        if self.connector_direction_group == "any":
            return True
        return a.get("direction_group") == b.get("direction_group")

    def _build_buckets(self, cell_size: float):
        buckets = defaultdict(list)
        for i, row in enumerate(self.rows):
            buckets[(math.floor(row["_x"] / cell_size), math.floor(row["_y"] / cell_size))].append(i)
        return buckets

    @staticmethod
    def _nearby(buckets, row, cell_size: float) -> Iterable[int]:
        bx = math.floor(row["_x"] / cell_size)
        by = math.floor(row["_y"] / cell_size)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                yield from buckets.get((bx + dx, by + dy), [])

    def _add_topology_connectors(self) -> int:
        if self.connector_distance <= 0.0:
            return 0

        buckets = self._build_buckets(self.connector_distance)
        endpoint_indices = [i for i in range(len(self.rows)) if self._is_endpoint(i)]

        added = 0
        for i in endpoint_indices:
            a = self.rows[i]
            for j in self._nearby(buckets, a, self.connector_distance):
                if i == j:
                    continue
                b = self.rows[j]
                if not self._same_group_ok(a, b):
                    continue
                d = xy_dist(a, b)
                if d > self.connector_distance:
                    continue
                if abs(i - j) == 1 and (
                    connect_next_is_true(self.rows[min(i, j)])
                    or connect_next_is_true(self.rows[max(i, j)])
                ):
                    continue
                if a.get("lane_id") == b.get("lane_id") and row_parent(a) == row_parent(b):
                    continue
                cost = d + self.connector_penalty
                if self._add_edge(self.graph, self._edge_set, i, j, cost, "topology_connector"):
                    added += 1
                if self._add_edge(self.graph, self._edge_set, j, i, cost, "topology_connector"):
                    added += 1
        return added

    def set_blocked(self, parent_segment_id: str) -> None:
        if parent_segment_id:
            self.blocked_parents.add(parent_segment_id)

    def clear_blocked(self) -> None:
        self.blocked_parents.clear()

    def is_row_blocked(self, idx: int) -> bool:
        return row_parent(self.rows[idx]) in self.blocked_parents

    def edge_allowed(self, u: int, v: int) -> bool:
        return not self.is_row_blocked(u) and not self.is_row_blocked(v)

    def nearest_node(self, x: float, y: float, direction_group: str = "") -> Tuple[int, float]:
        best_idx = -1
        best_dist = float("inf")
        for i, row in enumerate(self.rows):
            if self.is_row_blocked(i):
                continue
            if direction_group and row.get("direction_group") != direction_group:
                continue
            d = math.hypot(row["_x"] - x, row["_y"] - y)
            if d < best_dist:
                best_idx = i
                best_dist = d
        if best_idx < 0:
            raise RuntimeError("Snap icin uygun, unblocked waypoint bulunamadi.")
        return best_idx, best_dist

    def dijkstra(self, start_idx: int, goal_idx: int) -> Tuple[Optional[List[int]], float]:
        if self.is_row_blocked(start_idx) or self.is_row_blocked(goal_idx):
            return None, float("inf")

        distances = [float("inf")] * len(self.graph)
        previous = [-1] * len(self.graph)
        distances[start_idx] = 0.0
        heap = [(0.0, start_idx)]

        while heap:
            cur_d, u = heapq.heappop(heap)
            if cur_d != distances[u]:
                continue
            if u == goal_idx:
                break
            for v, cost, _kind in self.graph[u]:
                if not self.edge_allowed(u, v):
                    continue
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

    def route_yaw_deg(self, route_indices: Sequence[int], pos: int) -> float:
        if pos < len(route_indices) - 1:
            a = self.rows[route_indices[pos]]
            b = self.rows[route_indices[pos + 1]]
            return math.degrees(math.atan2(b["_y"] - a["_y"], b["_x"] - a["_x"]))
        if pos > 0:
            return self.route_yaw_deg(route_indices, pos - 1)
        return self.rows[route_indices[pos]]["_yaw"]


class DynamicRouteManager(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_route_manager_node")

        self.declare_parameter("csv_path", DEFAULT_CSV)
        self.declare_parameter("goal_x", None)
        self.declare_parameter("goal_y", None)
        self.declare_parameter("connector_distance", 0.75)
        self.declare_parameter("connector_penalty", 0.03)
        self.declare_parameter("undirected_base", True)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("snap_direction_group", "")
        self.declare_parameter("connector_direction_group", "same")
        self.declare_parameter("max_snap_distance", 0.0)
        self.declare_parameter("preserve_yaw", False)
        self.declare_parameter("allow_empty_path_on_no_route", False)

        self.csv_path = self.get_parameter("csv_path").value
        self.goal_x = self.optional_float_parameter("goal_x")
        self.goal_y = self.optional_float_parameter("goal_y")
        self.snap_direction_group = self.get_parameter("snap_direction_group").value
        self.max_snap_distance = float(self.get_parameter("max_snap_distance").value)
        self.preserve_yaw = bool(self.get_parameter("preserve_yaw").value)
        self.allow_empty_path_on_no_route = bool(self.get_parameter("allow_empty_path_on_no_route").value)

        self.graph = WaypointGraph(
            csv_path=self.csv_path,
            connector_distance=float(self.get_parameter("connector_distance").value),
            connector_penalty=float(self.get_parameter("connector_penalty").value),
            undirected_base=bool(self.get_parameter("undirected_base").value),
            connector_direction_group=self.get_parameter("connector_direction_group").value,
        )

        self.pose: Optional[PoseStamped] = None
        self.current_path = Path()
        self.current_path.header.frame_id = "map"
        self.status_text = "NO_POSE"
        self.route_indices: List[int] = []
        self.last_total_cost = float("inf")

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, "/waypoints/path", 10)
        self.status_pub = self.create_publisher(String, "/waypoints/status", status_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, "/localization/pose", self.on_pose, 10
        )
        self.event_sub = self.create_subscription(
            String, "/traffic_sign/event", self.on_traffic_event, 10
        )
        self.clicked_point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        publish_period = float(self.get_parameter("publish_period").value)
        self.timer = self.create_timer(publish_period, self.publish_current_route)

        self.get_logger().info(
            f"DynamicRouteManager loaded {len(self.graph.rows)} nodes from {self.csv_path} | "
            f"raw_edges={self.graph.raw_edge_count} traversal_edges={self.graph.base_edge_count} "
            f"topology_edges={self.graph.topology_edge_count} "
            f"goal={self.goal_log_text()}"
        )
        self.publish_status("NO_POSE" if self.has_goal() else "NO_GOAL")

    def optional_float_parameter(self, name: str) -> Optional[float]:
        param = self.get_parameter(name)
        if param.type_ == Parameter.Type.NOT_SET or param.value is None:
            return None
        return float(param.value)

    def has_goal(self) -> bool:
        return self.goal_x is not None and self.goal_y is not None

    def goal_log_text(self) -> str:
        if not self.has_goal():
            return "unset"
        return f"({self.goal_x:.3f}, {self.goal_y:.3f})"

    def on_pose(self, msg: PoseStamped) -> None:
        had_pose = self.pose is not None
        self.pose = msg
        if not had_pose:
            self.plan_from_current_pose("ACTIVE")

    def clicked_point_callback(self, msg: PointStamped) -> None:
        self.goal_x = float(msg.point.x)
        self.goal_y = float(msg.point.y)
        self.get_logger().info(
            f"New RViz goal from /clicked_point: x={self.goal_x:.3f}, y={self.goal_y:.3f}"
        )
        self.replan_from_current_pose(reason="RVIZ_CLICKED_POINT")

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.get_logger().info(
            f"New RViz goal from /goal_pose: x={self.goal_x:.3f}, y={self.goal_y:.3f}"
        )
        self.replan_from_current_pose(reason="RVIZ_GOAL_POSE")

    def replan_from_current_pose(self, reason: str) -> bool:
        return self.plan_from_current_pose(f"REPLANNED:{reason}")

    def on_traffic_event(self, msg: String) -> None:
        data = msg.data.strip()
        if data.startswith("NO_ENTRY:"):
            parent_id = data.split(":", 1)[1].strip()
            if not parent_id:
                self.get_logger().warn("NO_ENTRY event geldi ama parent_segment_id bos.")
                return
            self.graph.set_blocked(parent_id)
            self.publish_status("BLOCKED_SEGMENT_ADDED")
            self.get_logger().warn(
                f"Blocked segment added: {parent_id} | blocked_count={len(self.graph.blocked_parents)}"
            )
            self.plan_from_current_pose("REPLANNED")
            return

        if data == "CLEAR_BLOCKS":
            self.graph.clear_blocked()
            self.get_logger().warn("All blocked segments cleared.")
            self.plan_from_current_pose("REPLANNED")
            return

        if data in {"TRAFFIC_LIGHT_RED", "RED_LIGHT"} or data.startswith("TRAFFIC_LIGHT_RED:"):
            self.publish_status("TRAFFIC_LIGHT_RED")
            self.get_logger().warn("Traffic light red event received. Route unchanged.")
            return

        if data in {"TRAFFIC_LIGHT_GREEN", "GREEN_LIGHT"}:
            self.publish_status("ACTIVE" if self.route_indices else "NO_ROUTE")
            self.get_logger().info("Traffic light green event received.")
            return

        self.get_logger().warn(f"Unknown /traffic_sign/event message: '{data}'")

    def plan_from_current_pose(self, success_status: str) -> bool:
        if self.pose is None:
            self.publish_status("NO_POSE")
            self.get_logger().warn("Cannot plan route: current /localization/pose is not available.")
            return False

        if not self.has_goal():
            self.publish_status("NO_GOAL")
            self.get_logger().warn("Cannot plan route: goal is not set. Use RViz /clicked_point or /goal_pose.")
            return False

        sx = self.pose.pose.position.x
        sy = self.pose.pose.position.y
        try:
            start_idx, start_snap = self.graph.nearest_node(sx, sy, self.snap_direction_group)
            goal_idx, goal_snap = self.graph.nearest_node(
                float(self.goal_x), float(self.goal_y), self.snap_direction_group
            )
        except Exception as exc:
            self.get_logger().error(f"Snap failed: {exc}")
            self.handle_no_route()
            return False

        if self.max_snap_distance > 0.0 and (
            start_snap > self.max_snap_distance or goal_snap > self.max_snap_distance
        ):
            self.get_logger().error(
                f"Snap distance over limit: start={start_snap:.3f} "
                f"goal={goal_snap:.3f} limit={self.max_snap_distance:.3f}"
            )
            self.handle_no_route()
            return False

        route, total_cost = self.graph.dijkstra(start_idx, goal_idx)
        if route is None:
            self.get_logger().error(
                f"NO_ROUTE start_idx={start_idx} goal_idx={goal_idx} "
                f"blocked={sorted(self.graph.blocked_parents)}"
            )
            self.handle_no_route()
            return False

        self.route_indices = route
        self.last_total_cost = total_cost
        self.current_path = self.make_path(route)
        self.publish_status(success_status)
        self.get_logger().info(
            f"Route planned: status={success_status} nodes={len(route)} "
            f"edges={max(0, len(route) - 1)} cost={total_cost:.3f} "
            f"start_idx={start_idx} snap={start_snap:.3f} "
            f"goal_idx={goal_idx} snap={goal_snap:.3f} "
            f"blocked_count={len(self.graph.blocked_parents)}"
        )
        self.publish_current_route()
        return True

    def handle_no_route(self) -> None:
        if self.allow_empty_path_on_no_route:
            self.current_path = Path()
            self.current_path.header.frame_id = "map"
            self.route_indices = []
        self.publish_status("NO_ROUTE")

    def make_path(self, route_indices: Sequence[int]) -> Path:
        msg = Path()
        msg.header.frame_id = "map"

        for pos, idx in enumerate(route_indices):
            row = self.graph.rows[idx]
            yaw_deg = row["_yaw"] if self.preserve_yaw else self.graph.route_yaw_deg(route_indices, pos)
            yaw = math.radians(yaw_deg)
            q = quaternion_from_euler(0.0, 0.0, yaw)

            pose = PoseStamped()
            lane_id = row.get("lane_id", "")
            direction_group = row.get("direction_group", "")
            parent_segment_id = row_parent(row)
            pose.header.frame_id = f"map|{lane_id}|{direction_group}|{parent_segment_id}"
            pose.pose.position.x = row["_x"]
            pose.pose.position.y = row["_y"]
            pose.pose.position.z = float(row.get("z", 0.5) or 0.5)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            msg.poses.append(pose)
        return msg

    def publish_current_route(self) -> None:
        now = self.get_clock().now().to_msg()
        self.current_path.header.stamp = now
        self.current_path.header.frame_id = "map"
        for pose in self.current_path.poses:
            pose.header.stamp = now
        if self.current_path.poses or self.allow_empty_path_on_no_route:
            self.path_pub.publish(self.current_path)
        self.publish_status(self.status_text)

    def publish_status(self, text: str) -> None:
        self.status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DynamicRouteManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
