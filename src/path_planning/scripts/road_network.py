#!/usr/bin/env python3
"""
road_network.py — Gazebo compact_city.world yol ağ modeli

Asphalt bloklarının merkezlerinden iki şeritli yol ağı oluşturur.
Her yol, sağ ve sol şerit olarak ayrılır (şerit genişliği = 2.0m).

Kullanım:
  from road_network import RoadNetwork
  net = RoadNetwork()
  lanes = net.get_all_lanes()
"""

import math
import heapq
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Set


# ═══════════════════════════════════════════════════════════════════════
#  Yol Blok Verileri — compact_city.world'den çıkarıldı
# ═══════════════════════════════════════════════════════════════════════

TILE_SIZE = 5.0       # Her asphalt blok 5x5 metre
LANE_OFFSET = 1.25    # Merkez çizgisinden şerit merkezine mesafe (5m / 4 = 1.25m)
ROAD_Z = 0.5          # Waypoint z yüksekliği (yol yüzeyi biraz üstü)

# Yatay yollar (E-W) → y sabittir
HORIZONTAL_ROADS = {
    'north':  {'y': 32.5, 'x_range': (-27.5, 27.5), 'step': 5.0},
    'mid_n':  {'y': 17.5, 'x_range': (-27.5, 27.5), 'step': 5.0},
    'mid_s':  {'y': -2.5, 'x_range': (-27.5, 27.5), 'step': 5.0},
    'south':  {'y': -22.5, 'x_range': (-27.5, 27.5), 'step': 5.0},
}

# Dikey yollar (N-S) → x sabittir
VERTICAL_ROADS = {
    'west':       {'x': -27.5, 'y_ranges': [(27.5, 22.5), (12.5, 2.5), (-7.5, -17.5)]},
    'inner_west': {'x': -12.5, 'y_ranges': [(27.5, 22.5), (12.5, 2.5), (-7.5, -17.5)]},
    'inner_east': {'x':  12.5, 'y_ranges': [(27.5, 22.5), (12.5, 2.5), (-7.5, -17.5)]},
    'east':       {'x':  27.5, 'y_ranges': [(27.5, 22.5), (12.5, 2.5), (-7.5, -17.5)]},
    'spur':       {'x':   2.5, 'y_ranges': [(-27.5, -32.5)]},
}

# Kavşak merkezleri (yatay ve dikey yolların kesişimi)
INTERSECTION_CENTERS = []
for hy in [32.5, 17.5, -2.5, -22.5]:
    for vx in [-27.5, -12.5, 12.5, 27.5]:
        INTERSECTION_CENTERS.append((vx, hy))

# Döner kavşak
ROUNDABOUT_CENTER = (-12.59, -2.52)
ROUNDABOUT_RADIUS = 7.0


@dataclass
class LanePoint:
    """Şerit üzerinde bir nokta."""
    x: float
    y: float
    z: float = ROAD_Z
    yaw: float = 0.0  # radyan — aracın gittiği yön

    def distance_to(self, other: 'LanePoint') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)


@dataclass
class LaneSegment:
    """Tek bir şerit segmenti — doğrusal, iki nokta arası."""
    name: str
    points: List[LanePoint] = field(default_factory=list)
    direction: str = ''  # 'east', 'west', 'north', 'south'


@dataclass
class GraphNode:
    """Yol ağı graf düğümü."""
    id: str
    x: float
    y: float
    neighbors: Dict[str, float] = field(default_factory=dict)  # neighbor_id → distance


class RoadNetwork:
    """
    compact_city.world yol ağı — iki şeritli model.
    
    Her yol iki şerit olarak modellenir:
    - Sağ şerit: yol merkezinden +offset (sağda)
    - Sol şerit: yol merkezinden -offset (solda)
    
    Araç sağ şeritte gider (Türkiye trafik kuralı).
    """

    def __init__(self, lane_offset: float = LANE_OFFSET,
                 waypoint_spacing: float = 1.0):
        """
        Args:
            lane_offset: Yol merkezinden şerit merkezine mesafe (m)
            waypoint_spacing: İki waypoint arası mesafe (m)
        """
        self.lane_offset = lane_offset
        self.wp_spacing = waypoint_spacing
        self.segments: List[LaneSegment] = []
        self.graph_nodes: Dict[str, GraphNode] = {}

        self._build_horizontal_lanes()
        self._build_vertical_lanes()
        self._build_intersection_connectors()
        self._build_roundabout_lanes()
        self._build_graph()

    # ═══════════════════════════════════════════════════════════════════
    #  Yatay Yol Şeritleri (E-W)
    # ═══════════════════════════════════════════════════════════════════
    def _build_horizontal_lanes(self):
        """Yatay yollar → iki şerit: doğuya giden (alt), batıya giden (üst). SAĞDAN AKAN TRAFİK"""
        for name, info in HORIZONTAL_ROADS.items():
            y_center = info['y']
            x_min, x_max = info['x_range']
            hw = TILE_SIZE / 2.0

            # Doğuya giden şerit (Right-hand traffic: South of center -> y - offset)
            east_lane = LaneSegment(name=f'{name}_east', direction='east')
            y_east = y_center - self.lane_offset
            x = x_min
            while x <= x_max + 0.01:
                dist_to_roundabout = math.sqrt((x - ROUNDABOUT_CENTER[0])**2 + (y_east - ROUNDABOUT_CENTER[1])**2)
                in_intersection = False
                for (cx, cy) in INTERSECTION_CENTERS:
                    if abs(x - cx) < hw and abs(y_center - cy) < hw:
                        in_intersection = True
                        break
                        
                if dist_to_roundabout > ROUNDABOUT_RADIUS - 1.0 and not in_intersection:
                    east_lane.points.append(LanePoint(x=x, y=y_east, yaw=0.0))
                x += self.wp_spacing
            
            if east_lane.points: self.segments.append(east_lane)

            # Batıya giden şerit (Right-hand traffic: North of center -> y + offset)
            west_lane = LaneSegment(name=f'{name}_west', direction='west')
            y_west = y_center + self.lane_offset
            x = x_max
            while x >= x_min - 0.01:
                dist_to_roundabout = math.sqrt((x - ROUNDABOUT_CENTER[0])**2 + (y_west - ROUNDABOUT_CENTER[1])**2)
                in_intersection = False
                for (cx, cy) in INTERSECTION_CENTERS:
                    if abs(x - cx) < hw and abs(y_center - cy) < hw:
                        in_intersection = True
                        break

                if dist_to_roundabout > ROUNDABOUT_RADIUS - 1.0 and not in_intersection:
                    west_lane.points.append(LanePoint(x=x, y=y_west, yaw=math.pi))
                x -= self.wp_spacing
                
            if west_lane.points: self.segments.append(west_lane)

    # ═══════════════════════════════════════════════════════════════════
    #  Dikey Yol Şeritleri (N-S)
    # ═══════════════════════════════════════════════════════════════════
    def _build_vertical_lanes(self):
        """Dikey yollar → iki şerit: kuzeye giden (sağ), güneye giden (sol). SAĞDAN AKAN TRAFİK"""
        for name, info in VERTICAL_ROADS.items():
            x_center = info['x']
            hw = TILE_SIZE / 2.0

            for i, (y_start, y_end) in enumerate(info['y_ranges']):
                seg_name = f'{name}_seg{i}'
                ys = max(y_start, y_end)
                ye = min(y_start, y_end)

                # Güneye giden (Right-hand traffic: West of center -> x - offset)
                south_lane = LaneSegment(name=f'{seg_name}_south', direction='south')
                x_south = x_center - self.lane_offset
                y = ys
                while y >= ye - 0.01:
                    dist_to_roundabout = math.sqrt((x_south - ROUNDABOUT_CENTER[0])**2 + (y - ROUNDABOUT_CENTER[1])**2)
                    in_intersection = False
                    for (cx, cy) in INTERSECTION_CENTERS:
                        if abs(x_center - cx) < hw and abs(y - cy) < hw:
                            in_intersection = True
                            break

                    if dist_to_roundabout > ROUNDABOUT_RADIUS - 1.0 and not in_intersection:
                        south_lane.points.append(LanePoint(x=x_south, y=y, yaw=-math.pi / 2))
                    y -= self.wp_spacing
                if south_lane.points: self.segments.append(south_lane)

                # Kuzeye giden (Right-hand traffic: East of center -> x + offset)
                north_lane = LaneSegment(name=f'{seg_name}_north', direction='north')
                x_north = x_center + self.lane_offset
                y = ye
                while y <= ys + 0.01:
                    dist_to_roundabout = math.sqrt((x_north - ROUNDABOUT_CENTER[0])**2 + (y - ROUNDABOUT_CENTER[1])**2)
                    in_intersection = False
                    for (cx, cy) in INTERSECTION_CENTERS:
                        if abs(x_center - cx) < hw and abs(y - cy) < hw:
                            in_intersection = True
                            break

                    if dist_to_roundabout > ROUNDABOUT_RADIUS - 1.0 and not in_intersection:
                        north_lane.points.append(LanePoint(x=x_north, y=y, yaw=math.pi / 2))
                    y += self.wp_spacing
                if north_lane.points: self.segments.append(north_lane)

    # ═══════════════════════════════════════════════════════════════════
    #  Kavşak Dönüş Bağlantıları
    # ═══════════════════════════════════════════════════════════════════
    def _build_intersection_connectors(self):
        """
        Kavşaklarda dönüş yayları oluştur.
        (Sağdan akan trafik kurallarına göre Sağ ve Sol dönüşler, Quadratic Bezier)
        """
        for (cx, cy) in INTERSECTION_CENTERS:
            dist_to_roundabout = math.sqrt((cx - ROUNDABOUT_CENTER[0])**2 + (cy - ROUNDABOUT_CENTER[1])**2)
            if dist_to_roundabout < ROUNDABOUT_RADIUS + 2.0:
                continue

            L = self.lane_offset
            HW = TILE_SIZE / 2.0
            
            def check_dir(dir_name):
                # Harita sınırları (yolların sonu)
                xmin, xmax = -27.5, 27.5
                ymin, ymax = -22.5, 32.5
                # Yol merkezine göre dışarı taşmıyorsak o yönde yol vardır:
                if dir_name == "west": return cx > xmin
                if dir_name == "east": return cx < xmax
                if dir_name == "north": return cy < ymax
                if dir_name == "south": return cy > ymin
                return True

            has_w = check_dir("west")
            has_e = check_dir("east")
            has_n = check_dir("north")
            has_s = check_dir("south")

            # turns listesi:
            # name, has_in, has_out, start_x, start_y, start_yaw, end_x, end_y, end_yaw, (c_dx, c_dy) -> tangents intersection!
            turns = []

            # 1. West Input (Heading East, arriving at cx-HW)
            # Arrives at: y = cy - L
            if has_w and has_s: # Right turn to South
                turns.append(('turn_right_west_to_south', cx - HW, cy - L, 0.0, cx - L, cy - HW, -math.pi/2, cx - L, cy - L))
            if has_w and has_n: # Left turn to North
                turns.append(('turn_left_west_to_north', cx - HW, cy - L, 0.0, cx + L, cy + HW, math.pi/2, cx + L, cy - L))
                
            # 2. East Input (Heading West, arriving at cx+HW)
            # Arrives at: y = cy + L
            if has_e and has_n: # Right turn to North
                turns.append(('turn_right_east_to_north', cx + HW, cy + L, math.pi, cx + L, cy + HW, math.pi/2, cx + L, cy + L))
            if has_e and has_s: # Left turn to South
                turns.append(('turn_left_east_to_south', cx + HW, cy + L, math.pi, cx - L, cy - HW, -math.pi/2, cx - L, cy + L))
                
            # 3. South Input (Heading North, arriving at cy-HW)
            # Arrives at: x = cx + L
            if has_s and has_e: # Right turn to East
                turns.append(('turn_right_south_to_east', cx + L, cy - HW, math.pi/2, cx + HW, cy - L, 0.0, cx + L, cy - L))
            if has_s and has_w: # Left turn to West
                turns.append(('turn_left_south_to_west', cx + L, cy - HW, math.pi/2, cx - HW, cy + L, math.pi, cx + L, cy + L))

            # 4. North Input (Heading South, arriving at cy+HW)
            # Arrives at: x = cx - L
            if has_n and has_w: # Right turn to West
                turns.append(('turn_right_north_to_west', cx - L, cy + HW, -math.pi/2, cx - HW, cy + L, math.pi, cx - L, cy + L))
            if has_n and has_e: # Left turn to East
                turns.append(('turn_left_north_to_east', cx - L, cy + HW, -math.pi/2, cx + HW, cy - L, 0.0, cx - L, cy - L))

            for s_name, sx, sy, syaw, ex, ey, eyaw, ctx, cty in turns:
                seg = LaneSegment(name=f'{s_name}_at_{cx:.0f}_{cy:.0f}', direction=s_name)
                # Bezier egrisi uzerinde noktalar
                import math as m
                dist = math.sqrt((ex - sx)**2 + (ey - sy)**2)
                # Arcs are long, left turns are longer than right turns
                n_points = max(5, int(dist * 1.5 / self.wp_spacing))
                for j in range(n_points + 1):
                    t = j / n_points
                    px = (1 - t)**2 * sx + 2 * (1 - t) * t * ctx + t**2 * ex
                    py = (1 - t)**2 * sy + 2 * (1 - t) * t * cty + t**2 * ey
                    
                    diff = (eyaw - syaw + math.pi) % (2 * math.pi) - math.pi
                    yaw = syaw + diff * t
                    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                    seg.points.append(LanePoint(x=px, y=py, yaw=yaw))
                self.segments.append(seg)

            # ── Düz geçişler ──
            straights = []
            if has_e and has_w:
                straights.append(('through_west', cx + HW, cy + L, cx - HW, cy + L, math.pi))
                straights.append(('through_east', cx - HW, cy - L, cx + HW, cy - L, 0.0))
            if has_n and has_s:
                straights.append(('through_south', cx - L, cy + HW, cx - L, cy - HW, -math.pi / 2))
                straights.append(('through_north', cx + L, cy - HW, cx + L, cy + HW, math.pi / 2))

            for s_name, sx, sy, ex, ey, yaw in straights:
                seg = LaneSegment(name=f'{s_name}_at_{cx:.0f}_{cy:.0f}', direction=s_name)
                dist = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2)
                n_pts = max(2, int(dist / self.wp_spacing))
                for j in range(n_pts + 1):
                    t = j / n_pts
                    seg.points.append(LanePoint(x=sx + (ex - sx) * t, y=sy + (ey - sy) * t, yaw=yaw))
                self.segments.append(seg)

    #  Döner Kavşak Şeritleri
    # ═══════════════════════════════════════════════════════════════════
    def _build_roundabout_lanes(self):
        """Döner kavşak — saat yönünün tersine dairesel şerit."""
        cx, cy = ROUNDABOUT_CENTER
        r_inner = ROUNDABOUT_RADIUS - self.lane_offset * 1.5
        r_outer = ROUNDABOUT_RADIUS - self.lane_offset * 0.5

        # Dış şerit (ana döner kavşak — saat yönünün tersine)
        seg = LaneSegment(name='roundabout_outer', direction='ccw')
        n_pts = max(24, int(2 * math.pi * r_outer / self.wp_spacing))
        for j in range(n_pts):
            angle = 2 * math.pi * j / n_pts  # 0'dan 2π'ye
            px = cx + r_outer * math.cos(angle)
            py = cy + r_outer * math.sin(angle)
            # Teğet yönü (saat yönünün tersi → angle + π/2)
            yaw = angle + math.pi / 2
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            seg.points.append(LanePoint(x=px, y=py, yaw=yaw))
        # Döngüyü kapat
        if seg.points:
            seg.points.append(LanePoint(
                x=seg.points[0].x, y=seg.points[0].y, yaw=seg.points[0].yaw))
        self.segments.append(seg)

    # ═══════════════════════════════════════════════════════════════════
    #  Graf Oluştur (A* / Dijkstra için)
    # ═══════════════════════════════════════════════════════════════════
    def _build_graph(self):
        """
        Yol ağını graph olarak oluştur.
        Her kavşak bir düğüm, yol segmentleri kenar.
        """
        # Kavşakları düğüm olarak ekle
        for (cx, cy) in INTERSECTION_CENTERS:
            node_id = f'int_{cx:.0f}_{cy:.0f}'
            self.graph_nodes[node_id] = GraphNode(id=node_id, x=cx, y=cy)

        # Döner kavşağı da ekle
        cx, cy = ROUNDABOUT_CENTER
        self.graph_nodes['roundabout'] = GraphNode(id='roundabout', x=cx, y=cy)

        # Kenarları ekle — aynı yoldaki ardışık kavşaklar arasında
        # Yatay yollar
        for road_info in HORIZONTAL_ROADS.values():
            y = road_info['y']
            x_nodes = sorted([cx for (cx, cy_n) in INTERSECTION_CENTERS
                             if abs(cy_n - y) < 0.1])
            for i in range(len(x_nodes) - 1):
                n1 = f'int_{x_nodes[i]:.0f}_{y:.0f}'
                n2 = f'int_{x_nodes[i + 1]:.0f}_{y:.0f}'
                dist = abs(x_nodes[i + 1] - x_nodes[i])
                if n1 in self.graph_nodes and n2 in self.graph_nodes:
                    self.graph_nodes[n1].neighbors[n2] = dist
                    self.graph_nodes[n2].neighbors[n1] = dist

        # Dikey yollar
        for road_info in VERTICAL_ROADS.values():
            x = road_info['x']
            y_nodes = sorted([cy for (cx_n, cy) in INTERSECTION_CENTERS
                             if abs(cx_n - x) < 0.1], reverse=True)
            for i in range(len(y_nodes) - 1):
                n1 = f'int_{x:.0f}_{y_nodes[i]:.0f}'
                n2 = f'int_{x:.0f}_{y_nodes[i + 1]:.0f}'
                dist = abs(y_nodes[i] - y_nodes[i + 1])
                if n1 in self.graph_nodes and n2 in self.graph_nodes:
                    self.graph_nodes[n1].neighbors[n2] = dist
                    self.graph_nodes[n2].neighbors[n1] = dist

    # ═══════════════════════════════════════════════════════════════════
    #  Rota Planlama (A*)
    # ═══════════════════════════════════════════════════════════════════
    def find_nearest_node(self, x: float, y: float) -> str:
        """Verilen koordinata en yakın graph düğümünü bul."""
        min_dist = float('inf')
        nearest = ''
        for node in self.graph_nodes.values():
            d = math.sqrt((node.x - x) ** 2 + (node.y - y) ** 2)
            if d < min_dist:
                min_dist = d
                nearest = node.id
        return nearest

    def plan_route(self, start_x: float, start_y: float,
                   goal_x: float, goal_y: float,
                   direction: str = 'right') -> List[LanePoint]:
        """
        A* ile rota hesapla ve iki şeritli waypoint listesi oluştur.

        Args:
            start_x, start_y: Başlangıç koordinatları
            goal_x, goal_y: Bitiş koordinatları
            direction: 'right' → sağ şerit (varsayılan)

        Returns:
            Sıralı waypoint listesi (LanePoint)
        """
        start_node = self.find_nearest_node(start_x, start_y)
        goal_node = self.find_nearest_node(goal_x, goal_y)

        if start_node == goal_node:
            return []

        # A*
        path = self._astar(start_node, goal_node)
        if not path:
            return []

        # Kavşak listesinden waypoint listesi oluştur
        waypoints = self._path_to_waypoints(path, direction)
        return waypoints

    def _astar(self, start: str, goal: str) -> List[str]:
        """A* graf arama."""
        if start not in self.graph_nodes or goal not in self.graph_nodes:
            return []

        goal_node = self.graph_nodes[goal]
        open_set: List[Tuple[float, str]] = [(0.0, start)]
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Yolu geri izle
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            node = self.graph_nodes[current]
            for neighbor, dist in node.neighbors.items():
                tentative_g = g_score.get(current, float('inf')) + dist
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # Heuristic: Euclidean distance
                    n_node = self.graph_nodes[neighbor]
                    h = math.sqrt((n_node.x - goal_node.x) ** 2 +
                                  (n_node.y - goal_node.y) ** 2)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, neighbor))

        return []

    def _path_to_waypoints(self, path: List[str],
                           direction: str = 'right') -> List[LanePoint]:
        """
        Kavşak listesini iki şeritli waypoint listesine çevir.

        path: ['int_-28_32', 'int_-28_18', 'int_-28_-2', ...]
        """
        waypoints: List[LanePoint] = []
        offset = self.lane_offset if direction == 'right' else -self.lane_offset

        for i in range(len(path) - 1):
            n1 = self.graph_nodes[path[i]]
            n2 = self.graph_nodes[path[i + 1]]

            dx = n2.x - n1.x
            dy = n2.y - n1.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.01:
                continue

            # Hareket yönü
            yaw = math.atan2(dy, dx)

            # Şerit offseti (yolun sağ tarafı)
            # Yönün sağına dik → yaw - π/2
            offset_x = offset * math.cos(yaw - math.pi / 2)
            offset_y = offset * math.sin(yaw - math.pi / 2)

            # Bu segment boyunca waypoint'ler oluştur
            n_points = max(2, int(dist / self.wp_spacing))
            for j in range(n_points):
                t = j / n_points
                px = n1.x + dx * t + offset_x
                py = n1.y + dy * t + offset_y
                waypoints.append(LanePoint(x=px, y=py, yaw=yaw))

        # Son nokta
        if path:
            last = self.graph_nodes[path[-1]]
            if waypoints:
                waypoints.append(LanePoint(
                    x=last.x, y=last.y,
                    yaw=waypoints[-1].yaw
                ))

        return waypoints

    # ═══════════════════════════════════════════════════════════════════
    #  Yardımcı Fonksiyonlar
    # ═══════════════════════════════════════════════════════════════════
    @staticmethod
    def _dir_to_yaw(direction: str) -> float:
        """Yön adından yaw açısı (radyan)."""
        dirs = {
            'east': 0.0,
            'west': math.pi,
            'north': math.pi / 2,
            'south': -math.pi / 2,
        }
        return dirs.get(direction, 0.0)

    def get_all_lanes(self) -> List[LaneSegment]:
        """Tüm şerit segmentlerini döndür."""
        return self.segments

    def get_intersection_nodes(self) -> List[Tuple[float, float]]:
        """Tüm kavşak merkezlerini döndür."""
        return INTERSECTION_CENTERS.copy()

    def get_road_info(self) -> dict:
        """Yol ağı özet bilgisi."""
        return {
            'tile_size': TILE_SIZE,
            'lane_offset': self.lane_offset,
            'num_h_roads': len(HORIZONTAL_ROADS),
            'num_v_roads': len(VERTICAL_ROADS),
            'num_intersections': len(INTERSECTION_CENTERS),
            'num_segments': len(self.segments),
            'num_graph_nodes': len(self.graph_nodes),
            'total_lane_points': sum(len(s.points) for s in self.segments),
        }

    def list_nodes(self) -> List[dict]:
        """Tüm düğümleri ve komşularını listele."""
        result = []
        for node in self.graph_nodes.values():
            result.append({
                'id': node.id,
                'x': node.x,
                'y': node.y,
                'neighbors': list(node.neighbors.keys()),
            })
        return result


# ═══════════════════════════════════════════════════════════════════════
#  Ana Program — Test
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    net = RoadNetwork(waypoint_spacing=1.0)
    info = net.get_road_info()
    print(f"\n{'═' * 50}")
    print(f"  ROAD NETWORK SUMMARY")
    print(f"{'═' * 50}")
    for k, v in info.items():
        print(f"  {k}: {v}")

    print(f"\n  GRAPH NODES:")
    for node in net.list_nodes():
        print(f"    {node['id']:>20s}  ({node['x']:6.1f}, {node['y']:6.1f})  "
              f"→ {', '.join(node['neighbors'])}")
