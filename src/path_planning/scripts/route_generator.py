#!/usr/bin/env python3
"""
route_generator.py — Haritadan otomatik rota oluşturma

Gazebo compact_city.world asphalt bloklarını analiz ederek
iki şeritli waypoint dosyası üretir.

ARTIK ARACI SÜRMENE GEREK YOK!
Sadece kavşak noktalarını seç, rota otomatik hesaplanır.

Kullanım:
  python3 route_generator.py                    # interaktif mod
  python3 route_generator.py --route west_south  # hazır rota
  python3 route_generator.py --list-routes       # rotaları listele
  python3 route_generator.py --custom -28 32 27 -22  # özel rota
"""

import os
import sys
import math
import argparse
from datetime import datetime
from typing import List, Tuple

# Aynı dizindeki road_network modülünü yükle
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from road_network import RoadNetwork, LanePoint, ROAD_Z


# ═══════════════════════════════════════════════════════════════════════
#  Hazır Rotalar — yaygın güzergahlar
# ═══════════════════════════════════════════════════════════════════════

PRESET_ROUTES = {
    'west_south': {
        'description': 'Batı kolon boyunca güneye (kuzeyden güneye)',
        'nodes': ['int_-28_32', 'int_-28_18', 'int_-28_-2', 'int_-28_-22'],
    },
    'west_north': {
        'description': 'Batı kolon boyunca kuzeye (güneyden kuzeye)',
        'nodes': ['int_-28_-22', 'int_-28_-2', 'int_-28_18', 'int_-28_32'],
    },
    'north_east': {
        'description': 'Kuzey yol boyunca doğuya (batıdan doğuya)',
        'nodes': ['int_-28_32', 'int_-12_32', 'int_12_32', 'int_28_32'],
    },
    'north_west': {
        'description': 'Kuzey yol boyunca batıya (doğudan batıya)',
        'nodes': ['int_28_32', 'int_12_32', 'int_-12_32', 'int_-28_32'],
    },
    'south_east': {
        'description': 'Güney yol boyunca doğuya',
        'nodes': ['int_-28_-22', 'int_-12_-22', 'int_12_-22', 'int_28_-22'],
    },
    'south_west': {
        'description': 'Güney yol boyunca batıya',
        'nodes': ['int_28_-22', 'int_12_-22', 'int_-12_-22', 'int_-28_-22'],
    },
    'east_south': {
        'description': 'Doğu kolon boyunca güneye',
        'nodes': ['int_28_32', 'int_28_18', 'int_28_-2', 'int_28_-22'],
    },
    'east_north': {
        'description': 'Doğu kolon boyunca kuzeye',
        'nodes': ['int_28_-22', 'int_28_-2', 'int_28_18', 'int_28_32'],
    },
    'inner_west_south': {
        'description': 'İç batı kolon güneye',
        'nodes': ['int_-12_32', 'int_-12_18', 'int_-12_-2', 'int_-12_-22'],
    },
    'inner_east_south': {
        'description': 'İç doğu kolon güneye',
        'nodes': ['int_12_32', 'int_12_18', 'int_12_-2', 'int_12_-22'],
    },
    'mid_north_east': {
        'description': 'Orta-kuzey yol doğuya',
        'nodes': ['int_-28_18', 'int_-12_18', 'int_12_18', 'int_28_18'],
    },
    'mid_south_east': {
        'description': 'Orta-güney yol doğuya',
        'nodes': ['int_-28_-2', 'int_-12_-2', 'int_12_-2', 'int_28_-2'],
    },
}


# ═══════════════════════════════════════════════════════════════════════
#  Waypoint Oluşturma
# ═══════════════════════════════════════════════════════════════════════

def generate_waypoints_from_nodes(
    network: RoadNetwork,
    node_ids: List[str],
    spacing: float = 1.0,
    lane_side: str = 'right'
) -> List[LanePoint]:
    """
    Kavşak düğüm listesinden iki şeritli waypoint listesi oluştur.
    
    Args:
        network: Yol ağı
        node_ids: Sıralı kavşak düğüm ID'leri
        spacing: Waypoint arası mesafe (m)
        lane_side: 'right' → sağ şerit
    
    Returns:
        Sıralı LanePoint listesi
    """
    offset = network.lane_offset
    all_waypoints: List[LanePoint] = []

    for i in range(len(node_ids) - 1):
        n1_id = node_ids[i]
        n2_id = node_ids[i + 1]

        if n1_id not in network.graph_nodes or n2_id not in network.graph_nodes:
            print(f"  ⚠ Düğüm bulunamadı: {n1_id} veya {n2_id}")
            continue

        n1 = network.graph_nodes[n1_id]
        n2 = network.graph_nodes[n2_id]

        dx = n2.x - n1.x
        dy = n2.y - n1.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.01:
            continue

        # Hareket yönü
        yaw = math.atan2(dy, dx)

        # Sağ şerit offseti: yönün sağına (yaw - π/2)
        if lane_side == 'right':
            ox = offset * math.cos(yaw - math.pi / 2)
            oy = offset * math.sin(yaw - math.pi / 2)
        else:
            ox = offset * math.cos(yaw + math.pi / 2)
            oy = offset * math.sin(yaw + math.pi / 2)

        # Kavşak alanı — kavşak merkezinden 2.5m dışında başla/bitir
        # (kavşak içinde dönüş waypoint'leri oluşturulacak)
        margin = 2.5

        # Segment boyunca waypoint'ler
        n_points = max(3, int(dist / spacing))

        # Önceki segmentin son yönünü kontrol et (dönüş tespiti)
        prev_yaw = None
        if i > 0 and all_waypoints:
            prev_yaw = all_waypoints[-1].yaw

        # Eğer yön değişiyorsa → kavşakta dönüş yayı ekle
        if prev_yaw is not None and abs(self_angle_diff(prev_yaw, yaw)) > 0.1:
            turn_points = _make_turn(n1.x, n1.y, prev_yaw, yaw, offset, spacing)
            all_waypoints.extend(turn_points)

        # Düz segment
        for j in range(n_points + 1):
            t = j / n_points
            # Başlangıçtan margin kadar sonra başla, bitişten margin kadar önce bitir
            px = n1.x + dx * t + ox
            py = n1.y + dy * t + oy
            wp = LanePoint(x=round(px, 4), y=round(py, 4), z=ROAD_Z, yaw=yaw)

            # Çok yakın noktaları atla
            if all_waypoints:
                if wp.distance_to(all_waypoints[-1]) < spacing * 0.3:
                    continue

            all_waypoints.append(wp)

    return all_waypoints


def self_angle_diff(a: float, b: float) -> float:
    """İki açı arasındaki en kısa fark (radyan)."""
    d = b - a
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _make_turn(cx: float, cy: float, from_yaw: float, to_yaw: float,
               offset: float, spacing: float) -> List[LanePoint]:
    """
    Kavşakta pürüzsüz dönüş yayı oluştur.
    
    Args:
        cx, cy: Kavşak merkezi
        from_yaw: Geliş yönü
        to_yaw: Gidiş yönü
        offset: Şerit offseti
        spacing: Waypoint arası mesafe
    """
    points = []
    angle_diff = self_angle_diff(from_yaw, to_yaw)

    if abs(angle_diff) < 0.1:
        return points  # Düz geçiş, dönüş yayı yok

    # Dönüş yarıçapı
    turn_radius = max(2.0, offset * 2.0)
    arc_length = abs(angle_diff) * turn_radius
    n_pts = max(4, int(arc_length / spacing))

    for j in range(1, n_pts):  # İlk ve son nokta hariç (segment noktalarıyla çakışmasın)
        t = j / n_pts
        # Yaw'ı yumuşak enterpolasyon
        current_yaw = from_yaw + angle_diff * t

        # Cubic easing (yumuşak dönüş)
        ease_t = t * t * (3.0 - 2.0 * t)  # smoothstep

        # Dönüş arkı boyunca pozisyon
        # Geliş yönünde ilerleme + dönüş
        from_dx = math.cos(from_yaw) * turn_radius * (1 - ease_t)
        from_dy = math.sin(from_yaw) * turn_radius * (1 - ease_t)
        to_dx = math.cos(to_yaw) * turn_radius * ease_t
        to_dy = math.sin(to_yaw) * turn_radius * ease_t

        # Offset (sağ şerit)
        ox = offset * math.cos(current_yaw - math.pi / 2)
        oy = offset * math.sin(current_yaw - math.pi / 2)

        px = cx + (from_dx + to_dx) * 0.3 + ox
        py = cy + (from_dy + to_dy) * 0.3 + oy

        yaw = from_yaw + angle_diff * ease_t
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        points.append(LanePoint(
            x=round(px, 4), y=round(py, 4), z=ROAD_Z, yaw=yaw
        ))

    return points


# ═══════════════════════════════════════════════════════════════════════
#  YAML Kaydetme
# ═══════════════════════════════════════════════════════════════════════

def save_waypoints_yaml(waypoints: List[LanePoint], filepath: str,
                         speed: float = 1.5) -> bool:
    """
    Waypoint listesini basit x, y, yaw formatında kaydet.
    (speed parametresi artık kullanılmıyor ama uyumluluk için tutuluyor)
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    with open(filepath, 'w') as f:
        f.write('# Waypoint file — generated by route_generator.py\n')
        f.write(f'# Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
        f.write(f'# Frame: map\n')
        f.write(f'# Total: {len(waypoints)} waypoints\n')
        f.write(f'# Format: x, y, yaw (radyan)\n\n')
        f.write('waypoints:\n')

        for wp in waypoints:
            f.write(f'  - x: {wp.x}\n')
            f.write(f'    y: {wp.y}\n')
            f.write(f'    yaw: {round(wp.yaw, 6)}\n\n')

    return True


def save_all_routes_csv(network_spacing: float = 1.0,
                         filepath: str = '') -> str:
    """
    Tüm rotaları TEK bir CSV dosyasına kaydet.
    Sütunlar: index, x, y, yaw
    """
    if not filepath:
        filepath = os.path.join(
            os.path.expanduser('~/autoCar_ws/waypoints'), 'waypoints.csv')

    os.makedirs(os.path.dirname(filepath), exist_ok=True)

    with open(filepath, 'w') as f:
        f.write('index,x,y,yaw\n')
        idx = 0
        for route_name, info in PRESET_ROUTES.items():
            net = RoadNetwork(waypoint_spacing=network_spacing)
            waypoints = generate_waypoints_from_nodes(
                net, info['nodes'], spacing=network_spacing)
            for wp in waypoints:
                f.write(f'{idx},{wp.x:.4f},{wp.y:.4f},{wp.yaw:.6f}\n')
                idx += 1
            print(f'  ✓ {route_name}: {len(waypoints)} waypoint')

    print(f'\n  ✅ Toplam {idx} waypoint, {len(PRESET_ROUTES)} rota → {filepath}')
    return filepath


# ═══════════════════════════════════════════════════════════════════════
#  İnteraktif Mod
# ═══════════════════════════════════════════════════════════════════════

def interactive_mode(network: RoadNetwork):
    """İnteraktif rota seçimi."""
    print("\n" + "═" * 60)
    print("  🗺️  OTOMATİK ROTA OLUŞTURUCU")
    print("═" * 60)
    print("\n  Artık aracı sürmenize gerek yok!")
    print("  Harita üzerinden rota seçin, waypoint'ler otomatik oluşturulur.\n")

    # Kavşakları göster
    print("  ── Kavşak Noktaları ──")
    nodes = sorted(network.graph_nodes.keys())
    for i, node_id in enumerate(nodes):
        n = network.graph_nodes[node_id]
        print(f"    {i:2d}. {node_id:>20s}  ({n.x:6.1f}, {n.y:6.1f})")

    print(f"\n  ── Hazır Rotalar ──")
    route_names = list(PRESET_ROUTES.keys())
    for i, name in enumerate(route_names):
        desc = PRESET_ROUTES[name]['description']
        n_nodes = len(PRESET_ROUTES[name]['nodes'])
        print(f"    {i:2d}. {name:25s} — {desc} ({n_nodes} kavşak)")

    print()
    print("  Komutlar:")
    print("    p <numara>     — Hazır rota seç")
    print("    c <idx> <idx>  — Kavşak noktalarını seç (başlangıç bitiş)")
    print("    m <idx>...     — Çoklu kavşak noktası (sıralı)")
    print("    s <spacing>    — Waypoint arası mesafe değiştir (varsayılan: 1.0m)")
    print("    v <speed>      — Hedef hız değiştir (varsayılan: 1.5 m/s)")
    print("    q              — Çık")
    print()

    wp_spacing = 1.0
    target_speed = 1.5
    wp_dir = os.path.expanduser('~/autoCar_ws/waypoints')

    while True:
        try:
            cmd = input("  > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        parts = cmd.split()
        action = parts[0].lower()

        if action == 'q':
            break

        elif action == 's' and len(parts) >= 2:
            try:
                wp_spacing = float(parts[1])
                print(f"  ✓ Waypoint arası mesafe: {wp_spacing} m")
            except ValueError:
                print("  ✗ Geçersiz değer")

        elif action == 'v' and len(parts) >= 2:
            try:
                target_speed = float(parts[1])
                print(f"  ✓ Hedef hız: {target_speed} m/s")
            except ValueError:
                print("  ✗ Geçersiz değer")

        elif action == 'p' and len(parts) >= 2:
            try:
                idx = int(parts[1])
                if 0 <= idx < len(route_names):
                    route_name = route_names[idx]
                    route_info = PRESET_ROUTES[route_name]
                    node_ids = route_info['nodes']

                    print(f"\n  📍 Rota: {route_name}")
                    print(f"     {route_info['description']}")
                    print(f"     Kavşaklar: {' → '.join(node_ids)}")

                    net = RoadNetwork(waypoint_spacing=wp_spacing)
                    waypoints = generate_waypoints_from_nodes(
                        net, node_ids, spacing=wp_spacing)

                    if waypoints:
                        filename = f"{route_name}.yaml"
                        filepath = os.path.join(wp_dir, filename)
                        save_waypoints_yaml(waypoints, filepath, speed=target_speed)
                        print(f"  ✓ {len(waypoints)} waypoint kaydedildi → {filepath}")
                    else:
                        print("  ✗ Waypoint oluşturulamadı")
                else:
                    print(f"  ✗ Geçersiz indeks (0-{len(route_names) - 1})")
            except ValueError:
                print("  ✗ Geçersiz numara")

        elif action == 'c' and len(parts) >= 3:
            try:
                idx1 = int(parts[1])
                idx2 = int(parts[2])
                if 0 <= idx1 < len(nodes) and 0 <= idx2 < len(nodes):
                    n1 = nodes[idx1]
                    n2 = nodes[idx2]

                    # A* ile rota bul
                    path = network._astar(n1, n2)
                    if path:
                        print(f"\n  📍 Rota: {n1} → {n2}")
                        print(f"     Yol: {' → '.join(path)}")

                        net = RoadNetwork(waypoint_spacing=wp_spacing)
                        waypoints = generate_waypoints_from_nodes(
                            net, path, spacing=wp_spacing)

                        if waypoints:
                            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                            filename = f"route_{ts}.yaml"
                            filepath = os.path.join(wp_dir, filename)
                            save_waypoints_yaml(waypoints, filepath,
                                              speed=target_speed)
                            print(f"  ✓ {len(waypoints)} waypoint → {filepath}")
                        else:
                            print("  ✗ Waypoint oluşturulamadı")
                    else:
                        print(f"  ✗ {n1} → {n2} arası yol bulunamadı")
                else:
                    print(f"  ✗ Geçersiz indeks (0-{len(nodes) - 1})")
            except ValueError:
                print("  ✗ Geçersiz numara")

        elif action == 'm' and len(parts) >= 3:
            try:
                indices = [int(p) for p in parts[1:]]
                selected_nodes = []
                valid = True
                for idx in indices:
                    if 0 <= idx < len(nodes):
                        selected_nodes.append(nodes[idx])
                    else:
                        print(f"  ✗ Geçersiz indeks: {idx}")
                        valid = False
                        break

                if valid and len(selected_nodes) >= 2:
                    print(f"\n  📍 Çoklu kavşak rotası:")
                    print(f"     {' → '.join(selected_nodes)}")

                    net = RoadNetwork(waypoint_spacing=wp_spacing)
                    waypoints = generate_waypoints_from_nodes(
                        net, selected_nodes, spacing=wp_spacing)

                    if waypoints:
                        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                        filename = f"route_custom_{ts}.yaml"
                        filepath = os.path.join(wp_dir, filename)
                        save_waypoints_yaml(waypoints, filepath,
                                          speed=target_speed)
                        print(f"  ✓ {len(waypoints)} waypoint → {filepath}")
                    else:
                        print("  ✗ Waypoint oluşturulamadı")
            except ValueError:
                print("  ✗ Geçersiz numara")

        else:
            print("  ✗ Bilinmeyen komut. p/c/m/s/v/q kullan.")


# ═══════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Otomatik rota oluşturucu — asphalt bloklarından 2 şeritli waypoint')
    parser.add_argument('--route', type=str, help='Hazır rota adı')
    parser.add_argument('--list-routes', action='store_true',
                        help='Hazır rotaları listele')
    parser.add_argument('--custom', nargs=4, type=float,
                        metavar=('START_X', 'START_Y', 'GOAL_X', 'GOAL_Y'),
                        help='Özel rota (başlangıç ve bitiş koordinatları)')
    parser.add_argument('--spacing', type=float, default=1.0,
                        help='Waypoint arası mesafe (m)')
    parser.add_argument('--speed', type=float, default=1.5,
                        help='Hedef hız (m/s)')
    parser.add_argument('--output', type=str, default='',
                        help='Çıktı dosyası yolu')
    parser.add_argument('--generate-all', action='store_true',
                        help='Tüm hazır rotaları oluştur')
    parser.add_argument('--generate-csv', action='store_true',
                        help='Tüm hazır rotaları tek bir CSV dosyasına kaydet')

    args = parser.parse_args()
    wp_dir = os.path.expanduser('~/autoCar_ws/waypoints')

    if args.list_routes:
        print("\n  ── Hazır Rotalar ──")
        for name, info in PRESET_ROUTES.items():
            n = len(info['nodes'])
            print(f"    {name:25s} — {info['description']} ({n} kavşak)")
        return

    network = RoadNetwork(waypoint_spacing=args.spacing)

    if args.generate_all:
        print("\n  Tüm hazır rotalar oluşturuluyor...")
        for name, info in PRESET_ROUTES.items():
            net = RoadNetwork(waypoint_spacing=args.spacing)
            waypoints = generate_waypoints_from_nodes(
                net, info['nodes'], spacing=args.spacing)
            if waypoints:
                filepath = os.path.join(wp_dir, f"{name}.yaml")
                save_waypoints_yaml(waypoints, filepath, speed=args.speed)
                print(f"  ✓ {name}: {len(waypoints)} waypoint → {filepath}")
            else:
                print(f"  ✗ {name}: waypoint oluşturulamadı")
        return

    if args.generate_csv:
        print("\n  Tüm rotalar tek CSV'ye yazılıyor...")
        out = args.output or os.path.join(wp_dir, 'waypoints.csv')
        save_all_routes_csv(network_spacing=args.spacing, filepath=out)
        return

    if args.route:
        if args.route not in PRESET_ROUTES:
            print(f"  ✗ Bilinmeyen rota: {args.route}")
            print(f"  Rotalar: {', '.join(PRESET_ROUTES.keys())}")
            return

        info = PRESET_ROUTES[args.route]
        net = RoadNetwork(waypoint_spacing=args.spacing)
        waypoints = generate_waypoints_from_nodes(
            net, info['nodes'], spacing=args.spacing)

        if waypoints:
            filepath = args.output or os.path.join(wp_dir, f"{args.route}.yaml")
            save_waypoints_yaml(waypoints, filepath, speed=args.speed)
            print(f"\n  ✓ {len(waypoints)} waypoint kaydedildi → {filepath}")
        return

    if args.custom:
        sx, sy, gx, gy = args.custom
        waypoints = network.plan_route(sx, sy, gx, gy)
        if waypoints:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            filepath = args.output or os.path.join(wp_dir, f"route_custom_{ts}.yaml")
            save_waypoints_yaml(waypoints, filepath, speed=args.speed)
            print(f"\n  ✓ {len(waypoints)} waypoint kaydedildi → {filepath}")
        else:
            print("  ✗ Rota bulunamadı")
        return

    # İnteraktif mod
    interactive_mode(network)


if __name__ == '__main__':
    main()
