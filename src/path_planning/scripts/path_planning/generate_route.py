#!/usr/bin/env python3
"""
--------------------------------------------------------------------------------
Brief: A* (A-Star) Algoritması ile Rota Planlayıcı
Description:
  `generate_full_map.py` çıktısı olan `full_road_map.csv` dosyasından 
  sürülebilir noktaları/yönleri (`waypoints`)  ve çevresel engelleri (`road_grid_4wide.txt`) referans alarak 
  başlangıç koordinatlarından hedefe giden, araç boyutlarına çarpmayan optümum 
  en kisa yolu A* (A-Star) aracılığıyla belirler (yaw farkına ceza keserek).
  En son yörüngeyi (path smoothing) düzleştirir ve "planned_route.csv" dosyasına yazar.
Kullanıldığı Yer: Hedef tabanlı bir rotaya gidilmesi istendiğinde bağımsız path runner olarak
                  örneğin PID takip edicilerinden hemen önce.
--------------------------------------------------------------------------------
"""

import numpy as np
import csv
import math
import heapq
import sys

def load_grid(grid_file):
    """Harita dosyasını yükle"""
    grid = []
    with open(grid_file, 'r') as f:
        for line in f:
            row = [int(c) for c in line.strip() if c in '01']
            if row:
                grid.append(row)
    return np.array(grid)

def world_to_grid(world_x, world_y, grid_shape=(68, 59)):
    """Dünya koordinatlarını grid koordinatlarına çevir"""
    grid_height, grid_width = grid_shape
    min_x, max_x = -28.8 * 1.5, 28.8 * 1.5
    min_y, max_y = -32.5 * 1.5, 33.8 * 1.5
    x_span = max_x - min_x
    y_span = max_y - min_y
    grid_x = round((world_x - min_x) / x_span * (grid_width - 1))
    grid_y = round((max_y - world_y) / y_span * (grid_height - 1))
    return int(grid_x), int(grid_y)

def load_waypoints(waypoint_file):
    """Waypoints yükle (x, y, z, yaw)"""
    waypoints = {}
    with open(waypoint_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            yaw = float(row['yaw'])
            # Harita koordinat sistemine dönüştür
            grid_x, grid_y = world_to_grid(x, y)
            
            if (grid_x, grid_y) not in waypoints:
                waypoints[(grid_x, grid_y)] = {'yaws': [], 'points': []}
            
            waypoints[(grid_x, grid_y)]['yaws'].append(yaw)
            waypoints[(grid_x, grid_y)]['points'].append((x, y, z, yaw))
            
    return waypoints

def heuristic(pos, goal):
    """Öklid uzaklığı heuristic"""
    return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

def get_neighbors(pos):
    """8 komşu (çapraz dahil)"""
    x, y = pos
    neighbors = [
        (x+1, y), (x-1, y), (x, y+1), (x, y-1),
        (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)
    ]
    return neighbors

def calculate_movement_yaw(current, neighbor):
    """Grid üzerinde hareketin yönünü hesapla (derece cinsinden)"""
    dx = neighbor[0] - current[0]
    dy = neighbor[1] - current[1]
    
    # Y ekseni grid iterasyonunda aşağı doğru artar, 
    # bu yüzden matematiksel açı için Y'yi tersine çeviriyoruz
    yaw_rad = math.atan2(-dy, dx)
    yaw_deg = math.degrees(yaw_rad)
    
    if yaw_deg <= -180:
        yaw_deg += 360
    elif yaw_deg > 180:
        yaw_deg -= 360
        
    return yaw_deg

def angular_difference(angle1, angle2):
    """İki açı arasındaki en kısa farkı bul (derece)"""
    diff = abs((angle1 - angle2 + 180) % 360 - 180)
    return diff

def is_valid_with_yaw(grid, waypoints, current, neighbor):
    """Konum geçerli mi ve waypoint yaw yönüne uygun mu?"""
    h, w = grid.shape
    
    if not (0 <= neighbor[0] < w and 0 <= neighbor[1] < h):
        return False
    if grid[neighbor[1], neighbor[0]] != 1:
        return False
        
    if neighbor in waypoints:
        wp_yaws = waypoints[neighbor]['yaws']
        mov_yaw = calculate_movement_yaw(current, neighbor)
        
        # Hareket yönü, bu hücredeki WAYPOINT yönlerinden EN AZ BİRİNE uygun olmalı
        # Tolerans: ±60 derece (şerit merkezine uyum)
        is_valid_direction = False
        for wp_yaw in wp_yaws:
            diff = angular_difference(mov_yaw, wp_yaw)
            if diff <= 60:  # 60 derece tolerans
                is_valid_direction = True
                break
                
        # Hiçbir yaw değere uymuyorsa, yanlış şeritten gidiyordur
        if not is_valid_direction:
            return False
            
    return True

def a_star(grid, waypoints, start, goal):
    """A* algoritması (yaw yönlendirmeli)"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    closed_set = set()
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
        
        if current == goal:
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path
        
        closed_set.add(current)
        
        for neighbor in get_neighbors(current):
            # Yaw tabanlı geçerlilik kontrolü
            if neighbor in closed_set or not is_valid_with_yaw(grid, waypoints, current, neighbor):
                continue
            
            dx = abs(neighbor[0] - current[0])
            dy = abs(neighbor[1] - current[1])
            move_cost = 1.414 if (dx == 1 and dy == 1) else 1.0
            
            # Yaw cezası (opsiyonel) -Waypoint'in yönüne tam uymayan hareketlere ceza ver
            if neighbor in waypoints:
                wp_yaws = waypoints[neighbor]['yaws']
                mov_yaw = calculate_movement_yaw(current, neighbor)
                
                # En uygun yaw değerini bul
                min_diff = min([angular_difference(mov_yaw, y) for y in wp_yaws])
                
                # Yön farkına göre ek cost (fark büyüdükçe agresif ceza büyüsün)
                yaw_penalty = (min_diff / 90.0) * 2.0
                move_cost += yaw_penalty
            
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return []

def calculate_yaw_from_points(p1, p2):
    """İki nokta arasında yaw hesapla"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    if dx == 0 and dy == 0:
        return 0.0
    
    yaw_rad = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw_rad)
    
    if yaw_deg <= -180:
        yaw_deg += 360
    elif yaw_deg > 180:
        yaw_deg -= 360
    
    return yaw_deg

def grid_to_world(grid_x, grid_y, grid_shape=(68, 59)):
    """
    Grid koordinatlarını dünya koordinatlarına çevir
    Grid boyutu: 68 satır x 59 sütun
    1.5x Ölçeklenmiş CSV aralığı: X[-43.2, 43.2] Y[-48.75, 50.7]
    """
    # Grid boyutu
    grid_height, grid_width = grid_shape  # 68, 59
    
    # CSV koordinat aralığı (1.5x)
    min_x, max_x = -28.8 * 1.5, 28.8 * 1.5
    min_y, max_y = -32.5 * 1.5, 33.8 * 1.5
    
    # Span
    x_span = max_x - min_x  
    y_span = max_y - min_y  
    
    # X artarak büyür (soldan sağa)
    x = min_x + (grid_x / (grid_width - 1)) * x_span
    # Y azalarak küçülür (yukarıdan aşağı) - Y eksenini TERS ÇEVİR
    y = max_y - (grid_y / (grid_height - 1)) * y_span
    
    return x, y

def smooth_path(path_world, weight_data=0.5, weight_smooth=0.2, tolerance=0.00001):
    """
    Yolu yumuşatmak için Gradient Descent tabanlı smoother.
    path_world: Dünya koordinatlarında [(x, y), (x, y), ...] listesi.
    """
    new_path = [[p[0], p[1]] for p in path_world]
    change = tolerance
    
    # Gradient descent iterasyonu
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path_world) - 1):
            for j in range(2):
                aux = new_path[i][j]
                
                # new = old + alpha*(data - new) + beta*(prev + next - 2*new)
                new_path[i][j] += weight_data * (path_world[i][j] - new_path[i][j]) + \
                                  weight_smooth * (new_path[i-1][j] + new_path[i+1][j] - 2.0 * new_path[i][j])
                
                change += abs(aux - new_path[i][j])
                
    return [(p[0], p[1]) for p in new_path]

def plan_route(grid_file, waypoint_file, start_x, start_y, goal_x, goal_y, output_csv):
    """Rota planla ve yaw değerleri ekle"""
    
    print("=" * 60)
    print("  ROTA PLANLAMA (A* + YAW)")
    print("=" * 60)
    print()
    
    # Grid yükle
    print("Grid yükleniyor...")
    grid = load_grid(grid_file)
    print(f"✓ Grid: {grid.shape}")
    
    # Waypoints yükle
    print("Waypoints yükleniyor...")
    waypoints = load_waypoints(waypoint_file)
    print(f"✓ Waypoints: {len(waypoints)}")
    
    # Başlangıç ve hedefi grid koordinatlarına çevir
    start = (int(round(start_x)), int(round(start_y)))
    goal = (int(round(goal_x)), int(round(goal_y)))
    
    print()
    print(f"Başlangıç: {start} → ({start_x:.1f}, {start_y:.1f})")
    print(f"Hedef: {goal} → ({goal_x:.1f}, {goal_y:.1f})")
    print()
    
    # A* çalıştır
    print("A* ile rota hesaplanıyor (yaw-aware)...")
    path = a_star(grid, waypoints, start, goal)
    
    if not path:
        print("✗ Rota bulunamadı!")
        return False
    
    print(f"✓ Rota bulundu: {len(path)} nokta")
    print()
    
    # Dünya koordinatlarına çevirme
    print("Path dünya koordinatlarına dönüştürülüyor...")
    world_path = [grid_to_world(gx, gy) for gx, gy in path]
    
    # Yolu yumuşat
    print("✓ Path smoothing algoritması uygulanıyor (Gradient Descent)...")
    smoothed_world_path = smooth_path(world_path, weight_data=0.5, weight_smooth=0.3, tolerance=0.00001)
    
    # Waypoint ve yaw bilgilerini ekle
    print("Yaw değerleri hesaplanıyor...")
    route_points = []
    
    for i, (gx, gy) in enumerate(path):
        wx, wy = smoothed_world_path[i] # Artık yumuşatılmış dünya koordinatları
        
        # Waypoints'ten yaw bul, yoksa ardışık noktalardan hesapla
        if i < len(path) - 1:
            next_wx, next_wy = smoothed_world_path[i + 1]
            
            # Gerçek dünya yönelimi ile yaw değeri hesapla
            dx = next_wx - wx
            dy = next_wy - wy
            mov_yaw = math.degrees(math.atan2(dy, dx))
            if mov_yaw <= -180: mov_yaw += 360
            elif mov_yaw > 180: mov_yaw -= 360
            
            if (gx, gy) in waypoints:
                # Hücredeki yaw'lardan hareket yönüne en uygun olanını seç
                wp_yaws = waypoints[(gx, gy)]['yaws']
                yaw = min(wp_yaws, key=lambda y: angular_difference(mov_yaw, y))
            else:
                yaw = mov_yaw
        else:
            # Son nokta için önceki noktanın yaw'ını kullan veya hücrede varsa onu al
            if (gx, gy) in waypoints:
                yaw = waypoints[(gx, gy)]['yaws'][0]
            else:
                yaw = route_points[-1]['yaw'] if route_points else 0.0
        
        route_points.append({
            'step': i + 1,
            'lane_id': 'route_001',
            'segment_id': 'route_001_seg_001',
            'direction': 'forward',
            'x': wx,
            'y': wy,
            'z': 0.5,
            'yaw': yaw,
            'connect_next': 1 if i < len(path) - 1 else 0,
            'grid_x': gx,
            'grid_y': gy
        })
    
    # CSV'ye yaz
    print(f"Rota dosyasına yazılıyor: {output_csv}")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                'step', 'lane_id', 'segment_id', 'direction',
                'x', 'y', 'z', 'yaw', 'connect_next', 'grid_x', 'grid_y'
            ]
        )
        writer.writeheader()
        for point in route_points:
            writer.writerow({
                'step': point['step'],
                'lane_id': point['lane_id'],
                'segment_id': point['segment_id'],
                'direction': point['direction'],
                'x': f"{point['x']:.2f}",
                'y': f"{point['y']:.2f}",
                'z': f"{point['z']:.2f}",
                'yaw': f"{point['yaw']:.2f}",
                'connect_next': point['connect_next'],
                'grid_x': point['grid_x'],
                'grid_y': point['grid_y']
            })
    
    print(f"✓ {len(route_points)} nokta kaydedildi")
    print()
    print("=" * 60)
    print("İlk 10 nokta:")
    print("=" * 60)
    print(f"{'Step':<6} {'X':<8} {'Y':<8} {'Yaw':<8} {'Konum':<15}")
    print("-" * 60)
    
    for point in route_points[:10]:
        print(f"{point['step']:<6} {point['x']:<8.2f} {point['y']:<8.2f} {point['yaw']:<8.2f} ({point['grid_x']}, {point['grid_y']})")
    
    print(f"\n... ({len(route_points) - 10} daha)\n")
    
    return True

if __name__ == "__main__":
    grid_file = "/home/ranim/autoCar_ws/matrices/road_grid_4wide.txt"
    waypoint_file = "/home/ranim/autoCar_ws/waypoints/two_way_lane_waypoints.csv"
    output_csv = "/home/ranim/autoCar_ws/waypoints/planned_route.csv"
    
    # Parametreleri dogrudan alalim
    if len(sys.argv) < 5:
        print("Eksik arguman")
        sys.exit(1)
        
    start_x = float(sys.argv[1])
    start_y = float(sys.argv[2])
    goal_x = float(sys.argv[3])
    goal_y = float(sys.argv[4])
    
    plan_route(grid_file, waypoint_file, start_x, start_y, goal_x, goal_y, output_csv)
