#!/usr/bin/env python3
"""
Path Planning Visualization Script
Harita üzerinde rotayı gerçek zamanlı görselleştirir
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Circle
import sys
import os
import csv
from collections import deque
import heapq

def load_grid(grid_file):
    """Harita dosyasını yükle (1=yol, 0=engel)"""
    grid = []
    with open(grid_file, 'r') as f:
        for line in f:
            row = [int(c) for c in line.strip() if c in '01']
            if row:
                grid.append(row)
    return np.array(grid)

def load_waypoints(waypoint_file):
    """Waypoint CSV dosyasını yükle"""
    waypoints = []
    with open(waypoint_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Header atla
        for row in reader:
            if len(row) >= 3:
                x, y = float(row[1]), float(row[2])
                waypoints.append((x, y))
    return waypoints

def heuristic(p1, p2):
    """Euclidean distance heuristic"""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_valid(grid, pos):
    """Konum harita içinde geçerli ve yürünebilir mi?"""
    h, w = grid.shape
    if 0 <= pos[0] < w and 0 <= pos[1] < h:
        return grid[pos[1], pos[0]] == 1
    return False

def get_neighbors(pos):
    """8 komşu (çapraz hareket dahil)"""
    x, y = pos
    neighbors = [
        (x+1, y),   # Doğu
        (x-1, y),   # Batı
        (x, y+1),   # Güney
        (x, y-1),   # Kuzey
        (x+1, y+1), # Güney-Doğu
        (x-1, y+1), # Güney-Batı
        (x+1, y-1), # Kuzey-Doğu
        (x-1, y-1), # Kuzey-Batı
    ]
    return neighbors

def a_star(grid, start, goal):
    """A* pathfinding algoritması"""
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
            if not is_valid(grid, neighbor) or neighbor in closed_set:
                continue
            
            # Çapraz hareket maliyeti daha yüksek
            dx = abs(neighbor[0] - current[0])
            dy = abs(neighbor[1] - current[1])
            move_cost = 1.414 if (dx == 1 and dy == 1) else 1.0
            
            tentative_g = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return []  # Path bulunamadı

def plot_path(grid, start, goal, path=None, waypoints=None):
    """Path'i matplotlib ile görselleştir"""
    fig, ax = plt.subplots(figsize=(16, 12))
    
    # 1. Grid'i göster
    grid_display = np.where(grid == 1, 1.0, 0.0)
    ax.imshow(grid_display, cmap='Greys', origin='upper', alpha=0.7, extent=[0, grid.shape[1], grid.shape[0], 0])
    
    # 2. Waypoint'leri göster (hafif mavi)
    if waypoints:
        waypoints_array = np.array(waypoints)
        valid_wp = waypoints_array[
            (waypoints_array[:, 0] >= 0) & (waypoints_array[:, 0] < grid.shape[1]) &
            (waypoints_array[:, 1] >= 0) & (waypoints_array[:, 1] < grid.shape[0])
        ]
        if len(valid_wp) > 0:
            ax.scatter(valid_wp[:, 0], valid_wp[:, 1], c='lightblue', s=5, alpha=0.3, label=f'Waypoints ({len(waypoints)} total)')
    
    # 3. Path'i çiz
    if path and len(path) > 1:
        path_array = np.array(path)
        # Yolu düz çizgi ile göster
        ax.plot(path_array[:, 0], path_array[:, 1], 'g-', linewidth=2.5, alpha=0.8, label=f'Path ({len(path)} steps)', zorder=10)
        
        # Path üzerindeki noktaları göster
        ax.scatter(path_array[:, 0], path_array[:, 1], c='lime', s=30, alpha=0.6, zorder=9, edgecolors='darkgreen', linewidth=0.5)
        
        # Yönü gösteren oklar ekle (her N adımda bir)
        arrow_interval = max(1, len(path) // 10)
        for i in range(0, len(path) - 1, arrow_interval):
            p1 = path[i]
            p2 = path[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            
            # Orta noktadan başla, kısaltılmış ok
            mid_x = p1[0] + dx * 0.4
            mid_y = p1[1] + dy * 0.4
            arrow_dx = dx * 0.3
            arrow_dy = dy * 0.3
            
            arrow = FancyArrowPatch(
                (mid_x, mid_y),
                (mid_x + arrow_dx, mid_y + arrow_dy),
                arrowstyle='->', mutation_scale=20, 
                color='darkgreen', alpha=0.7, linewidth=1.5, zorder=11
            )
            ax.add_patch(arrow)
    
    # 4. Başlangıç ve hedef noktaları
    if start:
        circle_start = Circle((start[0], start[1]), 1.5, color='green', alpha=0.8, zorder=12, label='Start')
        ax.add_patch(circle_start)
        ax.plot(start[0], start[1], 'g*', markersize=25, zorder=13)
    
    if goal:
        circle_goal = Circle((goal[0], goal[1]), 1.5, color='red', alpha=0.8, zorder=12, label='Goal')
        ax.add_patch(circle_goal)
        ax.plot(goal[0], goal[1], 'r*', markersize=25, zorder=13)
    
    # 5. Grid ayarları
    ax.set_xlabel('X (grid units)', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y (grid units)', fontsize=13, fontweight='bold')
    
    if start and goal:
        title = f'Path Planning: Start({start[0]},{start[1]}) → Goal({goal[0]},{goal[1]}) | Distance: {len(path)-1 if path else 0} steps'
    else:
        title = 'Path Planning Visualization'
    
    ax.set_title(title, fontsize=15, fontweight='bold', pad=20)
    ax.legend(loc='upper left', fontsize=11, framealpha=0.9)
    ax.grid(True, alpha=0.2, linestyle='--')
    ax.set_xlim(0, grid.shape[1])
    ax.set_ylim(grid.shape[0], 0)
    
    plt.tight_layout()
    return fig, ax

def main():
    if len(sys.argv) < 7:
        print("Kullanım: python3 visualize_path.py <grid.txt> <waypoints.csv> <start_x> <start_y> <goal_x> <goal_y> [--no-waypoints]")
        print("\nÖrnek:")
        print("  python3 visualize_path.py matrices/road_grid_4wide.txt waypoints/full_road_map.csv 3 3 50 17")
        print("  python3 visualize_path.py matrices/road_grid_4wide.txt waypoints/full_road_map.csv 3 3 50 17 --no-waypoints")
        sys.exit(1)
    
    grid_file = sys.argv[1]
    waypoint_file = sys.argv[2]
    start_x, start_y = int(sys.argv[3]), int(sys.argv[4])
    goal_x, goal_y = int(sys.argv[5]), int(sys.argv[6])
    show_waypoints = "--no-waypoints" not in sys.argv
    
    # Dosyaları kontrol et
    if not os.path.exists(grid_file):
        print(f"❌ Hata: Harita dosyası bulunamadı: {grid_file}")
        sys.exit(1)
    if not os.path.exists(waypoint_file):
        print(f"❌ Hata: Waypoint dosyası bulunamadı: {waypoint_file}")
        sys.exit(1)
    
    # Verileri yükle
    print("📍 Harita yükleniyor...")
    grid = load_grid(grid_file)
    print(f"   Grid boyutu: {grid.shape[1]}x{grid.shape[0]}")
    
    print("📍 Waypoint'ler yükleniyor...")
    waypoints = load_waypoints(waypoint_file) if show_waypoints else None
    if waypoints:
        print(f"   Toplam waypoint: {len(waypoints)}")
    
    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    
    # Başlangıç ve hedef noktalarının geçerliliğini kontrol et
    if not is_valid(grid, start):
        print(f"❌ Hata: Başlangıç noktası ({start_x}, {start_y}) geçersiz!")
        sys.exit(1)
    if not is_valid(grid, goal):
        print(f"❌ Hata: Hedef noktası ({goal_x}, {goal_y}) geçersiz!")
        sys.exit(1)
    
    print(f"🔍 A* algoritması ile path hesaplanıyor...")
    print(f"   Başlangıç: {start}")
    print(f"   Hedef: {goal}")
    
    path = a_star(grid, start, goal)
    
    if path:
        print(f"✅ Path bulundu! {len(path)} adım, {len(path)-1} hareket")
        print(f"   Tahmini mesafe: {len(path)-1:.1f} grid birimi")
    else:
        print(f"❌ Path bulunamadı!")
        path = None
    
    # Görselleştir
    print("📊 Görselleştiriliyor...")
    fig, ax = plot_path(grid, start, goal, path=path, waypoints=waypoints)
    
    print("🎨 Plot gösteriliyor...")
    plt.show()

if __name__ == "__main__":
    main()
