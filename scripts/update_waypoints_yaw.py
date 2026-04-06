#!/usr/bin/env python3
"""
Waypoint CSV dosyasını oku ve her nokta için hareket yönüne göre yaw değerini güncelle
Direction -> Yaw mapping (derece cinsinden):
  - Doğu (1, 0):        0°
  - KuzeyDoğu (1, -1):  45°
  - Kuzey (0, -1):      90°
  - KuzeyBatı (-1, -1): 135°
  - Batı (-1, 0):       180°
  - GüneyBatı (-1, 1):  225°
  - Güney (0, 1):       270°
  - GüneyDoğu (1, 1):   315°
"""

import csv
import math
import sys
import os

def calculate_yaw_from_points(p1, p2):
    """
    İki nokta arasındaki yaw açısını atan2(Δy, Δx) ile hesapla
    Sonuç derece cinsinden (-180, 180] aralığında
    
    Koordinat sistemi:
    - X: sağa pozitif (Doğu) → 0°
    - Y: yukarı pozitif (Kuzey) → 90°
    """
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    
    dx = x2 - x1
    dy = y2 - y1
    
    # atan2 radyan cinsinden, Y up (matematiksel sistem)
    yaw_rad = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw_rad)
    
    # (-180, 180] aralığına normalize et
    if yaw_deg <= -180:
        yaw_deg += 360
    elif yaw_deg > 180:
        yaw_deg -= 360
    
    return yaw_deg

def update_waypoints_yaw(input_csv, output_csv=None):
    """
    Waypoints CSV dosyasını oku ve yaw değerlerini güncelle
    Yaw = atan2(Δy, Δx) ardışık waypoints arasında
    """
    if output_csv is None:
        output_csv = input_csv
    
    # CSV dosyasını oku
    waypoints = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append(row)
    
    if len(waypoints) < 2:
        print(f"Hata: {len(waypoints)} waypoint var. En az 2 gerekli.")
        return False
    
    print(f"✓ {input_csv}'dan {len(waypoints)} waypoint okundu")
    
    # Yaw değerlerini hesapla
    updated_waypoints = []
    
    for i, wp in enumerate(waypoints):
        x = float(wp['x'])
        y = float(wp['y'])
        z = float(wp.get('z', 0.5))
        
        if i == 0:
            # İlk nokta: sonraki noktaya doğru olan yaw
            if i + 1 < len(waypoints):
                next_x = float(waypoints[i+1]['x'])
                next_y = float(waypoints[i+1]['y'])
                yaw = calculate_yaw_from_points((x, y), (next_x, next_y))
            else:
                yaw = 0.0
        else:
            # Diğer noktalar: önceki noktadan bu noktaya olan yaw
            prev_x = float(waypoints[i-1]['x'])
            prev_y = float(waypoints[i-1]['y'])
            yaw = calculate_yaw_from_points((prev_x, prev_y), (x, y))
        
        updated_waypoints.append({
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        })
    
    # Güncellenmiş waypoints'i CSV'ye yaz
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'yaw'])
        writer.writeheader()
        for wp in updated_waypoints:
            writer.writerow({
                'x': f"{wp['x']:.1f}",
                'y': f"{wp['y']:.1f}",
                'z': f"{wp['z']:.1f}",
                'yaw': f"{wp['yaw']:.2f}"
            })
    
    print(f"✓ {output_csv}'ye yaw değerleri ile güncellendi")
    
    # İlk 15 örneği göster
    print(f"\n--- İLK 15 WAYPOINT (atan2 ile hesaplanan yaw) ---")
    print(f"{'#':<3} {'X':<8} {'Y':<8} {'Z':<6} {'YAW(°)':<8} {'Yön':<15}")
    print("-" * 55)
    
    for i, wp in enumerate(updated_waypoints[:15]):
        yaw = wp['yaw']
        
        # Yön adlandırması
        if -22.5 <= yaw < 22.5:
            direction = "Doğu →"
        elif 22.5 <= yaw < 67.5:
            direction = "KuzeyDoğu ↗"
        elif 67.5 <= yaw < 112.5:
            direction = "Kuzey ↑"
        elif 112.5 <= yaw < 157.5:
            direction = "KuzeyBatı ↖"
        elif 157.5 <= yaw or yaw < -157.5:
            direction = "Batı ←"
        elif -157.5 <= yaw < -112.5:
            direction = "GüneyBatı ↙"
        elif -112.5 <= yaw < -67.5:
            direction = "Güney ↓"
        elif -67.5 <= yaw < -22.5:
            direction = "GüneyDoğu ↘"
        else:
            direction = "?"
        
        print(f"{i+1:<3} {wp['x']:<8.1f} {wp['y']:<8.1f} {wp['z']:<6.1f} {yaw:<8.2f} {direction:<15}")
    
    print(f"\n... ({len(updated_waypoints) - 15} daha)")
    
    # İstatistikler
    yaw_values = [wp['yaw'] for wp in updated_waypoints]
    print(f"\nYaw İstatistikleri:")
    print(f"  Min: {min(yaw_values):.2f}°")
    print(f"  Max: {max(yaw_values):.2f}°")
    print(f"  Ortalama: {sum(yaw_values)/len(yaw_values):.2f}°")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else input_file
    else:
        input_file = "/home/ranim/autoCar_ws/waypoints/full_road_map.csv"
        output_file = input_file
    
    print(f"========================================")
    print(f"  WAYPOINT YAW HESAPLAMA")
    print(f"========================================\n")
    
    if not os.path.exists(input_file):
        print(f"Hata: {input_file} bulunamadı!")
        sys.exit(1)
    
    update_waypoints_yaw(input_file, output_file)
    print(f"\n========================================\n")
