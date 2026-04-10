#!/usr/bin/env python3
"""
--------------------------------------------------------------------------------
Brief: Rota Haritası Üreticisi (Road Map Generator)
Description: 
  'road_network.py' içinde tanımlanan asfalt yol ve kavşak geometrisini (`RoadNetwork`) 
  okuyarak haritadaki tüm sürülebilir yolları (~1m aralıklarla waypoint olarak) oluşturur.
  Bu waypoint'lere 'update_waypoints_yaw.py' yardımıyla atan2 yönelim değerlerini ekler
  ve çıktı olarak 'waypoints/full_road_map.csv' dosyasını yaratır.
Kullanıldığı Yer: `./scripts/generate_route.sh` pipeline'ı.
--------------------------------------------------------------------------------
"""
import math
import csv
import os
import sys
import subprocess

# Mevcut scripti içe aktarabilmek için yolu ekleyelim
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from road_network import RoadNetwork

def export_all_road_network_to_csv(output_path):
    """
    RoadNetwork sınıfındaki tüm yol segmentlerini (sol/sağ şeritler) 
    yaklaşık 1.0m aralıklarla CSV olarak dışa aktarır.
    """
    rn = RoadNetwork(waypoint_spacing=1.0) # 1.0m aralıklarla oluştur
    all_waypoints = []
    
    # RoadNetwork.segments içindeki tüm LaneSegment nesnelerini gezelim
    for segment in rn.segments:
        for point in segment.points:
            all_waypoints.append({
                'x': round(float(point.x), 2),
                'y': round(float(point.y), 2),
                'z': round(float(point.z), 2),
                'yaw': round(float(point.yaw), 2)
            })

    # CSV Yazma
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'yaw'])
        writer.writeheader()
        writer.writerows(all_waypoints)
        
    print(f"✓ {len(all_waypoints)} waypoint oluşturuldu")
    print(f"✓ Kaydedilen yer: {output_path}\n")
    
    return output_path

def calculate_waypoint_yaws(csv_path):
    """
    Waypoints CSV dosyasındaki yaw değerlerini atan2(Δy, Δx) ile hesapla
    """
    print("Yaw değerleri hesaplanıyor (atan2 formülü)...")
    
    # Workspace root'u bul
    # generate_full_map.py: /home/ranim/autoCar_ws/src/path_planning/scripts/generate_full_map.py
    # Workspace: /home/ranim/autoCar_ws
    script_path = os.path.abspath(__file__)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_path))))
    yaw_script = os.path.join(workspace_root, 'scripts', 'update_waypoints_yaw.py')
    
    if not os.path.exists(yaw_script):
        print(f"✗ Hata: Yaw hesaplama scripti bulunamadı: {yaw_script}")
        return False
    
    try:
        result = subprocess.run(
            ['python3', yaw_script, csv_path],
            check=False,
            capture_output=True,
            text=True
        )
        
        # Çıktısı göster
        if result.stdout:
            print(result.stdout)
        
        if result.returncode != 0:
            print(f"✗ Hata: Yaw hesaplaması başarısız (exit code: {result.returncode})")
            if result.stderr:
                print(f"Hata detayı:\n{result.stderr}")
            return False
        
        print("✓ Yaw değerleri otomatik olarak hesaplandı\n")
        return True
        
    except Exception as e:
        print(f"✗ Exception: {e}")
        return False

if __name__ == "__main__":
    output = "/home/ranim/autoCar_ws/waypoints/full_road_map.csv"
    
    print("=" * 60)
    print("  YOL HARITASI OLUŞTURUCU")
    print("  + Otomatik Yaw Hesaplaması")
    print("=" * 60)
    print()
    
    # 1. Waypoints oluştur
    print("[1/2] Waypoints oluşturuluyor...")
    csv_file = export_all_road_network_to_csv(output)
    
    # 2. Yaw değerlerini hesapla
    print("[2/2] Yaw değerleri hesaplanıyor...")
    calculate_waypoint_yaws(csv_file)
    
    print("=" * 60)
    print("✓ TAMAMLANDI!")
    print("=" * 60)
