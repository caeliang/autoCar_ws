#!/usr/bin/env python3
import math
import csv
import os
import sys

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
        
    print(f"Başarıyla {len(all_waypoints)} waypoint oluşturuldu. Kaydedilen yer: {output_path}")

if __name__ == "__main__":
    output = "/home/ranim/autoCar_ws/waypoints/full_road_map.csv"
    export_all_road_network_to_csv(output)
