#!/usr/bin/env python3
"""
Bu script, OpenDRIVE (.xodr) formatındaki dinamik harita dosyasını ayrıştırarak, 
yaklaşık 1.0m aralıklarla x, y, z, yaw bilgilerini içeren bir CSV dosyasına dönüştürür. 
Bu sayede, haritadaki yolların geometrisi, otonom aracın takip edebileceği şekilde noktalara dönüştürülmüş olur. 
Script, herhangi bir hardcoded (elle girilmiş) koordinat içermemektedir ve doğrudan .
xodr dosyasından verileri dinamik olarak okur.
"""
import xml.etree.ElementTree as ET
import math
import csv
import os
import argparse

def parse_opendrive_and_export(xodr_file, output_csv, ds=1.0):
    """
    OpenDRIVE (.xodr) dinamik harita formatını ayrıştırır.
    Herhangi bir hardcoded (elle girilmiş) kordinat içermez.
    """
    if not os.path.exists(xodr_file):
        print(f"Hata: Belirtilen dinamik harita dosyasi bulunamadi: {xodr_file}")
        return

    tree = ET.parse(xodr_file)
    root = tree.getroot()
    waypoints = []
    
    # Haritadaki butun dinamik yol parcalarini (road) tara
    for road in root.findall('road'):
        plan_view = road.find('planView')
        if plan_view is None: 
            continue
            
        # Geometrileri (Duz yol: line, Kavis: arc) cikar
        for geom in plan_view.findall('geometry'):
            x0 = float(geom.attrib['x'])
            y0 = float(geom.attrib['y'])
            hdg0 = float(geom.attrib['hdg']) # Baslangic yonelim acisi (yaw)
            length = float(geom.attrib['length'])
            
            curr_s = 0.0 # Yol boyu adimlama
            
            # 1. Duz Cizgi (Line) hesaplamasi
            if geom.find('line') is not None:
                while curr_s <= length:
                    px = x0 + curr_s * math.cos(hdg0)
                    py = y0 + curr_s * math.sin(hdg0)
                    waypoints.append({
                        'x': round(px, 3), 'y': round(py, 3), 'z': 0.0, 'yaw': round(hdg0, 3)
                    })
                    curr_s += ds
                    
            # 2. Yay/Kavis (Arc) hesaplamasi
            elif geom.find('arc') is not None:
                arc = geom.find('arc')
                curv = float(arc.attrib['curvature'])
                while curr_s <= length:
                    if curv == 0.0: # curvature 0 ise aslinda duzluktur
                        px = x0 + curr_s * math.cos(hdg0)
                        py = y0 + curr_s * math.sin(hdg0)
                        yaw = hdg0
                    else:
                        yaw = hdg0 + curr_s * curv
                        px = x0 + (math.sin(yaw) - math.sin(hdg0)) / curv
                        py = y0 - (math.cos(yaw) - math.cos(hdg0)) / curv
                        
                    # Yön açısını radyan olarak -pi, pi arasina normalize et
                    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                    waypoints.append({
                        'x': round(px, 3), 'y': round(py, 3), 'z': 0.0, 'yaw': round(yaw, 3)
                    })
                    curr_s += ds
                    
    # Verileri otonom aracin senin belirledigin formatindaki CSV'ye bas
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'yaw'])
        writer.writeheader()
        writer.writerows(waypoints)
        
    print(f"[{xodr_file}] basariyla islendi!")
    print(f"Toplam {len(waypoints)} waypoint -> '{output_csv}' dosyasina dinamik olarak aktarildi.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="OpenDRIVE (.xodr) to Waypoints CSV parser")
    parser.add_argument('-i', '--input', type=str, default='maps/custom_map.xodr', help="OpenDrive harita dosyasi")
    parser.add_argument('-o', '--output', type=str, default='waypoints/dynamic_road_map.csv', help="Cikti alinacak CSV dosyasi")
    parser.add_argument('-res', '--resolution', type=float, default=1.0, help="Noktalar arasi mesafe (metre)")
    
    args = parser.parse_args()
    parse_opendrive_and_export(args.input, args.output, args.resolution)
