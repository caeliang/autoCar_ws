#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def visualize_waypoints(csv_path):
    if not os.path.exists(csv_path):
        print(f"Hata: {csv_path} bulunamadı.")
        return

    # CSV oku
    df = pd.read_csv(csv_path)

    plt.figure(figsize=(10, 10))
    
    # Rota çizgisi
    
    # Waypoint noktaları
    plt.scatter(df['x'], df['y'], c='blue', s=2, label='Waypoints')

    # Başlangıç ve Bitiş
    plt.scatter(df['x'].iloc[0], df['y'].iloc[0], c='red', s=50, label='Başlangıç', marker='o')
    plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], c='purple', s=50, label='Bitiş', marker='x')

    # Yön gösterimi (Seyrek oklar)
    


    plt.title(f"Waypoint Rotası Görselleştirme ({len(df)} Nokta)")
    plt.xlabel("X (metre)")
    plt.ylabel("Y (metre)")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    
    output_img = csv_path.replace('.csv', '.png')
    plt.savefig(output_img)
    print(f"✓ Görselleştirme kaydedildi: {output_img}")
    plt.show()

if __name__ == "__main__":
    # Eğer komut satırı argümanı varsa onu kullan, yoksa varsayılanı kullan
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "/home/ranim/autoCar_ws/waypoints/full_road_map.csv"
    
    visualize_waypoints(csv_file)
