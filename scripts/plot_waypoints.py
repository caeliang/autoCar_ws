#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle
import os
import sys
import argparse
import csv

def get_grid_bounds(waypoint_file):
    with open(waypoint_file, 'r') as f:
        reader = csv.DictReader(f)
        points = list(reader)
    if not points: return None
    x_vals = [float(p['x']) for p in points]
    y_vals = [float(p['y']) for p in points]
    return {
        'min_x': min(x_vals), 'max_x': max(x_vals),
        'min_y': min(y_vals), 'max_y': max(y_vals)
    }

def visualize_waypoints(csv_path, grid_path=None, base_map_csv=None, show_arrows=True):
    if not os.path.exists(csv_path):
        print(f"Hata: {csv_path} bulunamadı.")
        return

    df = pd.read_csv(csv_path)

    fig, ax = plt.subplots(figsize=(14, 12))

    # Grid / Harita Sınırları
    bounds = None
    if base_map_csv and os.path.exists(base_map_csv):
        bounds = get_grid_bounds(base_map_csv)
        
        # Base waypoints'i hafif mavi olarak göster
        base_df = pd.read_csv(base_map_csv)
        ax.scatter(base_df['x'], base_df['y'], c='lightblue', s=5, alpha=0.3, label=f'Map ({len(base_df)})')

    # Eğer grid varsa bas
    if grid_path and os.path.exists(grid_path) and bounds:
        grid = []
        with open(grid_path, 'r') as f:
            for line in f:
                r = [int(c) for c in line.strip() if c in '01']
                if r: grid.append(r)
        
        if grid:
            grid_arr = np.array(grid)
            grid_display = np.where(grid_arr == 1, 1.0, 0.0)
            grid_display = np.flipud(grid_display)
            extent = [bounds['min_x'], bounds['max_x'], bounds['min_y'], bounds['max_y']]
            ax.imshow(grid_display, cmap='Greys', origin='lower', alpha=0.7, extent=extent, aspect='auto')

    # Ana Rota çizgisi ve Noktaları
    ax.plot(df['x'], df['y'], 'g-', linewidth=2.5, alpha=0.8, label=f'Path ({len(df)} steps)', zorder=10)
    ax.scatter(df['x'], df['y'], c='lime', s=30, alpha=0.6, zorder=9, edgecolors='darkgreen', linewidth=0.5)

    # Başlangıç ve Bitiş
    if len(df) > 0:
        start_x, start_y = df['x'].iloc[0], df['y'].iloc[0]
        end_x, end_y = df['x'].iloc[-1], df['y'].iloc[-1]
        
        circle_start = Circle((start_x, start_y), 1.5, color='green', alpha=0.8, zorder=12, label='Start')
        ax.add_patch(circle_start)
        ax.plot(start_x, start_y, 'g*', markersize=25, zorder=13)
        
        circle_goal = Circle((end_x, end_y), 1.5, color='red', alpha=0.8, zorder=12, label='Goal')
        ax.add_patch(circle_goal)
        ax.plot(end_x, end_y, 'r*', markersize=25, zorder=13)

    # Yön okları
    if show_arrows and 'yaw' in df.columns and len(df) > 1:
        arrow_interval = max(1, len(df) // 10)
        for i in range(0, len(df) - 1, arrow_interval):
            p1_x, p1_y = df['x'].iloc[i], df['y'].iloc[i]
            p2_x, p2_y = df['x'].iloc[i+1], df['y'].iloc[i+1]
            dx, dy = p2_x - p1_x, p2_y - p1_y
            
            mid_x = p1_x + dx * 0.4
            mid_y = p1_y + dy * 0.4
            
            arrow_dx = dx * 0.3
            arrow_dy = dy * 0.3
            
            arrow = FancyArrowPatch((mid_x, mid_y), (mid_x + arrow_dx, mid_y + arrow_dy),
                                    arrowstyle='->', mutation_scale=20, color='darkgreen', alpha=0.7, linewidth=1.5, zorder=11)
            ax.add_patch(arrow)

    plt.title(f"Rota / Waypoint Görselleştirme ({len(df)} Nokta)", fontsize=14, fontweight='bold', pad=20)
    plt.xlabel("X (world meters)")
    plt.ylabel("Y (world meters)")
    plt.legend(loc='upper left')
    plt.grid(True, alpha=0.2, linestyle='--')
    plt.axis('equal')
    
    output_img = csv_path.replace('.csv', '.png')
    plt.savefig(output_img, dpi=120, bbox_inches='tight')
    print(f"✓ Görselleştirme kaydedildi: {output_img}")
    # plt.show() # Display command skipped for terminal usage

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Çok amaçlı Waypoint / Rota görselleştirici")
    parser.add_argument("csv_file", help="Çizilecek CSV rotası/waypoints (örn. planned_route.csv)")
    parser.add_argument("--grid", help="Arkaplan için grid txt dosyası", default=None)
    parser.add_argument("--map", help="Referans genel harita CSV dosyası", default=None)
    parser.add_argument("--no-arrows", action="store_true", help="Yön oklarını gösterme")
    args = parser.parse_args()
    
    visualize_waypoints(args.csv_file, args.grid, args.map, show_arrows=not args.no_arrows)
