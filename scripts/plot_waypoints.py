#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import os
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

def get_color_for_yaw(yaw):
    y = (yaw % 360 + 360) % 360
    if y >= 330 or y <= 30: return 'red'        # Doğu
    elif 60 <= y <= 120: return 'blue'          # Kuzey
    elif 150 <= y <= 210: return 'green'        # Batı
    elif 240 <= y <= 300: return 'black'        # Güney
    else: return 'orange'                       # Kavşak/Dönüş

def iter_connected_runs(group):
    if group.empty:
        return
    rows = group.sort_values('step') if 'step' in group.columns else group
    current = []
    records = list(rows.to_dict('records'))
    for idx, row in enumerate(records):
        current.append(row)
        connect_next = int(row.get('connect_next', 1)) if not pd.isna(row.get('connect_next', 1)) else 1
        if connect_next == 0 or idx == len(records) - 1:
            if len(current) >= 2:
                yield pd.DataFrame(current)
            current = []

def draw_grouped_paths(ax, df, is_route):
    group_cols = [c for c in ['direction', 'lane_id', 'segment_id'] if c in df.columns]
    if not group_cols:
        first = True
        for run in iter_connected_runs(df):
            ax.plot(
                run['x'], run['y'],
                color='darkgreen' if is_route else '#2677b8',
                linewidth=2.0 if is_route else 1.3,
                alpha=0.7,
                label=f'Path groups' if first else None,
                zorder=10,
            )
            first = False
        return

    first = True
    for _, group in df.groupby(group_cols, sort=False):
        for run in iter_connected_runs(group):
            ax.plot(
                run['x'], run['y'],
                color='darkgreen' if is_route else '#2677b8',
                linewidth=2.0 if is_route else 1.3,
                alpha=0.7,
                label=f'Path groups' if first else None,
                zorder=10,
            )
            first = False

def visualize_waypoints(csv_path, grid_path=None, base_map_csv=None, show_arrows=True, arrow_interval=1):
    if not os.path.exists(csv_path):
        print(f"Hata: {csv_path} bulunamadı.")
        return

    df = pd.read_csv(csv_path)
    is_route = 'step' in df.columns

    fig, ax = plt.subplots(figsize=(14, 12))

    # Grid / Harita Sınırları
    bounds = None
    if base_map_csv and os.path.exists(base_map_csv):
        bounds = get_grid_bounds(base_map_csv)
        base_df = pd.read_csv(base_map_csv)
        # Eğer bu bir rota ise, alt haritayı gri çizelim
        ax.scatter(base_df['x'], base_df['y'], c='lightgray', s=5, alpha=0.4, label=f'Map ({len(base_df)})')

    if not bounds and len(df) > 0:
        bounds = {
            'min_x': df['x'].min(), 'max_x': df['x'].max(),
            'min_y': df['y'].min(), 'max_y': df['y'].max()
        }

    # Grid arka planı (Koyu gri asfalt blokları gibi)
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
            # Extent calculation
            extent = [bounds['min_x']-2.5, bounds['max_x']+2.5, bounds['min_y']-2.5, bounds['max_y']+2.5]
            ax.imshow(grid_display, cmap='Greys', origin='lower', alpha=0.45, extent=extent, aspect='auto')

    # Eger bu CSV planlanmıs bir ROTA ise cizgi ve markorler ekle
    if is_route:
        draw_grouped_paths(ax, df, is_route)
        ax.scatter(df['x'], df['y'], c='lime', s=20, alpha=0.8, zorder=9, edgecolors='darkgreen', linewidth=0.5)

        if len(df) > 0:
            start_x, start_y = df['x'].iloc[0], df['y'].iloc[0]
            end_x, end_y = df['x'].iloc[-1], df['y'].iloc[-1]
            ax.add_patch(Circle((start_x, start_y), 1.5, color='green', alpha=0.8, zorder=12, label='Start'))
            ax.add_patch(Circle((end_x, end_y), 1.5, color='red', alpha=0.8, zorder=12, label='Goal'))
    else:
        # Full Map modunda ufak markerlar
        pass

    # Yön Okları (Tam olarak görseldeki renklendirme standartı ile)
    if show_arrows and 'yaw' in df.columns and len(df) > 0:
        df_subset = df.iloc[::arrow_interval].copy()
        
        U = np.cos(np.deg2rad(df_subset['yaw']))
        V = np.sin(np.deg2rad(df_subset['yaw']))
        
        if is_route:
            colors = 'darkgreen'
            scale = 45
            width = 0.005
        else:
            colors = [get_color_for_yaw(y) for y in df_subset['yaw']]
            scale = 45 
            width = 0.0025

        ax.quiver(df_subset['x'], df_subset['y'], U, V, 
                  color=colors, scale=scale, width=width, headwidth=4, headlength=5, 
                  pivot='mid', alpha=0.9, zorder=11)

    title_mode = "Waypoints Yaw Görselleştirmesi" if not is_route else "Rota (Path) Görselleştirmesi"
    title_file = os.path.relpath(csv_path, start=os.getcwd())
    
    title = f"{title_mode}\n{title_file}\nToplam: {len(df)} waypoint, Gösterilen: her {arrow_interval}. nokta"
    plt.title(title, fontsize=12, fontweight='bold', pad=10)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        plt.legend(loc='upper left')
        
    plt.grid(True, alpha=0.2, linestyle='-')
    plt.axis('equal')
    
    output_img = csv_path.replace('.csv', '.png')
    plt.savefig(output_img, dpi=120, bbox_inches='tight')
    print(f"✓ Görselleştirme kaydedildi: {output_img}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="Çizilecek CSV rotası/waypoints")
    parser.add_argument("--grid", help="Arkaplan için grid txt", default=None)
    parser.add_argument("--map", help="Referans genel map CSV", default=None)
    parser.add_argument("--no-arrows", action="store_true", help="Yön oklarını gösterme")
    parser.add_argument("--arrow-interval", type=int, default=1, help="Ok çizim aralığı (ör: 1)")
    args = parser.parse_args()
    
    visualize_waypoints(args.csv_file, args.grid, args.map, not args.no_arrows, args.arrow_interval)
