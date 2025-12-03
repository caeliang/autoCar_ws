#!/usr/bin/env python3
"""
Read fusion_data.csv and write a static plot (PNG/SVG/PDF).

Usage:
  python3 plot_csv_static.py --csv /path/to/fusion_data.csv --out /path/to/output.svg

Defaults:
  csv: ~/autoCar_ws/src/sensor_fusion_pkg/log/fusion_data.csv
  out: ~/autoCar_ws/src/sensor_fusion_pkg/log/fusion_plot.svg
"""
import argparse
import os
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    """Read CSV file with columns: time,odom_x,odom_y,lidar_x,lidar_y,fused_x,fused_y"""
    times = []
    odom_x = []
    odom_y = []
    lidar_x = []
    lidar_y = []
    fused_x = []
    fused_y = []
    
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = int(row['time'])
            except Exception:
                continue
            times.append(t)
            odom_x.append(float(row.get('odom_x', '0') or 0))
            odom_y.append(float(row.get('odom_y', '0') or 0))
            # Handle missing lidar values
            lx = row.get('lidar_x', '')
            ly = row.get('lidar_y', '')
            lidar_x.append(float(lx) if lx else None)
            lidar_y.append(float(ly) if ly else None)
            fused_x.append(float(row.get('fused_x', '0') or 0))
            fused_y.append(float(row.get('fused_y', '0') or 0))
    
    return times, odom_x, odom_y, lidar_x, lidar_y, fused_x, fused_y


def make_plot(times, ox, oy, lx, ly, fx, fy, out_path, width, height, dpi):
    """Create trajectory plot with odom, lidar, and fused data"""
    if not times:
        print('No data in CSV')
        return 1

    fig, ax = plt.subplots(figsize=(width, height))
    
    # Plot odom trajectory
    ax.plot(ox, oy, label='Odom', color='blue', linewidth=1.5, alpha=0.7)
    
    # Plot lidar if available (filter out None values)
    if any(v is not None for v in lx):
        lx_clean = [x for x, y in zip(lx, ly) if x is not None and y is not None]
        ly_clean = [y for x, y in zip(lx, ly) if x is not None and y is not None]
        if lx_clean and ly_clean:
            ax.plot(lx_clean, ly_clean, label='Lidar', color='green', 
                   linewidth=1.5, alpha=0.7, marker='o', markersize=3)
    
    # Plot fused trajectory
    ax.plot(fx, fy, label='Fused (Kalman)', color='red', linestyle='--', linewidth=2)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Sensor Fusion: Odom + Lidar → Fused Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    plt.tight_layout()

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    
    # Determine format from file extension (svg/pdf/png)
    fmt = os.path.splitext(out_path)[1].lower().lstrip('.')
    if fmt not in ('png', 'svg', 'pdf'):
        fmt = 'svg'
        out_path = os.path.splitext(out_path)[0] + '.svg'
    
    fig.savefig(out_path, dpi=dpi, format=fmt, bbox_inches='tight')
    print(f'Wrote plot to {out_path}')
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', default=os.path.expanduser('~/autoCar_ws/src/sensor_fusion_pkg/log/fusion_data.csv'))
    p.add_argument('--out', default=os.path.expanduser('~/autoCar_ws/src/sensor_fusion_pkg/log/fusion_plot.svg'))
    p.add_argument('--dpi', type=int, default=300, help='DPI for raster outputs (png)')
    p.add_argument('--width', type=float, default=12.0, help='Figure width in inches')
    p.add_argument('--height', type=float, default=8.0, help='Figure height in inches')
    args = p.parse_args()

    if not os.path.exists(args.csv):
        print('CSV file not found:', args.csv)
        return 2

    times, ox, oy, lx, ly, fx, fy = read_csv(args.csv)
    return make_plot(times, ox, oy, lx, ly, fx, fy, args.out, args.width, args.height, args.dpi)


if __name__ == '__main__':
    raise SystemExit(main())
