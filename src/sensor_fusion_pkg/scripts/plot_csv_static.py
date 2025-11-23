#!/usr/bin/env python3
"""
Read fusion_log.csv and write a static PNG with odom vs fused trajectories.
Usage:
  python3 plot_csv_static.py --csv /path/to/fusion_log.csv --out /path/to/output.png
Defaults:
  csv: ~/autoCar_ws/fusion_log.csv
  out: ~/autoCar_ws/fusion_plot.png
"""
import argparse
import os
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime


def read_csv(path):
    times = []
    odom_x = []
    odom_y = []
    fused_x = []
    fused_y = []
    lidar = []
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
            fused_x.append(float(row.get('fused_x', '0') or 0))
            fused_y.append(float(row.get('fused_y', '0') or 0))
            lidar.append(row.get('lidar_avg', ''))
    return times, odom_x, odom_y, fused_x, fused_y, lidar


def make_plot(times, ox, oy, fx, fy, out_path):
    if not times:
        print('No data in CSV')
        return 1
    # convert ms since epoch to seconds relative to first sample
    t0 = times[0] / 1000.0
    t = [(ts / 1000.0) - t0 for ts in times]
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    ax.plot(ox, oy, label='Odom', color='blue', linewidth=1)
    ax.plot(fx, fy, label='Fused', color='red', linestyle='--', linewidth=1)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Odom vs Fused Trajectory')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    # Determine format from file extension (svg/pdf/png)
    fmt = os.path.splitext(out_path)[1].lower().lstrip('.')
    if fmt not in ('png', 'svg', 'pdf'):
        fmt = 'svg'
        out_path = os.path.splitext(out_path)[0] + '.svg'
    fig.savefig(out_path, dpi=DPI, format=fmt)
    print(f'Wrote plot to {out_path}')
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', default=os.path.expanduser('~/autoCar_ws/fusion_log.csv'))
    p.add_argument('--out', default=os.path.expanduser('~/autoCar_ws/fusion_plot.svg'))
    p.add_argument('--dpi', type=int, default=300, help='DPI for raster outputs (png)')
    p.add_argument('--width', type=float, default=10.0, help='Figure width in inches')
    p.add_argument('--height', type=float, default=6.0, help='Figure height in inches')
    args = p.parse_args()

    global DPI, FIG_W, FIG_H
    DPI = args.dpi
    FIG_W = args.width
    FIG_H = args.height

    if not os.path.exists(args.csv):
        print('CSV file not found:', args.csv)
        return 2

    times, ox, oy, fx, fy, lidar = read_csv(args.csv)
    return make_plot(times, ox, oy, fx, fy, args.out)

if __name__ == '__main__':
    raise SystemExit(main())
