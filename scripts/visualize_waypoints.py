#!/usr/bin/env python3
import argparse
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def visualize(csv_path: str, show_yaw: bool, output: str = None) -> str:
    if not os.path.exists(csv_path):
        raise RuntimeError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError(f"CSV is empty: {csv_path}")
    for column in ("x", "y"):
        if column not in df.columns:
            raise RuntimeError(f"Missing required column '{column}' in {csv_path}")

    output = output or os.path.splitext(csv_path)[0] + "_visualized.png"
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_title(f"Waypoint route\n{csv_path}\nTotal: {len(df)}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")

    ax.plot(df["x"], df["y"], color="#d62728", linewidth=2.0, label="recorded route")
    ax.scatter(df["x"], df["y"], color="#1f77b4", s=14, alpha=0.8, label="waypoints")
    ax.scatter([df["x"].iloc[0]], [df["y"].iloc[0]], color="#2ca02c", s=120, marker="o", label="start", zorder=5)
    ax.scatter([df["x"].iloc[-1]], [df["y"].iloc[-1]], color="#d62728", s=150, marker="*", label="goal", zorder=5)

    if show_yaw and "yaw" in df.columns:
        interval = max(1, len(df) // 120)
        yaw_rad = df["yaw"].iloc[::interval].apply(math.radians)
        ax.quiver(
            df["x"].iloc[::interval],
            df["y"].iloc[::interval],
            yaw_rad.apply(math.cos),
            yaw_rad.apply(math.sin),
            color="#111111",
            scale=45,
            width=0.003,
            headwidth=4,
            headlength=5,
            pivot="mid",
            alpha=0.85,
            label="yaw",
        )

    ax.legend(loc="best")
    fig.savefig(output, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize recorded waypoint CSV")
    parser.add_argument("csv_file")
    parser.add_argument("--show-yaw", action="store_true")
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = visualize(args.csv_file, args.show_yaw, args.output)
    print(f"Saved visualization: {output}")


if __name__ == "__main__":
    main()
