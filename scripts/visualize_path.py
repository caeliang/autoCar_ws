#!/usr/bin/env python3
"""
Visualize A* generated path on grid map
Usage: python3 visualize_path.py <grid_file> <waypoints_csv> <start_x> <start_y> <goal_x> <goal_y>
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for headless environments
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv

def load_grid(grid_file):
    """Load grid from text file (0=passable, 1=obstacle)"""
    grid = []
    try:
        with open(grid_file, 'r') as f:
            for line in f:
                row = [int(c) for c in line.strip() if c in '01']
                if row:
                    grid.append(row)
        return np.array(grid)
    except Exception as e:
        print(f"Error loading grid: {e}")
        return None

def load_waypoints(csv_file):
    """Load waypoints from CSV file"""
    waypoints = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                waypoints.append((x, y))
        return waypoints
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return []

def grid_to_world(grid_x, grid_y, grid_shape=(68, 59)):
    """Convert grid coordinates to world coordinates (MATCHES generate_route.py exactly)"""
    grid_height, grid_width = grid_shape
    min_x, max_x = -28.8 * 1.5, 28.8 * 1.5
    min_y, max_y = -32.5 * 1.5, 33.8 * 1.5
    x_span = max_x - min_x
    y_span = max_y - min_y
    world_x = min_x + (grid_x / (grid_width - 1)) * x_span
    world_y = max_y - (grid_y / (grid_height - 1)) * y_span
    return world_x, world_y


def world_to_grid(world_x, world_y, grid_shape=(68, 59)):
    """Convert world coordinates to grid coordinates (MATCHES generate_route.py exactly)"""
    grid_height, grid_width = grid_shape
    min_x, max_x = -28.8 * 1.5, 28.8 * 1.5
    min_y, max_y = -32.5 * 1.5, 33.8 * 1.5
    x_span = max_x - min_x
    y_span = max_y - min_y
    grid_x = round((world_x - min_x) / x_span * (grid_width - 1))
    grid_y = round((max_y - world_y) / y_span * (grid_height - 1))
    return int(grid_x), int(grid_y)


def main():
    if len(sys.argv) != 7:
        print("Usage: python3 visualize_path.py <grid_file> <waypoints_csv> <start_x> <start_y> <goal_x> <goal_y>")
        sys.exit(1)
    
    grid_file = sys.argv[1]
    csv_file = sys.argv[2]
    start_x = float(sys.argv[3])
    start_y = float(sys.argv[4])
    goal_x = float(sys.argv[5])
    goal_y = float(sys.argv[6])
    
    # Load data
    grid = load_grid(grid_file)
    waypoints = load_waypoints(csv_file)
    
    if grid is None:
        print("Could not load grid")
        return
    
    if not waypoints:
        print("Could not load waypoints")
        return
    
    print(f"✓ Loaded grid: {grid.shape}")
    print(f"✓ Loaded {len(waypoints)} waypoints")
    print(f"✓ Start: ({start_x}, {start_y}) → Goal: ({goal_x}, {goal_y})")
    
    # Create figure
    fig, ax = plt.subplots(figsize=(16, 14))
    
    # Plot grid (0 = free space in white, 1 = obstacle in dark)
    grid_display = np.where(grid == 0, 1, 0)  # Invert for visualization
    ax.imshow(grid_display, cmap='gray', origin='upper', alpha=0.3, extent=[-28.8 * 1.5, 28.8 * 1.5, -32.5 * 1.5, 33.8 * 1.5])
    
    # Plot waypoints as a line
    if len(waypoints) > 1:
        wx = [w[0] for w in waypoints]
        wy = [w[1] for w in waypoints]
        ax.plot(wx, wy, 'b-', linewidth=1.5, alpha=0.7, label=f'Path ({len(waypoints)} points)')
        
        # Plot waypoint markers every 10 points
        for i in range(0, len(waypoints), max(1, len(waypoints)//10)):
            ax.plot(waypoints[i][0], waypoints[i][1], 'bo', markersize=4, alpha=0.6)
    
    # Plot start and goal
    # Arguments are passed as Grid Coordinates (e.g. 3 3 and 45 55)
    start_grid = (start_x, start_y)
    goal_grid = (goal_x, goal_y)
    
    start_world = grid_to_world(start_grid[0], start_grid[1])
    goal_world = grid_to_world(goal_grid[0], goal_grid[1])
    
    ax.plot(start_world[0], start_world[1], 'g*', markersize=20, label=f'Start ({start_x}, {start_y})')
    ax.plot(goal_world[0], goal_world[1], 'r*', markersize=20, label=f'Goal ({goal_x}, {goal_y})')
    
    # Title and labels
    ax.set_xlabel('World X (meters)', fontsize=12)
    ax.set_ylabel('World Y (meters)', fontsize=12)
    ax.set_title(f'A* Path: ({start_x},{start_y}) → ({goal_x},{goal_y}) | {len(waypoints)} waypoints', fontsize=14, fontweight='bold')
    ax.legend(fontsize=11, loc='upper left')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    
    # Save to file
    output_path = os.path.expanduser('~/autoCar_ws/planned_path_visualization.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved to: {output_path}")
    
    # Try to show if display is available
    try:
        if os.environ.get('DISPLAY'):
            plt.show()
    except:
        pass

if __name__ == '__main__':
    main()
