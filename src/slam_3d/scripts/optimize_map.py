#!/usr/bin/env python3
"""
Map Optimization Script for Localization
Cleans and optimizes PCD maps for better localization performance

Steps:
1. Voxel Grid Filter - Downsample point cloud
2. Statistical Outlier Removal - Remove noise
3. Ground Segmentation - Separate ground points
4. Height Filtering - Keep only driving-relevant height
"""

import sys
import numpy as np
import open3d as o3d
from pathlib import Path


def print_banner():
    """Print script banner"""
    print('\n' + '='*60)
    print('║         MAP OPTIMIZATION FOR LOCALIZATION            ║')
    print('='*60 + '\n')


def load_pcd(file_path):
    """Load PCD file"""
    print(f'📂 Loading: {file_path}')
    pcd = o3d.io.read_point_cloud(str(file_path))
    points_before = len(pcd.points)
    print(f'   ✅ Loaded {points_before:,} points\n')
    return pcd, points_before


def voxel_downsample(pcd, voxel_size=0.1):
    """Apply voxel grid filter for downsampling"""
    print(f'🔽 Step 1: Voxel Grid Filter (voxel size: {voxel_size}m)')
    points_before = len(pcd.points)
    
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    points_after = len(pcd_downsampled.points)
    reduction = (1 - points_after/points_before) * 100
    print(f'   ✅ {points_before:,} → {points_after:,} points ({reduction:.1f}% reduction)\n')
    
    return pcd_downsampled


def remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """Remove statistical outliers (noise)"""
    print(f'🧹 Step 2: Statistical Outlier Removal')
    points_before = len(pcd.points)
    
    pcd_clean, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    
    points_after = len(pcd_clean.points)
    removed = points_before - points_after
    print(f'   ✅ Removed {removed:,} outliers ({removed/points_before*100:.1f}%)')
    print(f'   ✅ Remaining: {points_after:,} points\n')
    
    return pcd_clean


def segment_ground(pcd, distance_threshold=0.2):
    """Segment ground plane using RANSAC"""
    print(f'🌍 Step 3: Ground Segmentation (RANSAC)')
    points_before = len(pcd.points)
    
    # Find ground plane
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000
    )
    
    [a, b, c, d] = plane_model
    print(f'   ✅ Ground plane: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0')
    
    # Remove ground points
    pcd_no_ground = pcd.select_by_index(inliers, invert=True)
    
    ground_points = len(inliers)
    points_after = len(pcd_no_ground.points)
    print(f'   ✅ Removed {ground_points:,} ground points ({ground_points/points_before*100:.1f}%)')
    print(f'   ✅ Remaining: {points_after:,} points\n')
    
    return pcd_no_ground


def filter_height(pcd, min_height=0.3, max_height=3.0):
    """Keep only points within driving-relevant height"""
    print(f'📏 Step 4: Height Filtering ({min_height}m - {max_height}m)')
    points_before = len(pcd.points)
    
    # Get points as numpy array
    points = np.asarray(pcd.points)
    
    # Filter by Z coordinate
    mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    
    # Create new point cloud
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(points[mask])
    
    # Copy colors if available
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        pcd_filtered.colors = o3d.utility.Vector3dVector(colors[mask])
    
    points_after = len(pcd_filtered.points)
    removed = points_before - points_after
    print(f'   ✅ Removed {removed:,} points outside height range ({removed/points_before*100:.1f}%)')
    print(f'   ✅ Remaining: {points_after:,} points\n')
    
    return pcd_filtered


def save_pcd(pcd, output_path):
    """Save optimized PCD file"""
    print(f'💾 Saving optimized map: {output_path}')
    o3d.io.write_point_cloud(str(output_path), pcd, write_ascii=False)
    print(f'   ✅ Saved successfully!\n')


def main():
    print_banner()
    
    # Check arguments
    if len(sys.argv) < 2:
        print('❌ Usage: ./optimize_map.py <input_map.pcd> [output_name]')
        print('   Example: ./optimize_map.py maps/my_map.pcd my_map_optimized')
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    
    if not input_file.exists():
        print(f'❌ Error: File not found: {input_file}')
        sys.exit(1)
    
    # Output filename
    if len(sys.argv) >= 3:
        output_name = sys.argv[2]
    else:
        output_name = input_file.stem + '_optimized'
    
    output_file = input_file.parent / f'{output_name}.pcd'
    
    # Load PCD
    pcd, original_points = load_pcd(input_file)
    
    # Optimization pipeline
    print('🔄 Starting optimization pipeline...\n')
    
    # Step 1: Voxel downsampling
    pcd = voxel_downsample(pcd, voxel_size=0.1)
    
    # Step 2: Outlier removal
    pcd = remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0)
    
    # Step 3: Ground segmentation
    pcd = segment_ground(pcd, distance_threshold=0.2)
    
    # Step 4: Height filtering
    pcd = filter_height(pcd, min_height=0.3, max_height=3.0)
    
    # Save result
    save_pcd(pcd, output_file)
    
    # Summary
    final_points = len(pcd.points)
    total_reduction = (1 - final_points/original_points) * 100
    
    print('='*60)
    print('║                    SUMMARY                            ║')
    print('='*60)
    print(f'  Original points:  {original_points:,}')
    print(f'  Optimized points: {final_points:,}')
    print(f'  Total reduction:  {total_reduction:.1f}%')
    print(f'  Output file:      {output_file}')
    print('='*60)
    print('\n✅ Map optimization complete!')
    print(f'\n📊 To visualize: pcl_viewer {output_file}')
    print(f'🔧 To use in localization: Update your launch file with this map\n')


if __name__ == '__main__':
    main()
