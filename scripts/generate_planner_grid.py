#!/usr/bin/env python3
"""
PCD to Planner Grid - Binaları koruyan versiyon
"""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from pcd_pipeline import (
    PointCloudLoader,
    VoxelGridFilter,
    MatrixConverter,
    SmartGridProcessor,
    FileWriter
)

def main():
    pcd_file = 'maps/test_pcd.pcd'
    output_dir = 'output_grids'
    
    # Parametreler - binaları görebilmek için ayarlı
    voxel_size = 0.1  # Daha küçük = daha detaylı
    grid_resolution = 0.05  # Daha küçük = daha yüksek çözünürlük
    inflation_radius = 0.3  # Robot güvenlik mesafesi
    height_range = (-0.5, 2.0)  # Zemin ve binalar
    
    print("="*70)
    print("PCD → Planner Grid (Binaları Koruyan Versiyon)")
    print("="*70)
    
    # 1. Load
    print("\n1. Loading PCD...")
    loader = PointCloudLoader()
    points = loader.load(pcd_file)
    print(f"   ✓ {len(points)} points loaded")
    
    # 2. Voxel filter
    print("\n2. Voxel filtering...")
    voxel_filter = VoxelGridFilter(voxel_size, height_range)
    filtered_points = voxel_filter.filter(points)
    print(f"   ✓ {len(filtered_points)} points after filtering")
    
    # 3. Binary occupancy
    print("\n3. Creating binary occupancy grid...")
    converter = MatrixConverter()
    occupancy_grid, origin, resolution = converter.convert_to_occupancy_grid(
        filtered_points, 
        grid_resolution
    )
    print(f"   ✓ Grid: {occupancy_grid.shape[1]}×{occupancy_grid.shape[0]}")
    print(f"   ✓ Occupied cells: {np.sum(occupancy_grid==1)} ({100*np.sum(occupancy_grid==1)/occupancy_grid.size:.2f}%)")
    
    # 4. Smart processing
    print("\n4. Smart processing (preserving buildings)...")
    print(f"   a) Fill small holes only (<100 cells)")
    print(f"   b) Binary inflation ({inflation_radius}m safety margin)")
    
    processor = SmartGridProcessor(
        inflation_radius_meters=inflation_radius,
        fill_small_holes_only=True
    )
    planner_grid = processor.process(occupancy_grid, resolution)
    
    occupied_before = np.sum(occupancy_grid == 1)
    occupied_after = np.sum(planner_grid == 1)
    
    print(f"   ✓ Before: {occupied_before} obstacle cells")
    print(f"   ✓ After: {occupied_after} obstacle cells (+{occupied_after-occupied_before})")
    print(f"   ✓ Buildings preserved!")
    
    # 5. Save
    print("\n5. Saving files...")
    os.makedirs(output_dir, exist_ok=True)
    
    # Planner grid (final)
    base = os.path.join(output_dir, 'planner_grid')
    planner_pgm = (planner_grid * 100).astype(np.uint8)
    FileWriter.save_pgm(planner_pgm, f"{base}.pgm")
    FileWriter.save_yaml(f"{base}.yaml", origin, resolution)
    FileWriter.save_numpy(planner_grid, f"{base}.npy")
    print(f"   ✓ {base}.pgm/yaml/npy")
    
    # Original occupancy (comparison)
    base_occ = os.path.join(output_dir, 'occupancy_original')
    occ_pgm = (occupancy_grid * 100).astype(np.uint8)
    FileWriter.save_pgm(occ_pgm, f"{base_occ}.pgm")
    FileWriter.save_numpy(occupancy_grid, f"{base_occ}.npy")
    print(f"   ✓ {base_occ}.pgm/npy")
    
    print("\n" + "="*70)
    print("✓ Done! Buildings are preserved in the grid")
    print(f"  Free cells: {np.sum(planner_grid==0):,} ({100*np.sum(planner_grid==0)/planner_grid.size:.1f}%)")
    print(f"  Obstacle cells: {np.sum(planner_grid==1):,} ({100*np.sum(planner_grid==1)/planner_grid.size:.1f}%)")
    print("="*70)
    
    # Show samples
    print("\nSample - Building area:")
    obs_idx = np.where(planner_grid == 1)
    if len(obs_idx[0]) > 1000:
        y, x = obs_idx[0][1000], obs_idx[1][1000]
        sample = planner_grid[y-5:y+6, x-5:x+6]
        print(f"Position [{y},{x}] (11x11):")
        for row in sample:
            print(' '.join(str(int(v)) for v in row))
    
    print("\nSample - Free area:")
    free_idx = np.where(planner_grid == 0)
    if len(free_idx[0]) > 100:
        y, x = free_idx[0][len(free_idx[0])//2], free_idx[1][len(free_idx[1])//2]
        sample = planner_grid[y-5:y+6, x-5:x+6]
        print(f"Position [{y},{x}] (11x11):")
        for row in sample:
            print(' '.join(str(int(v)) for v in row))
    
    return planner_grid

if __name__ == '__main__':
    main()
