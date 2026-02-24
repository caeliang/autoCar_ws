"""Voxel Grid Filter"""
import numpy as np

class VoxelGridFilter:
    def __init__(self, voxel_size=0.1, height_threshold=(-2.0, 2.0)):
        self.voxel_size = voxel_size
        self.height_threshold = height_threshold
    
    def filter(self, points):
        if len(points) == 0:
            return np.array([])
        
        # Height filter
        min_z, max_z = self.height_threshold
        mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        filtered = points[mask]
        
        if len(filtered) == 0:
            return np.array([])
        
        # Voxel downsampling
        voxel_coords = np.floor(filtered[:, :2] / self.voxel_size).astype(int)
        unique_voxels = {}
        for i, (vx, vy) in enumerate(voxel_coords):
            key = (vx, vy)
            if key not in unique_voxels:
                unique_voxels[key] = []
            unique_voxels[key].append(i)
        
        result = []
        for indices in unique_voxels.values():
            avg_point = np.mean(filtered[indices], axis=0)
            result.append(avg_point)
        
        return np.array(result)
