"""Matrix Converter"""
import numpy as np

class MatrixConverter:
    def __init__(self):
        self.grid = None
        self.origin = None
        self.resolution = None
    
    def convert_to_occupancy_grid(self, points, grid_resolution=0.05):
        if len(points) == 0:
            return np.array([[]]), (0, 0), grid_resolution
        
        xy_points = points[:, :2]
        x_min, y_min = np.min(xy_points, axis=0)
        x_max, y_max = np.max(xy_points, axis=0)
        
        grid_width = int(np.ceil((x_max - x_min) / grid_resolution)) + 1
        grid_height = int(np.ceil((y_max - y_min) / grid_resolution)) + 1
        
        self.resolution = grid_resolution
        self.origin = (x_min, y_min)
        
        grid = np.zeros((grid_height, grid_width), dtype=np.uint8)
        
        for x, y in xy_points:
            grid_x = int((x - x_min) / grid_resolution)
            grid_y = int((y - y_min) / grid_resolution)
            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                grid[grid_y, grid_x] = 1
        
        self.grid = grid
        return grid, self.origin, self.resolution
