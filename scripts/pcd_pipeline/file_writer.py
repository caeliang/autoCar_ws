"""File Writer"""
import numpy as np
from pathlib import Path

class FileWriter:
    @staticmethod
    def save_pgm(grid, filepath):
        height, width = grid.shape
        with open(filepath, 'w') as f:
            f.write('P2\n')
            f.write(f'{width} {height}\n')
            f.write('255\n')
            for row in grid:
                f.write(' '.join(map(str, row)) + '\n')
    
    @staticmethod
    def save_yaml(filepath, origin, resolution):
        image_name = Path(filepath).stem + '.pgm'
        with open(filepath, 'w') as f:
            f.write(f'image: {image_name}\n')
            f.write(f'resolution: {resolution}\n')
            f.write(f'origin: [{origin[0]}, {origin[1]}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.196\n')
    
    @staticmethod
    def save_numpy(array, filepath):
        np.save(filepath, array)
