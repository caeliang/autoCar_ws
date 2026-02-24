"""Point Cloud Loader"""
import numpy as np
import struct

class PointCloudLoader:
    def load(self, pcd_file):
        try:
            return self._load_ascii(pcd_file)
        except:
            return self._load_binary(pcd_file)
    
    def _load_ascii(self, pcd_file):
        points = []
        with open(pcd_file, 'r') as f:
            data_started = False
            for line in f:
                if line.startswith('DATA'):
                    data_started = True
                    continue
                if data_started:
                    try:
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except:
                        continue
        return np.array(points)
    
    def _load_binary(self, pcd_file):
        with open(pcd_file, 'rb') as f:
            header_lines = []
            while True:
                line = f.readline().decode('ascii')
                header_lines.append(line)
                if line.startswith('POINTS'):
                    num_points = int(line.split()[1])
                if line.startswith('DATA'):
                    break
            points = []
            for _ in range(num_points):
                data = f.read(12)
                if len(data) < 12:
                    break
                x, y, z = struct.unpack('fff', data)
                points.append([x, y, z])
        return np.array(points)
