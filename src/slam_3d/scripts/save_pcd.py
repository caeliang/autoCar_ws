#!/usr/bin/env python3
"""
Save Point Cloud as PCD - Simple and Fast!
Just subscribes to topic and saves directly
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import struct
import sys


class PCDSaver(Node):
    def __init__(self, map_name):
        super().__init__('pcd_saver')
        self.map_name = map_name
        self.cloud_data = None
        
        # Subscribe to octomap point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.cloud_callback,
            10)
        
        print(f'\n╔══════════════════════════════════════╗')
        print(f'║   PCD Map Saver                      ║')
        print(f'╚══════════════════════════════════════╝\n')
        print(f'Map name: {map_name}')
        print(f'Waiting for point cloud data...\n')
    
    def cloud_callback(self, msg):
        self.cloud_data = msg
        self.get_logger().info(f'Received cloud with {msg.width * msg.height} points')
    
    def save_pcd(self):
        if self.cloud_data is None:
            print('✗ No point cloud data received yet!')
            print('  Make sure 3D SLAM is running and has built some map.')
            return False
        
        import os
        maps_dir = '/home/ranim/autoCar_ws/maps'
        os.makedirs(maps_dir, exist_ok=True)
        
        pcd_filename = os.path.join(maps_dir, f'{self.map_name}.pcd')
        
        # Extract points
        points = []
        for point in pc2.read_points(self.cloud_data, field_names=("x", "y", "z"), skip_nans=True):
            points.append(point)
        
        if len(points) == 0:
            print('✗ No valid points in cloud!')
            return False
        
        # Write PCD file
        with open(pcd_filename, 'w') as f:
            # Header
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            
            # Data
            for point in points:
                f.write(f'{point[0]} {point[1]} {point[2]}\n')
        
        print(f'\n✓ SUCCESS: Point cloud saved!')
        print(f'  File: {pcd_filename}')
        print(f'  Points: {len(points)}')
        print(f'\nTo visualize:')
        print(f'  - pcl_viewer {pcd_filename}')
        print(f'  - Or open in CloudCompare')
        print(f'  - Or RViz: Add PointCloud2, set topic to file\n')
        
        return True


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else 'my_3d_map'
    
    rclpy.init()
    saver = PCDSaver(map_name)
    
    # Wait for data (max 10 seconds)
    import time
    start = time.time()
    while rclpy.ok() and (time.time() - start) < 10.0:
        rclpy.spin_once(saver, timeout_sec=0.5)
        if saver.cloud_data is not None:
            break
    
    if saver.cloud_data is None:
        print('✗ Timeout waiting for point cloud')
        print('  Check: ./test_3d_slam.sh')
    else:
        saver.save_pcd()
    
    saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
