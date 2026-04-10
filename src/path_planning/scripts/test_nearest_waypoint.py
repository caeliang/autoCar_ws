#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import subprocess
import math

class NearestWaypointTester(Node):
    def __init__(self):
        super().__init__('test_nearest_waypoint')
        
        # Arka planda C++ node'larını başlatıyoruz
        self.get_logger().info("Gerekli C++ node'ları (tracker ve finder) arka planda başlatılıyor...")
        
        # Bu alt süreçleri saklıyoruz ki script kapanırken onları da kapatalım
        self.tracker_process = subprocess.Popen(["ros2", "run", "path_planning", "waypoint_tracker"])
        self.finder_process = subprocess.Popen(["ros2", "run", "path_planning", "car_index_finder"])
        
        # Sadece C++ node'larinin sonuclarini dinleyecegiz
        self.sub_wp = self.create_subscription(Point, '/prius/nearest_waypoint', self.wp_callback, 10)
        self.sub_grid = self.create_subscription(Point, '/prius/car_matrix_index', self.grid_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/prius/odom', self.odom_callback, 10)
        
        self.latest_wp = None
        self.last_grid = None
        self.latest_car_pos = None

        self.get_logger().info("Test aracı başlatıldı.")
        self.get_logger().info("Waypoint Tracker ve Car Index Finder node'larının çıktıları bekleniyor...")

    def odom_callback(self, msg):
        self.latest_car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def wp_callback(self, msg):
        self.latest_wp = msg

    def grid_callback(self, msg):
        if self.latest_wp is not None and self.latest_car_pos is not None:
            # Gelen grid matris degerlerini ekrana yazdir
            grid_x = int(msg.x)
            grid_y = int(msg.y)
            
            # Sadece matris indeksi degistiginde veya ilk defa alindiginda yazdir
            current_grid = (grid_x, grid_y)
            if self.last_grid != current_grid:
                self.last_grid = current_grid
                
                # O anki arac konumu ile wp arasindaki mesafeyi hesapla
                dist = math.dist(self.latest_car_pos, (self.latest_wp.x, self.latest_wp.y))
                
                self.get_logger().info(
                    f"\nAraç Konumu: ({self.latest_car_pos[0]:.2f}, {self.latest_car_pos[1]:.2f}) | "
                    f"Hedeflenen En Yakın WP: ({self.latest_wp.x:.2f}, {self.latest_wp.y:.2f}) | "
                    f"Aralarındaki Mesafe: {dist:.2f} metre\n"
                    f"-> Matris İndeksi Geldi: [Y(Row): {grid_y}, X(Col): {grid_x}]"
                )

def main(args=None):
    rclpy.init(args=args)
    node = NearestWaypointTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nTest sonlandırıldı, arka plan süreçleri kapatılıyor...")
    finally:
        # Alt C++ node'larını güvenli şekilde öldür
        node.tracker_process.terminate()
        node.finder_process.terminate()
        node.tracker_process.wait()
        node.finder_process.wait()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
