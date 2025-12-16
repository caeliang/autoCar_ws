#!/usr/bin/env python3
"""
Prius Arabasını Gazebo'da Kontrol Et - Test Scripti
Topic: /prius_hybrid_123/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class PriusController(Node):
    def __init__(self):
        super().__init__('prius_simple_mover')
        
        self.publisher_ = self.create_publisher(
            Twist,
            '/prius_hybrid_123/cmd_vel',
            10
        )
        
        self.get_logger().info('🚗 Prius Kontrol Başlatıldı')
        self.get_logger().info('Topic: /prius_hybrid_123/cmd_vel')
    
    def move_forward(self, speed=5.0, duration=5.0):
        """Arabanın ileri gitmesi"""
        self.get_logger().info(f'➡️  İleri gidiyor: {speed} m/s, {duration} saniye')
        self._send_velocity(speed, 0.0, duration)
    
    def move_backward(self, speed=3.0, duration=5.0):
        """Arabanın geri gitmesi"""
        self.get_logger().info(f'⬅️  Geri gidiyor: {speed} m/s, {duration} saniye')
        self._send_velocity(-speed, 0.0, duration)
    
    def turn_left(self, angular_vel=1.0, duration=3.0):
        """Arabanın sola dönmesi"""
        self.get_logger().info(f'🔄 Sola dönüyor: {angular_vel} rad/s, {duration} saniye')
        self._send_velocity(0.0, angular_vel, duration)
    
    def turn_right(self, angular_vel=1.0, duration=3.0):
        """Arabanın sağa dönmesi"""
        self.get_logger().info(f'🔄 Sağa dönüyor: {angular_vel} rad/s, {duration} saniye')
        self._send_velocity(0.0, -angular_vel, duration)
    
    def stop(self):
        """Arabanın durması"""
        self.get_logger().info('🛑 Durduruluyor...')
        self._send_velocity(0.0, 0.0, 0.5)
    
    def _send_velocity(self, linear, angular, duration):
        """Hız komutu gönder"""
        msg = Twist()
        msg.linear.x = linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    controller = PriusController()
    
    try:
        print("\n" + "="*50)
        print("PRIUS ARAÇ KONTROL TESTI")
        print("="*50)
        
        # Test sequence
        time.sleep(1)
        
        print("\n1️⃣  Test 1: İleri hareket (5 m/s, 5 saniye)")
        controller.move_forward(5.0, 5.0)
        time.sleep(1)
        
        print("\n2️⃣  Test 2: Geri hareket (3 m/s, 3 saniye)")
        controller.move_backward(3.0, 3.0)
        time.sleep(1)
        
        print("\n3️⃣  Test 3: Sola dönüş (2 rad/s, 4 saniye)")
        controller.turn_left(2.0, 4.0)
        time.sleep(1)
        
        print("\n4️⃣  Test 4: Sağa dönüş (2 rad/s, 4 saniye)")
        controller.turn_right(2.0, 4.0)
        time.sleep(1)
        
        print("\n5️⃣  Test 5: Kombinasyon (2 m/s + 0.5 rad/s sola, 3 saniye)")
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.5
        start = time.time()
        while (time.time() - start) < 3.0:
            controller.publisher_.publish(msg)
            time.sleep(0.05)
        controller.stop()
        
        print("\n✅ Tüm testler tamamlandı!")
        print("="*50 + "\n")
        
    except KeyboardInterrupt:
        print("\n⏹️  Durduruldu")
    except Exception as e:
        print(f"❌ Hata: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
