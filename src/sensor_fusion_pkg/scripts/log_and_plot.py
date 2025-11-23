#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import csv
import os

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        self.odom_x, self.odom_y = [], []
        self.fused_x, self.fused_y = [], []

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/fused_odom', self.fused_cb, 10)

        # output file paths
        self.odom_file = os.path.join(os.getcwd(), "odom_data.csv")
        self.fused_file = os.path.join(os.getcwd(), "fused_data.csv")

    def odom_cb(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
        self.plot_data()

    def fused_cb(self, msg):
        self.fused_x.append(msg.pose.pose.position.x)
        self.fused_y.append(msg.pose.pose.position.y)
        self.plot_data()

    def plot_data(self):
        plt.clf()
        plt.plot(self.odom_x, self.odom_y, label='Odom', color='blue')
        plt.plot(self.fused_x, self.fused_y, label='Fused', linestyle='--', color='red')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Odometry vs Fused Odometry')
        plt.legend()
        plt.pause(0.01)

    def save_data(self):
        # Save odom
        with open(self.odom_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x','y'])
            writer.writerows(zip(self.odom_x, self.odom_y))
        self.get_logger().info(f"Odom data saved to {self.odom_file}")

        # Save fused
        with open(self.fused_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x','y'])
            writer.writerows(zip(self.fused_x, self.fused_y))
        self.get_logger().info(f"Fused data saved to {self.fused_file}")


def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    plt.ion()
    plt.show()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
