#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
import serial
import time
import math

SERIAL_PORT = "/dev/ttyUSB1"  
BAUD_RATE = 115200

class Scanning(Node):
    def __init__(self):
        super().__init__('multi_line_lidar')
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  
            time.sleep(1) 
            self.get_logger().info(f'Serial connected on {SERIAL_PORT}')
        except serial.SerialException:
            self.get_logger().error(f'Failed to connect to {SERIAL_PORT}')
            self.serial = None

        processing_rate = 0.05
        self.count = 0
        self.direction = 1
        self.latest_scan = None
        self.cloud = []

        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        qos_profile_pc2 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/cloud', qos_profile_pc2)

        self.timer = self.create_timer(processing_rate, self.process)
    
    def scan_callback(self,msg):
        self.latest_scan = msg

    def process(self):
        if self.latest_scan is None:
            self.get_logger().warn('No LiDAR data received yet!')
            return
        for i, range_value in enumerate(self.latest_scan.ranges):
            if self.latest_scan.range_min <= range_value <= self.latest_scan.range_max:
                angle_rad = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
                if 3.14 <= angle_rad <= 6.283:
                    stepper_angle = -math.radians(self.count*1.8)
                else:
                    stepper_angle = math.radians(self.count*1.8)

                x = range_value * math.cos(angle_rad) * math.cos(stepper_angle)
                y = range_value * math.sin(angle_rad) * math.cos(stepper_angle)
                z = range_value * math.sin(stepper_angle)
                
                self.cloud.append((x, y, z))

        cloud_msg = point_cloud2.create_cloud_xyz32(self.latest_scan.header, self.cloud)
        self.pointcloud_publisher.publish(cloud_msg)
        
        self.Stepper(self.direction)

        if(self.count==50):
            self.direction = -1
        if(self.count==-50):
            self.direction = 1

        self.count +=self.direction

    def Stepper(self,dir):
        angle_str = f"{dir}\n"
        self.serial.write(angle_str.encode())  
        self.get_logger().info(f'Sent angle: {dir}')
            
def main():
    rclpy.init()
    node = Scanning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
