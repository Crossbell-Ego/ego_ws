#!/usr/bin/env python3
"""
測試 madgwick 濾波器節點架構的腳本
發布模擬 IMU 數據來驗證濾波器工作正常
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class TestIMUPublisher(Node):
    def __init__(self):
        super().__init__('test_imu_publisher')
        
        # 發布到 /imu/data_raw 主題
        self.publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # 每隔 0.01 秒發布一次 (100Hz)
        self.timer = self.create_timer(0.01, self.publish_imu_data)
        
        self.counter = 0
        self.get_logger().info('測試 IMU 發布器啟動，發布模擬數據到 /imu/data_raw')

    def publish_imu_data(self):
        msg = Imu()
        
        # 設置時間戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # 模擬角速度 (rad/s) - 緩慢旋轉
        t = self.counter * 0.01
        msg.angular_velocity.x = 0.1 * math.sin(t)
        msg.angular_velocity.y = 0.1 * math.cos(t)  
        msg.angular_velocity.z = 0.05 * math.sin(2 * t)
        
        # 模擬線性加速度 (m/s^2) - 包含重力
        msg.linear_acceleration.x = 0.2 * math.sin(t * 0.5)
        msg.linear_acceleration.y = 0.2 * math.cos(t * 0.5)
        msg.linear_acceleration.z = 9.81 + 0.1 * math.sin(t * 2)  # 重力 + 小擾動
        
        # 設置協方差矩陣
        msg.angular_velocity_covariance = [0.01] * 9
        msg.linear_acceleration_covariance = [0.01] * 9
        msg.orientation_covariance = [-1.0] * 9  # 未知方向
        
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 100 == 0:  # 每秒記錄一次
            self.get_logger().info(f'已發布 {self.counter} 條模擬 IMU 數據')

def main(args=None):
    rclpy.init(args=args)
    
    test_publisher = TestIMUPublisher()
    
    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        test_publisher.get_logger().info('測試 IMU 發布器停止')
    finally:
        test_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()