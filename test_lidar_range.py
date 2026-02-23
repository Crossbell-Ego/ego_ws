#!/usr/bin/env python3
"""
測試雷達角度範圍腳本
檢查雷達是否只輸出前方60度的數據
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarRangeTestNode(Node):
    def __init__(self):
        super().__init__('lidar_range_test')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('雷達角度範圍測試節點已啟動...')
        self.get_logger().info('等待雷達數據...')

    def scan_callback(self, msg):
        """處理雷達掃描數據"""
        # 計算角度範圍
        angle_min_deg = math.degrees(msg.angle_min)
        angle_max_deg = math.degrees(msg.angle_max)
        angle_increment_deg = math.degrees(msg.angle_increment)
        
        # 統計有效數據點
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        
        self.get_logger().info(f'=== 雷達角度範圍檢測結果 ===')
        self.get_logger().info(f'最小角度: {angle_min_deg:.1f}°')
        self.get_logger().info(f'最大角度: {angle_max_deg:.1f}°')
        self.get_logger().info(f'角度增量: {angle_increment_deg:.2f}°')
        self.get_logger().info(f'總掃描角度: {angle_max_deg - angle_min_deg:.1f}°')
        self.get_logger().info(f'總數據點數: {len(ranges)}')
        self.get_logger().info(f'有效數據點數: {len(valid_ranges)}')
        self.get_logger().info(f'數據覆蓋率: {len(valid_ranges)/len(ranges)*100:.1f}%')
        
        # 檢查是否符合60度範圍設定
        expected_range = 60.0  # 期望的角度範圍
        actual_range = angle_max_deg - angle_min_deg
        
        if abs(actual_range - expected_range) < 1.0:  # 允許1度誤差
            self.get_logger().info(f'✅ 角度範圍設定正確: {actual_range:.1f}° (期望: {expected_range}°)')
        else:
            self.get_logger().warn(f'❌ 角度範圍不符合期望: {actual_range:.1f}° (期望: {expected_range}°)')
        
        self.get_logger().info('=' * 40)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRangeTestNode()
    
    try:
        # 只運行一次檢測
        rclpy.spin_once(node, timeout_sec=5.0)
        rclpy.spin_once(node, timeout_sec=5.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()