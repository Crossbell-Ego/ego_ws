#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Common libraries
import math
import numpy as np
import time
import os
from egocar_laser.common import SinglePID

# Constants
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class LaserAvoidanceNode(Node):
    """
    激光避障節點 - 使用 LiDAR 數據進行實時避障
    """
    def __init__(self, name='laser_avoidance_node'):
        super().__init__(name)
        
        # 初始化參數
        self._declare_parameters()
        
        # 狀態變量
        self.right_warning = 0
        self.left_warning = 0 
        self.front_warning = 0
        self.joy_active = False
        self.moving = False
        self.last_command_time = time.time()
        
        # PID 控制器
        self.pid_controller = SinglePID(0.2, 0.0, 0.15)
        
        # 創建訂閱者
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.joy_state_sub = self.create_subscription(
            Bool, '/JoyState', self.joy_state_callback, 10)
            
        # 創建發布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定時器
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('激光避障節點初始化成功')

    def _declare_parameters(self):
        """聲明 ROS2 參數"""
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6) 
        self.declare_parameter('laser_angle', 30.0)
        self.declare_parameter('response_distance', 0.40)
        self.declare_parameter('obstacle_valid_angle', 4.0)
        self.declare_parameter('enable_avoidance', True)
        self.declare_parameter('avoidance_delay', 0.09)  # 避障延遲時間（秒）
    def _update_parameters(self):
        """更新參數值"""
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.laser_angle = self.get_parameter('laser_angle').get_parameter_value().double_value
        self.response_distance = self.get_parameter('response_distance').get_parameter_value().double_value
        self.obstacle_valid_angle = self.get_parameter('obstacle_valid_angle').get_parameter_value().double_value
        self.enable_avoidance = self.get_parameter('enable_avoidance').get_parameter_value().bool_value
        self.avoidance_delay = self.get_parameter('avoidance_delay').get_parameter_value().double_value

    def control_loop(self):
        """主控制循環"""
        self._update_parameters()
        
        # 如果手柄控制激活或避障關閉，停止避障
        if self.joy_active or not self.enable_avoidance:
            if self.moving:
                self._publish_stop_command()
                self.moving = False
            return

    def joy_state_callback(self, msg):
        """手柄狀態回調"""
        if isinstance(msg, Bool):
            self.joy_active = msg.data
            if self.joy_active:
                self.get_logger().info('手柄控制已啟動，停止避障功能')

    def _publish_stop_command(self):
        """發布停止命令"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _publish_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """發布速度命令"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        self.last_command_time = time.time()
    def laser_callback(self, scan_data):
        """激光掃描數據回調"""
        if not isinstance(scan_data, LaserScan):
            return
            
        # 轉換為 numpy 數組以提高性能
        ranges = np.array(scan_data.ranges)
        
        # 過濾無效數據
        ranges[np.isinf(ranges)] = scan_data.range_max
        ranges[np.isnan(ranges)] = scan_data.range_max
        
        # 重置警告計數
        self.right_warning = 0
        self.left_warning = 0
        self.front_warning = 0
        
        # 按照原始檔案的邏輯進行角度檢測
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            # 右側檢測區域: -10 到 -10-LaserAngle 度
            if -10 > angle > -10 - self.laser_angle:
                if ranges[i] < self.response_distance:
                    self.right_warning += 1
                    
            # 左側檢測區域: 10+LaserAngle 到 10 度
            if 10 + self.laser_angle > angle > 10:
                if ranges[i] < self.response_distance:
                    self.left_warning += 1
                    
            # 前方檢測區域: -10 到 10 度
            if abs(angle) < 10:
                if ranges[i] <= self.response_distance:
                    self.front_warning += 1
        
        # 計算有效障礙物數量閾值（按照原始檔案的計算方式）
        self.valid_num = int(self.obstacle_valid_angle / (scan_data.angle_increment * RAD2DEG))
        
        # 執行避障決策
        self._execute_avoidance_strategy()

    def _execute_avoidance_strategy(self):
        """執行避障策略"""
        if self.joy_active or not self.enable_avoidance:
            return
            
        self.moving = True
        
        # 使用在 laser_callback 中計算的 valid_num
        valid_num = getattr(self, 'valid_num', 3)  # 如果沒有計算出來則使用預設值
        
        # 按照原始檔案的 10 種避障情況
        if self.front_warning > valid_num and self.left_warning > valid_num and self.right_warning > valid_num:
            # 情況 1: 前方、左側、右側都有障礙物 - 後退並右轉
            self.get_logger().info('1, 左右都有障礙物，右轉')
            self._publish_velocity_command(-0.15, -self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning > valid_num and self.left_warning <= valid_num and self.right_warning > valid_num:
            # 情況 2: 前方和右側有障礙物 - 左轉
            self.get_logger().info('2, 中間右側有障礙物，左轉')
            self._publish_velocity_command(self.linear_speed, 0.0)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning > valid_num and self.left_warning > valid_num and self.right_warning <= valid_num:
            # 情況 4: 前方和左側有障礙物 - 右轉
            self.get_logger().info('4. 中間左側有障礙物，右轉')
            self._publish_velocity_command(0.0, -self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
                
        elif self.front_warning > valid_num and self.left_warning < valid_num and self.right_warning < valid_num:
            # 情況 6: 僅前方有障礙物 - 左轉
            self.get_logger().info('6, 中間有障礙物，左轉')
            self._publish_velocity_command(0.0, self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning < valid_num and self.left_warning > valid_num and self.right_warning > valid_num:
            # 情況 7: 左側和右側有障礙物 - 右轉
            self.get_logger().info('7. 左右有障礙物，右轉')
            self._publish_velocity_command(0.0, -self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning < valid_num and self.left_warning > valid_num and self.right_warning <= valid_num:
            # 情況 8: 左側有障礙物 - 右轉
            self.get_logger().info('8, 左側有障礙物，右轉')
            self._publish_velocity_command(0.0, -self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning < valid_num and self.left_warning <= valid_num and self.right_warning > valid_num:
            # 情況 9: 右側有障礙物 - 左轉
            self.get_logger().info('9, 右側有障礙物，左轉')
            self._publish_velocity_command(0.0, self.angular_speed)
            time.sleep(self.avoidance_delay)  # 動態延遲時間
            
        elif self.front_warning <= valid_num and self.left_warning <= valid_num and self.right_warning <= valid_num:
            # 情況 10: 無障礙物 - 前進
            self.get_logger().info('10, 無障礙物，前進')
            self._publish_velocity_command(self.linear_speed, 0.0)
            time.sleep(self.avoidance_delay)  # 動態延遲時間

def main():
    """主函數"""
    rclpy.init()
    
    try:
        # 創建激光避障節點
        laser_avoidance_node = LaserAvoidanceNode("laser_avoidance_node")
        laser_avoidance_node.get_logger().info("激光避障節點啟動成功")
        
        # 運行節點
        rclpy.spin(laser_avoidance_node)
        
    except KeyboardInterrupt:
        laser_avoidance_node.get_logger().info("正在關閉激光避障節點...")
    except Exception as e:
        laser_avoidance_node.get_logger().error(f"激光避障節點發生錯誤: {e}")
    finally:
        # 清理資源
        if 'laser_avoidance_node' in locals():
            laser_avoidance_node._publish_stop_command()
            laser_avoidance_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
