#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#commom lib
import math
import numpy as np
import time,os
from time import sleep
from egocar_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        
        
        
        
        #declareparam
        self.declare_parameter("linear",0.4)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",40.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.priorityAngle = 30  # 40
        self.Moving = False
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def car_cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        
        # 添加調試信息
        angle_min_deg = scan_data.angle_min * RAD2DEG
        angle_max_deg = scan_data.angle_max * RAD2DEG
        
        # 過濾無效數據
        ranges[np.isinf(ranges)] = scan_data.range_max
        ranges[np.isnan(ranges)] = scan_data.range_max
        
        offset = 0.5
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

            # 簡化角度檢測邏輯 - 直接檢測前方區域
            if abs(angle) <= self.priorityAngle:  # 前方優先區域
                if ranges[i] < (self.ResponseDist + offset) and ranges[i] > 0.1:
                    frontDistList.append(ranges[i])
                    frontDistIDList.append(angle)
            elif abs(angle) <= self.LaserAngle:  # 擴展檢測區域
                if ranges[i] > 0.1:  # 過濾過近的無效數據
                    minDistList.append(ranges[i])
                    minDistIDList.append(angle)
        if len(frontDistIDList) != 0:
            minDist = min(frontDistList)
            minDistID = frontDistIDList[frontDistList.index(minDist)]
        elif len(minDistList) != 0:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]
        else:
            # 如果沒有檢測到任何有效距離數據，使用默認值或跳過
            self.get_logger().warn("No valid distance data found, skipping this scan")
            return
        if self.Joy_active or self.Switch == True:
            if self.Moving == True:
                self.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        velocity = Twist()
        
        # 添加安全檢查：如果距離太近，停止運動
        if minDist < 0.15:  # 15cm以內停止
            self.get_logger().warn(f"Object too close: {minDist:.2f}m, stopping motion")
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.pub_vel.publish(velocity)
            return
            
        # 確保最小距離不會太小，避免PID計算異常
        if minDist < 0.2:
            minDist = 0.2
            
        if abs(minDist - self.ResponseDist) < 0.1:
            minDist = self.ResponseDist
            
        # PID控制計算
        linear_output = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(minDistID)) / 72, 0)
        
        # 限制線性速度範圍，避免過大的速度
        velocity.linear.x = max(-0.5, min(0.5, linear_output))
        
        # 角度控制
        if minDistID > 0:
            velocity.angular.z = -ang_pid_compute
        else:
            velocity.angular.z = ang_pid_compute
            
        # 限制角速度範圍
        velocity.angular.z = max(-2.0, min(2.0, velocity.angular.z))
        
        if abs(ang_pid_compute) < 0.02:
            velocity.angular.z = 0.0
            
        # 添加調試信息
        if minDist < 0.3:
            self.get_logger().info(f"Close object detected: dist={minDist:.2f}m, angle={minDistID:.1f}°, "
                                   f"linear_vel={velocity.linear.x:.2f}, angular_vel={velocity.angular.z:.2f}")
            
        self.pub_vel.publish(velocity)

def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker_4ROS")
    print ("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.car_cancel()
        laser_tracker.destroy_node()
        rclpy.shutdown()
