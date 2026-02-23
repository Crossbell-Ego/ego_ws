#!/usr/bin/env python3
# encoding: utf-8

import os
import time
import getpass
import threading
import sys

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from time import sleep

# Message and Service Types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Bool
from actionlib_msgs.msg import GoalID
# 使用正確的 egocar_msgs 套件
from egocar_msgs.msg import ArmJoint
from egocar_msgs.srv import RobotArmArray

class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Yahboom Joy Teleop Node for ROS2 is starting...")

        # 狀態變數 (State Variables)
        self.Joy_active = False
        self.Buzzer_active = False
        self.gripper_active = True
        self.getArm_active = True
        self.loop_active = True
        self.RGBLight_index = 0
        self.geta_arm_index = 0
        self.cancel_time = time.time()
        self.linear_Gear = 1
        self.angular_Gear = 1
        self.arm_speed_factor = 0.4 # 新增這一行，數值越小，速度越慢 (可設定在 0.1 ~ 1.0 之間)

        # 初始化機械臂角度
        self.armjoint = ArmJoint()
        self.armjoint.run_time = 150  # ROS1: 10, ROS2單位通常是秒，這裡可能需要調整
         # 將此值加大可以讓手臂運動更慢、更平滑。建議值 100-300
        self.arm_joints = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]

        # 參數宣告 (Parameter Declaration)
        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.get_logger().info(f"Speed limits: x={self.xspeed_limit}, y={self.yspeed_limit}, angular={self.angular_speed_limit}")

        # 建立發布者 (Publishers)
        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(Int32, "RGBLight", 10)
        self.pub_Arm = self.create_publisher(ArmJoint, "TargetAngle", 10)

        # 建立訂閱者 (Subscribers)
        self.sub_Joy = self.create_subscription(Joy, 'joy', self.buttonCallback, 10)
        self.sub_Arm = self.create_subscription(ArmJoint, "ArmAngleUpdate", self.Armcallback, 10)

        # 建立服務客戶端 (Service Client)
        self.srv_arm_client = self.create_client(RobotArmArray, "CurrentAngle")

        # 檢查使用者
        self.user_name = getpass.getuser()
        self.get_logger().info(f"Current user: {self.user_name}")

    def arm_ctrl_thread(self, id, direction):
        while self.loop_active:
            self.arm_joints[id - 1] += direction * self.arm_speed_factor
            # Boundary checks
            if id == 5:
                self.arm_joints[id - 1] = max(0, min(270, self.arm_joints[id - 1]))
            elif id == 6:
                self.arm_joints[id - 1] = max(30, min(180, self.arm_joints[id - 1]))
            else:
                self.arm_joints[id - 1] = max(0, min(180, self.arm_joints[id - 1]))

            self.armjoint.id = id
            # 在 ROS2 中，msg 的欄位名稱必須與 .msg 文件中定義的完全一樣
            self.armjoint.angle = float(self.arm_joints[id - 1])
            self.pub_Arm.publish(self.armjoint)
            sleep(0.03)

    def pub_armjoint(self, id, direction):
        self.loop_active = True
        arm_thread = threading.Thread(target=self.arm_ctrl_thread, args=(id, direction))
        arm_thread.setDaemon(True)
        arm_thread.start()

    def Armcallback(self, msg):
        if not isinstance(msg, ArmJoint): return
        if len(msg.joints) != 0:
            self.arm_joints = list(msg.joints)
        else:
            self.arm_joints[msg.id - 1] = msg.angle

    def srv_arm_request(self):
        """非同步請求當前的機械臂角度"""
        if not self.srv_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "CurrentAngle" not available, skipping initial arm angle fetch.')
            self.getArm_active = False
            return

        request = RobotArmArray.Request()
        request.apply = "GetArmJoints"

        self.future = self.srv_arm_client.call_async(request)
        self.future.add_done_callback(self.srv_arm_response_callback)

    def srv_arm_response_callback(self, future):
        try:
            response = future.result()
            if response.angles:
                valid_angles = [angle for angle in response.angles if angle != -1]
                if len(valid_angles) == len(self.arm_joints):
                    self.arm_joints = [float(a) for a in response.angles]
                    self.get_logger().info(f"Successfully fetched initial arm joints: {self.arm_joints}")
                else:
                    self.get_logger().warn("Fetched arm angles seem invalid, using default.")
            else:
                 self.get_logger().warn("Service 'CurrentAngle' returned empty angles.")
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')
        finally:
            # 無論成功或失敗，都標記為已獲取，避免重複請求
            self.getArm_active = False

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return

        # 首次回調時，非同步獲取機械臂初始角度
        if self.getArm_active:
            self.srv_arm_request()
            return # 等待服務回應後再處理按鍵

        # 根據搖桿按鈕數量判斷是哪種模式
        # 注意: 這裡的 button 數量判斷可能需要根據您的實際手把調整
        if len(joy_data.buttons) >= 15:
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        # 根據您之前提供的 /joy 訊息，您的搖桿沒有 axes[9]
        # 我們將其改為 'START' 按鈕，即 buttons[11] (假設)
        # R2 按下時 axes[4] 為 -1.0
        if joy_data.axes[4] <= -0.9: self.cancel_nav()
        
        # 停止機械臂連續移動
        # 這裡的邏輯是當所有相關按鍵都放開時，停止執行緒
        if joy_data.buttons[0] == 0 and joy_data.buttons[1] == 0 and \
           joy_data.buttons[3] == 0 and joy_data.buttons[4] == 0 and \
           joy_data.buttons[6] == 0 and joy_data.axes[5] > -0.9 and \
           joy_data.axes[6] == 0 and joy_data.axes[7] == 0:
            self.loop_active = False
        else:
            # 機械臂控制 (請根據您的手把重新對應)
            # Arm control (Please remap according to your joy)
            if joy_data.buttons[3] == 1: self.pub_armjoint(1, -1)  # X
            if joy_data.buttons[1] == 1: self.pub_armjoint(1, 1)   # B
            if joy_data.buttons[0] == 1: self.pub_armjoint(2, -1)  # A
            if joy_data.buttons[4] == 1: self.pub_armjoint(2, 1)   # Y
            if joy_data.axes[6] != 0: self.pub_armjoint(3, -joy_data.axes[6])
            if joy_data.axes[7] != 0: self.pub_armjoint(4, joy_data.axes[7])
            
            # Gripper / 5th axis toggle
            if joy_data.buttons[10] == 1: 
                # 增加延遲避免短時間內重複觸發
                if (time.time() - self.cancel_time) > 0.5:
                    self.gripper_active = not self.gripper_active
                    self.get_logger().info(f"Gripper mode: {'Active' if self.gripper_active else 'Inactive'}")
                    self.cancel_time = time.time()

            # L2 and L1 control
            if self.gripper_active:
                if joy_data.axes[5] <= -0.9: self.pub_armjoint(6, -1) # L2
                if joy_data.buttons[6] == 1: self.pub_armjoint(6, 1) # L1
            else:
                if joy_data.axes[5] <= -0.9: self.pub_armjoint(5, -1) # L2
                if joy_data.buttons[6] == 1: self.pub_armjoint(5, 1) # L1

        # RGBLight
        if joy_data.buttons[7] == 1:
            if (time.time() - self.cancel_time) > 0.5:
                light_msg = Int32()
                light_msg.data = self.RGBLight_index
                self.pub_RGBLight.publish(light_msg)
                self.RGBLight_index = (self.RGBLight_index + 1) % 7 # 0-6
                self.cancel_time = time.time()

        # Buzzer
        if joy_data.buttons[11] == 1:
            if (time.time() - self.cancel_time) > 0.5:
                self.Buzzer_active = not self.Buzzer_active
                buzzer_msg = Bool()
                buzzer_msg.data = self.Buzzer_active
                self.pub_Buzzer.publish(buzzer_msg)
                self.cancel_time = time.time()

        # Gear control
        if joy_data.buttons[13] == 1: # 左搖桿按下
            if (time.time() - self.cancel_time) > 0.5:
                if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
                elif self.linear_Gear < 0.4: self.linear_Gear = 2.0 / 3
                else: self.linear_Gear = 1.0
                self.get_logger().info(f"Linear Gear changed to: {self.linear_Gear:.2f}")
                self.cancel_time = time.time()

        if joy_data.buttons[14] == 1: # 右搖桿按下
            if (time.time() - self.cancel_time) > 0.5:
                if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
                elif self.angular_Gear < 0.3: self.angular_Gear = 1.0 / 2
                elif self.angular_Gear < 0.6: self.angular_Gear = 3.0 / 4
                else: self.angular_Gear = 1.0
                self.get_logger().info(f"Angular Gear changed to: {self.angular_Gear:.2f}")
                self.cancel_time = time.time()

        # Motion control
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        # 只有在遙控模式啟動時才發布速度
        if self.Joy_active:
            self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        # PC模式的邏輯與Jetson模式類似，但按鍵對應不同
        # 您可以參照user_jetson的修改方式，自行對應PC手把的按鍵
        self.get_logger().info("PC joy stick is not fully configured yet.")
        pass

    def filter_data(self, value):
        return value if abs(value) > 0.2 else 0.0

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            self.get_logger().info(f"Joy control {'activated' if self.Joy_active else 'deactivated'}.")
            
            # 發布狀態
            state_msg = Bool()
            state_msg.data = self.Joy_active
            self.pub_JoyState.publish(state_msg)
            
            # 關閉蜂鳴器和取消導航目標
            buzzer_msg = Bool()
            buzzer_msg.data = False
            self.pub_Buzzer.publish(buzzer_msg)
            self.pub_goal.publish(GoalID())
            
            # 如果是停止遙控，發送停止指令
            if not self.Joy_active:
                self.pub_cmdVel.publish(Twist())
                
            self.cancel_time = now_time

def main(args=None):
    rclpy.init(args=args)
    joy_node = JoyTeleop('joy_ctrl')
    try:
        rclpy.spin(joy_node)
    except KeyboardInterrupt:
        joy_node.get_logger().info("Shutting down joy node.")
    finally:
        # 清理並關閉
        joy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
