#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import sys
import math
import time
import threading
import numpy as np
from math import pi
from time import sleep

# 假設 Rosmaster_Lib.py 與此文件在同一目錄下
from Rosmaster_Lib import Rosmaster

# 匯入 ROS2 標準訊息
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from sensor_msgs.msg import Imu, MagneticField, JointState

# 匯入自訂義訊息與服務 (請確保 egocar_msgs 套件已在您的工作區中)
from egocar_msgs.msg import ArmJoint
from egocar_msgs.srv import RobotArmArray

import traceback

class YahboomcarDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.get_logger().info("節點 'driver_node' 初始化中...")
        
        # --- 參數宣告 (取代 ROS1 的 get_param 和 dynamic_reconfigure) ---
        # 基礎參數
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('xlinear_speed_limit', 1.0)
        self.declare_parameter('ylinear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)

        # PID 參數
        self.declare_parameter('Kp', 1.5)
        self.declare_parameter('Ki', 0.3)
        self.declare_parameter('Kd', 0.2)

        # 速度範圍參數
        self.declare_parameter('linear_max', 1.0)
        self.declare_parameter('linear_min', 0.0)
        self.declare_parameter('angular_max', 5.0)
        self.declare_parameter('angular_min', 0.0)

        # 機械臂角度參數
        self.declare_parameter('SetArmjoint', False)
        self.declare_parameter('joint1', 90.0)
        self.declare_parameter('joint2', 145.0)
        self.declare_parameter('joint3', 0.0)
        self.declare_parameter('joint4', 45.0)
        self.declare_parameter('joint5', 90.0)
        self.declare_parameter('joint6', 30.0)

        # 獲取初始參數值
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.Prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.xlinear_limit = self.get_parameter('xlinear_speed_limit').get_parameter_value().double_value
        self.ylinear_limit = self.get_parameter('ylinear_speed_limit').get_parameter_value().double_value
        self.angular_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # --- 初始化硬體 ---
        # 弧度轉角度
        self.RA2DE = 180 / pi
        try:
            self.car = Rosmaster()
            self.car.set_car_type(2) # 設定為 X3 Plus
            self.get_logger().info("Rosmaster 控制器初始化成功。")
        except Exception as e:
            self.get_logger().error(f"無法初始化 Rosmaster 控制器: {e}")
            self.get_logger().error("請檢查序列埠權限 '/dev/myserial' 是否正確。")
            sys.exit(1)
            
        self.car.create_receive_threading()
        
        # 啟用硬體自動回報功能，讓硬體能回傳實際的速度和感測器數據
        self.car.set_auto_report_state(True, False)
        self.get_logger().info("硬體自動回報功能已啟用。")

        # --- ROS2 通訊介面設定 ---
        # 訂閱者 (Subscribers)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 10)
        self.sub_Buzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 10)
        self.sub_Arm = self.create_subscription(ArmJoint, "TargetAngle", self.Armcallback, 10)

        # 發布者 (Publishers)
        self.ArmPubUpdate = self.create_publisher(ArmJoint, "ArmAngleUpdate", 100)
        self.EdiPublisher = self.create_publisher(Float32, 'edition', 100)
        self.velPublisher = self.create_publisher(Twist, '/vel_raw', 50)
        self.imuPublisher = self.create_publisher(Imu, '/imu/imu_raw', 100)
        self.magPublisher = self.create_publisher(MagneticField, 'mag/mag_raw', 100)
        self.staPublisher = self.create_publisher(JointState, 'joint_states', 100)
        self.volPublisher = self.create_publisher(Float32, 'voltage', 100)

        # 服務 (Services)
        self.srv_armAngle = self.create_service(RobotArmArray, "CurrentAngle", self.srv_Armcallback)

        # 參數回呼 (取代 dynamic_reconfigure)
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # --- 初始化小車狀態 ---
        self.car.set_car_motion(0.0, 0.0, 0.0)
        # 初始機械臂角度
        self.joints = [
            self.get_parameter('joint1').get_parameter_value().double_value,
            self.get_parameter('joint2').get_parameter_value().double_value,
            self.get_parameter('joint3').get_parameter_value().double_value,
            self.get_parameter('joint4').get_parameter_value().double_value,
            self.get_parameter('joint5').get_parameter_value().double_value,
            self.get_parameter('joint6').get_parameter_value().double_value
        ]
        self.car.set_uart_servo_angle_array([int(j) for j in self.joints], 1000)
        self.get_logger().info(f"機械臂初始角度設定為: {self.joints}")

        # 建立一個 50ms (20Hz) 的定時器來發布資料
        self.timer = self.create_timer(0.05, self.pub_data_callback)

        self.get_logger().info("驅動節點初始化完成。")

    def pub_data_callback(self):
        """
        定時器回呼函式，發布小車運動速度、IMU、磁力計、電池電壓與版本等資料
        """
        try:
            imu = Imu()
            twist = Twist()
            battery = Float32()
            edition = Float32()
            mag = MagneticField()

            # 先取資料
            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            mx, my, mz = self.car.get_magnetometer_data()
            vx, vy, angular = self.car.get_motion_data()
            ver = self.car.get_version()  # 可能是 float 或 -1（int）

            # 僅使用 Python float，禁止 int / numpy scalar
            edition.data = float(ver) if isinstance(ver, (int, float)) else 0.0
            battery.data = float(self.car.get_battery_voltage())

            # IMU
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = float(ax)
            imu.linear_acceleration.y = float(ay)
            imu.linear_acceleration.z = float(az)
            imu.angular_velocity.x = float(gx)
            imu.angular_velocity.y = float(gy)
            imu.angular_velocity.z = float(gz)

            # 磁力計
            mag.header.stamp = self.get_clock().now().to_msg()
            mag.header.frame_id = self.imu_link
            mag.magnetic_field.x = float(mx)
            mag.magnetic_field.y = float(my)
            mag.magnetic_field.z = float(mz)

            # 速度
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(angular)

            # 發布
            self.imuPublisher.publish(imu)
            self.magPublisher.publish(mag)
            self.velPublisher.publish(twist)
            self.volPublisher.publish(battery)
            self.EdiPublisher.publish(edition)

            self.joints_states_update()

        except Exception as e:
            # 額外印出型別便於查錯
            self.get_logger().error(f"發布資料時出錯: {e} "
                                    f"(types: ver={type(ver)}, v={type(vx)}, "
                                    f"ax={type(ax)}, mx={type(mx)})")

    def joints_states_update(self):
        """
        發布關節狀態
        """
        try:
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = "joint_states"
            
            # 根據是否有前綴來設定關節名稱
            if len(self.Prefix) == 0:
                state.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"]
            else:
                state.name = [self.Prefix + "/arm_joint1", self.Prefix + "/arm_joint2",
                              self.Prefix + "/arm_joint3", self.Prefix + "/arm_joint4",
                              self.Prefix + "/arm_joint5", self.Prefix + "/grip_joint"]
            
            joints_rad = [0.0] * 6
            current_joints = self.joints[:]
            
            # 確保所有關節值都是有效的數值
            for i in range(len(current_joints)):
                try:
                    current_joints[i] = float(current_joints[i])
                except (ValueError, TypeError):
                    current_joints[i] = 90.0  # 預設值
            
            # 將舵機角度轉換為弧度，以符合 JointState 的要求
            # 注意: 這裡的轉換關係需要根據您的 URDF 模型進行精確調整
            # 1-5號臂，中心點90度，範圍0-180
            for i in range(5):
                joints_rad[i] = math.radians(current_joints[i] - 90)
            # 6號爪子，範圍30-180 -> 0-90
            gripper_angle = np.interp(current_joints[5], [30, 180], [0, 90])
            joints_rad[5] = math.radians(gripper_angle - 90) # 假設中心點為90

            state.position = joints_rad
            self.staPublisher.publish(state)
        except Exception as e:
            self.get_logger().error(f"更新關節狀態時出錯: {e}")

    def cmd_vel_callback(self, msg):
        """
        小車運動控制，訂閱者回呼函式
        """
        if not isinstance(msg, Twist): return
        
        # 根據參數限制速度
        vx = np.clip(msg.linear.x, -self.xlinear_limit, self.xlinear_limit)
        vy = np.clip(msg.linear.y, -self.ylinear_limit, self.ylinear_limit)
        angular = np.clip(msg.angular.z, -self.angular_limit, self.angular_limit)

        self.car.set_car_motion(vx, vy, angular)

    def Armcallback(self, msg):
        """
        機械臂控制回呼
        """
        if not isinstance(msg, ArmJoint): return
        arm_joint = ArmJoint()
        try:
            # 控制一組關節
            if len(msg.joints) != 0:
                # 確保所有關節值都是有效的數值
                valid_joints = []
                for j in msg.joints:
                    try:
                        valid_joints.append(float(j))
                    except (ValueError, TypeError):
                        valid_joints.append(90.0)  # 預設值
                
                self.joints = valid_joints
                int_joints = [int(j) for j in valid_joints]
                self.car.set_uart_servo_angle_array(int_joints, msg.run_time)
                arm_joint.joints = valid_joints
                self.ArmPubUpdate.publish(arm_joint)
            # 控制單一關節
            else:
                try:
                    angle = float(msg.angle)
                    self.joints[msg.id - 1] = angle
                    self.car.set_uart_servo_angle(msg.id, int(angle), msg.run_time)
                    arm_joint.id = msg.id
                    arm_joint.angle = angle
                    self.ArmPubUpdate.publish(arm_joint)
                except (ValueError, TypeError) as e:
                    self.get_logger().warn(f"關節角度無效，忽略指令: {e}")
                    return
        except Exception as e:
            self.get_logger().error(f"機械臂控制時出錯: {e}")
            return
            
        sleep(0.01)
        self.joints_states_update()

    def srv_Armcallback(self, request, response):
        """
        服务端，提供機械臂當前關節角度
        """
        try:
            joints = self.car.get_uart_servo_angle_array()
            if joints:
                # 確保所有關節角度都是有效的浮點數
                response.angles = []
                for j in joints:
                    try:
                        if j is not None:
                            response.angles.append(float(j))
                        else:
                            response.angles.append(0.0)
                    except (ValueError, TypeError) as e:
                        self.get_logger().warn(f"關節角度轉換失敗，使用預設值: {e}")
                        response.angles.append(0.0)
            else:
                # 如果無法獲取關節角度，使用預設值
                response.angles = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]  # 預設角度
        except Exception as e:
            self.get_logger().error(f"獲取機械臂角度時出錯: {e}")
            response.angles = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]  # 預設角度
        return response

    def RGBLightcallback(self, msg):
        """
        RGB 流水燈控制
        """
        if not isinstance(msg, Int32): return
        self.car.set_colorful_effect(msg.data, 6, parm=1)
        sleep(0.01)

    def Buzzercallback(self, msg):
        """
        蜂鳴器控制
        """
        if not isinstance(msg, Bool): return
        if msg.data:
            self.car.set_beep(1)
        else:
            self.car.set_beep(0)
        sleep(0.01)
        
    def parameters_callback(self, params):
        """
        參數動態調整回呼函式
        """
        successful = True
        for param in params:
            param_name = param.name
            param_value = param.value
            
            self.get_logger().info(f"參數 '{param_name}' 被修改為: {param_value}")
            
            # 根據參數名稱執行相應操作
            if param_name == 'Kp' or param_name == 'Ki' or param_name == 'Kd':
                # 在 ROS2 中，我們需要獲取所有 PID 參數才能設定
                kp = self.get_parameter('Kp').get_parameter_value().double_value
                ki = self.get_parameter('Ki').get_parameter_value().double_value
                kd = self.get_parameter('Kd').get_parameter_value().double_value
                # 如果是剛觸發的參數，它的值還沒被 set
                if param_name == 'Kp': kp = param_value
                if param_name == 'Ki': ki = param_value
                if param_name == 'Kd': kd = param_value
                self.car.set_pid_param(kp, ki, kd)
                self.get_logger().info(f"設定 PID 參數為: Kp={kp}, Ki={ki}, Kd={kd}")
            
            elif param_name == 'SetArmjoint' and param_value == True:
                # 當 SetArmjoint 被設為 True 時，使用參數中的角度值設定機械臂
                joints_to_set = [
                    int(self.get_parameter('joint1').get_parameter_value().double_value),
                    int(self.get_parameter('joint2').get_parameter_value().double_value),
                    int(self.get_parameter('joint3').get_parameter_value().double_value),
                    int(self.get_parameter('joint4').get_parameter_value().double_value),
                    int(self.get_parameter('joint5').get_parameter_value().double_value),
                    int(self.get_parameter('joint6').get_parameter_value().double_value)
                ]
                self.car.set_uart_servo_angle_array(joints_to_set, run_time=1000)
                self.joints = [float(j) for j in joints_to_set]
                self.get_logger().info(f"透過參數設定機械臂角度為: {joints_to_set}")
                
                # 重設 SetArmjoint 為 False，使其成為一個觸發器
                # 建立一個新的執行緒來延遲設定參數，避免在回呼中直接設定
                def reset_param():
                    time.sleep(0.5)
                    self.set_parameters([Parameter('SetArmjoint', Parameter.Type.BOOL, False)])
                threading.Thread(target=reset_param).start()

        return SetParametersResult(successful=successful)

    def cancel_node(self):
        """
        節點關閉時執行的清理工作
        """
        self.get_logger().info("正在關閉節點...")
        self.timer.cancel()
        if hasattr(self, 'car') and self.car:
            self.car.set_car_motion(0, 0, 0)
            self.get_logger().info("已停止小車運動。")
        self.destroy_subscription(self.sub_cmd_vel)
        self.destroy_subscription(self.sub_RGBLight)
        self.destroy_subscription(self.sub_Buzzer)
        self.destroy_subscription(self.sub_Arm)
        self.get_logger().info("節點已清理。")

def main(args=sys.argv):
    rclpy.init(args=args)
    driver_node = None
    try:
        driver_node = YahboomcarDriver()
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if driver_node:
            driver_node.get_logger().error(f"節點執行時發生未預期錯誤: {e}")
            driver_node.get_logger().error(traceback.format_exc())
    finally:
        if driver_node:
            driver_node.cancel_node()
            driver_node.destroy_node()
        rclpy.shutdown()
        print("ROS2 節點已完全關閉。")

if __name__ == '__main__':
    main()
