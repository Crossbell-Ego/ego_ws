#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 導入必要的標準函式庫
import sys
import math
import threading
from time import sleep

# 導入您的底層硬體控制庫
from Rosmaster_Lib import Rosmaster

# 導入 ROS2 必要的函式庫
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from sensor_msgs.msg import Imu, MagneticField, JointState

class YahboomcarDriver(Node):
    """
    Yahboom 公司 Rosmaster 系列車型的 ROS2 驅動節點。
    這個節點會：
    1. 初始化與底層控制板的序列通訊。
    2. 訂閱 /cmd_vel 主題，接收速度指令。
    3. 將速度指令轉換並發送到底層控制板，驅動馬達。
    4. 定期從底層讀取感測器數據（如 IMU、電壓、里程計），並發佈到對應的 ROS 主題。
    """
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("節點 'yahboomcar_driver' 初始化開始...")

        # === 參數宣告 ===
        # 宣告並獲取參數。這些參數可以在啟動時由 launch 文件動態配置。
        self.declare_parameter('car_type', 'x3plus')
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value

        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value

        self.declare_parameter('com_port', '/dev/ttyUSB0')
        self.com_port = self.get_parameter('com_port').get_parameter_value().string_value
        
        # 宣告並獲取速度限制參數
        self.declare_parameter('xlinear_limit', 0.7)  # X3Plus 的最大前後速度 (m/s)
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value

        self.declare_parameter('ylinear_limit', 0.7)  # X3Plus 的最大橫移速度 (m/s)
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value

        self.declare_parameter('angular_limit', 3.2)  # X3Plus 的最大旋轉速度 (rad/s)
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value

        self.get_logger().info(f"車型: {self.car_type}, 序列埠: {self.com_port}")
        self.get_logger().info(f"速度限制: x-linear={self.xlinear_limit}, y-linear={self.ylinear_limit}, angular={self.angular_limit}")

        # === 初始化 Rosmaster 控制庫 ===
        # 根據車型名稱找到對應的 ID
        car_type_map = {'x3': 1, 'x3plus': 2, 'r2': 5}
        car_id = car_type_map.get(self.car_type.lower(), 2) # 預設為 x3plus

        # 實例化 Rosmaster 物件，並傳入正確的車型 ID 和序列埠
        try:
            self.car = Rosmaster(car_type=car_id, com=self.com_port, debug=False)
            # 啟動一個獨立執行緒來持續接收來自下位機的數據
            self.car.create_receive_threading()
            self.get_logger().info("成功連接到底層控制器並啟動接收執行緒。")
        except Exception as e:
            self.get_logger().error(f"無法初始化 Rosmaster 控制器: {e}")
            self.get_logger().error("請檢查序列埠 '{self.com_port}' 是否正確，以及是否有讀寫權限。")
            rclpy.shutdown()
            sys.exit(1)

        # === 訂閱者 (Subscribers) ===
        # 訂閱 'cmd_vel' 主題，當有 Twist 訊息發佈時，呼叫 cmd_vel_callback 函式
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # 訂閱 RGB 燈效控制
        self.sub_RGBLight = self.create_subscription(Int32, 'RGBLight', self.RGBLight_callback, 10)
        # 訂閱蜂鳴器控制
        self.sub_Buzzer = self.create_subscription(Bool, 'Buzzer', self.Buzzer_callback, 10)

        # === 發布者 (Publishers) ===
        # 建立各種感測器數據的發布者
        self.pub_edition = self.create_publisher(Float32, 'edition', 10)
        self.pub_voltage = self.create_publisher(Float32, 'voltage', 10)
        self.pub_vel = self.create_publisher(Twist, 'vel_raw', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 100)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 100)

        # === 計時器 (Timer) ===
        # 建立一個 0.1 秒觸發一次的計時器，用於定期發布感測器數據
        self.timer = self.create_timer(0.1, self.publish_data)
        
        self.get_logger().info("節點 'yahboomcar_driver' 初始化完成。")

    def cmd_vel_callback(self, msg):
        """
        處理來自 /cmd_vel 主題的 Twist 訊息。
        """
        if not isinstance(msg, Twist):
            self.get_logger().warn("收到的訊息類型不是 Twist，已忽略。")
            return

        # === 速度限制 ===
        # 將收到的速度指令限制在設定的最大值和最小值之間，防止超速
        vx = max(-self.xlinear_limit, min(self.xlinear_limit, msg.linear.x))
        vy = max(-self.ylinear_limit, min(self.ylinear_limit, msg.linear.y))
        vz = max(-self.angular_limit, min(self.angular_limit, msg.angular.z))

        # 呼叫底層函式庫來控制小車運動
        self.car.set_car_motion(vx, vy, vz)

    def RGBLight_callback(self, msg):
        """控制 RGB 燈效。"""
        if not isinstance(msg, Int32): return
        self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzer_callback(self, msg):
        """控制蜂鳴器。"""
        if not isinstance(msg, Bool): return
        if msg.data:
            self.car.set_beep(1)
        else:
            self.car.set_beep(0)

    def publish_data(self):
        """
        由計時器定期呼叫，從底層讀取數據並以 ROS 訊息發布。
        """
        time_stamp = self.get_clock().now()

        # 獲取並發布韌體版本
        edition = Float32()
        edition.data = float(self.car.get_version())
        self.pub_edition.publish(edition)
        
        # 獲取並發布電池電壓
        battery = Float32()
        battery.data = float(self.car.get_battery_voltage())
        self.pub_voltage.publish(battery)

        # 獲取並發布原始速度（里程計）
        vx, vy, vz = self.car.get_motion_data()
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(vz)
        self.pub_vel.publish(twist)

        # 獲取並發布 IMU 數據（加速度計和陀螺儀）
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        
        imu = Imu()
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        self.pub_imu.publish(imu)

        # 獲取並發布磁力計數據
        mx, my, mz = self.car.get_magnetometer_data()
        mag = MagneticField()
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = float(my)
        mag.magnetic_field.z = float(mz)
        self.pub_mag.publish(mag)

    def on_shutdown(self):
        """節點關閉時執行的清理工作。"""
        self.get_logger().info("節點關閉中...")
        # 停止小車運動
        self.car.set_car_motion(0, 0, 0)
        # 關閉蜂鳴器
        self.car.set_beep(0)
        # 關閉序列埠
        del self.car

def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)
    
    # 建立驅動節點
    driver_node = YahboomcarDriver('yahboomcar_driver_node')
    
    try:
        # 進入循環，等待回調函式被觸發
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理並關閉節點
        driver_node.on_shutdown()
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
