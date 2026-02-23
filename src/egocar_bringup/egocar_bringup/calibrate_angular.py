#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, radians, pi, degrees
import PyKDL
import traceback

class CalibrateAngularNode(Node):
    """
    一個用於校準機器人角度運動的ROS2節點。
    此節點透過發布 /cmd_vel 指令讓機器人旋轉，並監聽TF來獲取里程計的角度變化。
    透過比較目標角度和實際旋轉角度，可以幫助使用者調整 odom_angular_scale_correction 參數，
    以達到更精確的旋轉控制。
    """
    def __init__(self, name):
        """
        節點初始化。
        """
        super().__init__(name)
        self.get_logger().info(f"節點 '{name}' 初始化中...")

        # 1. 宣告並獲取參數
        self._declare_and_get_params()

        # 2. 初始化ROS2通訊介面
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 初始化狀態變數
        self.is_running_test = False       # 標記是否正在執行測試
        self.total_rotated_angle = 0.0     # 本次測試已旋轉的總角度（弧度）
        self.last_odom_angle = 0.0         # 上一個時間點的里程計角度
        self.direction = 1                 # 旋轉方向，1為正，-1為負

        # 等待TF座標轉換準備就緒
        self.wait_for_tf_ready()

        # 建立主迴圈計時器
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info("角度校準節點初始化完成，等待 'start_test' 參數設為 true 來開始測試。")

    def _declare_and_get_params(self):
        """
        集中宣告和獲取所有ROS參數。
        """
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('test_angle', 360.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('tolerance', 1.05)
        self.declare_parameter('odom_angular_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

    def wait_for_tf_ready(self):
        """
        在啟動時等待TF座標轉換準備好，避免後續查找失敗。
        """
        self.get_logger().info(f"正在等待 '{self.odom_frame}' -> '{self.base_frame}' 的TF座標轉換...")
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info("TF座標轉換已就緒。")
                return
            except TransformException as e:
                #self.get_logger().warn(f"等待TF中: {e}")
                rclpy.spin_once(self, timeout_sec=1.0)

    def timer_callback(self):
        """
        主迴圈，根據 'start_test' 參數控制測試的開始、執行與結束。
        """
        start_test_param = self.get_parameter('start_test').get_parameter_value().bool_value

        if start_test_param:
            # 如果參數為True，且測試尚未開始，則啟動新一輪測試
            if not self.is_running_test:
                self.start_new_test()
            # 執行測試的單一步驟
            self.run_test_step()
        elif self.is_running_test:
            # 如果參數從True變為False，表示外部指令停止測試
            self.stop_test("測試被外部命令手動停止。")

    def start_new_test(self):
        """
        重置狀態變數，並開始一個新的校準測試。
        """
        self.get_logger().info("==================== 開始新的角度校準測試 ====================")
        self.is_running_test = True
        self.total_rotated_angle = 0.0
        self.last_odom_angle = self._get_odom_angle()

        # 獲取當前測試參數並顯示
        test_angle_deg = self.get_parameter('test_angle').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance_rad = radians(self.get_parameter('tolerance').get_parameter_value().double_value)
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        
        self.target_angle = radians(test_angle_deg) * self.direction

        self.get_logger().info(f"測試目標: 旋轉 {degrees(self.target_angle):.2f} 度")
        self.get_logger().info(f"使用參數: 速度={self.speed:.2f} rad/s, 容忍度={degrees(self.tolerance_rad):.2f} 度, 修正係數={self.odom_angular_scale_correction:.4f}")


    def run_test_step(self):
        """
        執行單步的旋轉控制和角度計算。
        """
        error = self.target_angle - self.total_rotated_angle

        # 如果誤差大於容忍度，繼續旋轉
        if abs(error) > self.tolerance_rad:
            move_cmd = Twist()
            move_cmd.angular.z = copysign(self.speed, error)
            self.cmd_vel_pub.publish(move_cmd)

            # 計算自上次更新以來的角度變化
            current_odom_angle = self._get_odom_angle()
            delta_angle = self._normalize_angle(current_odom_angle - self.last_odom_angle)
            
            # 應用修正係數
            corrected_delta = delta_angle * self.odom_angular_scale_correction
            
            self.total_rotated_angle += corrected_delta
            self.last_odom_angle = current_odom_angle

            self.get_logger().info(f"目標: {degrees(self.target_angle):.2f}, 當前: {degrees(self.total_rotated_angle):.2f}, 誤差: {degrees(error):.2f}")
        else:
            # 誤差在容忍度內，測試完成
            self.get_logger().info(f"已達到目標角度。最終旋轉角度: {degrees(self.total_rotated_angle):.2f} 度")
            self.stop_test("已達到目標角度。")

    def stop_test(self, reason: str):
        """
        停止測試，發布停止指令，並重置狀態。
        """
        self.get_logger().info(f"==================== 測試結束: {reason} ====================")
        
        # 發送停止指令
        self.cmd_vel_pub.publish(Twist())
        
        self.is_running_test = False
        
        # 將 'start_test' 參數重設為 False，以便下次觸發
        param = Parameter('start_test', Parameter.Type.BOOL, False)
        self.set_parameters([param])
        
        # 反轉下次旋轉的方向
        self.direction *= -1
        self.get_logger().info(f"下次測試將朝相反方向旋轉。請再次設定 'start_test' 為 true 以開始。")


    def _get_odom_angle(self) -> float:
        """
        從TF獲取當前的里程計偏航角(yaw)。
        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now)
            
            q = trans.transform.rotation
            rotation = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
            
            # 從四元數獲取RPY角度，[2]代表Z軸的旋轉(Yaw)
            return rotation.GetRPY()[2]
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'獲取TF座標轉換時出錯: {e}')
            return self.last_odom_angle # 返回上一次的值以避免突變

    def _normalize_angle(self, angle: float) -> float:
        """
        將角度標準化到 -pi 到 pi 的範圍內。
        """
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def on_shutdown(self):
        """
        節點關閉時執行的清理工作。
        """
        self.get_logger().info("正在關閉角度校準節點...")
        self.timer.cancel()
        # 確保機器人停止運動
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("已發送停止指令，節點已清理。")

def main(args=None):
    """
    主函式，ROS2節點的入口點。
    """
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrateAngularNode('calibrate_angular_node')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"節點執行時發生未預期錯誤: {e}")
            node.get_logger().error(traceback.format_exc())
    finally:
        if node:
            # 確保在退出時執行清理工作
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()
        print("ROS2 節點已完全關閉。")

if __name__ == '__main__':
    main()

