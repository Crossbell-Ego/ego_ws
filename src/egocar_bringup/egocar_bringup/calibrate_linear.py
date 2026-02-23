#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import sqrt, pow, copysign
import traceback

class CalibrateLinear(Node):
    """
    本節點用於校準小車的線性運動里程計。
    透過控制小車行走一個預設距離，並與里程計回報的距離進行比較，
    來計算出一個修正係數 (odom_linear_scale_correction)。
    """
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"節點 '{name}' 初始化中...")

        # --- 1. 參數宣告與初始化 ---
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.declare_parameter('direction', True)  # True 為 X 軸, False 為 Y 軸
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')

        # 讀取初始參數值
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.direction_is_x = self.get_parameter('direction').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        # --- 2. ROS2 通訊介面 ---
        # 建立一個發布者，發布速度指令到 /cmd_vel 主題
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 5)

        # 建立 TF 監聽器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 變數初始化
        self.position_x_start = 0.0
        self.position_y_start = 0.0
        
        # --- 3. 核心邏輯 ---
        # 建立一個 20Hz (0.05s) 的計時器，用於主邏輯循環
        self.timer = self.create_timer(0.05, self.on_timer)

        # 註冊參數回呼函式
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info("等待 TF 轉換... (odom -> base_footprint)")
        # 建議在啟動時給 TF 一點時間來準備就緒
        try:
            self.tf_buffer.can_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=10.0))
            self.get_logger().info("TF 轉換已就緒。")
        except TransformException as e:
            self.get_logger().error(f"TF 轉換超時: {e}")
            self.get_logger().warn("請確保 robot_state_publisher 與 odom 發布節點正在運行。")

        self.get_logger().info(f"節點 '{name}' 初始化完成。")


    def parameters_callback(self, params):
        """
        當 ROS 參數被外部更改時，此回呼函式會被觸發。
        """
        for param in params:
            self.get_logger().info(f"參數 '{param.name}' 被修改為: {param.value}")
            if param.name == 'start_test' and param.value is True:
                # 當 start_test 被設為 True，記錄當前位置作為起點
                current_pos = self.get_position()
                if current_pos:
                    self.position_x_start = current_pos.transform.translation.x
                    self.position_y_start = current_pos.transform.translation.y
                    self.start_test = True
                    self.get_logger().info(f"測試開始！起點: (x={self.position_x_start:.3f}, y={self.position_y_start:.3f})")
                else:
                    self.get_logger().error("無法獲取當前位置，測試無法開始。")
                    # 將參數設回 False，表示測試未成功啟動
                    return SetParametersResult(successful=False)
            
            # 動態更新其他參數
            elif param.name == 'test_distance':
                self.test_distance = param.value
            elif param.name == 'speed':
                self.speed = param.value
            elif param.name == 'tolerance':
                self.tolerance = param.value
            elif param.name == 'odom_linear_scale_correction':
                self.odom_linear_scale_correction = param.value
            elif param.name == 'direction':
                self.direction_is_x = param.value

        return SetParametersResult(successful=True)


    def on_timer(self):
        """
        計時器主迴圈，負責在測試進行時控制小車運動。
        """
        # 如果測試未啟動，則不執行任何操作
        if not self.start_test:
            return

        move_cmd = Twist()
        current_pos = self.get_position()
        
        if not current_pos:
            self.get_logger().warn("暫時無法獲取 TF 位置，跳過此次迴圈。")
            return

        # 計算當前位置與起點的距離
        dx = current_pos.transform.translation.x - self.position_x_start
        dy = current_pos.transform.translation.y - self.position_y_start
        distance = sqrt(pow(dx, 2) + pow(dy, 2))
        
        # 應用校準係數
        distance *= self.odom_linear_scale_correction
        
        # 計算與目標距離的誤差
        error = distance - self.test_distance
        
        self.get_logger().info(f"目標距離: {self.test_distance:.3f} m, 當前行走距離: {distance:.3f} m, 誤差: {error:.3f} m")

        # 檢查是否已達到目標距離 (誤差在容忍範圍內)
        if abs(error) < self.tolerance:
            self.start_test = False
            # 透過 ROS 參數服務將 start_test 設回 False，方便外部工具觀察狀態
            self.set_parameters([Parameter('start_test', Parameter.Type.BOOL, False)])
            self.cmd_vel_pub.publish(Twist()) # 發送停止指令
            self.get_logger().info("校準測試完成！小車已停止。")
            
            # 根據行走的軸向，計算並建議新的 odom_linear_scale_correction 值
            actual_travel = sqrt(pow(dx, 2) + pow(dy, 2)) # 使用未經校正的原始里程計距離
            if actual_travel > 0.1: # 避免除以零或過小的數
                suggested_correction = self.test_distance / actual_travel
                self.get_logger().info(f"--- 校準建議 ---")
                self.get_logger().info(f"目標距離: {self.test_distance:.3f} m")
                self.get_logger().info(f"里程計回報距離: {actual_travel:.3f} m")
                self.get_logger().info(f"建議的新 'odom_linear_scale_correction' 值為: {suggested_correction:.4f}")
            
        else:
            # 如果未達到目標，繼續前進
            if self.direction_is_x: # 沿 X 軸移動
                move_cmd.linear.x = copysign(self.speed, -error)
            else: # 沿 Y 軸移動
                move_cmd.linear.y = copysign(self.speed, -error)
            
            self.cmd_vel_pub.publish(move_cmd)


    def get_position(self):
        """
        安全地獲取 odom -> base_frame 的 TF 轉換。
        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.1)
            )
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"獲取 TF 轉換失敗: {e}")
            return None


    def on_shutdown(self):
        """
        節點關閉時的清理工作。
        """
        self.get_logger().info("節點正在關閉...")
        # 確保小車停止
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("已發送停止指令，節點清理完畢。")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear("calibrate_linear")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(f"節點執行時發生未預期錯誤:\n{traceback.format_exc()}")
    finally:
        # 節點關閉時執行清理
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
