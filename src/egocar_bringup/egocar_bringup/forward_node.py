#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time

# 匯入與此檔案放在同一個資料夾的 Rosmaster_Lib 函式庫
from .Rosmaster_Lib import Rosmaster

class MoveForwardNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("節點 '%s' 已啟動" % name)

        # 初始化 Rosmaster 控制庫
        # *** 重要 ***：請確認您的 Jetson Orin Nano 連接底層控制板的序列埠號
        # 您可以在本地終端機使用 `ls /dev/tty*` 來查找，通常是 /dev/ttyUSB0, /dev/ttyACM0 或 /dev/ttyTHS1
        try:
            self.bot = Rosmaster(com="/dev/myserial", debug=False)
            self.bot.create_receive_threading()
            self.get_logger().info("Rosmaster 控制庫初始化成功！")
        except Exception as e:
            self.get_logger().error("Rosmaster 控制庫初始化失敗：%s" % str(e))
            self.get_logger().warn("請檢查序列埠號是否正確，以及是否有權限讀寫。")
            self.get_logger().warn("您可能需要執行 `sudo chmod 666 /dev/ttyUSB0` (請替換成您的埠號)")
            rclpy.shutdown()
            return

        # 建立一個每 0.1 秒觸發一次的計時器，並指定回呼函式為 self.timer_callback
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("小車將開始前進...")

    def timer_callback(self):
        """
        計時器回呼函式，會被重複呼叫。
        """
        # 呼叫 set_car_run 函式
        # state=1 代表前進
        # speed=30 代表速度 (範圍: -100 到 100)
        self.bot.set_car_run(1, 30)

    def stop_robot(self):
        """
        停止小車的函式
        """
        self.get_logger().info("正在停止小車...")
        # state=0 代表停止
        self.bot.set_car_run(0, 0)
        time.sleep(0.1) # 給予一點時間確保指令送達
        self.bot.ser.close()
        self.get_logger().info("小車已停止，序列埠已關閉。")


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode("move_forward_node")
    try:
        # 讓節點持續運行，直到被外部中斷 (例如按下 Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 當使用者按下 Ctrl+C 時，會執行這裡的程式碼
        node.get_logger().info("接收到關閉指令 (Ctrl+C)")
    finally:
        # 無論如何，最後都會停止機器人並關閉節點
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
