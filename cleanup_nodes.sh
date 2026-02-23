#!/bin/bash
# ROS2 節點完全清理腳本

echo "正在停止所有相關節點..."

# 停止雷達相關進程
pkill -f ydlidar
pkill -f laser_avoidance
pkill -f static_tf_pub_laser

# 停止launch相關進程
pkill -f "ros2 launch"
pkill -f launch_ros

# 停止其他可能的節點
pkill -f egocar_laser
pkill -f egocar_bringup

# 等待進程完全終止
sleep 2

echo "檢查剩餘的ROS2節點："
ros2 node list

echo "清理完成！"