#!/usr/bin/python3
# 專用於60度前方掃描的YDLidar啟動檔案

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # 直接在啟動檔案中定義60度參數
    lidar_60_params = {
        'port': '/dev/ydlidar',
        'frame_id': 'laser',
        'ignore_array': '',
        'baudrate': 512000,
        'lidar_type': 0,
        'device_type': 0,
        'sample_rate': 20,
        'abnormal_check_count': 4,
        'resolution_fixed': True,
        'reversion': False,
        'inverted': True,
        'auto_reconnect': True,
        'isSingleChannel': False,
        'intensity': False,
        'support_motor_dtr': True,
        'angle_max': 30.0,  # 30度
        'angle_min': -30.0,  # -30度
        'range_max': 64.0,
        'range_min': 0.01,
        'frequency': 10.0,
        'invalid_range_is_inf': False
    }

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_60_params],
        namespace='/',
    )
    
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser'],
    )

    return LaunchDescription([
        driver_node,
        tf2_node,
    ])