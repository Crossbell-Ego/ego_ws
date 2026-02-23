#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    GroupAction,
    ExecuteProcess,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    完整的SLAM系統啟動文件
    包含：激光雷達、機器人基礎系統、SLAM工具箱
    
    優化特性：
    - 智能啟動順序和延遲控制
    - 可配置的日誌級別和RViz支持
    - 系統健康監控
    - 性能優化參數
    """
    
    # ==================== 參數聲明 ====================
    # 先聲明所有參數
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用模擬時間（Gazebo仿真時設為true）'
    )
        
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('egocar_nav'),
            'params',
            'slam_toolbox_params.yaml'
        ]),
        description='SLAM工具箱參數文件的完整路徑'
    )
    
    declare_slam_delay = DeclareLaunchArgument(
        'slam_delay',
        default_value='3.0',
        description='SLAM節點延遲啟動時間（秒）'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='是否自動啟動RViz'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='日誌級別：debug, info, warn, error'
    )
    
    declare_enable_health_check = DeclareLaunchArgument(
        'enable_health_check',
        default_value='true',
        description='是否啟用系統健康檢查'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('egocar_nav'),
            'rviz',
            'slam_toolbox.rviz'
        ]),
        description='RViz配置文件路徑'
    )

    # 參數聲明後才能使用LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_delay = LaunchConfiguration('slam_delay')
    enable_rviz = LaunchConfiguration('enable_rviz')
    log_level = LaunchConfiguration('log_level')
    enable_health_check = LaunchConfiguration('enable_health_check')
    rviz_config = LaunchConfiguration('rviz_config')

    # ==================== 系統啟動信息 ====================
    
    startup_info = LogInfo(
        msg='🚀 啟動完整SLAM系統 - 機器人基礎系統 + 激光雷達 + SLAM工具箱'
    )

    # ==================== 系統組件啟動 ====================
    
    # 1. 機器人基礎系統（包含驅動、里程計、TF等）
    robot_bringup_group = GroupAction([
        LogInfo(msg='📡 啟動機器人基礎系統...'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('egocar_bringup'),
                '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ])
    
    # 2. 激光雷達系統
    lidar_bringup_group = GroupAction([
        LogInfo(msg='🔍 啟動激光雷達系統...'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('egocar_nav'),
                '/launch/laser_bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ])

    # 3. SLAM工具箱節點（性能優化配置）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                # 性能優化的SLAM參數
                'scan_queue_size': 10,           # 適中的掃描隊列（減少內存使用）
                'transform_timeout': 0.2,        # 更快的變換超時
                'tf_buffer_duration': 10.0,      # 適中的TF緩存時間
                'enable_interactive_mode': True,  # 啟用交互模式
                'map_update_interval': 2.0,      # 適中的地圖更新間隔
                'resolution': 0.05,              # 地圖解析度
                'max_laser_range': 30.0,         # 適中的最大激光範圍
                'minimum_time_interval': 0.5,    # 增加時間間隔（減少計算負載）
                'transform_publish_period': 0.05, # 降低TF發布頻率
                'map_start_at_dock': True,       # 從起始點開始建圖
                'throttle_scans': 1,             # 不跳過掃描
                'correlation_search_space_dimension': 0.3,  # 減少搜索空間
                'correlation_search_space_resolution': 0.01,
                'correlation_search_space_smear_deviation': 0.1,
                'loop_search_maximum_distance': 3.0,  # 減少迴路搜索距離
                'do_loop_closing': True,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_maximum_variance_coarse': 0.3,
                'optimization_frequency': 0.5,   # 降低優化頻率
                'max_iterations': 3,             # 減少最大迭代次數
            }
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata')
        ],
        # 條件性設置日誌級別（避免debug模式的過多輸出）
        arguments=[
            '--ros-args', 
            '--log-level', 
            ['slam_toolbox:=', log_level]
        ]
    )

    # 4. 延遲啟動SLAM（確保所有依賴系統已就緒）
    delayed_slam_toolbox = TimerAction(
        period=slam_delay,
        actions=[
            LogInfo(msg='🗺️ 啟動SLAM工具箱...'),
            slam_toolbox_node
        ]
    )

    # 5. 可選的RViz啟動
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 延遲啟動RViz（等待SLAM系統完全就緒）
    delayed_rviz = TimerAction(
        period=8.0,  # 在SLAM啟動後再等5秒
        actions=[
            LogInfo(msg='🎨 啟動RViz可視化界面...'),
            rviz_node
        ],
        condition=IfCondition(enable_rviz)
    )

    # ==================== 診斷和監控 ====================
    
    # 系統健康檢查腳本
    health_check_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "🔍 系統健康檢查..." && '
            'ros2 topic list | grep -E "(map|scan|odom)" && '
            'echo "✅ 關鍵話題檢查完成" && '
            'ros2 topic echo /map_metadata --once --timeout 5 > /dev/null 2>&1 && '
            'echo "✅ 地圖數據正常發布" || echo "⚠️ 地圖數據尚未發布"'
        ],
        output='screen',
        shell=False,
        condition=IfCondition(enable_health_check)
    )
    
    delayed_health_check = TimerAction(
        period=12.0,  # 12秒後檢查系統狀態
        actions=[health_check_cmd],
        condition=IfCondition(enable_health_check)
    )

    # SLAM性能監控
    performance_monitor = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 15 && '
            'echo "📊 SLAM性能監控..." && '
            'ros2 topic hz /map --window 10 --timeout 5 2>/dev/null | head -1 || echo "地圖更新頻率: 檢測中..." && '
            'ros2 topic hz /scan --window 10 --timeout 3 2>/dev/null | head -1 || echo "激光掃描頻率: 檢測中..."'
        ],
        output='screen',
        shell=False,
        condition=IfCondition(enable_health_check)
    )

    # ==================== 啟動描述組裝 ====================
    
    ld = LaunchDescription()

    # 首先添加參數聲明（必須在使用LaunchConfiguration之前）
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params_file)
    ld.add_action(declare_slam_delay)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_log_level)
    ld.add_action(declare_enable_health_check)
    ld.add_action(declare_rviz_config)

    # 添加啟動信息
    ld.add_action(startup_info)

    # 按順序添加系統組件
    ld.add_action(robot_bringup_group)       # 1. 機器人基礎系統
    ld.add_action(lidar_bringup_group)       # 2. 激光雷達系統
    ld.add_action(delayed_slam_toolbox)      # 3. 延遲啟動SLAM
    ld.add_action(delayed_rviz)              # 4. 可選的RViz
    ld.add_action(delayed_health_check)      # 5. 系統健康檢查
    ld.add_action(performance_monitor)       # 6. 性能監控

    return ld


if __name__ == '__main__':
    """
    🚀 優化的SLAM系統啟動文件使用指南
    
    ================================
    基本使用：
    ================================
    
    # 標準啟動（推薦）
    ros2 launch egocar_nav complete_slam_launch.py
    
    # 啟動並自動打開RViz
    ros2 launch egocar_nav complete_slam_launch.py enable_rviz:=true
    
    ================================
    高級配置：
    ================================
    
    # 自定義SLAM延遲時間（如果系統啟動較慢）
    ros2 launch egocar_nav complete_slam_launch.py slam_delay:=5.0
    
    # 安靜模式（減少日誌輸出）
    ros2 launch egocar_nav complete_slam_launch.py log_level:=warn
    
    # 調試模式（詳細輸出）
    ros2 launch egocar_nav complete_slam_launch.py log_level:=debug
    
    # 禁用健康檢查（節省資源）
    ros2 launch egocar_nav complete_slam_launch.py enable_health_check:=false
    
    # 仿真模式
    ros2 launch egocar_nav complete_slam_launch.py use_sim_time:=true
    
    # 完整配置示例
    ros2 launch egocar_nav complete_slam_launch.py enable_rviz:=true slam_delay:=3.0 log_level:=info
    
    ================================
    性能優化說明：
    ================================
    
    本版本針對Jetson平台進行了以下優化：
    - 降低了SLAM計算頻率，減少CPU負載
    - 優化了內存使用（較小的緩存和隊列）
    - 智能的日誌級別控制
    - 可選的性能監控
    - 減少了不必要的調試輸出
    
    ================================
    修復說明：
    ================================
    
    v1.1 修復：
    - 修復了LaunchConfiguration變數必須在DeclareLaunchArgument之後使用的問題
    - 簡化了startup_info以避免複雜的變數引用
    - 確保參數聲明在LaunchDescription的最開始
    
    """
    pass