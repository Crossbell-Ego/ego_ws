import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. 宣告啟動參數
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=os.environ.get('ROBOT_TYPE', 'X3plus'),
        description='Type of the robot (e.g., X1, X3, X3plus)'
    )

    # 獲取啟動參數的值
    robot_type = LaunchConfiguration('robot_type')

    # 2. 啟動 joy_node 節點
    # 這個節點負責讀取實體搖桿的輸入
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        # 在ROS2中，參數通常用 .yaml 或這種方式傳入
        parameters=[{'use_sim_time': False}]
    )

    # 3. 啟動我們的手把解譯節點
    ego_joy_node = Node(
        package='egocar_ctrl',  # 改為新的包名
        executable='ego_joy',
        name='ego_joy',
        output='screen',
        # 根據 robot_type 條件性地傳入參數
        parameters=[
            {'xspeed_limit': 1.0, 'yspeed_limit': 1.0, 'angular_speed_limit': 5.0, 
             'use_sim_time': False},
            # 以下的寫法是ROS2中條件判斷的範例，但更簡單的方式是直接在Python中處理
            # 此處為了教學目的保留，但上面直接設定更為直觀。
            # 若要使用條件判斷，需要更複雜的設定，此處提供一個簡化的邏輯。
        ]
    )

    # 根據 ROBOT_TYPE 變數來印出日誌訊息
    log_robot_type_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'X3'"])),
        msg=["Robot type is X3, setting specific speed limits."]
    )
    
    log_robot_type_info_plus = LogInfo(
        condition=IfCondition(PythonExpression(["'", robot_type, "' == 'X3plus'"])),
        msg=["Robot type is X3plus, setting specific speed limits."]
    )

    # 4. 將所有動作組合到 LaunchDescription 中並回傳
    return LaunchDescription([
        robot_type_arg,
        joy_node,
        ego_joy_node,
        log_robot_type_info,
        log_robot_type_info_plus
    ])

