import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 設定顯示環境變數
    set_display_env = SetEnvironmentVariable(
        name='DISPLAY',
        value=':0.0'
    )

    # 宣告 use_gui 啟動參數，用於決定是否開啟圖形介面
    declare_use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Whether to use the joint_state_publisher_gui')

    # 獲取 use_gui 參數的值
    use_gui = LaunchConfiguration('use_gui')

    # 獲取 egocar_description 功能包的路徑
    pkg_path = get_package_share_directory('egocar_description')

    # 定義 URDF 模型的完整路徑
    urdf_file = os.path.join(pkg_path, 'urdf', 'egocar_X3plus.urdf')
    
    # 讀取 URDF 文件內容
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # 獲取 RViz2 配置文件的路徑
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'egocar.rviz')

    # 定義 robot_state_publisher 節點
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        additional_env={'DISPLAY': ':0.0'}
    )

    # 定義 joint_state_publisher 節點 (非 GUI 版本)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        additional_env={'DISPLAY': ':0.0'}
    )

    # 定義 joint_state_publisher_gui 節點 (GUI 版本)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        additional_env={'DISPLAY': ':0.0'}
    )

    # 定義 RViz2 節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        additional_env={'DISPLAY': ':0.0'}
    )

    # 將所有宣告和節點打包到 LaunchDescription 中並返回
    return LaunchDescription([
        set_display_env,
        declare_use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
