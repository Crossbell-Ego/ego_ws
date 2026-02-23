import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 聲明 use_sim_time 啟動參數，並將其預設值設為 'false'
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true')

    # 聲明並設定 slam_params_file 的路徑
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("egocar_nav"),
                                   'params', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file for the slam_toolbox node')

    # 讀取 use_sim_time 和 slam_params_file 的配置值
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # 啟動 slam_toolbox 節點
    start_sync_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # 正確的參數加載方式：
        # 1. 將 use_sim_time 放入一個字典中
        # 2. 將包含所有參數的 YAML 檔案也傳遞進來
        # ROS 2 會自動合併它們，並以 launch 檔案中的設定為優先
        parameters=[
            {'use_sim_time': use_sim_time},
            slam_params_file
        ]
        # remappings 在此處通常是多餘的，因為 scan_topic 已在 YAML 中定義
        # remappings=[('/scan', '/scan')] 
    )

    # 建立並返回 LaunchDescription 物件
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld

