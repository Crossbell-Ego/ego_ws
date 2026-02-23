import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory('egocar_bringup')
    description_pkg_dir = get_package_share_directory('egocar_description')

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    robot_type = LaunchConfiguration('robot_type', default='X3plus')
    
    urdf_path = os.path.join(description_pkg_dir, 'urdf', 'egocar_X3plus.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_localization_yaml_path = os.path.join(bringup_pkg_dir, 'param', 'robot_localization.yaml')
    rviz_config_path = os.path.join(description_pkg_dir, 'rviz', 'egocar.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='X3plus', description='Robot type'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to start RVIZ'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation/Gazebo clock'),

        # 1. 機器人狀態發布節點 (感測器相對於小車基座的位置的TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # 2. 底層驅動節點 - 已移除 remappings
        Node(
            package='egocar_bringup',
            executable='Mcnamu_driver_X3plus',
            name='driver_node', # 賦予節點一個名稱
            output='screen',
            parameters=[{
                'xlinear_speed_limit': 0.7,
                'ylinear_speed_limit': 0.7,
                'angular_speed_limit': 3.2,
                'imu_link': 'imu_link'
            }]
            # remappings 已移除，因為驅動節點現在直接發布正確的話題
        ),

        # 3. 發布里程計數據節點 - 已校準的修正參數
        Node(
            package='egocar_base_node',
            executable='base_node',
            name='odometry_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_footprint_frame': 'base_footprint',
                # 線性修正係數 (根據 calibrate_linear 校準結果)
                'linear_scale_x': 1.01,     # X 軸線性修正
                'linear_scale_y': 1.088,     # Y 軸線性修正
                # 角度修正係數 (根據 calibrate_angular 校準結果)
                'angular_scale': 1.005,     # 角度修正 (360°/343.45° ≈ 1.048)
            }],
            remappings=[
                ('/sub_vel', '/vel_raw'), # 訂閱驅動發布的 /vel_raw
                ('/pub_odom', '/odom_raw')
            ]
        ),

        # 4. IMU 數據過濾節點 - 添加關鍵參數以減少飄移
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'fixed_frame': 'base_footprint',
                'use_mag': False,
                'publish_tf': False,
                'use_magnetic_field_msg': False,
                'world_frame': 'enu',
                'orientation_stddev': 0.05,
                'angular_scale': 1.04,
                # 關鍵參數：減少飄移和抖動
                'gain': 0.1,              # 融合增益：平衡陀螺儀和加速度計
                'constant_dt': 0.01,      # 固定時間步長：確保穩定的積分
                'zeta': 0.0,              # 陀螺儀偏差補償：預設為 0
                'remove_gravity_vector': True,  # 移除重力向量：減少線性加速度干擾
            }],
            remappings=[
                # 修正：使用實際的 IMU 話題名稱
                ('imu/data_raw', '/imu/imu_raw'),     # 輸入：從驅動節點接收原始 IMU 數據
                ('imu/data', '/imu/imu_data')     # 輸出：濾波後的 IMU 數據給 EKF
            ]
        ),

        # 5. 擴展卡爾曼濾波節點 (odom -> base_footprint TF轉換)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[robot_localization_yaml_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        ),

        # 6. 手把控制節點 (使用新的 egocar_ctrl 包)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('egocar_ctrl'), 'launch', 'ego_joy_launch.py')
            )
        ),

        # 7. Rviz 視覺化節點
        Node(
            package='rviz2',
            executable='rviz2',
            name='odom_rviz',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(use_rviz)
        )
    ])
