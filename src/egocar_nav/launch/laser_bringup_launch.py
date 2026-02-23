from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 固定僅支援 X3plus 與 4ROS，若未設定環境變數則使用預設
    ROBOT_TYPE = os.getenv('ROBOT_TYPE', 'X3plus')
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE', '4ROS')

    # 僅允許合法值，避免誤選
    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value=ROBOT_TYPE,
        choices=['X3plus'],
        description='Robot type (only X3plus supported)'
    )

    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type',
        default_value=RPLIDAR_TYPE,
        choices=['4ROS'],
        description='RPLIDAR type (only 4ROS supported)'
    )

    # 僅啟動 X3 機型 bringup（X3plus 與 x3 共用）
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('egocar_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        condition=LaunchConfigurationEquals('robot_type', 'X3plus')
    )

    # 僅啟動 YDLIDAR 4ROS 驅動
    lidar_4ROS_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
            '/ydlidar_raw_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    return LaunchDescription([
        robot_type_arg,
        rplidar_type_arg,
        bringup_launch,
        lidar_4ROS_launch,
    ])
