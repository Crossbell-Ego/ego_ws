from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    # 僅支援 4ROS；若未設定環境變數則預設為 4ROS
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE', '4ROS')

    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type',
        default_value=RPLIDAR_TYPE,
        choices=['4ROS'],
        description='The type of RPLIDAR (only 4ROS supported)'
    )

    # 原 map_gmapping_4ros_launch.py：啟動雷射雷達與 GMapping
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('egocar_nav'), 'launch'),
            '/laser_bringup_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    slam_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    return LaunchDescription([
        rplidar_type_arg,
        laser_bringup_launch,
        slam_gmapping_node
    ])
