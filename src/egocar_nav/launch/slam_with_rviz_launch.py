from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 參數設定
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE', '4ROS')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    rplidar_type_arg = DeclareLaunchArgument(
        name='rplidar_type',
        default_value=RPLIDAR_TYPE,
        choices=['4ROS'],
        description='The type of RPLIDAR (only 4ROS supported)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # 啟動基礎系統（雷達 + 驅動）
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('egocar_nav'), 'launch'),
            '/laser_bringup_launch.py'
        ]),
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    # 啟動 GMapping SLAM 節點
    slam_gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
        condition=LaunchConfigurationEquals('rplidar_type', '4ROS')
    )

    # RViz 節點
    rviz_config_path = os.path.join(get_package_share_directory('egocar_nav'), 'rviz', 'map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        rplidar_type_arg,
        use_rviz_arg,
        laser_bringup_launch,
        slam_gmapping_node,
        rviz_node
    ])