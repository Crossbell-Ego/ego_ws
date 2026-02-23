from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    if not os.environ.get("PRINTED"):
        os.environ["PRINTED"] = "1"
        print("---------------------robot_type = x3---------------------")
    urdf_tutorial_path = get_package_share_path('egocar_description')
    default_model_path = urdf_tutorial_path / 'urdf/egocar_X3.urdf'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/egocar.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='false',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

    # 直接讀取 egocar_X3plus.urdf 檔案
    urdf_file_path = '/home/jetson/ego_ws/src/egocar_description/urdf/egocar_X3plus.urdf'
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    driver_node = Node(
        package='egocar_bringup',
        executable='Mcnamu_driver_X3',
    )

    base_node = Node(
        package='egocar_base_node',
        executable='base_node_X3',
        # 当使用ekf融合时，该tf有ekf发布
        parameters=[{
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }]
    )

    # IMU 濾波器節點 - 使用 madgwick 濾波器，內聯參數配置
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'base_frame': 'base_link',
            'gain': 0.1,
            'zeta': 0.0,
            'mag_bias_x': 0.0,
            'mag_bias_y': 0.0,
            'mag_bias_z': 0.0,
            'orientation_stddev': 0.1,
            'angular_velocity_stddev': 0.02,
            'linear_acceleration_stddev': 0.1,
            'stateless': False,
            'constant_dt': 0.01,
            'publish_debug_topics': False
        }]
    )

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    ego_joy_node = Node(
        package='egocar_ctrl',
        executable='ego_joy',
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # rviz_node
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        ego_joy_node,
        joy_node
    ])
