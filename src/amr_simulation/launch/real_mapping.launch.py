import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. Node RPLidar A1 (Membaca sensor fisik)
    start_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1M8 biasanya 115200
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # 2. Static Transform Publisher (Karena kamu belum punya roda/odometry)
    # Ini "membohongi" sistem seolah-olah robot diam, 
    # SLAM Toolbox akan dipaksa menebak pergerakan murni dari scan Lidar (Scan Matching)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )

    # 3. Otak SLAM Toolbox
    slam_config_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    start_slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {
                'use_sim_time': False, 
                'base_frame': 'base_link',
                'odom_frame': 'base_link', # KUNCI: Odom & Base disamakan agar murni pakai Laser
                'map_frame': 'map',
                'scan_topic': '/scan' 
            }
        ]
    )

    # 4. RViz2 untuk melihat hasil
    rviz_config_dir = os.path.join(get_package_share_directory('amr_simulation'), 'rviz', 'slam_config.rviz')
    
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        start_rplidar,
        tf_base_to_laser,
        start_slam_toolbox,
        start_rviz2
    ])
