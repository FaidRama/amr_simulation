import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    default_lidar_port = '/dev/rplidar' if os.path.exists('/dev/rplidar') else '/dev/ttyUSB0'

    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_lidar_port,
        description='Port serial RPLiDAR A1 (/dev/rplidar atau /dev/ttyUSB0)'
    )

    baud_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate RPLiDAR A1 (default 115200)'
    )

    frame_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='TF Frame ID untuk LaserScan'
    )

    filter_arg = DeclareLaunchArgument(
        'filter',
        default_value='True',
        description='Jalankan juga laser_filter_node untuk menyaring 4 tiang sasis'
    )

    # 1. Node Driver RPLiDAR A1
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # 2. Node Filter Sasis Robot (/scan -> /scan_filtered)
    filter_node = Node(
        package='amr_simulation',
        executable='laser_filter_node.py',
        name='laser_filter',
        output='screen',
        parameters=[{
            'min_range': 0.32,
            'max_range': 12.0,
        }],
        condition=IfCondition(LaunchConfiguration('filter'))
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_arg,
        filter_arg,
        rplidar_node,
        filter_node,
    ])
