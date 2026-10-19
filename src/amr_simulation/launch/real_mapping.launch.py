import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_arduino_arg = DeclareLaunchArgument(
        'use_arduino',
        default_value='True',
        description='Gunakan Arduino untuk Odometry & Motor Control'
    )
    
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

    # 2A. Node Arduino Bridge (Odometry Asli & Motor Control via /dev/ttyACM0)
    start_arduino_bridge = Node(
        package='amr_simulation',
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baudrate': 115200,
            'wheel_diameter': 0.127,
            'wheel_separation': 0.442,
            'ticks_per_rev': 537.0,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 2B. Static Transform Publisher (Fallback jika use_arduino:=False)
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=UnlessCondition(LaunchConfiguration('use_arduino'))
    )

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
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'minimum_travel_distance': 0.0,
                'minimum_travel_heading': 0.0,
                'map_update_interval': 2.0
            }
        ]
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='False', description='Jalankan RViz2')

    # 4. RViz2 untuk melihat hasil
    rviz_config_dir = os.path.join(get_package_share_directory('amr_simulation'), 'rviz', 'slam_config.rviz')
    
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        use_arduino_arg,
        rviz_arg,
        start_rplidar,
        start_arduino_bridge,
        tf_odom_to_base,
        tf_base_to_laser,
        start_slam_toolbox,
        start_rviz2
    ])
