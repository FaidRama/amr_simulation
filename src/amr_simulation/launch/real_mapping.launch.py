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

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='False',
        description='Aktifkan streaming kamera USB untuk pantau jarak jauh'
    )
    
    # 1. Driver RPLiDAR A1 Fisik via rplidar_bridge
    lidar_port = '/dev/rplidar' if os.path.exists('/dev/rplidar') else '/dev/ttyUSB0'
    start_rplidar = Node(
        package='amr_simulation',
        executable='rplidar_bridge.py',
        name='rplidar_bridge',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'auto_launch_driver': True,
            'quiet': True,  # Nonaktifkan spam log telemetri berkala (terminal bersih)
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    # 1B. Node Kamera USB (Streaming Ringan JPEG untuk Pantau Jarak Jauh)
    start_camera = Node(
        package='amr_simulation',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    # 2A. Node Arduino Bridge (Odometry Asli & Motor Control via /dev/arduino)
    start_arduino_bridge = Node(
        package='amr_simulation',
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/arduino',
            'baudrate': 115200,
            'wheel_diameter': 0.127,
            'wheel_separation': 0.458,
            'ticks_per_rev': 350.7,
            'cmd_vel_topic': '/cmd_vel',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 2B. Static Transform Publisher (Fallback jika use_arduino:=False)
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        respawn=True,
        respawn_delay=1.0,
        condition=UnlessCondition(LaunchConfiguration('use_arduino'))
    )

    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'laser'],
        respawn=True,
        respawn_delay=1.0
    )

    tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        respawn=True,
        respawn_delay=1.0
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        respawn=True,
        respawn_delay=1.0,
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    # 3. Filter Laser Sasis (Memotong pantulan 4 tiang aluminium sasis)
    start_laser_filter = Node(
        package='amr_simulation',
        executable='laser_filter_node.py',
        name='laser_filter',
        output='screen',
        parameters=[{
            'min_range': 0.34,
            'max_range': 12.0
        }],
        respawn=True,
        respawn_delay=2.0
    )

    # 4. Otak SLAM Toolbox
    slam_config_path = os.path.join(
        get_package_share_directory('amr_simulation'),
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
                'scan_topic': '/scan_filtered',
                'min_laser_range': 0.34,
                'map_update_interval': 1.0
            }
        ]
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='False', description='Jalankan RViz2')

    # 5. RViz2 untuk melihat hasil
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
        camera_arg,
        rviz_arg,
        start_rplidar,
        start_camera,
        start_laser_filter,
        start_arduino_bridge,
        tf_odom_to_base,
        tf_base_to_laser,
        tf_base_to_footprint,
        tf_base_to_camera,
        start_slam_toolbox,
        start_rviz2
    ])
