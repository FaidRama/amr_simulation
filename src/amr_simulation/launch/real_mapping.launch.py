import os
import time
import signal
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def _get_ancestor_pids():
    ancestors = set()
    pid = os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        try:
            with open(f"/proc/{pid}/status", "r") as f:
                for line in f:
                    if line.startswith("PPid:"):
                        pid = int(line.split()[1])
                        break
                else:
                    break
        except Exception:
            break
    return ancestors


def _cleanup_stale_processes():
    protected_pids = _get_ancestor_pids()
    patterns = [
        'real_navigation.launch.py',
        'real_mapping.launch.py',
        'localization_launch.py',
        'navigation_launch.py',
        'bt_navigator',
        'planner_server',
        'controller_server',
        'recoveries_server',
        'waypoint_follower',
        'map_server',
        'amcl',
        'lifecycle_manager',
        'async_slam_toolbox_node',
        'adaptive_velocity_node.py',
        'rplidar_bridge.py',
        'rplidar_composition',
        'arduino_bridge.py',
        'laser_filter_node.py',
        'static_transform_publisher',
    ]
    for pat in patterns:
        try:
            res = subprocess.run(['pgrep', '-f', pat], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            for line in res.stdout.splitlines():
                line = line.strip()
                if line.isdigit():
                    pid = int(line)
                    if pid not in protected_pids:
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except Exception:
                            pass
        except Exception:
            pass
    time.sleep(0.3)


def generate_launch_description():
    _cleanup_stale_processes()

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
            'quiet': True,
        }],
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
        arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'laser']
    )

    tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
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
        }]
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
