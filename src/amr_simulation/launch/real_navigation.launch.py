import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr = get_package_share_directory('amr_simulation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    default_map_path = os.path.join(pkg_amr, 'maps', 'peta_l1_ddc.yaml')
    src_params_path = os.path.join(os.getcwd(), 'src', 'amr_simulation', 'config', 'nav2_params.yaml')
    default_params_path = src_params_path if os.path.exists(src_params_path) else os.path.join(pkg_amr, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_amr, 'rviz', 'slam_config.rviz')

    # Launch Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path lengkap ke file peta .yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Path lengkap ke file nav2_params.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Gunakan waktu simulasi (False untuk robot fisik)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Otomatis jalankan lifecycle Nav2 stack'
    )

    use_arduino_arg = DeclareLaunchArgument(
        'use_arduino',
        default_value='True',
        description='Gunakan Arduino untuk Odometry & Motor Control'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Jalankan RViz2 untuk visualisasi di laptop'
    )

    # 1. Driver RPLiDAR A1 Fisik via rplidar_bridge (Auto-detect port /dev/rplidar /dev/ttyUSB0 & auto-recovery)
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
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 2. Filter Laser Sasis (Memotong pantulan tiang di dalam 29 cm)
    start_laser_filter = Node(
        package='amr_simulation',
        executable='laser_filter_node.py',
        name='laser_filter',
        output='screen',
        parameters=[{
            'min_range': 0.29,
            'max_range': 12.0
        }],
        respawn=True,
        respawn_delay=2.0
    )

    # 3. Arduino Bridge (Odometry Asli & Motor Control via /dev/arduino)
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
            'cmd_vel_topic': '/cmd_vel_safe',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 4. Static Transforms (dilengkapi respawn=True agar tidak pernah putus jika ada cleanup proses)
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

    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        respawn=True,
        respawn_delay=1.0,
        condition=UnlessCondition(LaunchConfiguration('use_arduino'))
    )

    # 5. Pembersihan otomatis proses Nav2 yatim (zombie) dari sesi sebelumnya pada t=0.0s
    # Mencegah duplikasi action server (/compute_path_to_pose, /follow_path) yang memicu
    # "send_goal failed" dan crash exit code -11 pada bt_navigator saat diberi 2D Goal Pose.
    cleanup_stale_nav2 = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            'pkill -9 -f "(bt_navigator|planner_server|controller_server|recoveries_server|waypoint_follower|map_server|amcl|lifecycle_manager)" 2>/dev/null; true'
        ],
        output='log'
    )

    # 5B. Adaptive Velocity & Nav2 Bootstrap Node (dijalankan sejak t=0s agar TF map->odom langsung tersedia)
    start_adaptive_velocity = Node(
        package='amr_simulation',
        executable='adaptive_velocity_node.py',
        name='adaptive_velocity_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # 6A. Nav2 Localization (Map Server + AMCL) - dijalankan pada t=2.0s setelah sensor & TF siap
    start_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart')
        }.items()
    )

    delayed_localization = TimerAction(
        period=2.0,
        actions=[start_localization]
    )

    # 6B. Nav2 Navigation (Controller + Planner + Costmaps + BT Navigator) - dijalankan pada t=4.5s
    # setelah Map Server & AMCL aktif sehingga tidak pernah gagal "frame map does not exist"
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    delayed_navigation = TimerAction(
        period=4.5,
        actions=[start_navigation]
    )

    # 7. RViz2 Visualizer (Opsional di laptop)
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_arduino_arg,
        rviz_arg,
        cleanup_stale_nav2,
        start_rplidar,
        start_laser_filter,
        start_arduino_bridge,
        tf_base_to_laser,
        tf_base_to_footprint,
        tf_odom_to_base,
        start_adaptive_velocity,
        delayed_localization,
        delayed_navigation,
        start_rviz2
    ])