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

    # 1. Driver RPLiDAR A1 Fisik (/dev/ttyUSB0)
    start_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 2. Filter Laser Sasis (Memotong pantulan tiang di dalam 34 cm)
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
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 4. Static Transforms
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

    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=UnlessCondition(LaunchConfiguration('use_arduino'))
    )

    # 5. Nav2 Bringup (AMCL + Costmaps + Planner + Controller)
    # Output controller di-remap dari /cmd_vel ke /cmd_vel_nav
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart')
        }.items()
    )
 
    # 6. Adaptive Velocity Node (Local Planner Speed Governor)
    # Membaca /cmd_vel_nav dan /human_state, mempublikasikan output aman ke /cmd_vel
    src_adaptive_script = os.path.join(os.getcwd(), 'src', 'amr_simulation', 'scripts', 'adaptive_velocity_node.py')
    default_adaptive_script = src_adaptive_script if os.path.exists(src_adaptive_script) else os.path.join(pkg_amr, 'scripts', 'adaptive_velocity_node.py')
    
    start_adaptive_velocity = ExecuteProcess(
        cmd=['python3', default_adaptive_script],
        output='screen'
    )


    # Nav2 dan Adaptive Velocity dijalankan setelah sensor stabil (delay 3 detik)
    delayed_nav2_and_adaptor = TimerAction(
        period=3.0,
        actions=[start_nav2, start_adaptive_velocity]
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
        start_rplidar,
        start_laser_filter,
        start_arduino_bridge,
        tf_base_to_laser,
        tf_base_to_footprint,
        tf_odom_to_base,
        delayed_nav2_and_adaptor,
        start_rviz2
    ])