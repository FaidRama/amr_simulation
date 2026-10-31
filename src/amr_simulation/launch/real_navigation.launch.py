import os
import time
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _get_ancestor_pids():
    """Ambil semua PID leluhur (proses ros2 launch saat ini, bash, sshd, dll) agar tidak ikut terbunuh."""
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
    """
    Bersihkan secara sinkron seluruh proses navigasi/mapping yatim dari sesi sebelumnya
    SEBELUM node-node baru diluncurkan. Mengecualikan proses launch ini sendiri.
    """
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
    # 0. Bersihkan sisa proses sesi sebelumnya secara sinkron sebelum memulai node baru
    _cleanup_stale_processes()

    pkg_amr = get_package_share_directory('amr_simulation')
    pkg_bt_navigator = get_package_share_directory('nav2_bt_navigator')

    default_map_path = os.path.join(pkg_amr, 'maps', 'peta_l1_ddc.yaml')
    src_params_path = os.path.join(os.getcwd(), 'src', 'amr_simulation', 'config', 'nav2_params.yaml')
    default_params_path = src_params_path if os.path.exists(src_params_path) else os.path.join(pkg_amr, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_amr, 'rviz', 'slam_config.rviz')
    default_bt_xml_path = os.path.join(
        pkg_bt_navigator, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )

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

    # 1. Driver RPLiDAR A1 Fisik via rplidar_bridge (Sudah memiliki internal auto-recovery sendiri)
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
        }]
    )

    # 3. Arduino Bridge (Odometry Asli & Motor Control via /dev/arduino - memiliki internal auto-reconnect)
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
        condition=IfCondition(LaunchConfiguration('use_arduino'))
    )

    # 4. Static Transforms (FIKS PERMANEN: base_link -> laser & base_link -> base_footprint)
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

    # 5. Adaptive Velocity & Nav2 Bootstrap Node (dijalankan sejak t=0s agar TF map->odom langsung tersedia)
    start_adaptive_velocity = Node(
        package='amr_simulation',
        executable='adaptive_velocity_node.py',
        name='adaptive_velocity_node',
        output='screen'
    )

    # 6. Unified Nav2 Stack (Map Server + AMCL + Controller + Planner + Recoveries + BT Navigator + Waypoint Follower)
    # Dikelola oleh SATU lifecycle_manager_navigation secara berurutan:
    #   Tahap 1 (CONFIGURE): map_server -> amcl -> controller_server -> planner_server -> recoveries -> bt_navigator -> waypoint_follower
    #   Tahap 2 (ACTIVATE) : map_server -> amcl -> controller_server -> planner_server -> recoveries -> bt_navigator -> waypoint_follower
    # Dengan urutan ini, map_server baru mem-publish /map pada Tahap 2 (SETELAH planner_server selesai menginisialisasi
    # static_layer, obstacle_layer, dan inflation_layer pada Tahap 1), sehingga mencegah 100% bug race condition
    # "StaticLayer: Resizing costmap" vs "Using plugin inflation_layer" (exit code -11) di ROS 2 Foxy.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yaml_filename': LaunchConfiguration('map'),
        'default_bt_xml_filename': default_bt_xml_path,
        'autostart': LaunchConfiguration('autostart'),
        'map_subscribe_transient_local': 'true',
    }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    nav2_lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': nav2_lifecycle_nodes
        }]
    )

    delayed_nav2_stack = TimerAction(
        period=2.5,
        actions=[
            node_map_server,
            node_amcl,
            node_controller_server,
            node_planner_server,
            node_recoveries_server,
            node_bt_navigator,
            node_waypoint_follower,
            node_lifecycle_manager,
        ]
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

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    return LaunchDescription([
        stdout_linebuf_envvar,
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
        start_adaptive_velocity,
        delayed_nav2_stack,
        start_rviz2
    ])