import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    
    # 1. MANTRA FILTER LASER (Membersihkan bayangan tiang aluminium)
    start_laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[
            os.path.join(get_package_share_directory('amr_simulation'), 'config', 'laser_filter_config.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[
            ('scan', '/scan'),                # Sedot laser asli dari Gazebo
            ('scan_filtered', '/scan_filtered') # Buang hasil bersih ke sini
        ]
    )

    # 2. MANTRA OTAK SLAM (Tukang Gambar Peta)
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
                'use_sim_time': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                # SANGAT KRUSIAL: SLAM sekarang harus baca laser yang sudah disaring!
                'scan_topic': '/scan_filtered' 
            }
        ]
    )

    # Tambahkan path konfigurasi rviz di bagian atas (di bawah deklarasi variabel lain)
    rviz_config_dir = os.path.join(get_package_share_directory('amr_simulation'), 'rviz', 'slam_config.rviz')

    # MANTRA VISUALISASI RVIZ2 (Otomatis muat config)
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir], # IKI KUNCINE LE!
        parameters=[{'use_sim_time': True}]
    )

    # Memanggil semua pekerja untuk turun ke lapangan
    return LaunchDescription([
        start_laser_filter,
        start_slam_toolbox,
        start_rviz2
    ])