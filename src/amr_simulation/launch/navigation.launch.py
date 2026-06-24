import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Deklarasi alamat paket
    pkg_amr = get_package_share_directory('amr_simulation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # 2. Tunjuk lokasi sertifikat rumah, otak parameter, dan config rviz
    map_file = os.path.join(pkg_amr, 'maps', 'peta_rumah.yaml')
    params_file = os.path.join(pkg_amr, 'config', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(pkg_amr, 'rviz', 'slam_config.rviz')

    # 3. Panggil ekosistem Nav2 secara penuh
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file
        }.items()
    )

    # 4. MANTRA VISUALISASI RVIZ2 OTOMATIS (Biar tidak kerja manual!)
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}]
    )

    # Panggil Nav2 dan RViz2 bersamaan
    return LaunchDescription([
        start_nav2,
        start_rviz2
    ])