import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr = get_package_share_directory('amr_simulation')

    # Default map ke peta_l1_dc.yaml
    default_map = os.path.join(pkg_amr, 'maps', 'peta_l1_dc.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.join(os.getcwd(), 'src', 'amr_simulation', 'maps', 'peta_l1_dc.yaml')

    default_rviz = os.path.join(pkg_amr, 'rviz', 'slam_config.rviz')
    if not os.path.exists(default_rviz):
        default_rviz = os.path.join(os.getcwd(), 'src', 'amr_simulation', 'rviz', 'slam_config.rviz')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path lengkap ke file peta .yaml yang ingin dibuka'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Path ke konfigurasi RViz'
    )

    # 1. Map Server untuk memuat file YAML & PGM ke topic /map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )

    # 2. Lifecycle Manager untuk mengaktifkan Map Server otomatis
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. RViz2 Visualizer
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        map_arg,
        rviz_config_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz2_node
    ])
