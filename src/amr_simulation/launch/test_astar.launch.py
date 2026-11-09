#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 1. Direktori paket
    pkg_amr_simulation = get_package_share_directory('amr_simulation')

    # 2. File default
    default_map = os.path.join(pkg_amr_simulation, 'maps', 'peta_l1_dc.yaml')
    default_params = os.path.join(pkg_amr_simulation, 'config', 'astar_params.yaml')
    default_rviz = os.path.join(pkg_amr_simulation, 'rviz', 'astar_test.rviz')

    # 3. Deklarasi Argumen Launch
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Lokasi file peta .yaml'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Lokasi file konfigurasi parameter A* (astar_params.yaml)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi Gazebo jika True, atau wall-clock jika False'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Aktifkan otomatis lifecycle node Nav2'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Jalankan visualisasi RViz2 (set true jika di laptop dengan layar, false untuk headless Jetson)'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Broadcast TF virtual map->base_footprint dari astar_benchmark. Set false agar tidak konflik dengan uji_astar_dijkstra_skripsi.'
    )
    interactive_arg = DeclareLaunchArgument(
        'interactive',
        default_value='false',
        description='Jalankan node interaktif astar_benchmark.py untuk klik manual di RViz2'
    )
    start_x_arg = DeclareLaunchArgument('start_x', default_value='0.0', description='Titik X awal robot')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='0.0', description='Titik Y awal robot')
    start_yaw_arg = DeclareLaunchArgument('start_yaw', default_value='0.0', description='Orientasi awal robot (radian)')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    publish_tf = LaunchConfiguration('publish_tf')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    # Remappings TF
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Substitusi parameter dinamis ke file YAML
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 4. Node 1: Map Server (Menyediakan peta untuk Global Costmap & A*)
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 5. Node 2: Planner Server (Otak A* - menghitung jalur global tanpa controller/penggerak)
    node_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 6. Node 3: Lifecycle Manager (Hanya menghidupkan map_server dan planner_server)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_astar',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'planner_server']
        }]
    )

    # 7. Node 4: A* Benchmark & Tester Node (Penyadap goal, pemanggil action, & penghitung metrik)
    node_astar_benchmark = Node(
        package='amr_simulation',
        executable='astar_benchmark.py',
        name='astar_benchmark',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': publish_tf,
            'default_start_x': start_x,
            'default_start_y': start_y,
            'default_start_yaw': start_yaw
        }],
        condition=IfCondition(LaunchConfiguration('interactive'))
    )

    # 8. Visualisasi RViz2 (Opsional, hanya jika rviz:=true)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_astar',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    return LaunchDescription([
        stdout_linebuf_envvar,
        map_arg,
        params_arg,
        use_sim_time_arg,
        autostart_arg,
        rviz_arg,
        publish_tf_arg,
        interactive_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,

        node_map_server,
        node_planner_server,
        node_lifecycle_manager,
        node_astar_benchmark,
        node_rviz
    ])
