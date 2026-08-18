import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'robo_pathfinding_description'
    xacro_file = 'robo_pathfinding.xacro'
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', xacro_file)

    # 1. Cari lokasi folder package amr_simulation milikmu yang sudah di-build
    pkg_amr_simulation = get_package_share_directory('amr_simulation')
    
    
    # 1. Tentukan alamat rumah hasil curian
    rumah_world_path = os.path.join(
        pkg_amr_simulation,
        'worlds',
        'peta_dc.world'
    )

    # Parser XACRO ke URDF otomatis
    robot_desc = Command(['xacro ', xacro_path])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. Jalankan otak simulasi (Backend) dan PAKSA menelan rumah
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        # IKI LHO LE KUNCINE! Argumen world disuntikkan langsung ke gzserver
        launch_arguments={'world': rumah_world_path, 'verbose': 'true'}.items() 
    )

    # 3. MANTRA TERMINAL OTOMATIS: Meniru ketikan manualmu di terminal baru
    # Jeda 2 detik, lalu panggil gzclient polosan TANPA plugin EOL!
    gzclient_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 2 && gzclient'],
        output='screen'
    )
    
    # 4. Jatuhkan robot dari langit (ketinggian Z = 0.5 meter)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robo_pathfinding',
                   '-z', '0.5',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0'],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,  # Beri waktu 3 detik agar rumah selesai di-render
        actions=[spawn_entity]
    )

    # DAFTAR PEKERJA ABSOLUT (Tanpa ada yang tumpang tindih!)
    return LaunchDescription([
        node_robot_state_publisher,
        gzserver_cmd,
        gzclient_cmd,
        delayed_spawn
    ])