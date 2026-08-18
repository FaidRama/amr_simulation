import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_amr_simulation = get_package_share_directory('amr_simulation')

    # Deklarasi argumen: mode "slam" (untuk bikin peta) atau "nav" (untuk navigasi A*)
    slam_arg = DeclareLaunchArgument('slam', default_value='False', description='Jalankan SLAM untuk mapping')
    nav_arg = DeclareLaunchArgument('nav', default_value='True', description='Jalankan Navigasi A*')

    # Path ke file launch masing-masing
    gazebo_launch_path = os.path.join(pkg_amr_simulation, 'launch', 'gazebo.launch.py')
    slam_launch_path = os.path.join(pkg_amr_simulation, 'launch', 'slam.launch.py')
    nav_launch_path = os.path.join(pkg_amr_simulation, 'launch', 'navigation.launch.py')

    # 1. Selalu jalankan Gazebo Simulasi
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # 2. Jalankan SLAM jika argument slam:=true
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # 3. Jalankan Navigasi jika argument nav:=true
    start_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    return LaunchDescription([
        slam_arg,
        nav_arg,
        start_gazebo,
        start_slam,
        start_nav
    ])
