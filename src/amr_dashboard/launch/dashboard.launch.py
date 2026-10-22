import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file untuk AMR Dashboard (Mendukung Mode Simulasi & Mode Robot Nyata).
    Contoh penggunaan:
    - Mode Robot Nyata (Real):
      ros2 launch amr_dashboard dashboard.launch.py mode:=real use_sim_time:=False
    - Mode Simulasi (Gazebo):
      ros2 launch amr_dashboard dashboard.launch.py mode:=sim use_sim_time:=True
    """
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode operasi awal dashboard: "sim" (Gazebo) atau "real" (Robot Fisik)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Gunakan waktu simulasi Gazebo (True untuk sim, False untuk real)'
    )

    # 1. Waypoint Manager Node
    waypoint_manager_node = Node(
        package='amr_dashboard',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 2. Flask Dashboard Server
    # Ditunda 1.5 detik agar ROS2 nodes siap dulu
    dashboard_server = Node(
        package='amr_dashboard',
        executable='dashboard_server',
        name='dashboard_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'initial_mode': LaunchConfiguration('mode'),
        }]
    )

    delayed_dashboard = TimerAction(
        period=1.5,
        actions=[dashboard_server]
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        waypoint_manager_node,
        delayed_dashboard,
    ])
