import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file untuk AMR Dashboard (Robot Fisik).
    Contoh penggunaan:
      ros2 launch amr_dashboard dashboard.launch.py
    """
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Gunakan clock ROS2 eksternal (default False untuk robot fisik)'
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
        }]
    )

    delayed_dashboard = TimerAction(
        period=1.5,
        actions=[dashboard_server]
    )

    # 3. Robot Voice & Bluetooth Watchdog Node
    robot_voice_node = Node(
        package='amr_simulation',
        executable='test_robot_voice.py',
        name='robot_voice_node',
        output='screen',
        arguments=['--node'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        waypoint_manager_node,
        robot_voice_node,
        delayed_dashboard,
    ])
