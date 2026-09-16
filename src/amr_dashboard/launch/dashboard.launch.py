import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file untuk AMR Dashboard.
    Menjalankan:
    1. rosbridge_websocket (opsional, jika terinstall)
    2. waypoint_manager node
    3. Flask dashboard server
    """

    # 1. Rosbridge WebSocket (opsional — cek dulu apakah terinstall)
    # Jika tidak terinstall, dashboard tetap bisa jalan via REST API
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
            'retry_startup_delay': 5.0,
        }],
        # Condition: hanya jalan jika package ada
        # Jika error, dashboard tetap bisa diakses via REST
    )

    # 2. Waypoint Manager Node
    waypoint_manager_node = Node(
        package='amr_dashboard',
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Flask Dashboard Server (dijalankan sebagai process)
    # Ditunda 2 detik agar ROS2 nodes siap dulu
    dashboard_server = Node(
        package='amr_dashboard',
        executable='dashboard_server',
        name='dashboard_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_dashboard = TimerAction(
        period=2.0,
        actions=[dashboard_server]
    )

    return LaunchDescription([
        # rosbridge_node,  # Uncomment setelah install: sudo apt install ros-foxy-rosbridge-suite
        waypoint_manager_node,
        delayed_dashboard,
    ])
