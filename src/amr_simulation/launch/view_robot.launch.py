import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_sim = get_package_share_directory('amr_simulation')
    pkg_amr_desc = get_package_share_directory('Delivery_AMR_description')

    default_rviz_config = os.path.join(pkg_amr_sim, 'rviz', 'slam_config.rviz')
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'Delivery_AMR.xacro')

    # Proses Xacro secara lokal di laptop
    try:
        import xacro
        robot_description_config = xacro.process_file(xacro_file)
        robot_urdf = robot_description_config.toxml()
    except Exception as e:
        from launch.substitutions import Command
        robot_urdf = Command(['xacro ', xacro_file])

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path ke file konfigurasi RViz (.rviz)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Gunakan waktu simulasi (False untuk robot asli)'
    )

    # 1. Robot State Publisher (Berjalan 100% di Laptop - 0% beban Jetson)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 2. Joint State Publisher (Dummy status untuk sendi roda continuous)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 3. RViz2 Visualizer
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz2_node
    ])
