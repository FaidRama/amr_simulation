#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Node Uji Coba & Benchmarking Global Planner A* (Delivery_AMR)
Memungkinkan eksperimen parameter A* & Global Costmap di RViz2
tanpa robot bergerak (controller dinonaktifkan).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class AstarBenchmarkNode(Node):
    def __init__(self):
        super().__init__('astar_benchmark')

        # Deklarasi Parameter
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_start_y', 0.0)
        self.declare_parameter('default_start_yaw', 0.0)
        self.declare_parameter('planner_id', 'GridBased')

        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.start_x = self.get_parameter('default_start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('default_start_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('default_start_yaw').get_parameter_value().double_value
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value

        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        self.planning_in_progress = False
        self.start_calc_time = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Action Client ComputePathToPose (disediakan oleh planner_server Nav2)
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Publisher Rute (/plan) untuk divisualisasikan oleh RViz2
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        # Publisher Marker untuk menandai Start & Goal di RViz2
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_markers', 10)

        # Subscriber 2D Pose Estimate (Pengaturan Titik Start dari RViz2)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # Subscriber 2D Goal Pose (Titik Target dari RViz2)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Timer untuk broadcast TF dan Marker berkala (30 Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info('====================================================')
        self.get_logger().info('🚀 Node Uji Coba Parameter A* Siap!')
        self.get_logger().info(f'📍 Titik Start default: X={self.start_x:.2f} m, Y={self.start_y:.2f} m')
        self.get_logger().info('👉 Klik "2D Pose Estimate" di RViz2 untuk memindah posisi Start.')
        self.get_logger().info('👉 Klik "2D Goal Pose" di RViz2 untuk menguji rute A*.')
        self.get_logger().info('🛡️  Robot TIDAK AKAN BERGERAK. Hanya menghitung rute!')
        self.get_logger().info('====================================================')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """Menerima posisi awal baru dari RViz2 (2D Pose Estimate)."""
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y
        self.start_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        yaw_deg = math.degrees(self.start_yaw)
        print(f"\n\033[94m[START DIUBAH] -> Posisi Start: X = {self.start_x:.2f} m, Y = {self.start_y:.2f} m, Yaw = {yaw_deg:.1f}°\033[0m", flush=True)
        self.publish_markers()

    def goal_callback(self, msg: PoseStamped):
        """Menerima target dari RViz2 (2D Goal Pose) dan memanggil planner_server."""
        if self.planning_in_progress:
            self.get_logger().warn('⚠️ Perhitungan A* sebelumnya sedang berlangsung, mohon tunggu...')
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = euler_from_quaternion(msg.pose.orientation)

        print(f"\n\033[93m[GOAL DITERIMA] -> Target: X = {self.goal_x:.2f} m, Y = {self.goal_y:.2f} m\033[0m", flush=True)
        print("⏳ Menghubungi planner_server (A*)...", flush=True)

        self.publish_markers()

        # Tunggu Action Server jika belum online
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Action Server "compute_path_to_pose" belum siap! Pastikan planner_server sudah running.')
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = msg
        # Pastikan timestamp terisi dengan waktu sekarang
        if goal_msg.pose.header.stamp.sec == 0 and goal_msg.pose.header.stamp.nanosec == 0:
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        if not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = 'map'
        goal_msg.planner_id = self.planner_id

        self.planning_in_progress = True
        self.start_calc_time = time.perf_counter()

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Permintaan rute DITOLAK oleh planner_server!')
            self.planning_in_progress = False
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        elapsed_time_ms = (time.perf_counter() - self.start_calc_time) * 1000.0
        self.planning_in_progress = False

        result = future.result().result
        path = result.path

        # Jika ada waktu planning bawaan dari Nav2, kita juga bisa gunakan
        nav2_time_ms = result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6

        if len(path.poses) == 0:
            print("\n" + "=" * 65, flush=True)
            print("\033[91m❌ [A* GAGAL MENEMUKAN RUTE]\033[0m", flush=True)
            print("-" * 65, flush=True)
            print(f"⏱️  Waktu Komputasi    : {elapsed_time_ms:.2f} ms", flush=True)
            print(f"📍 Dari Titik Start  : X = {self.start_x:.2f} m, Y = {self.start_y:.2f} m", flush=True)
            print(f"🏁 Ke Titik Goal     : X = {self.goal_x:.2f} m, Y = {self.goal_y:.2f} m", flush=True)
            print("-" * 65, flush=True)
            print("💡 Kemungkinan Masalah:", flush=True)
            print("   1. Titik Goal berada di dalam obstacle / tembok (Tingkatkan parameter tolerance).", flush=True)
            print("   2. Titik Goal berada di area unknown dan allow_unknown diset False.", flush=True)
            print("   3. Nilai robot_radius atau inflation_radius terlalu besar sehingga lorong tertutup.", flush=True)
            print("=" * 65 + "\n", flush=True)
            return

        # Hitung Panjang Rute (Total Jarak Euclidean)
        total_distance = 0.0
        poses = path.poses
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            total_distance += math.hypot(p2.x - p1.x, p2.y - p1.y)

        # Hitung Jarak Garis Lurus (Euclidean Direct Distance)
        straight_dist = math.hypot(self.goal_x - self.start_x, self.goal_y - self.start_y)
        efficiency = (straight_dist / total_distance * 100.0) if total_distance > 0 else 100.0

        # Publikasikan rute ke /plan agar RViz2 menampilkannya
        self.path_pub.publish(path)

        # Cetak Hasil Benchmark Rapi dan Berwarna
        print("\n" + "=" * 65, flush=True)
        print("\033[92m           📊 HASIL UJI COBA GLOBAL PLANNER (A*)\033[0m", flush=True)
        print("=" * 65, flush=True)
        print(f" 🎯 Status Rute       : \033[92mBERHASIL (Path Ditemukan)\033[0m", flush=True)
        print(f" ⚙️  Planner Plugin    : {self.planner_id} (NavfnPlanner / A*)", flush=True)
        print(f" ⏱️  Waktu Komputasi   : \033[96m{elapsed_time_ms:.2f} ms\033[0m (Nav2 internal: {nav2_time_ms:.2f} ms)", flush=True)
        print(f" 📏 Panjang Rute A*   : \033[93m{total_distance:.2f} meter\033[0m", flush=True)
        print(f" 📐 Jarak Garis Lurus : {straight_dist:.2f} meter", flush=True)
        print(f" 📈 Rasio Efisiensi   : \033[95m{efficiency:.1f} %\033[0m (Garis lurus vs jarak rute)", flush=True)
        print(f" 🔢 Jumlah Waypoint   : {len(poses)} titik koordinat", flush=True)
        print(f" 📍 Titik Start       : X = {self.start_x:.2f} m, Y = {self.start_y:.2f} m", flush=True)
        print(f" 🏁 Titik Goal        : X = {self.goal_x:.2f} m, Y = {self.goal_y:.2f} m", flush=True)
        print("-" * 65, flush=True)
        print(" 💡 Rute hijau telah digambar di RViz2 (Topik: /plan).", flush=True)
        print(" 💡 Robot TIDAK BERGERAK. Kamu bisa ubah parameter di astar_params.yaml", flush=True)
        print("=" * 65 + "\n", flush=True)

    def timer_callback(self):
        """Broadcast TF dari map -> odom -> base_footprint secara kontinu."""
        now = self.get_clock().now().to_msg()

        if self.publish_tf:
            # 1. Transform: map -> odom
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = now
            t_map_odom.header.frame_id = 'map'
            t_map_odom.child_frame_id = 'odom'
            t_map_odom.transform.translation.x = 0.0
            t_map_odom.transform.translation.y = 0.0
            t_map_odom.transform.translation.z = 0.0
            t_map_odom.transform.rotation.w = 1.0

            # 2. Transform: odom -> base_footprint
            t_odom_base = TransformStamped()
            t_odom_base.header.stamp = now
            t_odom_base.header.frame_id = 'odom'
            t_odom_base.child_frame_id = 'base_footprint'
            t_odom_base.transform.translation.x = self.start_x
            t_odom_base.transform.translation.y = self.start_y
            t_odom_base.transform.translation.z = 0.0
            t_odom_base.transform.rotation = quaternion_from_euler(0.0, 0.0, self.start_yaw)

            self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])

    def publish_markers(self):
        """Membuat visual marker titik Start (Hijau) dan Goal (Merah) di RViz2."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: Titik Start (Silinder Hijau)
        m_start = Marker()
        m_start.header.stamp = now
        m_start.header.frame_id = 'map'
        m_start.ns = 'astar_test'
        m_start.id = 1
        m_start.type = Marker.CYLINDER
        m_start.action = Marker.ADD
        m_start.pose.position.x = self.start_x
        m_start.pose.position.y = self.start_y
        m_start.pose.position.z = 0.1
        m_start.pose.orientation = quaternion_from_euler(0.0, 0.0, self.start_yaw)
        m_start.scale.x = 0.4
        m_start.scale.y = 0.4
        m_start.scale.z = 0.15
        m_start.color.r = 0.1
        m_start.color.g = 0.9
        m_start.color.b = 0.1
        m_start.color.a = 0.8
        marker_array.markers.append(m_start)

        # Marker 2: Teks "START [A*]"
        m_start_text = Marker()
        m_start_text.header.stamp = now
        m_start_text.header.frame_id = 'map'
        m_start_text.ns = 'astar_test'
        m_start_text.id = 2
        m_start_text.type = Marker.TEXT_VIEW_FACING
        m_start_text.action = Marker.ADD
        m_start_text.pose.position.x = self.start_x
        m_start_text.pose.position.y = self.start_y
        m_start_text.pose.position.z = 0.4
        m_start_text.text = f"START ({self.start_x:.1f}, {self.start_y:.1f})"
        m_start_text.scale.z = 0.25
        m_start_text.color.r = 1.0
        m_start_text.color.g = 1.0
        m_start_text.color.b = 1.0
        m_start_text.color.a = 1.0
        marker_array.markers.append(m_start_text)

        # Marker 3: Titik Goal (Silinder Merah) jika sudah ada goal
        if self.goal_x is not None:
            m_goal = Marker()
            m_goal.header.stamp = now
            m_goal.header.frame_id = 'map'
            m_goal.ns = 'astar_test'
            m_goal.id = 3
            m_goal.type = Marker.CYLINDER
            m_goal.action = Marker.ADD
            m_goal.pose.position.x = self.goal_x
            m_goal.pose.position.y = self.goal_y
            m_goal.pose.position.z = 0.1
            m_goal.scale.x = 0.4
            m_goal.scale.y = 0.4
            m_goal.scale.z = 0.15
            m_goal.color.r = 0.9
            m_goal.color.g = 0.1
            m_goal.color.b = 0.1
            m_goal.color.a = 0.8
            marker_array.markers.append(m_goal)

            m_goal_text = Marker()
            m_goal_text.header.stamp = now
            m_goal_text.header.frame_id = 'map'
            m_goal_text.ns = 'astar_test'
            m_goal_text.id = 4
            m_goal_text.type = Marker.TEXT_VIEW_FACING
            m_goal_text.action = Marker.ADD
            m_goal_text.pose.position.x = self.goal_x
            m_goal_text.pose.position.y = self.goal_y
            m_goal_text.pose.position.z = 0.4
            m_goal_text.text = f"GOAL ({self.goal_x:.1f}, {self.goal_y:.1f})"
            m_goal_text.scale.z = 0.25
            m_goal_text.color.r = 1.0
            m_goal_text.color.g = 1.0
            m_goal_text.color.b = 1.0
            m_goal_text.color.a = 1.0
            marker_array.markers.append(m_goal_text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AstarBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
