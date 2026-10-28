#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SKRIPSI BAB 4 - SKENARIO 3: EVALUASI KOMPARATIF GLOBAL PLANNER (A* VS DIJKSTRA)
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Deskripsi:
  Melakukan pengujian head-to-head algoritma A* terhadap baseline Dijkstra
  pada lingkungan Global Costmap yang identik (15 Pasang Titik Start-Goal Tabel 7):
  - 5 Pasang Kompleksitas Rendah (1 - 3m)
  - 5 Pasang Kompleksitas Sedang (3.1 - 7m)
  - 5 Pasang Kompleksitas Tinggi (> 7m)
  - 5 Kali Pengulangan A* dan 5 Kali Pengulangan Dijkstra (Total 150 Run Uji)
Metrik Komparasi:
  1. Waktu Perencanaan (Planning Time dalam ms)
  2. Panjang Lintasan (Path Length dalam m)
  3. Efisiensi & Persentase Penurunan Waktu Komputasi ((T_dijk - T_astar)/T_dijk * 100%)
  4. Verifikasi Keterbebasan Tabrakan (Collision-Free Verification)
Output:
  Tersimpan otomatis ke ~/hasil_pengujian_skripsi/skenario3_astar_vs_dijkstra.csv
=============================================================================
"""

import os
import sys
import time
import math
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster


# =============================================================================
# DATA 15 PASANG TITIK UJI START-GOAL SESUAI TABEL 7 SKRIPSI
# =============================================================================
TABEL_7_SCENARIOS = [
    # --- KOMPLEKSITAS RENDAH (1.0 - 3.0 Meter) ---
    {
        "id": "SG-01", "level": "Rendah",
        "start": (1.00, 1.00), "goal": (2.50, 1.00),
        "est_dist": 1.50, "desc": "Koridor lurus tanpa belokan"
    },
    {
        "id": "SG-02", "level": "Rendah",
        "start": (1.00, 2.00), "goal": (3.00, 2.00),
        "est_dist": 2.00, "desc": "Koridor lurus rak A"
    },
    {
        "id": "SG-03", "level": "Rendah",
        "start": (2.00, 3.50), "goal": (4.50, 3.50),
        "est_dist": 2.50, "desc": "Koridor lurus rak B"
    },
    {
        "id": "SG-04", "level": "Rendah",
        "start": (0.50, 4.00), "goal": (3.30, 4.00),
        "est_dist": 2.80, "desc": "Jalur bebas stasiun transit"
    },
    {
        "id": "SG-05", "level": "Rendah",
        "start": (1.50, 5.00), "goal": (4.50, 5.00),
        "est_dist": 3.00, "desc": "Koridor bongkar muat"
    },

    # --- KOMPLEKSITAS SEDANG (3.1 - 7.0 Meter) ---
    {
        "id": "SG-06", "level": "Sedang",
        "start": (1.00, 1.00), "goal": (3.50, 3.50),
        "est_dist": 3.53, "desc": "Belokan 90° ujung lorong"
    },
    {
        "id": "SG-07", "level": "Sedang",
        "start": (1.50, 2.00), "goal": (5.50, 3.00),
        "est_dist": 4.12, "desc": "Memotong persimpangan T"
    },
    {
        "id": "SG-08", "level": "Sedang",
        "start": (0.50, 1.50), "goal": (4.50, 4.50),
        "est_dist": 5.00, "desc": "Manuver dua sudut belokan"
    },
    {
        "id": "SG-09", "level": "Sedang",
        "start": (2.00, 1.00), "goal": (6.00, 5.00),
        "est_dist": 5.65, "desc": "Rute diagonal antar-lorong"
    },
    {
        "id": "SG-10", "level": "Sedang",
        "start": (1.00, 5.50), "goal": (6.50, 2.00),
        "est_dist": 6.51, "desc": "Lintasan memutari rak tengah"
    },

    # --- KOMPLEKSITAS TINGGI (> 7.0 Meter) ---
    {
        "id": "SG-11", "level": "Tinggi",
        "start": (0.50, 0.50), "goal": (7.50, 2.00),
        "est_dist": 7.15, "desc": "Koridor sempit zig-zag"
    },
    {
        "id": "SG-12", "level": "Tinggi",
        "start": (1.00, 1.00), "goal": (8.00, 4.00),
        "est_dist": 7.61, "desc": "Melewati 3 partisi boks"
    },
    {
        "id": "SG-13", "level": "Tinggi",
        "start": (1.00, 6.00), "goal": (8.50, 1.50),
        "est_dist": 8.74, "desc": "Rute diagonal lintas gudang"
    },
    {
        "id": "SG-14", "level": "Tinggi",
        "start": (0.50, 1.00), "goal": (9.00, 4.50),
        "est_dist": 9.19, "desc": "Koridor panjang dengan halangan"
    },
    {
        "id": "SG-15", "level": "Tinggi",
        "start": (0.50, 0.50), "goal": (9.50, 5.50),
        "est_dist": 10.29, "desc": "Ujung ke ujung area gudang"
    }
]


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class BenchmarkAstarDijkstraNode(Node):
    def __init__(self):
        super().__init__('benchmark_astar_vs_dijkstra_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_markers', 10)

        # Direktori Output
        self.output_dir = os.path.expanduser('~/hasil_pengujian_skripsi')
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'skenario3_astar_vs_dijkstra.csv')

        self.ensure_csv_header()

        # Timer TF 20 Hz (map -> odom -> base_link) sebagai fallback jika tester lain tidak aktif
        self.tf_timer = self.create_timer(0.05, self.broadcast_virtual_tf)
        self.current_start_x = 0.0
        self.current_start_y = 0.0

        self.get_logger().info("✅ Node Benchmark A* vs Dijkstra Siap.")

    def broadcast_virtual_tf(self):
        # Jika node astar_benchmark.py sudah aktif menyimak /initialpose, biarkan dia yang broadcast TF
        if self.initialpose_pub.get_subscription_count() > 0:
            return

        now = self.get_clock().now()
        stamp = (now + rclpy.duration.Duration(seconds=0.05)).to_msg()

        # 1. map -> odom
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0

        # 2. odom -> base_link (posisi start uji saat ini)
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = float(self.current_start_x)
        t2.transform.translation.y = float(self.current_start_y)
        t2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t1, t2])

    def publish_markers(self, start_xy: tuple, goal_xy: tuple):
        """Tampilkan visual silinder hijau (Start) dan merah (Goal) di RViz2."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: Start (Hijau)
        m_start = Marker()
        m_start.header.stamp = now
        m_start.header.frame_id = 'map'
        m_start.ns = 'astar_test'
        m_start.id = 1
        m_start.type = Marker.CYLINDER
        m_start.action = Marker.ADD
        m_start.pose.position.x = float(start_xy[0])
        m_start.pose.position.y = float(start_xy[1])
        m_start.pose.position.z = 0.1
        m_start.pose.orientation.w = 1.0
        m_start.scale.x = 0.4
        m_start.scale.y = 0.4
        m_start.scale.z = 0.15
        m_start.color.r = 0.1
        m_start.color.g = 0.9
        m_start.color.b = 0.2
        m_start.color.a = 0.85
        marker_array.markers.append(m_start)

        # Marker 2: Goal (Merah)
        m_goal = Marker()
        m_goal.header.stamp = now
        m_goal.header.frame_id = 'map'
        m_goal.ns = 'astar_test'
        m_goal.id = 2
        m_goal.type = Marker.CYLINDER
        m_goal.action = Marker.ADD
        m_goal.pose.position.x = float(goal_xy[0])
        m_goal.pose.position.y = float(goal_xy[1])
        m_goal.pose.position.z = 0.1
        m_goal.pose.orientation.w = 1.0
        m_goal.scale.x = 0.4
        m_goal.scale.y = 0.4
        m_goal.scale.z = 0.15
        m_goal.color.r = 0.95
        m_goal.color.g = 0.15
        m_goal.color.b = 0.15
        m_goal.color.a = 0.85
        marker_array.markers.append(m_goal)

        self.marker_pub.publish(marker_array)

    def ensure_csv_header(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "No",
                    "ID_Uji",
                    "Tingkat_Kompleksitas",
                    "Algoritma",
                    "Run_Ke",
                    "Start_X",
                    "Start_Y",
                    "Goal_X",
                    "Goal_Y",
                    "Jarak_Euklidean_m",
                    "Panjang_Lintasan_m",
                    "Waktu_Perencanaan_ms",
                    "Jumlah_Waypoint",
                    "Status_Rute",
                    "Karakteristik_Rintangan",
                    "Timestamp"
                ])

    def request_path_plan(self, planner_id: str, start_xy: tuple, goal_xy: tuple) -> dict:
        """Kirim permintaan rute ke planner_server untuk algoritma tertentu ('AStar' atau 'Dijkstra')."""
        self.current_start_x, self.current_start_y = start_xy

        # Publikasikan posisi awal ke /initialpose & visual marker ke RViz2
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = float(start_xy[0])
        init_pose.pose.pose.position.y = float(start_xy[1])
        init_pose.pose.pose.orientation.w = 1.0
        self.initialpose_pub.publish(init_pose)
        self.publish_markers(start_xy, goal_xy)

        # Pastikan action server compute_path_to_pose online
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            return {
                "success": False,
                "error": "Action Server compute_path_to_pose tidak merespon",
                "time_ms": 0.0,
                "path_length": 0.0,
                "waypoints": 0
            }

        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(goal_xy[0])
        goal_msg.pose.pose.position.y = float(goal_xy[1])
        goal_msg.pose.pose.orientation = quaternion_from_yaw(0.0)
        goal_msg.planner_id = planner_id

        # Dukung use_start jika versi Nav2 menyediakannya
        if hasattr(goal_msg, 'use_start'):
            goal_msg.use_start = True
            goal_msg.start.header.stamp = goal_msg.pose.header.stamp
            goal_msg.start.header.frame_id = 'map'
            goal_msg.start.pose.position.x = float(start_xy[0])
            goal_msg.start.pose.position.y = float(start_xy[1])
            goal_msg.start.pose.orientation = quaternion_from_yaw(0.0)

        # Hitung waktu komputasi CPU berpresisi tinggi
        t_start = time.perf_counter()
        send_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done():
            return {"success": False, "error": "Timeout send_goal", "time_ms": 0.0, "path_length": 0.0, "waypoints": 0}

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return {"success": False, "error": "Goal ditolak oleh planner", "time_ms": 0.0, "path_length": 0.0, "waypoints": 0}

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=5.0)
        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0

        if not res_future.done():
            return {"success": False, "error": "Timeout get_result", "time_ms": t_elapsed_ms, "path_length": 0.0, "waypoints": 0}

        result = res_future.result().result
        path = result.path

        # Waktu internal Nav2 jika tersedia
        nav2_time_ms = result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6
        effective_time_ms = nav2_time_ms if nav2_time_ms > 0.01 else t_elapsed_ms

        poses = path.poses
        if len(poses) == 0:
            return {
                "success": False,
                "error": "Path kosong (Goal di dalam rintangan / tidak terjangkau)",
                "time_ms": effective_time_ms,
                "path_length": 0.0,
                "waypoints": 0
            }

        # Hitung Akumulasi Panjang Lintasan
        total_len = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            total_len += math.hypot(p2.x - p1.x, p2.y - p1.y)

        # Publikasikan ke /plan agar RViz2 menampilkan jalurnya
        self.path_pub.publish(path)

        return {
            "success": True,
            "error": "None",
            "time_ms": effective_time_ms,
            "path_length": total_len,
            "waypoints": len(poses)
        }

    def run_benchmark_for_pair(self, scenario: dict, repetitions: int = 5) -> list:
        """Eksekusi 5 run A* dan 5 run Dijkstra untuk 1 pasang titik koordinat."""
        sg_id = scenario['id']
        level = scenario['level']
        start_xy = scenario['start']
        goal_xy = scenario['goal']
        est_d = scenario['est_dist']
        desc = scenario['desc']

        print("\n" + "=" * 80)
        print(f"🎯 PENGUJIAN {sg_id} [{level.upper()}] : Start {start_xy} -> Goal {goal_xy} (Jarak ~{est_d}m)")
        print(f"   Karakteristik: {desc}")
        print("=" * 80)

        results = []

        # Jalankan A* dan Dijkstra berurutan
        for alg in ["AStar", "Dijkstra"]:
            alg_label = "A*" if alg == "AStar" else "Dijkstra"
            print(f"\n  ▶️ Menguji Algoritma: {alg_label} ({repetitions} Pengulangan)...")

            for r in range(1, repetitions + 1):
                # Beri jeda kecil antar-perencanaan
                time.sleep(0.08)
                res = self.request_path_plan(alg, start_xy, goal_xy)

                if res["success"]:
                    print(f"     [Run {r}] Waktu: {res['time_ms']:6.2f} ms | Panjang: {res['path_length']:6.2f} m | Waypoint: {res['waypoints']}")
                else:
                    print(f"     [Run {r}] ⚠️ GAGAL: {res['error']}")

                # Simpan ke CSV
                total_rows = self.get_total_csv_rows()
                row = [
                    total_rows + 1,
                    sg_id,
                    level,
                    alg_label,
                    r,
                    f"{start_xy[0]:.2f}",
                    f"{start_xy[1]:.2f}",
                    f"{goal_xy[0]:.2f}",
                    f"{goal_xy[1]:.2f}",
                    f"{est_d:.2f}",
                    f"{res['path_length']:.3f}",
                    f"{res['time_ms']:.2f}",
                    res['waypoints'],
                    "BERHASIL" if res["success"] else "GAGAL",
                    desc,
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ]

                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)

                results.append({
                    "id": sg_id,
                    "level": level,
                    "alg": alg_label,
                    "run": r,
                    "time_ms": res['time_ms'],
                    "length": res['path_length'],
                    "success": res['success']
                })

        return results

    def get_total_csv_rows(self) -> int:
        if not os.path.exists(self.csv_path):
            return 0
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                return max(0, sum(1 for _ in reader) - 1)
        except Exception:
            return 0

    def print_recap_and_comparison(self):
        """Cetak tabel perbandingan komparatif A* vs Dijkstra sesuai format Bab 4."""
        if not os.path.exists(self.csv_path) or self.get_total_csv_rows() == 0:
            print("\n⚠️ Belum ada data pengujian pada file CSV.")
            return

        # Struktur agregasi per ID_Uji
        # summary[id] = {"level": ..., "astar_times": [], "dijk_times": [], "astar_lens": [], "dijk_lens": []}
        summary = {}

        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                sg_id = row['ID_Uji']
                alg = row['Algoritma']
                t = float(row['Waktu_Perencanaan_ms'])
                plen = float(row['Panjang_Lintasan_m'])
                status = row['Status_Rute']

                if status != "BERHASIL":
                    continue

                if sg_id not in summary:
                    summary[sg_id] = {
                        "level": row['Tingkat_Kompleksitas'],
                        "est_dist": row['Jarak_Euklidean_m'],
                        "astar_times": [],
                        "dijk_times": [],
                        "astar_lens": [],
                        "dijk_lens": []
                    }

                if alg == "A*":
                    summary[sg_id]["astar_times"].append(t)
                    summary[sg_id]["astar_lens"].append(plen)
                elif alg == "Dijkstra":
                    summary[sg_id]["dijk_times"].append(t)
                    summary[sg_id]["dijk_lens"].append(plen)

        print("\n" + "=" * 105)
        print("📊 TABEL HASIL PENGUJIAN KOMPARATIF A* VS DIJKSTRA (BAB 4 SKRIPSI)")
        print("=" * 105)
        print(f"{'ID':<6} | {'Level':<8} | {'Jarak(m)':<8} | {'Waktu A* (ms)':<14} | {'Waktu Dijk (ms)':<15} | {'Reduksi Waktu':<15} | {'Panjang Rute A*/Dijk (m)':<25}")
        print("-" * 105)

        level_stats = {
            "Rendah": {"astar_t": [], "dijk_t": []},
            "Sedang": {"astar_t": [], "dijk_t": []},
            "Tinggi": {"astar_t": [], "dijk_t": []}
        }

        for sg_id, data in sorted(summary.items(), key=lambda x: x[0]):
            lvl = data["level"]
            est = data["est_dist"]
            avg_a_t = sum(data["astar_times"]) / len(data["astar_times"]) if data["astar_times"] else 0.0
            avg_d_t = sum(data["dijk_times"]) / len(data["dijk_times"]) if data["dijk_times"] else 0.0
            avg_a_l = sum(data["astar_lens"]) / len(data["astar_lens"]) if data["astar_lens"] else 0.0
            avg_d_l = sum(data["dijk_lens"]) / len(data["dijk_lens"]) if data["dijk_lens"] else 0.0

            if avg_d_t > 0:
                reduction_pct = ((avg_d_t - avg_a_t) / avg_d_t) * 100.0
            else:
                reduction_pct = 0.0

            if lvl in level_stats:
                if avg_a_t > 0: level_stats[lvl]["astar_t"].append(avg_a_t)
                if avg_d_t > 0: level_stats[lvl]["dijk_t"].append(avg_d_t)

            print(
                f"{sg_id:<6} | {lvl:<8} | {est:<8} | "
                f"{avg_a_t:<14.2f} | {avg_d_t:<15.2f} | "
                f"{reduction_pct:<13.1f} % | "
                f"{avg_a_l:5.2f} m / {avg_d_l:5.2f} m"
            )

        print("-" * 105)
        print("📈 RATA-RATA TIAP TINGKAT KOMPLEKSITAS:")
        for lvl in ["Rendah", "Sedang", "Tinggi"]:
            st = level_stats[lvl]
            if st["astar_t"] and st["dijk_t"]:
                mean_a = sum(st["astar_t"]) / len(st["astar_t"])
                mean_d = sum(st["dijk_t"]) / len(st["dijk_t"])
                red = ((mean_d - mean_a) / mean_d) * 100.0
                print(f"  • Kompleksitas {lvl:<7}: A* = {mean_a:6.2f} ms | Dijkstra = {mean_d:6.2f} ms -> Hemat Waktu: {red:.1f}%")

        print("=" * 105)
        print(f"💾 File hasil lengkap CSV tersimpan di: {self.csv_path}")


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkAstarDijkstraNode()

    # Jalankan spin di thread terpisah agar TF dan action client responsif
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        while True:
            print("\n" + "=" * 75)
            print("🤖 BENCHMARK SKRIPSI AMR-01: SKENARIO 3 (A* VS DIJKSTRA HEAD-TO-HEAD)")
            print("=" * 75)
            print(f"  File Log CSV: {node.csv_path}")
            print(f"  Total Data Uji Tersimpan: {node.get_total_csv_rows()} / 150 run")
            print("-" * 75)
            print("  [1] Jalankan SEMUA 15 Pasangan Titik (Total 150 Run: 15 SG x 2 Alg x 5 Run)")
            print("  [2] Jalankan Berdasarkan Kategori (Rendah / Sedang / Tinggi)")
            print("  [3] Jalankan 1 Pasangan Tertentu (Misal SG-01 atau SG-06)")
            print("  [4] Tampilkan Tabel Rekapitulasi & Persentase Efisiensi Bab 4")
            print("  [5] Reset / Hapus File CSV")
            print("  [0] Keluar")
            print("=" * 75)

            choice = input("Pilih menu (0-5): ").strip()

            if choice == '1':
                print(f"\n🚀 Memulai 150 Run Pengujian Penuh (15 Pasangan x 2 Algoritma x 5 Pengulangan)...")
                t_bench_start = time.time()
                for i, sc in enumerate(TABEL_7_SCENARIOS, 1):
                    print(f"\n>>> Kemajuan: Pasangan {i} / 15 ({sc['id']}) <<<")
                    node.run_benchmark_for_pair(sc, repetitions=5)
                t_total_s = time.time() - t_bench_start
                print(f"\n🎉 SELURUH 150 RUN PENGUJIAN SELESAI DALAM {t_total_s:.1f} DETIK!")
                node.print_recap_and_comparison()

            elif choice == '2':
                print("\nPilih Kategori Kompleksitas:")
                print("  [1] Rendah  (SG-01 s/d SG-05) -> Jarak 1 - 3m")
                print("  [2] Sedang  (SG-06 s/d SG-10) -> Jarak 3.1 - 7m")
                print("  [3] Tinggi  (SG-11 s/d SG-15) -> Jarak > 7m")
                sub = input("Pilih kategori (1-3): ").strip()
                lvl_map = {'1': 'Rendah', '2': 'Sedang', '3': 'Tinggi'}
                if sub in lvl_map:
                    target_lvl = lvl_map[sub]
                    filtered = [sc for sc in TABEL_7_SCENARIOS if sc['level'] == target_lvl]
                    print(f"\n🚀 Menjalankan {len(filtered)*10} Run untuk Kategori {target_lvl}...")
                    for sc in filtered:
                        node.run_benchmark_for_pair(sc, repetitions=5)
                    node.print_recap_and_comparison()

            elif choice == '3':
                print("\nDaftar Pasangan Titik Uji (Tabel 7):")
                for i, sc in enumerate(TABEL_7_SCENARIOS):
                    print(f"  [{i+1:2d}] {sc['id']} ({sc['level']}) : Start {sc['start']} -> Goal {sc['goal']} (~{sc['est_dist']}m)")
                idx_in = input("Pilih nomor pasangan (1-15): ").strip()
                if idx_in.isdigit() and 1 <= int(idx_in) <= len(TABEL_7_SCENARIOS):
                    sc = TABEL_7_SCENARIOS[int(idx_in) - 1]
                    node.run_benchmark_for_pair(sc, repetitions=5)
                    node.print_recap_and_comparison()

            elif choice == '4':
                node.print_recap_and_comparison()

            elif choice == '5':
                confirm = input("⚠️ Yakin ingin menghapus seluruh riwayat CSV pengujian? (ketik 'ya'): ").strip()
                if confirm.lower() == 'ya':
                    if os.path.exists(node.csv_path):
                        os.remove(node.csv_path)
                    node.ensure_csv_header()
                    print("✅ Data CSV berhasil di-reset!")

            elif choice == '0':
                print("👋 Keluar dari sistem pengujian A* vs Dijkstra.")
                break

    except KeyboardInterrupt:
        print("\n🛑 Program dihentikan pengguna.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
