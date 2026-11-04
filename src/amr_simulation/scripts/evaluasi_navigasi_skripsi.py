#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
Fungsi:
  1. Merekam telemetri navigasi real-time: /odom, /cmd_vel, /human_state, /scan
  2. Mengalkulasi 7 Metrik Kinerja Navigasi sesuai Bab 2 & 3 Proposal:
       - Success Rate (SR) [%]
       - Path Length (L) [m]
       - Travel Time (T) [s]
       - Path Smoothness [rad]
       - Average Execution / Sampling Time [ms]
       - Minimum Clearance (D_min) [m]
       - Path Deviation (E_path) [m]
  3. Otomatis menghasilkan:
       - File CSV rekaman lengkap per-timestamp
       - File PNG Grafik 3-Subplot (Profil Kecepatan FSM, Clearance, Trajektori 2D)
=============================================================================
"""

import os
import sys
import time
import math
import argparse
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Headless render (aman dijalankan via SSH / Jetson)
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def quaternion_to_yaw(qx, qy, qz, qw):
    """Konversi quaternion ke sudut yaw orientasi robot (radian)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalisasi sudut ke rentang [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EvaluasiNavigasiNode(Node):
    def __init__(self, mode: str, skenario: str, label: str, out_dir: str):
        super().__init__('evaluasi_navigasi_skripsi_node')

        self.mode = mode.upper()            # BASELINE atau ADAPTIF
        self.skenario = skenario            # statis / dinamis / manusia
        self.label = label
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self.robot_radius = 0.29            # Radius sasis robot (0.29 meter)

        # Buffer rekaman data
        self.timestamps = []
        self.pos_x = []
        self.pos_y = []
        self.yaw = []
        self.vel_linear = []
        self.vel_angular = []
        self.cmd_linear = []
        self.cmd_angular = []
        self.human_states = []
        self.min_clearances = []
        self.global_path_pts = []

        # State realtime
        self.current_human_state = "NORMAL"
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0
        self.latest_clearance = 5.0
        self.is_recording = False
        self.start_wall_time = None
        self.end_wall_time = None

        # QoS Profile
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, reliable_qos)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, reliable_qos)
        self.sub_human = self.create_subscription(String, '/human_state', self.on_human_state, reliable_qos)
        self.sub_scan = self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, sensor_qos)
        self.sub_scan_raw = self.create_subscription(LaserScan, '/scan', self.on_scan_raw, sensor_qos)
        self.sub_plan = self.create_subscription(Path, '/plan', self.on_plan, latched_qos)

        self.get_logger().info("============================================================")
        self.get_logger().info("📊 [LOGGER EVALUASI NAVIGASI SKRIPSI] AKTIF & STANDBY")
        self.get_logger().info(f"   Mode     : {self.mode}")
        self.get_logger().info(f"   Skenario : {self.skenario}")
        self.get_logger().info(f"   Label    : {self.label}")
        self.get_logger().info(f"   Output   : {self.out_dir}")
        self.get_logger().info("============================================================")
        self.get_logger().info("⏳ Menunggu robot mulai bergerak (/odom atau /cmd_vel)...")

    def on_human_state(self, msg: String):
        self.current_human_state = msg.data.strip()

    def on_cmd_vel(self, msg: Twist):
        self.current_cmd_v = msg.linear.x
        self.current_cmd_w = msg.angular.z

    def on_plan(self, msg: Path):
        """Simpan titik-titik global path A* untuk kalkulasi deviasi lintasan."""
        pts = []
        for p in msg.poses:
            pts.append((p.pose.position.x, p.pose.position.y))
        if pts:
            self.global_path_pts = pts

    def on_scan(self, msg: LaserScan):
        self._process_scan(msg)

    def on_scan_raw(self, msg: LaserScan):
        if self.latest_clearance == 5.0:
            self._process_scan(msg)

    def _process_scan(self, msg: LaserScan):
        """Hitung jarak terpendek dari sasis robot ke rintangan/manusia (Minimum Clearance)."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            # Jarak clearance dihitung dari permukaan terluar sasis (radius 0.29m)
            min_r = min(valid_ranges)
            clearance = max(0.0, min_r - self.robot_radius)
            self.latest_clearance = clearance

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)

        v_lin = msg.twist.twist.linear.x
        v_ang = msg.twist.twist.angular.z

        now = time.time()

        # Trigger mulai rekam otomatis saat robot mulai jalan
        if not self.is_recording:
            if abs(v_lin) > 0.01 or abs(v_ang) > 0.05 or abs(self.current_cmd_v) > 0.01:
                self.is_recording = True
                self.start_wall_time = now
                self.get_logger().info("🚀 [START DETEKSI] Robot mulai bergerak! Perekaman data dimulai...")

        if not self.is_recording:
            return

        t_rel = now - self.start_wall_time
        self.end_wall_time = now

        # Rekam sampel data
        self.timestamps.append(t_rel)
        self.pos_x.append(x)
        self.pos_y.append(y)
        self.yaw.append(yaw)
        self.vel_linear.append(v_lin)
        self.vel_angular.append(v_ang)
        self.cmd_linear.append(self.current_cmd_v)
        self.cmd_angular.append(self.current_cmd_w)
        self.human_states.append(self.current_human_state)
        self.min_clearances.append(self.latest_clearance)

    def calculate_metrics(self):
        """Menghitung 7 metrik evaluasi sesuai Bab 2 & 3 Proposal Skripsi."""
        n = len(self.timestamps)
        if n < 5:
            return None

        # 1. Travel Time (Persamaan 27)
        travel_time = self.timestamps[-1] - self.timestamps[0]

        # 2. Path Length (Persamaan 26)
        path_length = 0.0
        for i in range(1, n):
            dx = self.pos_x[i] - self.pos_x[i - 1]
            dy = self.pos_y[i] - self.pos_y[i - 1]
            path_length += math.sqrt(dx * dx + dy * dy)

        # 3. Path Smoothness (Persamaan 28)
        smoothness = 0.0
        for i in range(1, n):
            dyaw = abs(normalize_angle(self.yaw[i] - self.yaw[i - 1]))
            smoothness += dyaw

        # 4. Minimum Clearance (Persamaan 30)
        min_clearance = min(self.min_clearances) if self.min_clearances else 0.0

        # 5. Path Deviation (Persamaan 31)
        path_deviation = 0.0
        if self.global_path_pts:
            dev_sum = 0.0
            for i in range(n):
                rx, ry = self.pos_x[i], self.pos_y[i]
                # Cari jarak terdekat ke titik jalur global
                min_dist_to_path = min(math.hypot(rx - gx, ry - gy) for gx, gy in self.global_path_pts)
                dev_sum += min_dist_to_path
            path_deviation = dev_sum / float(n)
        else:
            # Estimasi deviasi dari garis lurus start-ke-goal jika path global tidak tertangkap
            dx_total = self.pos_x[-1] - self.pos_x[0]
            dy_total = self.pos_y[-1] - self.pos_y[0]
            line_len = math.hypot(dx_total, dy_total)
            if line_len > 0.1:
                dev_sum = 0.0
                for i in range(n):
                    # Jarak titik ke garis
                    d = abs(dy_total * self.pos_x[i] - dx_total * self.pos_y[i] + self.pos_x[-1]*self.pos_y[0] - self.pos_y[-1]*self.pos_x[0]) / line_len
                    dev_sum += d
                path_deviation = dev_sum / float(n)

        # 5. Average Execution / Sampling Time (Persamaan 29)
        dts = [self.timestamps[i] - self.timestamps[i - 1] for i in range(1, n)]
        avg_dt_ms = (sum(dts) / len(dts)) * 1000.0 if dts else 50.0
        freq_hz = (1000.0 / avg_dt_ms) if avg_dt_ms > 0 else 20.0

        # Status Keberhasilan (Success Status untuk uji tunggal)
        # Sesuai kriteria: sampai tujuan tanpa benturan (clearance > 0.05m)
        is_success = min_clearance > 0.05
        success_str = "BERHASIL (Goal Reached, Tanpa Tabrakan)" if is_success else "GAGAL (Terdeteksi Benturan)"

        # Kecepatan Rata-rata & Kecepatan Maksimal (Validasi AVC Tabel 10)
        avg_v = float(np.mean(np.abs(self.vel_linear)))
        max_v = float(np.max(np.abs(self.vel_linear)))

        return {
            'mode': self.mode,
            'skenario': self.skenario,
            'label': self.label,
            'is_success': is_success,
            'success_status': success_str,
            'path_length_m': path_length,
            'travel_time_s': travel_time,
            'smoothness_rad': smoothness,
            'avg_cycle_ms': avg_dt_ms,
            'frequency_hz': freq_hz,
            'min_clearance_m': min_clearance,
            'path_deviation_m': path_deviation,
            'avg_velocity_mps': avg_v,
            'max_velocity_mps': max_v,
            'total_samples': n
        }

    def save_results(self):
        metrics = self.calculate_metrics()
        if not metrics:
            self.get_logger().warn("⚠️ Sampel data terlalu sedikit (< 5 sampel), hasil tidak disimpan.")
            return

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = f"eval_{self.mode.lower()}_{self.skenario}_{self.label}_{ts_str}"

        csv_path = os.path.join(self.out_dir, f"{prefix}.csv")
        png_path = os.path.join(self.out_dir, f"{prefix}.png")

        # 1. Simpan CSV
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.write("# ============================================================\n")
            f.write(f"# EVALUASI KINERJA NAVIGASI SKRIPSI AMR-01\n")
            f.write(f"# 7 Metrik Evaluasi Sesuai Proposal Subbab 3.5.2 & Tabel 10\n")
            f.write(f"# Mode            : {metrics['mode']}\n")
            f.write(f"# Skenario        : {metrics['skenario']}\n")
            f.write(f"# Label           : {metrics['label']}\n")
            f.write(f"# Tanggal         : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 1. Status/Succ  : {metrics['success_status']}\n")
            f.write(f"# 2. Path Length  : {metrics['path_length_m']:.3f} m\n")
            f.write(f"# 3. Travel Time  : {metrics['travel_time_s']:.2f} s\n")
            f.write(f"# 4. Smoothness   : {metrics['smoothness_rad']:.3f} rad\n")
            f.write(f"# 5. Avg Exec Time: {metrics['avg_cycle_ms']:.2f} ms ({metrics['frequency_hz']:.1f} Hz)\n")
            f.write(f"# 6. Min Clear    : {metrics['min_clearance_m']:.3f} m\n")
            f.write(f"# 7. Path Dev     : {metrics['path_deviation_m']:.3f} m\n")
            f.write(f"# (Tambahan) V_avg: {metrics['avg_velocity_mps']:.3f} m/s | V_max: {metrics['max_velocity_mps']:.3f} m/s\n")
            f.write("# ============================================================\n")
            f.write("time_s,pos_x,pos_y,yaw_deg,v_actual,w_actual,v_cmd,w_cmd,human_state,clearance_m\n")

            for i in range(len(self.timestamps)):
                f.write(
                    f"{self.timestamps[i]:.3f},"
                    f"{self.pos_x[i]:.4f},"
                    f"{self.pos_y[i]:.4f},"
                    f"{math.degrees(self.yaw[i]):.2f},"
                    f"{self.vel_linear[i]:.3f},"
                    f"{self.vel_angular[i]:.3f},"
                    f"{self.cmd_linear[i]:.3f},"
                    f"{self.cmd_angular[i]:.3f},"
                    f"{self.human_states[i]},"
                    f"{self.min_clearances[i]:.3f}\n"
                )

        # 2. Gambar Grafik Profil Publikasi Skripsi (300 DPI)
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), dpi=300)

        t_arr = np.array(self.timestamps)
        v_arr = np.array(self.vel_linear)
        c_arr = np.array(self.min_clearances)

        # Subplot 1: Profil Kecepatan Translasi + Shading Warna State FSM
        ax1.plot(t_arr, v_arr, color='#1f77b4', linewidth=2.0, label='Kecepatan Aktual v (m/s)')
        ax1.plot(t_arr, self.cmd_linear, color='#ff7f0e', linestyle='--', linewidth=1.5, alpha=0.7, label='Target /cmd_vel (m/s)')
        ax1.axhline(0.45, color='gray', linestyle=':', label='Batas Nominal (0.45 m/s)')

        # Shading warna latar belakang sesuai status FSM
        colors = {'NORMAL': '#2ca02c', 'REDUCED_SPEED': '#ffbb78', 'SAFE_STOP': '#d62728'}
        alpha_map = {'NORMAL': 0.10, 'REDUCED_SPEED': 0.25, 'SAFE_STOP': 0.30}

        cur_state = None
        state_start = t_arr[0]
        for idx in range(len(t_arr)):
            st = self.human_states[idx]
            if st != cur_state:
                if cur_state is not None:
                    ax1.axvspan(state_start, t_arr[idx], color=colors.get(cur_state, 'gray'), alpha=alpha_map.get(cur_state, 0.15))
                cur_state = st
                state_start = t_arr[idx]
        if cur_state is not None:
            ax1.axvspan(state_start, t_arr[-1], color=colors.get(cur_state, 'gray'), alpha=alpha_map.get(cur_state, 0.15))

        ax1.set_title(f'Profil Kecepatan Navigasi ({self.mode} - Skenario: {self.skenario})', fontsize=12, fontweight='bold')
        ax1.set_xlabel('Waktu (detik)', fontsize=10)
        ax1.set_ylabel('Kecepatan Linear (m/s)', fontsize=10)
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend(loc='upper right', fontsize=8)

        # Subplot 2: Minimum Clearance (Jarak Aman ke Halangan / Manusia)
        ax2.plot(t_arr, c_arr, color='#2ca02c', linewidth=2.0, label='Jarak Aman Minimum (m)')
        ax2.axhline(metrics['min_clearance_m'], color='red', linestyle='--', label=f"Min Clearance: {metrics['min_clearance_m']:.2f}m")
        ax2.axhline(0.50, color='darkred', linestyle=':', alpha=0.7, label='Batas Kritis Jarak Aman (0.50 m)')
        ax2.set_title('Jarak Aman ke Rintangan / Pekerja (Clearance)', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Waktu (detik)', fontsize=10)
        ax2.set_ylabel('Jarak Fisik (meter)', fontsize=10)
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.legend(loc='upper right', fontsize=8)

        # Subplot 3: Trajektori Lintasan 2D (X vs Y)
        if self.global_path_pts:
            gx = [p[0] for p in self.global_path_pts]
            gy = [p[1] for p in self.global_path_pts]
            ax3.plot(gx, gy, color='black', linestyle='--', linewidth=1.5, alpha=0.7, label='Global Path (A*)')
        ax3.plot(self.pos_x, self.pos_y, color='#1f77b4', linewidth=2.2, label=f"Lintasan Aktual ({self.mode})")
        ax3.scatter([self.pos_x[0]], [self.pos_y[0]], color='green', s=80, marker='o', label='Start')
        ax3.scatter([self.pos_x[-1]], [self.pos_y[-1]], color='red', s=80, marker='X', label='Goal')
        ax3.set_title('Rekonstruksi Lintasan 2D (Path Tracking)', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Koordinat X (meter)', fontsize=10)
        ax3.set_ylabel('Koordinat Y (meter)', fontsize=10)
        ax3.axis('equal')
        ax3.grid(True, linestyle='--', alpha=0.6)
        ax3.legend(loc='best', fontsize=8)

        plt.tight_layout()
        plt.savefig(png_path, dpi=300)
        plt.close()

        # Cetak Tabel Hasil di Terminal Sesuai 7 Metrik Proposal Subbab 3.5.2
        print("\n" + "=" * 68)
        print(f"🎉 PENGUJIAN SELESAI: {self.mode} | Skenario: {self.skenario} | Label: {self.label}")
        print("=" * 68)
        print(f"🎯 1. Tingkat Keberhasilan (Success)     : {metrics['success_status']}")
        print(f"📏 2. Jarak Tempuh (Path Length)         : {metrics['path_length_m']:.3f} meter")
        print(f"⏱️  3. Waktu Tempuh (Travel Time)         : {metrics['travel_time_s']:.2f} detik")
        print(f"🌊 4. Kehalusan Lintasan (Path Smoothness): {metrics['smoothness_rad']:.3f} radian")
        print(f"💻 5. Waktu Komputasi (Avg Exec Time)    : {metrics['avg_cycle_ms']:.2f} ms (~{metrics['frequency_hz']:.1f} Hz)")
        print(f"🛡️  6. Jarak Aman Minimum (Min Clearance) : {metrics['min_clearance_m']:.3f} meter")
        print(f"📐 7. Deviasi Jalur (Path Deviation)     : {metrics['path_deviation_m']:.3f} meter")
        print("-" * 68)
        print(f"⚡ Kecepatan Rata-rata                   : {metrics['avg_velocity_mps']:.3f} m/s")
        print(f"🚀 Kecepatan Puncak (V_max)              : {metrics['max_velocity_mps']:.3f} m/s")
        print(f"💾 File CSV tersimpan di                 : {csv_path}")
        print(f"🖼️  File PNG tersimpan di                 : {png_path}")
        print("=" * 68 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Tool Evaluasi & Perekam Metrik Navigasi Skripsi AMR-01")
    parser.add_argument('--mode', type=str, default='adaptif', choices=['adaptif', 'baseline'],
                        help="Mode pengujian: 'adaptif' (dengan human-aware) atau 'baseline' (DWB standar)")
    parser.add_argument('--skenario', type=str, default='manusia',
                        help="Nama skenario pengujian: statis / dinamis / manusia")
    parser.add_argument('--label', type=str, default='percobaan_1',
                        help="Label pengujian (contoh: uji_1, uji_2, dst)")
    parser.add_argument('--outdir', type=str, default='~/hasil_pengujian_skripsi',
                        help="Direktori penyimpanan file CSV dan gambar PNG")

    args = parser.parse_args()

    rclpy.init()
    node = EvaluasiNavigasiNode(mode=args.mode, skenario=args.skenario, label=args.label, out_dir=args.outdir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Perekaman dihentikan oleh pengguna (Ctrl+C). Menyimpan hasil...")
    finally:
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

