#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SKRIPSI BAB 4 - SKENARIO 2: EVALUASI KINERJA PELACAKAN POSISI (NAV2_AMCL)
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Deskripsi:
  Mengevaluasi keandalan filter partikel AMCL dalam menentukan pose absolut robot
  (x, y, theta) di atas peta tersimpan (saved map):
  1. Mode Pengujian:
     - Mode 1 (Utama Skripsi): POSE TRACKING
       Menginisialisasi partikel lokal di sekitar datum via /initialpose (sebaran ±30 cm),
       lalu mengukur waktu konvergensi dan galat saat robot berputar/melacak posisi.
     - Mode 2: GLOBAL LOCALIZATION (Kidnapped Robot)
       Menyebarkan 2000 partikel acak ke seluruh peta via /reinitialize_global_localization.
  2. Uji Waktu Konvergensi (Time-to-Converge, t_conv):
     Waktu dihitung hingga matriks kovariansi pada /amcl_pose menyusut di bawah ambang batas:
     sigma_x <= 0.05m, sigma_y <= 0.05m, sigma_theta <= 0.05 rad (~2.86°).
  3. Uji Galat Posisi (e_pos) dan Orientasi (e_theta):
     e_pos   = sqrt((x_amcl - x_gt)^2 + (y_amcl - y_gt)^2)
     e_theta = |theta_amcl - theta_gt| (normalisasi [-pi, pi])
  4. Kriteria Kelulusan:
     - 5 kali pengulangan pada 5 titik datum (Total 25 kali pengujian)
     - t_conv <= 10.0 s
     - Rata-rata e_pos <= 0.05 m (5 cm)
     - Rata-rata e_theta <= 3.0°
  5. Penyimpanan Hasil:
     Tersimpan otomatis ke file CSV di ~/hasil_pengujian_skripsi/skenario2_evaluasi_amcl.csv
=============================================================================
"""

import os
import sys
import time
import math
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


def euler_from_quaternion(q):
    """Konversi quaternion ke yaw (radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalisasi sudut ke rentang [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# =============================================================================
# KONFIGURASI DEFAULT TITIK DATUM (GROUND TRUTH)
# =============================================================================
DEFAULT_DATUMS = {
    1: {"name": "Datum 1 (Home/Origin)",   "x": 0.000, "y": 0.000, "yaw_deg": 0.0},
    2: {"name": "Datum 2 (Koridor Utama)", "x": 2.000, "y": 0.000, "yaw_deg": 0.0},
    3: {"name": "Datum 3 (Persimpangan)",  "x": 3.500, "y": 1.500, "yaw_deg": 90.0},
    4: {"name": "Datum 4 (Pintu Kamar)",   "x": 4.800, "y": 2.500, "yaw_deg": 180.0},
    5: {"name": "Datum 5 (Drop Zone)",     "x": 1.200, "y": 3.800, "yaw_deg": -90.0},
}

# Ambang Batas Konvergensi AMCL (Standar Deviasi Maksimum)
THRESHOLD_SIGMA_XY = 0.050    # meter (5 cm)
THRESHOLD_SIGMA_YAW = 0.052   # radian (~3.0 derajat)

# Kriteria Kelulusan Skripsi
PASS_MAX_TCONV = 10.0   # detik
PASS_AVG_EPOS = 0.050   # meter (5.0 cm)
PASS_AVG_EYAW = 3.00    # derajat


class EvaluasiAMCLNode(Node):
    def __init__(self):
        super().__init__('evaluasi_amcl_node')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # /amcl_pose mendukung QoS transient local / volatile
        qos_amcl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            qos_amcl
        )

        # Publisher /initialpose (Dual QoS: TRANSIENT_LOCAL & VOLATILE untuk Foxy)
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_volatile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.initial_pose_pub_reliable = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos_transient
        )
        self.initial_pose_pub_volatile = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos_volatile
        )

        # Service Client untuk memicu Global Localization (sebar 2000 partikel acak)
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        # Mode Pengujian: 'tracking' (Pose Tracking lokal) atau 'global' (Global Relocalization)
        self.test_mode = 'tracking'

        # State AMCL Real-time
        self.current_pose = None
        self.covariance = None
        self.sigma_x = 999.0
        self.sigma_y = 999.0
        self.sigma_yaw = 999.0
        self.last_amcl_time = 0.0
        self.has_amcl_data = False

        # Direktori Output
        self.output_dir = os.path.expanduser('~/hasil_pengujian_skripsi')
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'skenario2_evaluasi_amcl.csv')
        self.config_path = os.path.join(self.output_dir, 'datum_config.json')

        # Subscriber ke /odom untuk melacak rotasi roda fisik aktual (Ground Truth Heading)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_odom_yaw = 0.0
        self.has_odom_data = False

        self.datums = self.load_datum_config()
        self.ensure_csv_header()

        self.get_logger().info("✅ Node Evaluasi AMCL Skripsi Siap (Mode Default: POSE TRACKING).")

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_odom_yaw = euler_from_quaternion(q)
        self.has_odom_data = True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q)

        cov = msg.pose.covariance
        # Indeks matriks kovariansi 6x6:
        # [0] = var(x), [7] = var(y), [35] = var(yaw)
        var_x = max(0.0, float(cov[0]))
        var_y = max(0.0, float(cov[7]))
        var_yaw = max(0.0, float(cov[35]))

        self.current_pose = {'x': float(p.x), 'y': float(p.y), 'yaw': yaw}
        self.covariance = cov
        self.sigma_x = math.sqrt(var_x)
        self.sigma_y = math.sqrt(var_y)
        self.sigma_yaw = math.sqrt(var_yaw)
        self.last_amcl_time = time.time()
        self.has_amcl_data = True

    def send_cmd(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        for _ in range(5):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.02)

    def send_initial_pose(self, x: float, y: float, yaw_rad: float, cov_xy: float = 0.09, cov_yaw: float = 0.068):
        """
        Kirim perkiraan pose awal ke AMCL untuk inisialisasi partikel lokal (Pose Tracking).
        cov_xy = 0.09 m^2 (sigma ~ 30 cm) -> sebaran partikel lokal di sekitar datum.
        cov_yaw = 0.068 rad^2 (sigma ~ 15 derajat).
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Quaternion dari yaw 2D
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Kovariansi awal
        cov = [0.0] * 36
        cov[0] = float(cov_xy)    # var(x)
        cov[7] = float(cov_xy)    # var(y)
        cov[35] = float(cov_yaw)  # var(yaw)
        msg.pose.covariance = cov

        self.initial_pose_pub_reliable.publish(msg)
        self.initial_pose_pub_volatile.publish(msg)

    def trigger_global_localization(self) -> bool:
        """Panggil service /reinitialize_global_localization untuk menyebarkan 2000 partikel."""
        if not self.global_loc_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Service /reinitialize_global_localization tidak merespon!")
            return False

        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.done() and future.result() is not None

    def load_datum_config(self) -> dict:
        """Muat konfigurasi titik datum dari file JSON jika ada, atau gunakan default."""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    data = json.load(f)
                    # Convert keys to int
                    return {int(k): v for k, v in data.items()}
            except Exception:
                pass
        return dict(DEFAULT_DATUMS)

    def save_datum_config(self):
        """Simpan konfigurasi titik datum ke file JSON."""
        try:
            with open(self.config_path, 'w') as f:
                json.dump(self.datums, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f"Gagal menyimpan config datum: {e}")

    def ensure_csv_header(self):
        """Buat file CSV dengan header lengkap jika belum ada."""
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "No",
                    "Datum_ID",
                    "Nama_Datum",
                    "Mode_Uji",
                    "Run_Ke",
                    "GT_X_m",
                    "GT_Y_m",
                    "GT_Yaw_deg",
                    "AMCL_X_m",
                    "AMCL_Y_m",
                    "AMCL_Yaw_deg",
                    "Waktu_Konvergensi_s",
                    "Galat_Posisi_m",
                    "Galat_Posisi_cm",
                    "Galat_Orientasi_deg",
                    "Status_TConv",
                    "Status_EPos",
                    "Status_EYaw",
                    "Status_Keseluruhan",
                    "Waktu_Pengujian"
                ])

    def get_next_run_number(self, datum_id: int) -> int:
        """Cari nomor run berikutnya untuk datum tertentu dari isi CSV."""
        if not os.path.exists(self.csv_path):
            return 1
        runs = []
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if int(row.get('Datum_ID', 0)) == datum_id:
                        runs.append(int(row.get('Run_Ke', 0)))
        except Exception:
            pass
        return (max(runs) + 1) if runs else 1

    def run_single_test(self, datum_id: int, run_idx: int) -> dict:
        """Eksekusi satu kali run pengujian konvergensi AMCL."""
        datum = self.datums[datum_id]
        gt_x = datum['x']
        gt_y = datum['y']
        gt_yaw_deg = datum['yaw_deg']
        gt_yaw_rad = math.radians(gt_yaw_deg)

        mode_desc = (
            "POSE TRACKING (Inisialisasi Partikel Lokal ±30 cm - Rekomendasi Skripsi)"
            if self.test_mode == 'tracking'
            else "GLOBAL LOCALIZATION (Kidnapped Robot 2000 Partikel Acak)"
        )

        print("\n" + "=" * 78)
        print(f"🚀 MEMULAI PENGUJIAN: {datum['name']} | RUN KE-{run_idx} DARI 5")
        print(f"⚙️ Mode Pengujian     : {mode_desc}")
        print("=" * 78)
        print(f"📍 Ground Truth Fisik : X = {gt_x:.3f} m, Y = {gt_y:.3f} m, Heading = {gt_yaw_deg:.1f}°")
        print(f"🎯 Target Ambang Batas: Sigma XY <= {THRESHOLD_SIGMA_XY*100:.1f} cm | Sigma Yaw <= {math.degrees(THRESHOLD_SIGMA_YAW):.1f}°")
        print(f"⏳ Maksimum Timeout   : 25.0 detik")
        print("-" * 78)
        print("👉 Pastikan robot sudah diletakkan tepat di atas tanda titik datum fisik.")
        input("Tekan [ENTER] di keyboard untuk menginisialisasi partikel & memulai rotasi...")

        # 1. Inisialisasi Partikel
        if self.test_mode == 'tracking':
            print("\n[1/3] 📍 Mengirim /initialpose di sekitar Datum (sebaran partikel lokal ±30 cm)...")
            self.send_initial_pose(gt_x, gt_y, gt_yaw_rad, cov_xy=0.09, cov_yaw=0.068)
            print("      ✅ Pose awal AMCL berhasil diset! Partikel terpusat pada titik datum.")
        else:
            print("\n[1/3] 💥 Memicu /reinitialize_global_localization (2000 partikel disebar acak)...")
            if self.trigger_global_localization():
                print("      ✅ Partikel AMCL berhasil disebarkan ke seluruh area peta!")
            else:
                print("      ⚠️ Service tidak merespon, melanjutkan dengan rotasi paksa...")

        time.sleep(0.6)

        # 2. Mulai Rotasi Robot & Pengukuran Waktu Konvergensi
        print("[2/3] 🔄 Robot mulai berputar di tempat (w = 0.35 rad/s) untuk pemindaian...")
        start_odom_yaw = self.current_odom_yaw if self.has_odom_data else None
        t_start = time.time()
        converged = False
        consecutive_stable = 0
        final_pose = None
        rot_speed = 0.35  # rad/s

        while rclpy.ok():
            time.sleep(0.05)
            now = time.time()
            elapsed = now - t_start

            # Putar robot perlahan di tempat
            self.send_cmd(0.0, rot_speed)

            # Cek status konvergensi
            if self.has_amcl_data and (now - self.last_amcl_time < 0.5):
                sig_xy = max(self.sigma_x, self.sigma_y)
                sig_yaw_deg = math.degrees(self.sigma_yaw)

                # Format tampilan live di satu baris terminal
                sys.stdout.write(
                    f"\r  ⏱️ [{elapsed:4.1f}s] Sigma X: {self.sigma_x*100:4.1f}cm | "
                    f"Sigma Y: {self.sigma_y*100:4.1f}cm | "
                    f"Sigma Yaw: {sig_yaw_deg:4.1f}° "
                )
                sys.stdout.flush()

                # Kondisi konvergensi: standar deviasi menyusut di bawah batas selama minimal 3 tick (0.15s)
                if (self.sigma_x <= THRESHOLD_SIGMA_XY and
                    self.sigma_y <= THRESHOLD_SIGMA_XY and
                    self.sigma_yaw <= THRESHOLD_SIGMA_YAW):
                    consecutive_stable += 1
                    if consecutive_stable >= 3:
                        converged = True
                        t_converge = elapsed
                        final_pose = dict(self.current_pose)
                        break
                else:
                    consecutive_stable = 0

            # Timeout safety
            if elapsed >= 25.0:
                print("\n⚠️ WAKTU MELEBIHI BATAS MAKSIMUM (TIMEOUT 25s)!")
                t_converge = elapsed
                final_pose = dict(self.current_pose) if self.current_pose else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                break

        # 3. Hentikan Robot
        self.stop_robot()
        time.sleep(0.3)
        stop_odom_yaw = self.current_odom_yaw if self.has_odom_data else None
        print("\n\n[3/3] 🛑 Robot berhenti. Menghitung galat estimasi vs ground truth...")

        # 4. Perhitungan Galat
        amcl_x = final_pose['x']
        amcl_y = final_pose['y']
        amcl_yaw = final_pose['yaw']
        amcl_yaw_deg = math.degrees(amcl_yaw)

        # Hitung sudut hadap estimasi dari odometri roda
        if start_odom_yaw is not None and stop_odom_yaw is not None:
            delta_rot_rad = normalize_angle(stop_odom_yaw - start_odom_yaw)
            odom_yaw_rad = normalize_angle(gt_yaw_rad + delta_rot_rad)
        else:
            delta_rot_rad = rot_speed * t_converge
            odom_yaw_rad = normalize_angle(gt_yaw_rad + delta_rot_rad)

        odom_yaw_deg = math.degrees(odom_yaw_rad)
        delta_rot_deg = math.degrees(delta_rot_rad)

        # Opsi Input Sudut Fisik Aktual (Manual Ground Truth)
        print("-" * 78)
        print("📐 VERIFIKASI GROUND TRUTH SUDUT HADAP (HEADING):")
        print(f"  • Estimasi Deteksi AMCL       : {amcl_yaw_deg:+.1f}°")
        print(f"  • Estimasi Putaran Roda (Odom): {odom_yaw_deg:+.1f}° (Rentan selip roda saat rotasi)")
        print(f"  💡 Pengukuran fisik manual (busur/meteran) jauh lebih valid secara ilmiah sebagai Ground Truth.")
        user_in = input(f"  👉 Masukkan Sudut Aktual Fisik Hasil Ukur Manual (°) [ENTER = pakai odom {odom_yaw_deg:+.1f}°]: ").strip()

        if user_in:
            try:
                actual_yaw_deg = float(user_in)
                actual_yaw_rad = math.radians(actual_yaw_deg)
                sudut_sumber = f"Ukur Manual ({actual_yaw_deg:+.1f}°)"
                print(f"  ✅ Menggunakan Sudut Aktual Manual Ground Truth: {actual_yaw_deg:+.1f}°")
            except ValueError:
                print(f"  ⚠️ Input tidak valid, beralih ke estimasi odom: {odom_yaw_deg:+.1f}°")
                actual_yaw_deg = odom_yaw_deg
                actual_yaw_rad = odom_yaw_rad
                sudut_sumber = f"Odometri Roda ({delta_rot_deg:+.1f}°)"
        else:
            actual_yaw_deg = odom_yaw_deg
            actual_yaw_rad = odom_yaw_rad
            sudut_sumber = f"Odometri Roda ({delta_rot_deg:+.1f}°)"

        dx = amcl_x - gt_x
        dy = amcl_y - gt_y
        e_pos_m = math.sqrt(dx * dx + dy * dy)
        e_pos_cm = e_pos_m * 100.0

        d_yaw_rad = abs(normalize_angle(amcl_yaw - actual_yaw_rad))
        e_yaw_deg = math.degrees(d_yaw_rad)

        status_tconv = "LULUS" if t_converge <= PASS_MAX_TCONV else "GAGAL"
        status_epos = "LULUS" if e_pos_m <= PASS_AVG_EPOS else "GAGAL"
        status_eyaw = "LULUS" if e_yaw_deg <= PASS_AVG_EYAW else "GAGAL"
        status_total = "LULUS" if (status_tconv == "LULUS" and status_epos == "LULUS" and status_eyaw == "LULUS") else "TIDAK LULUS"

        # Tampilkan Hasil Uji
        print("=" * 78)
        print(f"📊 HASIL UJI {datum['name']} - RUN {run_idx} ({self.test_mode.upper()}):")
        print(f"  • Waktu Konvergensi (t_conv) : {t_converge:.2f} detik  (Batas: <= {PASS_MAX_TCONV:.1f}s) -> [{status_tconv}]")
        print(f"  • Posisi Nyata vs Estimasi  : GT=({gt_x:.3f}, {gt_y:.3f}) | AMCL=({amcl_x:.3f}, {amcl_y:.3f})")
        print(f"  • Galat Posisi (e_pos)       : {e_pos_m:.3f} m ({e_pos_cm:.1f} cm)  (Batas: <= {PASS_AVG_EPOS*100:.1f} cm) -> [{status_epos}]")
        print(f"  • Sudut Nyata saat Berhenti : {actual_yaw_deg:+.1f}° [{sudut_sumber}]")
        print(f"  • Estimasi Sudut AMCL       : {amcl_yaw_deg:+.1f}°")
        print(f"  • Galat Orientasi (e_theta)  : {e_yaw_deg:.2f}°  (Batas: <= {PASS_AVG_EYAW:.1f}°) -> [{status_eyaw}]")
        print(f"  • KESIMPULAN RUN INI        : [{status_total}]")
        print("=" * 78)

        # Simpan ke CSV
        row_data = [
            self.get_total_csv_rows() + 1,
            datum_id,
            datum['name'],
            self.test_mode.upper(),
            run_idx,
            f"{gt_x:.4f}",
            f"{gt_y:.4f}",
            f"{actual_yaw_deg:.2f}",
            f"{amcl_x:.4f}",
            f"{amcl_y:.4f}",
            f"{amcl_yaw_deg:.2f}",
            f"{t_converge:.3f}",
            f"{e_pos_m:.4f}",
            f"{e_pos_cm:.2f}",
            f"{e_yaw_deg:.2f}",
            status_tconv,
            status_epos,
            status_eyaw,
            status_total,
            datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        ]

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row_data)

        print(f"💾 Data berhasil disimpan ke: {self.csv_path}")
        return {
            'datum_id': datum_id,
            'run': run_idx,
            't_conv': t_converge,
            'e_pos_cm': e_pos_cm,
            'e_yaw_deg': e_yaw_deg,
            'status': status_total
        }

    def get_total_csv_rows(self) -> int:
        if not os.path.exists(self.csv_path):
            return 0
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                return max(0, sum(1 for _ in reader) - 1)
        except Exception:
            return 0

    def print_recap_table(self):
        """Tampilkan rekapitulasi data dari CSV siap untuk laporan Bab 4."""
        if not os.path.exists(self.csv_path) or self.get_total_csv_rows() == 0:
            print("\n⚠️ Belum ada data pengujian di file CSV.")
            return

        print("\n" + "=" * 90)
        print("📋 REKAPITULASI PENGUJIAN SKENARIO 2 (EVALUASI PELACAKAN AMCL - BAB 4)")
        print("=" * 90)
        print(f"{'Datum':<16} | {'Mode':<8} | {'Run':<4} | {'t_conv (s)':<10} | {'e_pos (cm)':<10} | {'e_yaw (°)':<10} | {'Status':<10}")
        print("-" * 90)

        data_by_datum = {i: [] for i in range(1, 6)}
        all_tconv = []
        all_epos_cm = []
        all_eyaw = []

        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                d_id = int(row['Datum_ID'])
                t = float(row['Waktu_Konvergensi_s'])
                ep_cm = float(row['Galat_Posisi_cm'])
                ey_deg = float(row['Galat_Orientasi_deg'])
                st = row['Status_Keseluruhan']
                mode_str = row.get('Mode_Uji', 'TRACKING')

                data_by_datum[d_id].append({
                    'run': row['Run_Ke'],
                    't': t,
                    'ep_cm': ep_cm,
                    'ey': ey_deg,
                    'status': st,
                    'mode': mode_str
                })
                all_tconv.append(t)
                all_epos_cm.append(ep_cm)
                all_eyaw.append(ey_deg)

                print(f"{row['Nama_Datum'][:16]:<16} | {mode_str[:8]:<8} | {row['Run_Ke']:<4} | {t:<10.2f} | {ep_cm:<10.1f} | {ey_deg:<10.2f} | {st:<10}")

        print("-" * 90)
        print("📊 RATA-RATA TIAP TITIK DATUM (5 RUN):")
        for d_id, records in data_by_datum.items():
            if not records:
                continue
            avg_t = sum(r['t'] for r in records) / len(records)
            avg_ep = sum(r['ep_cm'] for r in records) / len(records)
            avg_ey = sum(r['ey'] for r in records) / len(records)
            d_name = self.datums[d_id]['name']
            print(f"  • {d_name:<20}: t_conv = {avg_t:.2f}s | e_pos = {avg_ep:.2f} cm | e_yaw = {avg_ey:.2f}° ({len(records)}/5 run)")

        if all_tconv:
            grand_avg_t = sum(all_tconv) / len(all_tconv)
            grand_avg_ep = sum(all_epos_cm) / len(all_epos_cm)
            grand_avg_ey = sum(all_eyaw) / len(all_eyaw)
            status_kriteria = (
                "MEMENUHI SYARAT (LULUS)"
                if (grand_avg_t <= PASS_MAX_TCONV and grand_avg_ep <= PASS_AVG_EPOS * 100 and grand_avg_ey <= PASS_AVG_EYAW)
                else "BELUM MEMENUHI"
            )
            print("=" * 90)
            print(f"🏆 RATA-RATA TOTAL KESELURUHAN ({len(all_tconv)}/25 PENGUJIAN):")
            print(f"  • Rata-rata Waktu Konvergensi : {grand_avg_t:.2f} detik  (Batas: <= {PASS_MAX_TCONV:.1f} s)")
            print(f"  • Rata-rata Galat Posisi (e_pos) : {grand_avg_ep:.2f} cm     (Batas: <= {PASS_AVG_EPOS*100:.1f} cm)")
            print(f"  • Rata-rata Galat Sudut (e_theta): {grand_avg_ey:.2f}°       (Batas: <= {PASS_AVG_EYAW:.1f}°)")
            print(f"  • KESIMPULAN AKHIR SKENARIO 2   : [{status_kriteria}]")
            print("=" * 90)

    def edit_datum_coordinates(self):
        """Menu interaktif untuk mengubah koordinat Ground Truth 5 titik acuan."""
        while True:
            print("\n" + "=" * 65)
            print("🛠️ UBAH KOORDINAT GROUND TRUTH TITIK ACUAN (DATUM)")
            print("=" * 65)
            if self.current_pose:
                cur_x = self.current_pose['x']
                cur_y = self.current_pose['y']
                cur_yaw = math.degrees(self.current_pose['yaw'])
                print(f"  📍 Pose Live AMCL Saat Ini : X={cur_x:6.3f}m, Y={cur_y:6.3f}m, Yaw={cur_yaw:5.1f}°")
                print("-" * 65)

            for i in range(1, 6):
                d = self.datums[i]
                print(f"  [{i}] {d['name']:<24} -> X={d['x']:6.3f}m, Y={d['y']:6.3f}m, Yaw={d['yaw_deg']:5.1f}°")
            print("  [0] Selesai & Kembali ke Menu Utama")
            print("-" * 65)

            choice = input("Pilih nomor datum yang ingin diedit (1-5, atau 0): ").strip()
            if choice == '0':
                break
            if choice in ('1', '2', '3', '4', '5'):
                idx = int(choice)
                cur = self.datums[idx]
                print(f"\nMengedit: {cur['name']}")
                if self.current_pose:
                    print("💡 Tips: Masukkan 'C' pada nama/koordinat untuk menyalin Pose Live AMCL secara instan.")

                name_in = input(f"Nama Datum [{cur['name']}]: ").strip()
                if name_in.upper() == 'C' and self.current_pose:
                    cur['x'] = round(self.current_pose['x'], 3)
                    cur['y'] = round(self.current_pose['y'], 3)
                    cur['yaw_deg'] = round(math.degrees(self.current_pose['yaw']), 1)
                    self.datums[idx] = cur
                    self.save_datum_config()
                    print(f"✅ Berhasil menyalin Pose Live AMCL ke {cur['name']}: X={cur['x']}, Y={cur['y']}, Yaw={cur['yaw_deg']}°")
                    continue

                if name_in:
                    cur['name'] = name_in

                x_in = input(f"Koordinat X (meter) [{cur['x']}]: ").strip()
                if x_in.upper() == 'C' and self.current_pose:
                    cur['x'] = round(self.current_pose['x'], 3)
                    cur['y'] = round(self.current_pose['y'], 3)
                    cur['yaw_deg'] = round(math.degrees(self.current_pose['yaw']), 1)
                    self.datums[idx] = cur
                    self.save_datum_config()
                    print(f"✅ Berhasil menyalin Pose Live AMCL: X={cur['x']}, Y={cur['y']}, Yaw={cur['yaw_deg']}°")
                    continue
                elif x_in:
                    try:
                        cur['x'] = float(x_in)
                    except ValueError:
                        print("Nilai X tidak valid, menggunakan nilai lama.")

                y_in = input(f"Koordinat Y (meter) [{cur['y']}]: ").strip()
                if y_in:
                    try:
                        cur['y'] = float(y_in)
                    except ValueError:
                        print("Nilai Y tidak valid, menggunakan nilai lama.")

                yaw_in = input(f"Sudut Hadap Yaw (derajat) [{cur['yaw_deg']}]: ").strip()
                if yaw_in:
                    try:
                        cur['yaw_deg'] = float(yaw_in)
                    except ValueError:
                        print("Nilai Yaw tidak valid, menggunakan nilai lama.")

                self.datums[idx] = cur
                self.save_datum_config()
                print("✅ Perubahan datum berhasil disimpan!")


def main(args=None):
    rclpy.init(args=args)
    node = EvaluasiAMCLNode()

    # Jalankan spin ringan di background thread agar callback /amcl_pose terus terupdate
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        while True:
            mode_badge = "POSE TRACKING (Inisialisasi Lokal ±30cm)" if node.test_mode == 'tracking' else "GLOBAL LOCALIZATION (Acak 2000 Partikel)"
            print("\n" + "=" * 70)
            print("🤖 BENCHMARK SKRIPSI AMR-01: SKENARIO 2 (EVALUASI PELACAKAN AMCL)")
            print("=" * 70)
            print(f"  Mode Uji Aktif      : 🎯 {mode_badge}")
            print(f"  File Log CSV        : {node.csv_path}")
            print(f"  Total Data Tersimpan: {node.get_total_csv_rows()} / 25 kali pengujian")
            print("-" * 70)
            print("  [1] Jalankan Uji Manual Bertahap (Pilih Titik Datum & Run)")
            print("  [2] Jalankan 5 Run Berurutan untuk 1 Titik Datum")
            print("  [3] Tampilkan Rekapitulasi Tabel & Statistik Bab 4")
            print("  [4] Ubah Koordinat Ground Truth 5 Titik Datum")
            print("  [M] Ganti Mode Pengujian (Pose Tracking <-> Global Localization)")
            print("  [5] Reset / Hapus Seluruh Data CSV")
            print("  [0] Keluar")
            print("=" * 70)

            choice = input("Pilih menu: ").strip()

            if choice.upper() == 'M':
                if node.test_mode == 'tracking':
                    node.test_mode = 'global'
                    print("🔄 Mode dialihkan ke: GLOBAL LOCALIZATION (Kidnapped Robot).")
                else:
                    node.test_mode = 'tracking'
                    print("🔄 Mode dialihkan ke: POSE TRACKING (Inisialisasi Partikel Lokal ±30cm - Rekomendasi Skripsi).")

            if choice == '1':
                print("\nPilih Titik Datum:")
                for i in range(1, 6):
                    d = node.datums[i]
                    next_run = node.get_next_run_number(i)
                    print(f"  [{i}] {d['name']} (Run berikutnya: {next_run}/5)")
                d_choice = input("Pilih nomor datum (1-5): ").strip()
                if d_choice in ('1', '2', '3', '4', '5'):
                    d_id = int(d_choice)
                    run_in = input(f"Nomor Run (1-5) [Default: {node.get_next_run_number(d_id)}]: ").strip()
                    run_idx = int(run_in) if run_in.isdigit() else node.get_next_run_number(d_id)
                    node.run_single_test(d_id, run_idx)

            elif choice == '2':
                print("\nPilih Titik Datum untuk 5 Run Otomatis:")
                for i in range(1, 6):
                    d = node.datums[i]
                    print(f"  [{i}] {d['name']}")
                d_choice = input("Pilih nomor datum (1-5): ").strip()
                if d_choice in ('1', '2', '3', '4', '5'):
                    d_id = int(d_choice)
                    print(f"\n🚀 Memulai 5 Run untuk {node.datums[d_id]['name']}...")
                    for r in range(1, 6):
                        node.run_single_test(d_id, r)
                        if r < 5:
                            input(f"\n👉 Selesai Run {r}. Siapkan robot di posisi awal lalu tekan [ENTER] untuk Run {r+1}...")

            elif choice == '3':
                node.print_recap_table()

            elif choice == '4':
                node.edit_datum_coordinates()

            elif choice == '5':
                confirm = input("⚠️ Yakin ingin menghapus seluruh riwayat CSV pengujian? (ketik 'ya'): ").strip()
                if confirm.lower() == 'ya':
                    if os.path.exists(node.csv_path):
                        os.remove(node.csv_path)
                    node.ensure_csv_header()
                    print("✅ Data CSV berhasil di-reset!")

            elif choice == '0':
                print("👋 Keluar dari sistem pengujian AMCL.")
                break

    except KeyboardInterrupt:
        print("\n🛑 Program dihentikan pengguna.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
