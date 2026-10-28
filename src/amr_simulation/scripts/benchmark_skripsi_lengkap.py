#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
UNIFIED BENCHMARK SYSTEM AMR-01 (PENGUJIAN GABUNGAN SKRIPSI & FIRMWARE)
=============================================================================
Penulis  : Divala Zahra Oktavia & Tim AMR-01 (Teknik Komputer UNNES 2026)
Deskripsi: Menggabungkan pengujian performa PID-PSO roda & akurasi odometri 
           (Skripsi Divala) dengan pengujian mode mandiri firmware / navigasi 
           (Skripsi Teman).
=============================================================================
"""

import os
import sys
import time
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


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


class UnifiedBenchmarkNode(Node):
    def __init__(self):
        super().__init__('unified_benchmark_node')

        # =====================================================================
        # 1. PARAMETER KENDALI PID (OPTIMAL PSO MULTI-SETPOINT)
        # =====================================================================
        # Nilai ini sinkron dengan amr_firmware.ino pada Arduino Uno:
        self.mode_tuning = "PSO_MULTI_SETPOINT" # Opsi: "PSO_MULTI_SETPOINT" atau "TRIAL_ERROR"
        
        # Parameter Motor Kiri (Left Motor)
        self.kp_l = 0.9531
        self.ki_l = 21.0829
        self.kd_l = 0.0000

        # Parameter Motor Kanan (Right Motor)
        self.kp_r = 0.7861
        self.ki_r = 23.2146
        self.kd_r = 0.0000

        # Parameter Alternatif untuk Uji Perbandingan Trial & Error (Jika diperlukan)
        self.manual_kp_l, self.manual_ki_l, self.manual_kd_l = 1.2000, 15.0000, 0.0000
        self.manual_kp_r, self.manual_ki_r, self.manual_kd_r = 1.0000, 15.0000, 0.0000

        # =====================================================================
        # 2. PARAMETER FISIK ROBOT AMR-01 (SESUAI PROPOSAL SKRIPSI)
        # =====================================================================
        self.wheel_diameter = 0.127       # D = 0.127 m (5 inch)
        self.wheel_separation = 0.483     # B = 0.483 m (Kalibrasi UMBmark: efektif kontak ban)

        # Publishers & Subscribers (Depth 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pid_l_sub = self.create_subscription(Point, '/pid/motor_left', self.pid_l_callback, 10)
        self.pid_r_sub = self.create_subscription(Point, '/pid/motor_right', self.pid_r_callback, 10)

        # Telemetri Real-time Odometri
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_received = False

        # Telemetri Real-time PID Motor
        self.pid_l = {'target': 0.0, 'actual': 0.0, 'pwm': 0.0}
        self.pid_r = {'target': 0.0, 'actual': 0.0, 'pwm': 0.0}
        self.pid_received = False

        # Direktori Penyimpanan Log Hasil Pengujian
        self.output_dir = os.path.expanduser('~/hasil_pengujian_skripsi')
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("✅ Unified Benchmark AMR-01 Aktif.")

    def get_pid_header_info(self) -> str:
        """Mengembalikan header teks parameter PID untuk dokumentasi file CSV."""
        return (
            f"# TUNING_METHOD: {self.mode_tuning}\n"
            f"# MOTOR_LEFT : Kp={self.kp_l:.4f}, Ki={self.ki_l:.4f}, Kd={self.kd_l:.4f}\n"
            f"# MOTOR_RIGHT: Kp={self.kp_r:.4f}, Ki={self.ki_r:.4f}, Kd={self.kd_r:.4f}\n"
            f"# WHEEL_DIAMETER: {self.wheel_diameter}m, WHEEL_BASE: {self.wheel_separation}m\n"
        )

    def print_active_pid_banner(self):
        """Menampilkan nilai konstanta PID yang aktif ke layar terminal."""
        print("-" * 75)
        print(f"  ⚙️  METODE TUNING AKTIF : [{self.mode_tuning}]")
        print(f"  • Motor Kiri  (L) : Kp = {self.kp_l:.4f} | Ki = {self.ki_l:.4f} | Kd = {self.kd_l:.4f}")
        print(f"  • Motor Kanan (R) : Kp = {self.kp_r:.4f} | Ki = {self.ki_r:.4f} | Kd = {self.kd_r:.4f}")
        print("-" * 75)

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = euler_from_quaternion(q)
        self.odom_received = True

    def pid_l_callback(self, msg: Point):
        self.pid_l['target'] = msg.x
        self.pid_l['actual'] = msg.y
        self.pid_l['pwm'] = msg.z
        self.pid_received = True

    def pid_r_callback(self, msg: Point):
        self.pid_r['target'] = msg.x
        self.pid_r['actual'] = msg.y
        self.pid_r['pwm'] = msg.z

    def send_cmd(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Kirim perintah stop beberapa kali secara aktif."""
        for _ in range(8):
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_for_connections(self):
        """Pastikan publisher /cmd_vel terhubung ke arduino_bridge & data odom aktif."""
        print("[*] Memeriksa koneksi ROS 2 ke arduino_bridge & data sensor odometri...")
        t_start = time.time()
        while (self.cmd_pub.get_subscription_count() == 0 or not self.odom_received) and (time.time() - t_start) < 3.5:
            rclpy.spin_once(self, timeout_sec=0.1)

        subs = self.cmd_pub.get_subscription_count()
        if subs > 0 and self.odom_received:
            print(f"[✅ OK] Terhubung ke {subs} listener di /cmd_vel & Odometri aktif (Yaw awal: {math.degrees(self.odom_yaw):.1f}°).")
        elif subs > 0:
            print("[⚠️ PERINGATAN] Listener /cmd_vel siap, tapi data /odom belum terdeteksi. Pastikan Arduino mengirim data.")
        else:
            print("[❌ ERROR] Belum mendeteksi listener di /cmd_vel. Jalankan ros2 run amr_simulation arduino_bridge.py!")

    def rpm_to_linear_velocity(self, rpm: float) -> float:
        """Hitung kecepatan linear v (m/s) dari target RPM roda (saat jalan lurus w=0)."""
        return (rpm * math.pi * self.wheel_diameter) / 60.0

    # =========================================================================
    # BAGIAN A.1: RESPON KECEPATAN RODA (STEP RESPONSE 60, 80, 100 RPM)
    # =========================================================================
    def run_step_response_test(self, target_rpm: float, duration_sec: float = 6.0):
        self.wait_for_connections()

        v_target = self.rpm_to_linear_velocity(target_rpm)
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN STEP KECEPATAN RODA: TARGET = {target_rpm:.1f} RPM")
        print(f"  Kecepatan Linier Ekivalen = {v_target:.4f} m/s | Durasi: {duration_sec} detik")
        self.print_active_pid_banner()
        print("=" * 70)

        print("[*] Bersiap... Robot mulai dalam 1 detik...")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"step_respon_{int(target_rpm)}rpm_{timestamp_str}.csv")

        log_data = []
        start_time = time.time()
        print(f"[*] 🚀 MOTOR BERPUTAR ({target_rpm} RPM)... Tekan Ctrl+C untuk membatalkan.")

        try:
            loop_count = 0
            while (time.time() - start_time) < duration_sec:
                t_now = time.time() - start_time
                self.send_cmd(v_target, 0.0)
                rclpy.spin_once(self, timeout_sec=0.04)

                target_l = self.pid_l['target']
                actual_l = self.pid_l['actual']
                pwm_l = self.pid_l['pwm']
                target_r = self.pid_r['target']
                actual_r = self.pid_r['actual']
                pwm_r = self.pid_r['pwm']

                err_l = target_l - actual_l
                err_r = target_r - actual_r

                log_data.append({
                    'time': t_now,
                    'target_L': target_l,
                    'actual_L': actual_l,
                    'pwm_L': pwm_l,
                    'err_L': err_l,
                    'target_R': target_r,
                    'actual_R': actual_r,
                    'pwm_R': pwm_r,
                    'err_R': err_r
                })

                loop_count += 1
                if loop_count % 10 == 0:
                    sys.stdout.write(
                        f"\r[t={t_now:4.1f}s] Roda KIRI: Act={actual_l:5.1f} (PWM={int(pwm_l):3d}) | "
                        f"Roda KANAN: Act={actual_r:5.1f} (PWM={int(pwm_r):3d})"
                    )
                    sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Dihentikan pengguna secara manual.")
        finally:
            self.stop_robot()
            print("\n[*] Robot Berhenti.")

        # Simpan ke CSV lengkap dengan Metadata Kp, Ki, Kd
        if log_data:
            with open(csv_filename, 'w') as f:
                f.write(self.get_pid_header_info())
                f.write("Time_s,Target_RPM_L,Actual_RPM_L,PWM_L,Error_L,Target_RPM_R,Actual_RPM_R,PWM_R,Error_R\n")
                for row in log_data:
                    f.write(
                        f"{row['time']:.3f},{row['target_L']:.2f},{row['actual_L']:.2f},{row['pwm_L']:.0f},{row['err_L']:.2f},"
                        f"{row['target_R']:.2f},{row['actual_R']:.2f},{row['pwm_R']:.0f},{row['err_R']:.2f}\n"
                    )
            print(f"[✅] Data berhasil disimpan ke: {csv_filename}")
            self.analyze_step_performance(log_data, target_rpm)

    def analyze_step_performance(self, log_data, target_rpm):
        """Kalkulasi otomatis karakteristik respon transien (Rise Time, Overshoot, Ess)."""
        if not log_data or target_rpm <= 0:
            return

        print("\n" + "-" * 70)
        print("  ANALISIS METRIK RESPON TRANSIEN SKRIPSI")
        print("-" * 70)

        for side, key_act in [('MOTOR KIRI', 'actual_L'), ('MOTOR KANAN', 'actual_R')]:
            actuals = [d[key_act] for d in log_data]
            times = [d['time'] for d in log_data]

            peak_val = max(actuals) if actuals else 0.0
            overshoot = max(0.0, ((peak_val - target_rpm) / target_rpm) * 100.0)

            val_10 = 0.10 * target_rpm
            val_90 = 0.90 * target_rpm
            t_10 = next((t for t, y in zip(times, actuals) if y >= val_10), None)
            t_90 = next((t for t, y in zip(times, actuals) if y >= val_90), None)
            rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

            last_samples = actuals[-int(len(actuals) * 0.2):]
            mean_final = sum(last_samples) / len(last_samples) if last_samples else 0.0
            ess = abs(target_rpm - mean_final)
            ess_pct = (ess / target_rpm) * 100.0

            print(f"[{side}]")
            print(f"  • Nilai Puncak (Peak)     : {peak_val:.2f} RPM")
            print(f"  • Overshoot (Mp)          : {overshoot:.2f} %")
            print(f"  • Rise Time (Tr 10%-90%)  : {rise_time:.3f} s" if rise_time else "  • Rise Time (Tr 10%-90%)  : -")
            print(f"  • Steady-State Value      : {mean_final:.2f} RPM")
            print(f"  • Steady-State Error (Ess): {ess:.2f} RPM ({ess_pct:.2f} %)")
        print("-" * 70)

    # =========================================================================
    # BAGIAN A.2: ODOMETRI GERAK LURUS (2.0 METER)
    # =========================================================================
    def run_odometry_straight_test(self, target_dist: float = 2.0, test_speed: float = 0.40):
        self.wait_for_connections()

        rpm_equiv = (test_speed / (math.pi * self.wheel_diameter)) * 60.0
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN AKURASI ODOMETRI: GERAK LURUS {target_dist:.2f} METER")
        print(f"  Kecepatan Uji: {test_speed:.2f} m/s (~{rpm_equiv:.1f} RPM)")
        self.print_active_pid_banner()
        print("=" * 70)

        input("[*] Posisikan robot tepat di garis start lantai, lalu tekan ENTER untuk mulai...")

        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_lurus_{int(target_dist)}m_{timestamp_str}.csv")

        log_data = []
        start_time = time.time()
        print("[*] 🚀 Robot bergerak maju lurus...")

        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                curr_x = self.odom_x
                curr_y = self.odom_y
                curr_yaw = self.odom_yaw

                dist_traveled = math.hypot(curr_x - start_x, curr_y - start_y)
                remaining = target_dist - dist_traveled
                t_now = time.time() - start_time

                log_data.append({
                    'time': t_now,
                    'x': curr_x, 'y': curr_y,
                    'yaw_deg': math.degrees(curr_yaw),
                    'dist_traveled': dist_traveled
                })

                if remaining <= 0.005:
                    break

                # Soft-Start Ramp 0.4s & Deselerasi halus di 25 cm terakhir
                if t_now < 0.40:
                    v_cmd = max(0.08, test_speed * (t_now / 0.40))
                elif remaining < 0.25:
                    v_cmd = max(0.08, test_speed * (remaining / 0.25))
                else:
                    v_cmd = test_speed

                yaw_err = normalize_angle(start_yaw - curr_yaw)
                w_cmd = 0.8 * yaw_err

                self.send_cmd(v_cmd, w_cmd)
                sys.stdout.write(f"\r[*] Jarak Odom: {dist_traveled:5.3f} m / {target_dist:.2f} m | Yaw={math.degrees(curr_yaw):+5.1f}°")
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Uji lurus dibatalkan pengguna.")
        finally:
            self.stop_robot()
            time.sleep(1.0)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        final_x = self.odom_x
        final_y = self.odom_y
        final_yaw = self.odom_yaw
        final_odom_dist = math.hypot(final_x - start_x, final_y - start_y)
        drift_y = abs(final_y - start_y)

        print("\n\n" + "=" * 70)
        print("  HASIL PENGUJIAN ODOMETRI GERAK LURUS")
        print("=" * 70)
        print(f"• Jarak Terbaca Odometri (D_odom)  : {final_odom_dist:.4f} meter")
        print(f"• Deviasi Lateral ke Samping (ΔY)  : {drift_y:.4f} meter")
        print(f"• Galat Orientasi Akhir (ΔYaw)     : {math.degrees(abs(final_yaw - start_yaw)):.2f}°")
        print("-" * 70)

        real_input = input(">> Masukkan jarak riil di lantai hasil pita ukur [m] (contoh: 2.01): ")
        try:
            d_real = float(real_input)
            err_dist = abs(d_real - final_odom_dist)
            err_pct = (err_dist / d_real) * 100.0
            print(f"\n[📊 EVALUASI AKURASI]: Selisih={err_dist*100:.2f} cm | Error={err_pct:.2f}% | Akurasi={100-err_pct:.2f}%")
        except ValueError:
            pass

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write(self.get_pid_header_info())
                f.write("Time_s,Odom_X,Odom_Y,Odom_Yaw_deg,Dist_Traveled_m\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['x']:.4f},{row['y']:.4f},{row['yaw_deg']:.2f},{row['dist_traveled']:.4f}\n")
            print(f"[✅] Log lintasan disimpan ke: {csv_filename}")

    # =========================================================================
    # BAGIAN A.3: ODOMETRI ROTASI DI TEMPAT (90°, 180°, 360°) - SEKALI JALAN
    # =========================================================================
    def run_odometry_rotation_test(self, target_deg: float = 90.0, rot_speed: float = 0.50):
        self.wait_for_connections()

        target_rad = math.radians(target_deg)
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN ODOMETRI: ROTASI DI TEMPAT {target_deg:.0f}° ({target_rad:.3f} rad)")
        print(f"  Kecepatan Sudut: {rot_speed:.2f} rad/s | Mode: 1 Kali Eksekusi Presisi")
        self.print_active_pid_banner()
        print("=" * 70)

        input("[*] Tandai posisi awal robot di lantai, lalu tekan ENTER untuk mulai...")

        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

        prev_yaw = self.odom_yaw
        accumulated_yaw = 0.0

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_rotasi_{int(target_deg)}deg_{timestamp_str}.csv")

        log_data = []
        start_time = time.time()
        
        # Safety Timeout: Untuk 90° maksimal 8 detik (mencegah berputar terus-menerus jika odometri drop)
        max_timeout = max(8.0, (target_deg / 90.0) * 6.0)
        print(f"[*] 🔄 Robot berputar 1 kali menuju {target_deg:.0f}° (Safety Timeout: {max_timeout:.0f}s)...")

        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                curr_yaw = self.odom_yaw

                dyaw = normalize_angle(curr_yaw - prev_yaw)
                accumulated_yaw += dyaw
                prev_yaw = curr_yaw

                t_now = time.time() - start_time
                log_data.append({
                    'time': t_now,
                    'yaw_accum_deg': math.degrees(abs(accumulated_yaw)),
                    'target_deg': target_deg
                })

                # KONDISI HENTI 1: Target sudut tercapai (toleransi presisi 0.3 derajat)
                remaining = target_rad - abs(accumulated_yaw)
                if remaining <= math.radians(0.3):
                    print(f"\n[🎯 TARGET TERCAPAI] Sudut {math.degrees(abs(accumulated_yaw)):.2f}° tercapai! Robot Berhenti.")
                    break

                # KONDISI HENTI 2: Pengaman Waktu Maksimal (Mencegah berputar berkali-kali)
                if t_now > max_timeout:
                    print(f"\n[⚠️ TIMEOUT] Batas waktu maksimal {max_timeout:.0f}s tercapai. Robot dipaksa berhenti.")
                    break

                # Perlambatan halus di 15 derajat terakhir (merayap hingga 0.12 rad/s agar presisi tanpa inersia)
                if remaining < math.radians(15.0):
                    w_cmd = max(0.12, rot_speed * (remaining / math.radians(15.0)))
                else:
                    w_cmd = rot_speed

                self.send_cmd(0.0, w_cmd)
                sys.stdout.write(f"\r[*] Sudut Putaran Odom: {math.degrees(abs(accumulated_yaw)):6.2f}° / {target_deg:.0f}°")
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Uji rotasi dibatalkan pengguna.")
        finally:
            self.stop_robot()
            time.sleep(1.0)

        final_deg = math.degrees(abs(accumulated_yaw))
        print("\n\n" + "=" * 70)
        print(f"• Sudut Putaran Terbaca Odometri : {final_deg:.2f}°")

        real_input = input(">> Masukkan sudut riil di lantai hasil ukur busur/laser [°] (contoh: 89.5): ")
        try:
            deg_real = float(real_input)
            err_deg = abs(deg_real - final_deg)
            err_pct = (err_deg / deg_real) * 100.0
            print(f"\n[📊 EVALUASI AKURASI]: Selisih={err_deg:.2f}° | Error={err_pct:.2f}% | Akurasi={100-err_pct:.2f}%")
        except ValueError:
            pass

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write(self.get_pid_header_info())
                f.write("Time_s,Yaw_Accum_deg,Target_deg\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['yaw_accum_deg']:.2f},{row['target_deg']:.2f}\n")
            print(f"[✅] Log rotasi disimpan ke: {csv_filename}")

    # =========================================================================
    # BAGIAN A.4: ODOMETRI PERSEGI TERTUTUP 2m x 2m (UMBMARK)
    # =========================================================================
    def run_odometry_square_test(self, side_length: float = 2.0, straight_speed: float = 0.30, turn_speed: float = 0.50):
        self.wait_for_connections()

        print("\n" + "=" * 70)
        print(f"  PENGUJIAN AKURASI ODOMETRI: PERSEGI TERTUTUP {side_length}m x {side_length}m")
        print("  Metode: UMBmark Benchmark (4 Sisi Lurus + 4 Belokan 90°)")
        self.print_active_pid_banner()
        print("=" * 70)

        input("[*] Beri tanda lakban titik tengah robot di lantai, lalu tekan ENTER...")

        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

        x0, y0, yaw0 = self.odom_x, self.odom_y, self.odom_yaw
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_persegi_{int(side_length)}x{int(side_length)}m_{timestamp_str}.csv")
        log_data = []
        start_time = time.time()

        for side in range(1, 5):
            # 1. Maju Lurus
            print(f"\n[▶ SISI {side}/4]: Maju lurus sejauh {side_length:.2f} m...")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            seg_start_x, seg_start_y, seg_start_yaw = self.odom_x, self.odom_y, self.odom_yaw

            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                cx, cy, cyaw = self.odom_x, self.odom_y, self.odom_yaw
                dist = math.hypot(cx - seg_start_x, cy - seg_start_y)
                rem = side_length - dist

                log_data.append({
                    'time': time.time() - start_time,
                    'x': cx, 'y': cy, 'yaw_deg': math.degrees(cyaw), 'phase': f'Side_{side}'
                })

                if rem <= 0.005:
                    break

                v_cmd = max(0.10, straight_speed * (rem / 0.20)) if rem < 0.20 else straight_speed
                w_cmd = 0.8 * normalize_angle(seg_start_yaw - cyaw)
                self.send_cmd(v_cmd, w_cmd)
                sys.stdout.write(f"\r  Maju: {dist:4.2f}m / {side_length:.2f}m")
                sys.stdout.flush()

            self.stop_robot()
            time.sleep(0.8)

            # 2. Belok 90 Derajat Kiri
            print(f"\n[🔄 SUDUT {side}/4]: Belok 90° ke kiri...")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            prev_yaw = self.odom_yaw
            accum_yaw = 0.0
            target_turn = math.radians(90.0)
            t_turn_start = time.time()

            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                cyaw = self.odom_yaw
                dyaw = normalize_angle(cyaw - prev_yaw)
                accum_yaw += dyaw
                prev_yaw = cyaw

                log_data.append({
                    'time': time.time() - start_time,
                    'x': self.odom_x, 'y': self.odom_y, 'yaw_deg': math.degrees(cyaw), 'phase': f'Turn_{side}'
                })

                rem_turn = target_turn - abs(accum_yaw)
                if rem_turn <= math.radians(0.8) or (time.time() - t_turn_start > 8.0):
                    break

                w_cmd = max(0.22, turn_speed * (rem_turn / math.radians(12.0))) if rem_turn < math.radians(12.0) else turn_speed
                self.send_cmd(0.0, w_cmd)
                sys.stdout.write(f"\r  Putar: {math.degrees(abs(accum_yaw)):5.1f}° / 90.0°")
                sys.stdout.flush()

            self.stop_robot()
            time.sleep(0.8)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        xf, yf, yawf = self.odom_x, self.odom_y, self.odom_yaw
        odom_drift = math.hypot(xf - x0, yf - y0)
        odom_yaw_drift = math.degrees(abs(normalize_angle(yawf - yaw0)))

        print("\n\n" + "=" * 70)
        print("  HASIL EVALUASI LINTASAN PERSEGI TERTUTUP (RETURN-TO-ORIGIN)")
        print("=" * 70)
        print(f"• Drift Posisi Akhir Odometri (ε_odom)   : {odom_drift:.4f} meter ({odom_drift * 100:.2f} cm)")
        print(f"• Drift Orientasi Akhir Odometri (Δθ)   : {odom_yaw_drift:.2f}°")
        print("-" * 70)

        real_input = input(">> Ukur jarak fisik dari tanda awal ke posisi henti akhir robot [cm]: ")
        try:
            real_drift_cm = float(real_input)
            print(f"\n[📊 HASIL UMBMARK]: Drift Real = {real_drift_cm:.2f} cm | Drift Odometri = {odom_drift * 100:.2f} cm")
        except ValueError:
            pass

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write(self.get_pid_header_info())
                f.write("Time_s,Odom_X,Odom_Y,Odom_Yaw_deg,Phase\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['x']:.4f},{row['y']:.4f},{row['yaw_deg']:.2f},{row['phase']}\n")
            print(f"[✅] Log lintasan persegi disimpan ke: {csv_filename}")

    # =========================================================================
    # BAGIAN B: PENGUJIAN MODE FIRMWARE / SKRIPSI TEMAN
    # =========================================================================
    def run_partner_test_mode(self, mode_name: str, lin_vel: float, ang_vel: float, duration_or_dist: float, is_rotation: bool = False):
        """Menjalankan pengujian mode navigasi/firmware teman dengan logging."""
        self.wait_for_connections()
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN FIRMWARE / MODE TEMAN: {mode_name}")
        print(f"  V = {lin_vel:.2f} m/s | W = {ang_vel:.2f} rad/s")
        print("=" * 70)

        input("[*] Posisikan robot pada titik uji, lalu tekan ENTER untuk mulai...")
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

        start_time = time.time()
        start_x, start_y, start_yaw = self.odom_x, self.odom_y, self.odom_yaw
        prev_yaw = start_yaw
        accum_yaw = 0.0

        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                t_now = time.time() - start_time

                if is_rotation:
                    dyaw = normalize_angle(self.odom_yaw - prev_yaw)
                    accum_yaw += dyaw
                    prev_yaw = self.odom_yaw
                    target_rad = math.radians(duration_or_dist)
                    rem_rot = target_rad - abs(accum_yaw)
                    if rem_rot <= math.radians(0.3) or t_now > 15.0:
                        break
                    w_cmd = max(0.12, ang_vel * (rem_rot / math.radians(15.0))) if rem_rot < math.radians(15.0) else ang_vel
                    self.send_cmd(0.0, w_cmd)
                    sys.stdout.write(f"\r[*] Putaran: {math.degrees(abs(accum_yaw)):5.1f}° / {duration_or_dist:.0f}°")
                else:
                    dist = math.hypot(self.odom_x - start_x, self.odom_y - start_y)
                    if dist >= duration_or_dist or t_now > 20.0:
                        break
                    sys.stdout.write(f"\r[*] Jarak: {dist:5.2f}m / {duration_or_dist:.2f}m")
                    self.send_cmd(lin_vel, ang_vel)

                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Dibatalkan pengguna.")
        finally:
            self.stop_robot()
            print("\n[*] Selesai. Robot Berhenti.")


# =============================================================================
# MENU UTAMA INTERAKTIF (BISA PILIH PENGUJIAN ANDA ATAU TEMAN)
# =============================================================================
def main():
    rclpy.init()
    node = UnifiedBenchmarkNode()

    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        while rclpy.ok():
            print("\n" + "=" * 75)
            print("  🤖 AMR-01 UNIFIED BENCHMARK SYSTEM // PILIHAN PENGUJIAN SKRIPSI")
            print("===========================================================================")
            node.print_active_pid_banner()
            print("  [A] PENGUJIAN SAYA (Divala: Kendali PID-PSO Motor & Akurasi Odometri)")
            print("  [B] PENGUJIAN TEMAN (Mode Firmware / Uji Navigasi Mandiri)")
            print("  [T] Ganti Profil Tuning PID (Saat ini: " + node.mode_tuning + ")")
            print("  [0] Keluar Sistem")
            print("=" * 75)

            kategori = input("Pilih Kategori Pengujian [A / B / T / 0]: ").strip().upper()

            # -----------------------------------------------------------------
            # TOGGLE / GANTI PROFIL TUNING PID
            # -----------------------------------------------------------------
            if kategori == 'T':
                print("\n" + "-" * 75)
                print("  PILIH PROFIL TUNING UNTUK LABEL DOKUMENTASI:")
                print("  [1] PID-PSO (Multi-Setpoint Optimal: Kp_L=0.9531, Ki_L=21.0829, Kd=0)")
                print("  [2] PID Manual Trial & Error (Pembanding)")
                print("-" * 75)
                t_pilih = input("Pilih profil [1/2]: ").strip()
                if t_pilih == '1':
                    node.mode_tuning = "PID_PSO_MULTI_SETPOINT"
                    node.kp_l, node.ki_l, node.kd_l = 0.9531, 21.0829, 0.0000
                    node.kp_r, node.ki_r, node.kd_r = 0.7861, 23.2146, 0.0000
                    print("[✅] Profil aktif diubah ke: PID-PSO Optimal.")
                elif t_pilih == '2':
                    node.mode_tuning = "PID_MANUAL_TRIAL_ERROR"
                    node.kp_l, node.ki_l, node.kd_l = node.manual_kp_l, node.manual_ki_l, node.manual_kd_l
                    node.kp_r, node.ki_r, node.kd_r = node.manual_kp_r, node.manual_ki_r, node.manual_kd_r
                    print("[✅] Profil aktif diubah ke: Manual Trial & Error.")
                continue

            # -----------------------------------------------------------------
            # KATEGORI A: PENGUJIAN SKRIPSI DIVALA (MENU 1 - 7)
            # -----------------------------------------------------------------
            elif kategori == 'A':
                print("\n" + "=" * 75)
                print("  --- [KATEGORI A] PENGUJIAN SKRIPSI DIVALA (PID-PSO & ODOMETRI) ---")
                node.print_active_pid_banner()
                print("  [1] Step Test Motor: 60 RPM  (~0.40 m/s)")
                print("  [2] Step Test Motor: 80 RPM  (~0.53 m/s)")
                print("  [3] Step Test Motor: 100 RPM (~0.66 m/s)")
                print("  [4] Multi-Step Motor: Bertingkat (60 -> 80 -> 100 RPM)")
                print("  [5] Uji Odometri: Gerak Lurus 2.0 Meter")
                print("  [6] Uji Odometri: Rotasi 1 Kali (Pilihan Sudut 90° / 180° / 360°)")
                print("  [7] Uji Odometri: Lintasan Persegi 2m x 2m (UMBmark)")
                print("  [9] Kembali ke Menu Utama")
                print("=" * 75)

                pilih_a = input("Pilih nomor pengujian Anda [1-7 / 9]: ").strip()
                if pilih_a == '1':
                    node.run_step_response_test(60.0, duration_sec=6.0)
                elif pilih_a == '2':
                    node.run_step_response_test(80.0, duration_sec=6.0)
                elif pilih_a == '3':
                    node.run_step_response_test(100.0, duration_sec=6.0)
                elif pilih_a == '4':
                    print("\n[*] Menjalankan uji bertingkat...")
                    node.run_step_response_test(60.0, duration_sec=5.0)
                    node.run_step_response_test(80.0, duration_sec=5.0)
                    node.run_step_response_test(100.0, duration_sec=5.0)
                elif pilih_a == '5':
                    node.run_odometry_straight_test(target_dist=2.0)
                elif pilih_a == '6':
                    sub_deg = input("Pilih sudut rotasi (90 / 180 / 360) [default 90]: ").strip() or "90"
                    try:
                        node.run_odometry_rotation_test(target_deg=float(sub_deg), rot_speed=0.50)
                    except ValueError:
                        print("[!] Sudut tidak valid.")
                elif pilih_a == '7':
                    node.run_odometry_square_test(side_length=2.0)
                elif pilih_a == '9':
                    continue

            # -----------------------------------------------------------------
            # KATEGORI B: PENGUJIAN TEMAN / MODE FIRMWARE
            # -----------------------------------------------------------------
            elif kategori == 'B':
                print("\n" + "=" * 75)
                print("  --- [KATEGORI B] PENGUJIAN TEMAN / MODE MANDIRI FIRMWARE ---")
                print("  [1] Uji Gerak Lurus 2.0 Meter (V = 0.33 m/s, W = 0.0)")
                print("  [2] Uji Rotasi Sudut 90 Derajat (W = 0.8 rad/s)")
                print("  [3] Uji Rotasi Sudut 180 Derajat (W = 0.8 rad/s)")
                print("  [4] Uji Rotasi Sudut 360 Derajat (W = 0.8 rad/s)")
                print("  [5] Uji Lintasan Persegi 2m x 2m (Firmware Mode)")
                print("  [9] Kembali ke Menu Utama")
                print("=" * 75)

                pilih_b = input("Pilih nomor pengujian teman [1-5 / 9]: ").strip()
                if pilih_b == '1':
                    node.run_partner_test_mode("Gerak Lurus 2m (Firmware Speed 0.33 m/s)", 0.33, 0.0, 2.0, False)
                elif pilih_b == '2':
                    node.run_partner_test_mode("Rotasi 90 Derajat (Firmware Speed 0.8 rad/s)", 0.0, 0.8, 90.0, True)
                elif pilih_b == '3':
                    node.run_partner_test_mode("Rotasi 180 Derajat (Firmware Speed 0.8 rad/s)", 0.0, 0.8, 180.0, True)
                elif pilih_b == '4':
                    node.run_partner_test_mode("Rotasi 360 Derajat (Firmware Speed 0.8 rad/s)", 0.0, 0.8, 360.0, True)
                elif pilih_b == '5':
                    node.run_odometry_square_test(side_length=2.0, straight_speed=0.33, turn_speed=0.8)
                elif pilih_b == '9':
                    continue

            elif kategori == '0':
                print("[*] Menutup sistem benchmark...")
                break
            else:
                print("[!] Pilihan kategori tidak valid, silakan ketik A, B, atau T.")

    except KeyboardInterrupt:
        print("\n[*] Interupsi terdeteksi. Mematikan robot...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        print("[*] Selesai. Terima kasih!")


if __name__ == '__main__':
    main()
