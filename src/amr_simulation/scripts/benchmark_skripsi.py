#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
BENCHMARK PENGUJIAN SKRIPSI AMR-01 (ROCK-SOLID ROS 2 EDITION)
1. PENGUJIAN PERFORMA KECEPATAN RODA (STEP RESPONSE: 60, 80, 100 RPM)
2. PENGUJIAN AKURASI ODOMETRI (LURUS 2M, ROTASI 90/180/360°, PERSEGI 2x2M)
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Robot   : Autonomous Mobile Robot Differential Drive (AMR-01)
Motor   : DC PG36 Planetary Gear + BTS7960 Driver
Komputasi: ROS 2 Foxy (Jetson) <--> Arduino Uno PID-PSO
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


class BenchmarkSkripsiNode(Node):
    def __init__(self):
        super().__init__('benchmark_skripsi_node')

        # Parameter Fisik AMR-01 Sesuai Proposal Skripsi
        self.wheel_diameter = 0.127       # D = 0.127 m (5 inch)
        self.wheel_separation = 0.458     # B = 0.458 m (jarak antar roda)

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

        self.get_logger().info("✅ Node Benchmark Skripsi AMR-01 Aktif.")

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
        """Pastikan publisher /cmd_vel sudah menemukan subscriber arduino_bridge."""
        print("[*] Memeriksa koneksi ROS 2 ke arduino_bridge...")
        t_start = time.time()
        while self.cmd_pub.get_subscription_count() == 0 and (time.time() - t_start) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        subs = self.cmd_pub.get_subscription_count()
        if subs > 0:
            print(f"[✅ OK] Terhubung ke {subs} listener di topik /cmd_vel (arduino_bridge siap).")
        else:
            print("[⚠️ PERINGATAN] Belum mendeteksi listener di /cmd_vel. Pastikan arduino_bridge.py sudah jalan!")

    def rpm_to_linear_velocity(self, rpm: float) -> float:
        """Hitung kecepatan linear v (m/s) dari target RPM roda (saat jalan lurus w=0)."""
        return (rpm * math.pi * self.wheel_diameter) / 60.0

    # =========================================================================
    # PENGUJIAN 1: RESPON KECEPATAN RODA (STEP RESPONSE 60, 80, 100 RPM)
    # =========================================================================
    def run_step_response_test(self, target_rpm: float, duration_sec: float = 6.0):
        self.wait_for_connections()

        v_target = self.rpm_to_linear_velocity(target_rpm)
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN STEP KECEPATAN RODA: TARGET = {target_rpm:.1f} RPM")
        print(f"  Kecepatan Linier Ekivalen = {v_target:.4f} m/s | Durasi: {duration_sec} detik")
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
                
                # Kirim perintah gerak kontinu (20 Hz)
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

        # Simpan ke CSV
        if log_data:
            with open(csv_filename, 'w') as f:
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

            # Rise Time (10% s.d 90%)
            val_10 = 0.10 * target_rpm
            val_90 = 0.90 * target_rpm
            t_10 = next((t for t, y in zip(times, actuals) if y >= val_10), None)
            t_90 = next((t for t, y in zip(times, actuals) if y >= val_90), None)
            rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

            # Steady-State Error (rata-rata 1 detik terakhir)
            last_samples = actuals[-int(len(actuals) * 0.2):]
            mean_final = sum(last_samples) / len(last_samples) if last_samples else 0.0
            ess = abs(target_rpm - mean_final)
            ess_pct = (ess / target_rpm) * 100.0

            print(f"[{side}]")
            print(f"  • Nilai Puncak (Peak)     : {peak_val:.2f} RPM")
            print(f"  • Overshoot (Mp)          : {overshoot:.2f} %")
            if rise_time:
                print(f"  • Rise Time (Tr 10%-90%)  : {rise_time:.3f} detik")
            else:
                print("  • Rise Time (Tr 10%-90%)  : -")
            print(f"  • Steady-State Value      : {mean_final:.2f} RPM")
            print(f"  • Steady-State Error (Ess): {ess:.2f} RPM ({ess_pct:.2f} %)")
        print("-" * 70)

    # =========================================================================
    # PENGUJIAN 2: AKURASI ODOMETRI - GERAK LURUS 2.0 METER
    # =========================================================================
    def run_odometry_straight_test(self, target_dist: float = 2.0, test_speed: float = 0.40):
        self.wait_for_connections()

        rpm_equiv = (test_speed / (math.pi * self.wheel_diameter)) * 60.0
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN AKURASI ODOMETRI: GERAK LURUS {target_dist:.2f} METER")
        print(f"  Kecepatan Uji: {test_speed:.2f} m/s (~{rpm_equiv:.1f} RPM)")
        print("=" * 70)

        # Tunggu pembacaan awal odometri
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        input("[*] Posisikan robot tepat di garis start lantai, lalu tekan ENTER untuk mulai...")

        # Pastikan pose awal fresh
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw

        print(f"[*] Posisi Awal Odom: X0={start_x:.4f} m, Y0={start_y:.4f} m, Yaw0={math.degrees(start_yaw):.2f}°")

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_lurus_{int(target_dist)}m_{timestamp_str}.csv")

        log_data = []
        start_time = time.time()
        ramp_duration = 0.4  # Akselerasi halus selama 400 ms
        print("[*] 🚀 Robot bergerak maju (dengan Soft-Start Ramp 0.4s & Heading Lock)...")

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
                    'x': curr_x,
                    'y': curr_y,
                    'yaw_deg': math.degrees(curr_yaw),
                    'dist_traveled': dist_traveled,
                })

                if remaining <= 0.005:
                    break

                # 1. Ramp akselerasi halus saat start & deselerasi di akhir
                if t_now < ramp_duration:
                    v_cmd = test_speed * (t_now / ramp_duration)
                    v_cmd = max(0.08, v_cmd)
                elif remaining < 0.25:
                    v_cmd = max(0.08, test_speed * (remaining / 0.25))
                else:
                    v_cmd = test_speed

                # 2. Koreksi sudut diperkuat ke 2.2
                yaw_err = normalize_angle(start_yaw - curr_yaw)
                w_cmd = max(-0.35, min(0.35, 2.2 * yaw_err))

                self.send_cmd(v_cmd, w_cmd)
                sys.stdout.write(f"\r[*] Jarak Odom: {dist_traveled:5.3f} m / {target_dist:.2f} m | Yaw={math.degrees(curr_yaw):+5.1f}° (Err={math.degrees(yaw_err):+4.2f}°)")
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
        real_input = input(">> Masukkan jarak riil di lantai hasil pita ukur/meteran (contoh: 2.01) [m]: ")
        try:
            d_real = float(real_input)
            err_dist = abs(d_real - final_odom_dist)
            err_pct = (err_dist / d_real) * 100.0
            print("\n[📊 HASIL EVALUASI AKURASI SKRIPSI]:")
            print(f"  • Ground Truth (Pita Ukur) : {d_real:.4f} m")
            print(f"  • Selisih Galat Translasi  : {err_dist:.4f} m ({err_dist * 100.0:.2f} cm)")
            print(f"  • Persentase Galat (%Ed)   : {err_pct:.2f} %")
            print(f"  • Akurasi Odometri         : {100.0 - err_pct:.2f} %")
        except ValueError:
            print("[!] Input ground truth tidak valid, dilewati.")

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write("Time_s,Odom_X,Odom_Y,Odom_Yaw_deg,Dist_Traveled_m\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['x']:.4f},{row['y']:.4f},{row['yaw_deg']:.2f},{row['dist_traveled']:.4f}\n")
            print(f"[✅] Log lintasan disimpan ke: {csv_filename}")

    # =========================================================================
    # PENGUJIAN 3: AKURASI ODOMETRI - ROTASI DI TEMPAT (90°, 180°, 360°)
    # =========================================================================
    def run_odometry_rotation_test(self, target_deg: float = 360.0, rot_speed: float = 0.35):
        self.wait_for_connections()

        target_rad = math.radians(target_deg)
        rpm_wheel = ((self.wheel_separation / 2.0 * rot_speed) / (math.pi * self.wheel_diameter)) * 60.0
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN AKURASI ODOMETRI: ROTASI DI TEMPAT {target_deg:.0f}° ({target_rad:.3f} rad)")
        print(f"  Kecepatan Sudut: {rot_speed:.2f} rad/s (~{rpm_wheel:.1f} RPM per roda)")
        print("=" * 70)

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        input("[*] Tandai sudut orientasi awal robot di lantai, lalu tekan ENTER untuk mulai...")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        prev_yaw = self.odom_yaw
        accumulated_yaw = 0.0

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_rotasi_{int(target_deg)}deg_{timestamp_str}.csv")

        log_data = []
        start_time = time.time()
        print("[*] 🔄 Robot berputar di tempat...")

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
                    'yaw_accum_deg': math.degrees(accumulated_yaw),
                    'target_deg': target_deg
                })

                remaining = abs(target_rad - abs(accumulated_yaw))
                if remaining <= math.radians(0.8):
                    break

                if remaining < math.radians(15.0):
                    w_cmd = max(0.12, rot_speed * (remaining / math.radians(15.0)))
                else:
                    w_cmd = rot_speed

                self.send_cmd(0.0, w_cmd)
                sys.stdout.write(f"\r[*] Akumulasi Putaran Odom: {math.degrees(accumulated_yaw):6.2f}° / {target_deg:.0f}°")
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Uji rotasi dibatalkan pengguna.")
        finally:
            self.stop_robot()
            time.sleep(1.0)

        final_deg = math.degrees(abs(accumulated_yaw))
        print("\n\n" + "=" * 70)
        print(f"• Sudut Putaran Terbaca Odometri : {final_deg:.2f}°")

        real_input = input(">> Masukkan sudut riil di lantai hasil ukur busur/laser (contoh: 358.5) [°]: ")
        try:
            deg_real = float(real_input)
            err_deg = abs(deg_real - final_deg)
            err_pct = (err_deg / deg_real) * 100.0
            print("\n[📊 HASIL EVALUASI AKURASI ROTASI SKRIPSI]:")
            print(f"  • Ground Truth Sudut   : {deg_real:.2f}°")
            print(f"  • Galat Orientasi (Δθ) : {err_deg:.2f}°")
            print(f"  • Persentase Galat (%Eθ): {err_pct:.2f} %")
            print(f"  • Akurasi Orientasi    : {100.0 - err_pct:.2f} %")
        except ValueError:
            print("[!] Input ground truth tidak valid, dilewati.")

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write("Time_s,Yaw_Accum_deg,Target_deg\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['yaw_accum_deg']:.2f},{row['target_deg']:.2f}\n")
            print(f"[✅] Log rotasi disimpan ke: {csv_filename}")

    # =========================================================================
    # PENGUJIAN 4: AKURASI ODOMETRI - PERSEGI TERTUTUP 2m x 2m (RETURN-TO-ORIGIN)
    # =========================================================================
    def run_odometry_square_test(self, side_length: float = 2.0, straight_speed: float = 0.25, turn_speed: float = 0.35):
        self.wait_for_connections()

        rpm_straight = (straight_speed / (math.pi * self.wheel_diameter)) * 60.0
        rpm_turn = ((self.wheel_separation / 2.0 * turn_speed) / (math.pi * self.wheel_diameter)) * 60.0
        print("\n" + "=" * 70)
        print(f"  PENGUJIAN AKURASI ODOMETRI: PERSEGI TERTUTUP {side_length}m x {side_length}m")
        print("  Metode: UMBmark Benchmark (4 Sisi Translasi + 4 Belokan 90°)")
        print(f"  • Sisi Lurus : {straight_speed:.2f} m/s (~{rpm_straight:.1f} RPM)")
        print(f"  • Sudut Belok: {turn_speed:.2f} rad/s (~{rpm_turn:.1f} RPM)")
        print("=" * 70)

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        input("[*] Beri tanda (lakban/spidol) tepat pada titik tengah robot di lantai, lalu tekan ENTER...")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        x0 = self.odom_x
        y0 = self.odom_y
        yaw0 = self.odom_yaw

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"odom_persegi_{int(side_length)}x{int(side_length)}m_{timestamp_str}.csv")
        log_data = []

        start_time = time.time()

        for side in range(1, 5):
            # 1. Tentukan Target Sudut Absolut untuk Sisi Ini (0°, 90°, 180°, 270°)
            expected_heading = normalize_angle(yaw0 + (side - 1) * (math.pi / 2.0))
            print(f"\n[▶ SISI {side}/4]: Maju lurus sejauh {side_length:.2f} m | Kunci Sudut = {math.degrees(expected_heading):+5.1f}°")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            seg_start_x = self.odom_x
            seg_start_y = self.odom_y
            t_seg_start = time.time()

            # Sub-langkah 1: Maju Lurus dengan Soft-Start 0.4s & Kunci Heading P=2.2
            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                cx = self.odom_x
                cy = self.odom_y
                cyaw = self.odom_yaw

                dist = math.hypot(cx - seg_start_x, cy - seg_start_y)
                rem = side_length - dist
                t_seg = time.time() - t_seg_start

                log_data.append({
                    'time': time.time() - start_time,
                    'x': cx, 'y': cy, 'yaw_deg': math.degrees(cyaw), 'phase': f'Side_{side}'
                })

                if rem <= 0.005:
                    break

                # Soft-Start Ramp 0.4s
                if t_seg < 0.40:
                    v_cmd = max(0.08, straight_speed * (t_seg / 0.40))
                # Deselerasi halus di 20 cm terakhir
                elif rem < 0.20:
                    v_cmd = max(0.08, straight_speed * (rem / 0.20))
                else:
                    v_cmd = straight_speed

                # Koreksi kemudi P kuat (Gain Kp = 2.2) mengunci target heading absolut
                yaw_err = normalize_angle(expected_heading - cyaw)
                w_cmd = max(-0.35, min(0.35, 2.2 * yaw_err))

                self.send_cmd(v_cmd, w_cmd)
                sys.stdout.write(f"\r  Maju: {dist:4.2f}m / {side_length:.2f}m | Yaw={math.degrees(cyaw):+5.1f}° (Err={math.degrees(yaw_err):+4.2f}°)")
                sys.stdout.flush()

            self.stop_robot()
            time.sleep(0.8)

            # Sub-langkah 2: Belok 90 Derajat Kiri dengan Kunci Sudut Absolut
            target_heading = normalize_angle(yaw0 + side * (math.pi / 2.0))
            print(f"\n[🔄 SUDUT {side}/4]: Belok 90° ke kiri | Target Absolut = {math.degrees(target_heading):+5.1f}°")

            while True:
                rclpy.spin_once(self, timeout_sec=0.03)
                cyaw = self.odom_yaw
                
                # Hitung sisa sudut belok menuju target absolut
                turn_err = normalize_angle(target_heading - cyaw)

                log_data.append({
                    'time': time.time() - start_time,
                    'x': self.odom_x, 'y': self.odom_y, 'yaw_deg': math.degrees(cyaw), 'phase': f'Turn_{side}'
                })

                if abs(turn_err) <= math.radians(0.8):
                    break

                # Perlambat di 15 derajat terakhir belokan
                if abs(turn_err) < math.radians(15.0):
                    w_mag = max(0.10, turn_speed * (abs(turn_err) / math.radians(15.0)))
                else:
                    w_mag = turn_speed

                w_cmd = math.copysign(w_mag, turn_err)
                self.send_cmd(0.0, w_cmd)
                sys.stdout.write(f"\r  Putar menuju {math.degrees(target_heading):+5.1f}° | Posisi={math.degrees(cyaw):+5.1f}° (Sisa={math.degrees(turn_err):+4.1f}°)")
                sys.stdout.flush()

            self.stop_robot()
            time.sleep(0.8)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        xf = self.odom_x
        yf = self.odom_y
        yawf = self.odom_yaw

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
            print("\n[📊 HASIL UMBMARK DRIFT BENCHMARK]:")
            print(f"  • Return-to-Origin Real Drift : {real_drift_cm:.2f} cm ({real_drift_cm / 100.0:.4f} m)")
            print(f"  • Odometry Internal Drift    : {odom_drift * 100:.2f} cm")
        except ValueError:
            pass

        if log_data:
            with open(csv_filename, 'w') as f:
                f.write("Time_s,Odom_X,Odom_Y,Odom_Yaw_deg,Phase\n")
                for row in log_data:
                    f.write(f"{row['time']:.3f},{row['x']:.4f},{row['y']:.4f},{row['yaw_deg']:.2f},{row['phase']}\n")
            print(f"[✅] Log lintasan persegi disimpan ke: {csv_filename}")


# =============================================================================
# MENU UTAMA INTERAKTIF
# =============================================================================
def main():
    rclpy.init()
    node = BenchmarkSkripsiNode()

    # Pemanasan event loop ROS 2
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        while rclpy.ok():
            print("\n" + "=" * 75)
            print("  🤖 AMR-01 BENCHMARK SYSTEM // EVALUASI PID-PSO & ODOMETRI")
            print("  Skripsi : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)")
            print("=" * 75)
            print("  --- PENGUJIAN 1: PERFORMA KECEPATAN RODA (STEP RESPONSE) ---")
            print("  [1] Step Test : 60 RPM  (~0.40 m/s)")
            print("  [2] Step Test : 80 RPM  (~0.53 m/s)")
            print("  [3] Step Test : 100 RPM (~0.66 m/s)")
            print("  [4] Multi-Step: Bertingkat (60 -> 80 -> 100 RPM)")
            print("\n  --- PENGUJIAN 2: AKURASI ODOMETRI (DEAD RECKONING BENCHMARK) ---")
            print("  [5] Uji Gerak Lurus 2.0 Meter       (v = 0.40 m/s ~ 60.2 RPM)")
            print("  [6] Uji Rotasi di Tempat 90/180/360° (w = 0.35 rad/s ~ 12.1 RPM)")
            print("  [7] Uji Persegi 2m x 2m (UMBmark)   (Lurus: 37.6 RPM | Belok: 12.1 RPM)")
            print("\n  --- KONTROL SISTEM ---")
            print("  [0] Keluar")
            print("=" * 75)

            pilihan = input("Pilih menu [0-7]: ").strip()

            if pilihan == '1':
                node.run_step_response_test(60.0, duration_sec=6.0)
            elif pilihan == '2':
                node.run_step_response_test(80.0, duration_sec=6.0)
            elif pilihan == '3':
                node.run_step_response_test(100.0, duration_sec=6.0)
            elif pilihan == '4':
                print("\n[*] Menjalankan uji bertingkat...")
                node.run_step_response_test(60.0, duration_sec=5.0)
                node.run_step_response_test(80.0, duration_sec=5.0)
                node.run_step_response_test(100.0, duration_sec=5.0)
            elif pilihan == '5':
                node.run_odometry_straight_test(target_dist=2.0)
            elif pilihan == '6':
                sub_deg = input("Pilih sudut rotasi (90 / 180 / 360) [default 360]: ").strip() or "360"
                try:
                    deg_val = float(sub_deg)
                    node.run_odometry_rotation_test(target_deg=deg_val)
                except ValueError:
                    print("[!] Sudut tidak valid.")
            elif pilihan == '7':
                node.run_odometry_square_test(side_length=2.0)
            elif pilihan == '0':
                print("[*] Menutup sistem benchmark...")
                break
            else:
                print("[!] Pilihan tidak valid, silakan coba lagi.")

    except KeyboardInterrupt:
        print("\n[*] Interupsi terdeteksi. Mematikan robot...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        print("[*] Selesai. Terima kasih!")


if __name__ == '__main__':
    main()