#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluasi_pid.py
Script Pengujian Respon Step PID Motor DC PG36 + BTS7960 AMR

Fungsi:
  - Mengirim perintah kecepatan step konstan (/cmd_vel) selama durasi tertentu (default 4 detik)
  - Otomatis mencatat telemetri Target RPM, Aktual RPM, Error, dan PWM ke file CSV
  - Otomatis menghitung metrik kendali performa:
      1. Rise Time (tr 10% - 90%)
      2. Maximum Overshoot (%OS)
      3. Settling Time (ts kriteria +-5%)
      4. Steady-State Error (ess dalam RPM dan %)
      5. RMSE (Root Mean Square Error)
      6. MAE (Mean Absolute Error)
  - Otomatis menghasilkan grafik kurva respon transien (PNG 300 DPI) untuk Skripsi

Penggunaan:
  python3 evaluasi_pid.py --vel 0.3 --duration 4.0 --label PID_PSO_1
  python3 evaluasi_pid.py --rpm 50 --duration 4.0 --label PID_ZIEGLER
"""

import os
import sys
import time
import math
import argparse
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


class PIDEvaluationNode(Node):
    def __init__(self, target_v: float, target_rpm: float, duration: float, label: str, show_plot: bool):
        super().__init__('pid_evaluation_node')

        self.wheel_diameter = 0.127       # D = 0.127 m (5 inci)
        self.wheel_separation = 0.458     # Jarak antar roda = 0.458 m

        self.target_v = target_v
        self.target_rpm = target_rpm
        self.duration = duration
        self.label = label
        self.show_plot = show_plot

        # Folder simpan CSV & Gambar
        self.output_dir = os.path.join(os.getcwd(), 'data_pid_evaluation')
        os.makedirs(self.output_dir, exist_ok=True)

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pid_l_sub = self.create_subscription(Point, '/pid/motor_left', self.pid_l_cb, 10)
        self.pid_r_sub = self.create_subscription(Point, '/pid/motor_right', self.pid_r_cb, 10)
 
        # Buffer telemetri real-time
        self.pid_l = {'target': 0.0, 'actual': 0.0, 'pwm': 0.0}
        self.pid_r = {'target': 0.0, 'actual': 0.0, 'pwm': 0.0}
        self.received_telemetry = False

    def pid_l_cb(self, msg: Point):
        self.pid_l['target'] = msg.x
        self.pid_l['actual'] = msg.y
        self.pid_l['pwm'] = msg.z
        self.received_telemetry = True

    def pid_r_cb(self, msg: Point):
        self.pid_r['target'] = msg.x
        self.pid_r['actual'] = msg.y
        self.pid_r['pwm'] = msg.z

    def send_cmd(self, v: float, w: float = 0.0):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Hentikan robot secara aktif."""
        for _ in range(10):
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_for_bridge(self):
        print("\n[*] Menghubungkan ke node arduino_bridge...")
        t_start = time.time()
        while self.cmd_pub.get_subscription_count() == 0 and (time.time() - t_start) < 4.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.cmd_pub.get_subscription_count() > 0:
            print("✅ Terhubung ke arduino_bridge.py!")
        else:
            print("⚠️ PERINGATAN: arduino_bridge.py belum terdeteksi aktif di /cmd_vel.")
            print("   Pastikan di terminal lain Anda sudah menjalankan: ros2 run amr_simulation arduino_bridge.py")

    def run_test(self):
        self.wait_for_bridge()

        print("\n" + "=" * 72)
        print("🤖 PENGUJIAN STEP RESPON KECEPATAN RODA (AMR PID BENCHMARK)")
        print(f"   • Label Pengujian    : \033[93m{self.label}\033[0m")
        print(f"   • Target Kecepatan   : \033[92m{self.target_v:.3f} m/s\033[0m  (~ \033[92m{self.target_rpm:.2f} RPM\033[0m)")
        print(f"   • Durasi Melaju      : {self.duration:.1f} detik")
        print(f"   • Folder Hasil       : {self.output_dir}")
        print("=" * 72)
        print("\033[91m⚠️ PERINGATAN KESELAMATAN:\033[0m")
        print(f"   Robot akan melaju lurus sejauh ~{self.target_v * self.duration:.1f} meter.")
        print("   Pastikan robot diletakkan di lantai dengan lintasan bebas hambatan!")
        print("-" * 72)

        try:
            input("👉 Tekan [ENTER] untuk MULAI menjalankan robot... (atau Ctrl+C untuk batal): ")
        except (KeyboardInterrupt, EOFError):
            print("\n[!] Dibatalkan pengguna.")
            return

        # 1. Fase Diam Awal (0.5 detik baseline)
        print("\n[*] ⏳ Baseline awal (0.5s diam)...")
        self.stop_robot()
        log_data = []

        start_time = time.time()
        pre_roll = 0.5
        while (time.time() - start_time) < pre_roll:
            t_now = time.time() - start_time
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
            log_data.append(self.record_sample(t_now, 0.0, 0.0))

        # 2. Fase Step Aktif (Melaju pada kecepatan target selama duration detik)
        print(f"[*] 🚀 MOTOR BERPUTAR: Target {self.target_v:.3f} m/s ({self.target_rpm:.1f} RPM)...")
        step_start_time = time.time()
        while (time.time() - step_start_time) < self.duration:
            t_now = time.time() - start_time
            self.send_cmd(self.target_v, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)

            sample = self.record_sample(t_now, self.target_rpm, self.target_v)
            log_data.append(sample)

            sys.stdout.write(
                f"\r   [t={t_now:4.2f}s] Kiri: Act={sample['act_l']:5.1f} RPM (PWM={int(sample['pwm_l']):3d}) | "
                f"Kanan: Act={sample['act_r']:5.1f} RPM (PWM={int(sample['pwm_r']):3d})"
            )
            sys.stdout.flush()

        # 3. Fase Berhenti (1.0 detik rem dan diam)
        print("\n[*] 🛑 Menghentikan robot (rem & catat settling)...")
        post_start_time = time.time()
        while (time.time() - post_start_time) < 1.0:
            t_now = time.time() - start_time
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
            log_data.append(self.record_sample(t_now, 0.0, 0.0))

        self.stop_robot()
        print("✅ Pengujian Selesai! Motor Berhenti.")

        # 4. Simpan CSV
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(self.output_dir, f"pid_eval_{self.label}_{timestamp_str}.csv")
        self.save_csv(csv_filename, log_data)

        # 5. Analisis Metrik & Cetak Tabel
        metrics_l = self.compute_control_metrics(log_data, side='L', t_step_start=pre_roll, duration=self.duration)
        metrics_r = self.compute_control_metrics(log_data, side='R', t_step_start=pre_roll, duration=self.duration)
        self.print_metrics_table(metrics_l, metrics_r)

        # 6. Generate Plot Grafik (PNG)
        png_filename = csv_filename.replace('.csv', '.png')
        self.generate_plot(log_data, metrics_l, metrics_r, png_filename, pre_roll)

    def record_sample(self, t_now: float, target_rpm: float, target_v: float):
        act_l = self.pid_l['actual']
        pwm_l = self.pid_l['pwm']
        act_r = self.pid_r['actual']
        pwm_r = self.pid_r['pwm']

        err_l = target_rpm - act_l
        err_r = target_rpm - act_r

        v_l = (act_l * math.pi * self.wheel_diameter) / 60.0
        v_r = (act_r * math.pi * self.wheel_diameter) / 60.0

        return {
            'time': t_now,
            'target_rpm': target_rpm,
            'target_v': target_v,
            'act_l': act_l,
            'pwm_l': pwm_l,
            'err_l': err_l,
            'v_l': v_l,
            'act_r': act_r,
            'pwm_r': pwm_r,
            'err_r': err_r,
            'v_r': v_r
        }

    def save_csv(self, filename: str, log_data: list):
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(
                "Time_s,Target_RPM,Actual_RPM_L,PWM_L,Error_L,Actual_V_L_ms,"
                "Actual_RPM_R,PWM_R,Error_R,Actual_V_R_ms\n"
            )
            for d in log_data:
                f.write(
                    f"{d['time']:.3f},{d['target_rpm']:.2f},{d['act_l']:.2f},{d['pwm_l']:.0f},{d['err_l']:.2f},{d['v_l']:.4f},"
                    f"{d['act_r']:.2f},{d['pwm_r']:.0f},{d['err_r']:.2f},{d['v_r']:.4f}\n"
                )
        print(f"\n[💾 CSV] Data tersimpan di : {filename}")

    def compute_control_metrics(self, log_data: list, side: str, t_step_start: float, duration: float):
        key_act = 'act_l' if side == 'L' else 'act_r'
        key_err = 'err_l' if side == 'L' else 'err_r'

        # Ambil hanya data selama fase aktif step (pre_roll s/d pre_roll + duration)
        active_samples = [d for d in log_data if t_step_start <= d['time'] <= (t_step_start + duration)]
        if not active_samples or self.target_rpm <= 0:
            return {}

        t_rel = np.array([d['time'] - t_step_start for d in active_samples])
        acts = np.array([d[key_act] for d in active_samples])
        errs = np.array([d[key_err] for d in active_samples])

        target = self.target_rpm

        # 1. Peak & Overshoot
        peak_val = float(np.max(acts))
        peak_time = float(t_rel[np.argmax(acts)])
        overshoot_pct = max(0.0, ((peak_val - target) / target) * 100.0)

        # 2. Rise Time (10% s.d. 90% dari target)
        val_10 = 0.10 * target
        val_90 = 0.90 * target
        t_10 = next((t for t, y in zip(t_rel, acts) if y >= val_10), None)
        t_90 = next((t for t, y in zip(t_rel, acts) if y >= val_90), None)
        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

        # 3. Settling Time (Kriteria +-5%)
        # Waktu terakhir sinyal berada di luar pita 5% sebelum stabil
        band_lower = 0.95 * target
        band_upper = 1.05 * target
        out_indices = np.where((acts < band_lower) | (acts > band_upper))[0]
        if len(out_indices) == 0:
            settling_time = 0.0
        else:
            last_out_idx = out_indices[-1]
            if last_out_idx < len(t_rel) - 1:
                settling_time = float(t_rel[last_out_idx + 1])
            else:
                settling_time = None  # Belum settle hingga akhir durasi

        # 4. Steady-State Value & Steady-State Error (Rata-rata 1.5 detik terakhir sebelum stop)
        steady_mask = t_rel >= (duration - 1.5)
        if np.any(steady_mask):
            steady_vals = acts[steady_mask]
            steady_val = float(np.mean(steady_vals))
        else:
            steady_val = float(acts[-1])

        ess = abs(target - steady_val)
        ess_pct = (ess / target) * 100.0

        # 5. RMSE & MAE
        rmse = float(np.sqrt(np.mean(errs ** 2)))
        mae = float(np.mean(np.abs(errs)))

        return {
            'side': 'Motor Kiri' if side == 'L' else 'Motor Kanan',
            'target': target,
            'peak_val': peak_val,
            'peak_time': peak_time,
            'overshoot': overshoot_pct,
            'rise_time': rise_time,
            'settling_time': settling_time,
            'steady_val': steady_val,
            'ess': ess,
            'ess_pct': ess_pct,
            'rmse': rmse,
            'mae': mae
        }

    def print_metrics_table(self, ml: dict, mr: dict):
        if not ml or not mr:
            return

        print("\n" + "=" * 76)
        print(f"📊 TABEL EVALUASI RESPON TRANSIEN & STEADY-STATE PID ({self.label})")
        print("=" * 76)
        print(f"{'Parameter Metrik Kendali':<32} | {'Motor Kiri (L)':<18} | {'Motor Kanan (R)':<18}")
        print("-" * 76)
        print(f"{'Target Kecepatan (Setpoint)':<32} | {ml['target']:>10.2f} RPM     | {mr['target']:>10.2f} RPM")
        print(f"{'Kecepatan Linier Target':<32} | {self.target_v:>10.3f} m/s     | {self.target_v:>10.3f} m/s")
        print(f"{'Nilai Puncak (Peak Value)':<32} | {ml['peak_val']:>10.2f} RPM     | {mr['peak_val']:>10.2f} RPM")
        
        ov_l_str = f"{ml['overshoot']:>10.2f} %"
        ov_r_str = f"{mr['overshoot']:>10.2f} %"
        print(f"{'Maximum Overshoot (%OS)':<32} | {ov_l_str:<18} | {ov_r_str:<18}")

        tr_l_str = f"{ml['rise_time']:>10.3f} s" if ml['rise_time'] else "      -     "
        tr_r_str = f"{mr['rise_time']:>10.3f} s" if mr['rise_time'] else "      -     "
        print(f"{'Rise Time (tr 10%-90%)':<32} | {tr_l_str:<18} | {tr_r_str:<18}")

        ts_l_str = f"{ml['settling_time']:>10.3f} s" if ml['settling_time'] else "Belum stabil "
        ts_r_str = f"{mr['settling_time']:>10.3f} s" if mr['settling_time'] else "Belum stabil "
        print(f"{'Settling Time (ts +-5%)':<32} | {ts_l_str:<18} | {ts_r_str:<18}")

        print(f"{'Steady-State Value':<32} | {ml['steady_val']:>10.2f} RPM     | {mr['steady_val']:>10.2f} RPM")
        print(f"{'Steady-State Error (ess)':<32} | {ml['ess']:>7.2f} RPM ({ml['ess_pct']:4.1f}%) | {mr['ess']:>7.2f} RPM ({mr['ess_pct']:4.1f}%)")
        print(f"{'RMSE (Root Mean Square Error)':<32} | {ml['rmse']:>10.2f} RPM     | {mr['rmse']:>10.2f} RPM")
        print(f"{'MAE (Mean Absolute Error)':<32} | {ml['mae']:>10.2f} RPM     | {mr['mae']:>10.2f} RPM")
        print("=" * 76)

    def generate_plot(self, log_data: list, ml: dict, mr: dict, filename: str, pre_roll: float):
        t_arr = np.array([d['time'] for d in log_data])
        target_arr = np.array([d['target_rpm'] for d in log_data])
        act_l_arr = np.array([d['act_l'] for d in log_data])
        act_r_arr = np.array([d['act_r'] for d in log_data])
        pwm_l_arr = np.array([d['pwm_l'] for d in log_data])
        pwm_r_arr = np.array([d['pwm_r'] for d in log_data])
        err_l_arr = np.array([d['err_l'] for d in log_data])
        err_r_arr = np.array([d['err_r'] for d in log_data])

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

        # --- SUBPLOT 1: RPM TRACKING & TOLERANCE BAND ---
        ax1.plot(t_arr, target_arr, 'k--', linewidth=1.8, label=f'Target ({self.target_rpm:.1f} RPM)')
        ax1.plot(t_arr, act_l_arr, 'b-', linewidth=1.7, alpha=0.9, label=f'Aktual Kiri (tr={ml.get("rise_time", 0) or 0:.2f}s, OS={ml.get("overshoot", 0):.1f}%)')
        ax1.plot(t_arr, act_r_arr, 'r-', linewidth=1.7, alpha=0.9, label=f'Aktual Kanan (tr={mr.get("rise_time", 0) or 0:.2f}s, OS={mr.get("overshoot", 0):.1f}%)')

        # Area Toleransi +-5%
        band_low = 0.95 * self.target_rpm
        band_high = 1.05 * self.target_rpm
        t_active = t_arr[(t_arr >= pre_roll) & (t_arr <= (pre_roll + self.duration))]
        if len(t_active) > 0:
            ax1.fill_between(t_active, band_low, band_high, color='green', alpha=0.12, label='Toleransi Settling +-5%')

        ax1.set_ylabel('Kecepatan (RPM)', fontsize=10, fontweight='bold')
        ax1.set_title(f'Evaluasi Respon Transien PID ({self.label}) - Step {self.target_v:.2f} m/s ({self.target_rpm:.1f} RPM)', fontsize=12, fontweight='bold', pad=10)
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='lower right', fontsize=9)

        # --- SUBPLOT 2: SINYAL KENDALI PWM ---
        ax2.plot(t_arr, pwm_l_arr, 'b-', linewidth=1.5, alpha=0.85, label='PWM Motor Kiri')
        ax2.plot(t_arr, pwm_r_arr, 'r-', linewidth=1.5, alpha=0.85, label='PWM Motor Kanan')
        ax2.set_ylabel('Sinyal PWM (0-255)', fontsize=10, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend(loc='upper right', fontsize=9)

        # --- SUBPLOT 3: TRACKING ERROR ---
        ax3.plot(t_arr, err_l_arr, 'b-', linewidth=1.5, alpha=0.85, label=f'Error Kiri (RMSE={ml.get("rmse", 0):.2f})')
        ax3.plot(t_arr, err_r_arr, 'r-', linewidth=1.5, alpha=0.85, label=f'Error Kanan (RMSE={mr.get("rmse", 0):.2f})')
        ax3.axhline(0, color='gray', linestyle='--', linewidth=1.0)
        ax3.set_xlabel('Waktu (detik)', fontsize=10, fontweight='bold')
        ax3.set_ylabel('Error e(t) RPM', fontsize=10, fontweight='bold')
        ax3.grid(True, linestyle=':', alpha=0.6)
        ax3.legend(loc='upper right', fontsize=9)

        plt.tight_layout()
        plt.savefig(filename, dpi=300)
        print(f"[📊 PLOT] Gambar grafik kurva disimpan ke : {filename}")

        if self.show_plot:
            try:
                plt.show()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Script Otomatis Evaluasi Performa Respon Transien PID AMR-01")
    parser.add_argument('--vel', type=float, default=0.30, help="Target kecepatan linear (m/s), default: 0.30 m/s")
    parser.add_argument('--rpm', type=float, default=None, help="Target RPM roda (opsional, jika diisi mengabaikan --vel)")
    parser.add_argument('--duration', type=float, default=4.0, help="Durasi melaju aktif dalam detik (default: 4.0s)")
    parser.add_argument('--label', type=str, default="PID_TEST", help="Label nama pengujian untuk file & grafik (contoh: PID_PSO_1)")
    parser.add_argument('--no-plot', action='store_true', help="Jangan buka GUI matplotlib (cocok untuk via SSH headless)")

    args = parser.parse_args()

    # Hitung konversi m/s <-> RPM
    wheel_diameter = 0.127
    if args.rpm is not None:
        target_rpm = float(args.rpm)
        target_v = (target_rpm * math.pi * wheel_diameter) / 60.0
    else:
        target_v = float(args.vel)
        target_rpm = (target_v * 60.0) / (math.pi * wheel_diameter)

    rclpy.init()
    node = PIDEvaluationNode(
        target_v=target_v,
        target_rpm=target_rpm,
        duration=args.duration,
        label=args.label,
        show_plot=not args.no_plot
    )

    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
