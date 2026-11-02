#!/usr/bin/env python3
"""
plot_step_response.py
Visualisasi Step Response & Identifikasi Parameter Model Dinamis Motor DC PG36:
  G(s) = K / (tau * s + 1) = b / (s + a)

Menghitung otomatis:
  1. Kecepatan Baseline y0 pada PWM 40
  2. Kecepatan Steady-State y_ss pada PWM 140 (Delta PWM = 100)
  3. Gain Statis K = Delta_y / Delta_u
  4. Time Constant tau (waktu capai 63.2% kenaikan)
  5. Fungsi Alih Kontinu G_kiri(s) dan G_kanan(s)
"""

import os
import sys
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt


def get_latest_csv(folder):
    files = glob.glob(os.path.join(folder, "step_response_*.csv"))
    if not files:
        return None
    return max(files, key=os.path.getctime)


def calculate_first_order_params(t_arr, rpm_arr, t_step=3.0, delta_u=100.0):
    # 1. Baseline y0 (rata-rata pada t = 2.0 s/d 2.95s)
    idx_base = np.where((t_arr >= 2.0) & (t_arr < 2.95))[0]
    y0 = np.mean(rpm_arr[idx_base]) if len(idx_base) > 0 else 0.0

    # 2. Steady-state y_ss (rata-rata pada t = 5.0 s/d 5.95s)
    idx_ss = np.where((t_arr >= 5.0) & (t_arr < 5.95))[0]
    yss = np.mean(rpm_arr[idx_ss]) if len(idx_ss) > 0 else 0.0

    delta_y = yss - y0
    if delta_y <= 0:
        return y0, yss, 0.0, 0.1, 0.0, 0.0

    # Gain K = delta_y / delta_u
    K = delta_y / delta_u

    # 3. Waktu konstan tau: saat y(t) mencapai 63.2% dari kenaikan
    y_target = y0 + (0.632 * delta_y)

    idx_after_step = np.where(t_arr >= t_step)[0]
    tau = 0.1 # default fallback
    t_target = t_step
    for idx in idx_after_step:
        if rpm_arr[idx] >= y_target:
            t_target = t_arr[idx]
            tau = max(0.02, t_target - t_step)
            break

    a = 1.0 / tau
    b = K / tau
    return y0, yss, K, tau, a, b


def main():
    candidate_dirs = [
        os.path.join(os.getcwd(), 'data_step_response'),
        os.path.expanduser('~/deliver_amr/data_step_response'),
        os.path.expanduser('~/Documents/amr_ws/data_step_response')
    ]

    csv_path = None
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        for folder in candidate_dirs:
            if os.path.exists(folder):
                csv_path = get_latest_csv(folder)
                if csv_path:
                    break

    if not csv_path or not os.path.exists(csv_path):
        print("❌ File CSV Step Response tidak ditemukan di folder data_step_response!")
        print("Jalankan pengujian terlebih dahulu: ros2 run amr_simulation step_response_monitor_node.py")
        return

    print(f"📊 Membaca data Step Response dari: {csv_path}")

    t_list, pwm_list, rpm_l_list, rpm_r_list = [], [], [], []

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 4:
                continue
            try:
                t = float(row[0])
                p = int(row[1])
                rl = float(row[2])
                rr = float(row[3])
                t_list.append(t)
                pwm_list.append(p)
                rpm_l_list.append(rl)
                rpm_r_list.append(rr)
            except ValueError:
                continue

    if not t_list:
        print("⚠️ File CSV kosong atau rusak.")
        return

    t_arr = np.array(t_list)
    pwm_arr = np.array(pwm_list)
    rpm_l_arr = np.array(rpm_l_list)
    rpm_r_arr = np.array(rpm_r_list)

    # Identifikasi Parameter Motor Kiri & Kanan
    y0_l, yss_l, K_l, tau_l, a_l, b_l = calculate_first_order_params(t_arr, rpm_l_arr)
    y0_r, yss_r, K_r, tau_r, a_r, b_r = calculate_first_order_params(t_arr, rpm_r_arr)

    print("\n" + "=" * 65)
    print("🎯 HASIL IDENTIFIKASI SISTEM DINAMIS MOTOR PG36:")
    print("=" * 65)
    print(f"📌 MOTOR KIRI:")
    print(f"   - Baseline (PWM 40)   : {y0_l:.2f} RPM")
    print(f"   - Steady (PWM 140)    : {yss_l:.2f} RPM (Delta = {yss_l - y0_l:.2f} RPM)")
    print(f"   - Gain Statis (K)     : {K_l:.4f} RPM/PWM")
    print(f"   - Time Constant (tau) : {tau_l:.4f} detik")
    print(f"   - FUNGSI ALIH G_L(s)  : {K_l:.4f} / ({tau_l:.4f}*s + 1)")
    print(f"                           = {b_l:.4f} / (s + {a_l:.4f})")
    print("-" * 65)
    print(f"📌 MOTOR KANAN:")
    print(f"   - Baseline (PWM 40)   : {y0_r:.2f} RPM")
    print(f"   - Steady (PWM 140)    : {yss_r:.2f} RPM (Delta = {yss_r - y0_r:.2f} RPM)")
    print(f"   - Gain Statis (K)     : {K_r:.4f} RPM/PWM")
    print(f"   - Time Constant (tau) : {tau_r:.4f} detik")
    print(f"   - FUNGSI ALIH G_R(s)  : {K_r:.4f} / ({tau_r:.4f}*s + 1)")
    print(f"                           = {b_r:.4f} / (s + {a_r:.4f})")
    print("=" * 65 + "\n")

    # Plot Gambar
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Subplot 1: Input PWM
    ax1.plot(t_arr, pwm_arr, 'k-', linewidth=2, label='Input Sinyal PWM')
    ax1.set_ylabel('Input PWM', fontsize=11, fontweight='bold')
    ax1.set_title('Step Response Open-Loop Motor PG36 (Baseline PWM 40 -> Step 140)', fontsize=12, fontweight='bold', pad=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    # Subplot 2: Respon Kecepatan Aktual & Fit Model
    ax2.plot(t_arr, rpm_l_arr, 'b-', alpha=0.85, linewidth=1.8, label=f'RPM Aktual Kiri (K={K_l:.3f}, tau={tau_l:.3f}s)')
    ax2.plot(t_arr, rpm_r_arr, 'r-', alpha=0.85, linewidth=1.8, label=f'RPM Aktual Kanan (K={K_r:.3f}, tau={tau_r:.3f}s)')

    # Model Prediksi Teoretis setelah Step (t >= 3.0)
    idx_step = np.where(t_arr >= 3.0)[0]
    if len(idx_step) > 0:
        t_model = t_arr[idx_step] - 3.0
        model_l = y0_l + (yss_l - y0_l) * (1.0 - np.exp(-t_model / tau_l))
        model_r = y0_r + (yss_r - y0_r) * (1.0 - np.exp(-t_model / tau_r))
        ax2.plot(t_arr[idx_step], model_l, 'b--', linewidth=1.5, alpha=0.7, label='Fit Model G_L(s)')
        ax2.plot(t_arr[idx_step], model_r, 'r--', linewidth=1.5, alpha=0.7, label='Fit Model G_R(s)')

    ax2.axvline(x=3.0, color='gray', linestyle=':', label='Saat Step Terjadi (t = 3.0s)')
    ax2.set_xlabel('Waktu (detik)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Kecepatan (RPM)', fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=9)

    plt.tight_layout()

    # Simpan Gambar PNG Resolusi Tinggi (300 DPI)
    output_png = csv_path.replace('.csv', '.png')
    plt.savefig(output_png, dpi=300)
    print(f"✅ Gambar grafik kurva dan identifikasi model berhasil disimpan:")
    print(f"   {output_png}\n")

    try:
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
