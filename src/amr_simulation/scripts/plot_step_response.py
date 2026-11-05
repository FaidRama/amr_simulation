#!/usr/bin/env python3
"""
plot_step_response.py
Visualisasi Step Response & Identifikasi Parameter Model Dinamis Motor DC PG36:
  G(s) = K / (tau * s + 1) = b / (s + a)

Fitur:
  1. Mendukung data mentah (RAW data tanpa filter EMA) serta file lama.
  2. Otomatis mendeteksi Baseline PWM dan Step PWM (misal 40->100, 120, 140, 160, 180).
  3. Menghitung Delta U, Delta Y, Gain K, dan Time Constant tau (Least Squares + 63.2%).
  4. Delimiter otomatis (koma ',' maupun titik koma ';').
  5. Mode perbandingan multi-file (--compare) untuk visualisasi perbandingan step 100 s/d 180.
"""

import os
import sys
import glob
import csv
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt


def extract_timestamp(filepath):
    """Ambil timestamp YYYYMMDD_HHMMSS dari nama file."""
    match = re.search(r'(\d{8}_\d{6})', os.path.basename(filepath))
    if match:
        return match.group(1)
    return ""


def get_latest_csv(folder):
    files = glob.glob(os.path.join(folder, "step_response_*.csv"))
    if not files:
        return None
    # Urutkan berdasarkan timestamp YYYYMMDD_HHMMSS dari nama file, fallback ke mtime
    return max(files, key=lambda f: (extract_timestamp(f), os.path.getmtime(f)))


def load_step_csv(csv_path):
    """Membaca file CSV step response dengan deteksi delimiter otomatis."""
    if not os.path.exists(csv_path):
        return None

    # Deteksi delimiter (koma atau titik koma)
    with open(csv_path, 'r', encoding='utf-8', errors='ignore') as f:
        sample = f.read(2048)
        delimiter = ';' if ';' in sample and sample.count(';') > sample.count(',') else ','

    t_list, pwm_list, rpm_l_list, rpm_r_list = [], [], [], []

    with open(csv_path, 'r', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f, delimiter=delimiter)
        for row in reader:
            if not row or len(row) < 4:
                continue
            # Lewati baris header atau komentar
            if row[0].startswith('#') or 't' in row[0].lower():
                continue
            try:
                t = float(row[0].strip())
                p = int(float(row[1].strip()))
                rl = float(row[2].strip())
                rr = float(row[3].strip())
                t_list.append(t)
                pwm_list.append(p)
                rpm_l_list.append(rl)
                rpm_r_list.append(rr)
            except ValueError:
                continue

    if not t_list:
        return None

    return {
        'filepath': csv_path,
        'filename': os.path.basename(csv_path),
        't': np.array(t_list),
        'pwm': np.array(pwm_list),
        'rpm_l': np.array(rpm_l_list),
        'rpm_r': np.array(rpm_r_list)
    }


def calculate_first_order_params(t_arr, pwm_arr, rpm_arr, t_step=3.0):
    """
    Menghitung parameter orde 1:
      K = Delta_y / Delta_u
      tau = waktu konstan via least-squares curve fit optimal & 63.2%
    """
    # 1. Baseline pada t = 2.0 s/d 2.95s
    idx_base = np.where((t_arr >= 2.0) & (t_arr < 2.95))[0]
    y0 = float(np.mean(rpm_arr[idx_base])) if len(idx_base) > 0 else 0.0
    u0 = float(np.mean(pwm_arr[idx_base])) if len(idx_base) > 0 else 40.0

    # 2. Steady-state pada t = 5.0 s/d 5.95s
    idx_ss = np.where((t_arr >= 5.0) & (t_arr < 5.95))[0]
    yss = float(np.mean(rpm_arr[idx_ss])) if len(idx_ss) > 0 else 0.0
    uss = float(np.mean(pwm_arr[idx_ss])) if len(idx_ss) > 0 else 140.0

    delta_u = max(1.0, uss - u0)
    delta_y = yss - y0

    if delta_y <= 0:
        return {
            'y0': y0, 'yss': yss, 'u0': u0, 'uss': uss,
            'delta_u': delta_u, 'delta_y': delta_y,
            'K': 0.0, 'tau': 0.1, 'tau_63': 0.1, 'a': 10.0, 'b': 0.0
        }

    # Gain Statis K = delta_y / delta_u
    K = delta_y / delta_u

    # 3. Optimasi Least Squares untuk mencari tau terbaik (0.02s s/d 2.00s)
    idx_step = np.where((t_arr >= t_step) & (t_arr <= 5.95))[0]
    if len(idx_step) > 5:
        t_fit = t_arr[idx_step] - t_step
        y_meas = rpm_arr[idx_step]
        tau_candidates = np.linspace(0.02, 2.0, 500)
        # Evaluasi kuadrat galat: y_model = y0 + delta_y * (1 - exp(-t / tau))
        models = y0 + delta_y * (1.0 - np.exp(-t_fit[:, None] / tau_candidates[None, :]))
        sq_errors = np.sum((models - y_meas[:, None]) ** 2, axis=0)
        best_tau = float(tau_candidates[np.argmin(sq_errors)])
    else:
        best_tau = 0.1

    # Waktu konstan metode klasik 63.2%
    y_target = y0 + (0.632 * delta_y)
    tau_63 = best_tau
    for idx in idx_step:
        if rpm_arr[idx] >= y_target:
            tau_63 = max(0.02, float(t_arr[idx] - t_step))
            break

    a = 1.0 / best_tau
    b = K / best_tau

    return {
        'y0': y0,
        'yss': yss,
        'u0': u0,
        'uss': uss,
        'delta_u': delta_u,
        'delta_y': delta_y,
        'K': K,
        'tau': best_tau,
        'tau_63': tau_63,
        'a': a,
        'b': b
    }


def plot_single(data, show_gui=False):
    """Membuat grafik analisis lengkap untuk satu file step response."""
    t_arr = data['t']
    pwm_arr = data['pwm']
    rpm_l_arr = data['rpm_l']
    rpm_r_arr = data['rpm_r']
    csv_path = data['filepath']

    res_l = calculate_first_order_params(t_arr, pwm_arr, rpm_l_arr)
    res_r = calculate_first_order_params(t_arr, pwm_arr, rpm_r_arr)

    u0 = int(round(res_l['u0']))
    uss = int(round(res_l['uss']))

    print("\n" + "=" * 68)
    print(f"🎯 HASIL IDENTIFIKASI SISTEM MOTOR PG36 (Base={u0} -> Step={uss} PWM):")
    print("=" * 68)
    print(f"📌 MOTOR KIRI:")
    print(f"   - Baseline (PWM {u0:3d}) : {res_l['y0']:.2f} RPM")
    print(f"   - Steady   (PWM {uss:3d}) : {res_l['yss']:.2f} RPM (Delta = +{res_l['delta_y']:.2f} RPM)")
    print(f"   - Gain Statis (K)     : {res_l['K']:.4f} RPM/PWM")
    print(f"   - Time Constant (tau) : {res_l['tau']:.4f} detik (Metode Least Squares)")
    print(f"   - Time Constant (63%) : {res_l['tau_63']:.4f} detik (Metode 63.2% Rise Time)")
    print(f"   - FUNGSI ALIH G_L(s)  : {res_l['K']:.4f} / ({res_l['tau']:.4f}*s + 1)")
    print(f"                           = {res_l['b']:.4f} / (s + {res_l['a']:.4f})")
    print("-" * 68)
    print(f"📌 MOTOR KANAN:")
    print(f"   - Baseline (PWM {u0:3d}) : {res_r['y0']:.2f} RPM")
    print(f"   - Steady   (PWM {uss:3d}) : {res_r['yss']:.2f} RPM (Delta = +{res_r['delta_y']:.2f} RPM)")
    print(f"   - Gain Statis (K)     : {res_r['K']:.4f} RPM/PWM")
    print(f"   - Time Constant (tau) : {res_r['tau']:.4f} detik (Metode Least Squares)")
    print(f"   - Time Constant (63%) : {res_r['tau_63']:.4f} detik (Metode 63.2% Rise Time)")
    print(f"   - FUNGSI ALIH G_R(s)  : {res_r['K']:.4f} / ({res_r['tau']:.4f}*s + 1)")
    print(f"                           = {res_r['b']:.4f} / (s + {res_r['a']:.4f})")
    print("=" * 68 + "\n")

    # Plot Gambar
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Subplot 1: Input PWM
    ax1.plot(t_arr, pwm_arr, 'k-', linewidth=2, label=f'Input PWM (Base {u0} -> Step {uss})')
    ax1.set_ylabel('Input PWM', fontsize=11, fontweight='bold')
    ax1.set_title(f'Step Response Open-Loop Motor PG36 (Base PWM {u0} -> Step {uss}) [Raw Encoder]',
                  fontsize=12, fontweight='bold', pad=10)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    # Subplot 2: Respon Kecepatan Aktual (Raw) & Fit Model
    ax2.plot(t_arr, rpm_l_arr, 'b-', alpha=0.75, linewidth=1.5,
             label=f'RPM Mentah Kiri (K={res_l["K"]:.3f}, tau={res_l["tau"]:.3f}s)')
    ax2.plot(t_arr, rpm_r_arr, 'r-', alpha=0.75, linewidth=1.5,
             label=f'RPM Mentah Kanan (K={res_r["K"]:.3f}, tau={res_r["tau"]:.3f}s)')

    # Model Prediksi Teoretis setelah Step (t >= 3.0)
    idx_step = np.where((t_arr >= 3.0) & (t_arr <= 6.0))[0]
    if len(idx_step) > 0:
        t_model = t_arr[idx_step] - 3.0
        model_l = res_l['y0'] + res_l['delta_y'] * (1.0 - np.exp(-t_model / res_l['tau']))
        model_r = res_r['y0'] + res_r['delta_y'] * (1.0 - np.exp(-t_model / res_r['tau']))
        ax2.plot(t_arr[idx_step], model_l, 'b--', linewidth=2.0, label='Fit Model G_L(s)')
        ax2.plot(t_arr[idx_step], model_r, 'r--', linewidth=2.0, label='Fit Model G_R(s)')

    ax2.axvline(x=3.0, color='gray', linestyle=':', label='Saat Step Terjadi (t = 3.0s)')
    ax2.set_xlabel('Waktu (detik)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Kecepatan (RPM)', fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=9)

    plt.tight_layout()

    output_png = csv_path.rsplit('.', 1)[0] + '.png'
    plt.savefig(output_png, dpi=300)
    print(f"✅ Gambar grafik kurva dan identifikasi model tersimpan:")
    print(f"   {output_png}\n")

    if show_gui:
        try:
            plt.show()
        except Exception:
            pass


def plot_comparison(datasets, output_png='data_step_response/step_response_comparison.png', show_gui=False):
    """Membandingkan beberapa pengujian step (100, 120, 140, 160, 180) dalam satu gambar."""
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 11), sharex=True)

    print("\n" + "=" * 78)
    print("📊 TABEL PERBANDINGAN IDENTIFIKASI MULTI-STEP RESPONSE:")
    print("=" * 78)
    print(f"{'Skenario':<20} | {'Motor Kiri (K | tau)':<25} | {'Motor Kanan (K | tau)':<25}")
    print("-" * 78)

    for i, data in enumerate(datasets):
        c = colors[i % len(colors)]
        t = data['t']
        pwm = data['pwm']
        rl = data['rpm_l']
        rr = data['rpm_r']

        res_l = calculate_first_order_params(t, pwm, rl)
        res_r = calculate_first_order_params(t, pwm, rr)

        u0 = int(round(res_l['u0']))
        uss = int(round(res_l['uss']))
        label_step = f"Step {u0}->{uss} PWM"

        print(f"{label_step:<20} | K={res_l['K']:6.4f}, tau={res_l['tau']:5.3f}s | K={res_r['K']:6.4f}, tau={res_r['tau']:5.3f}s")

        # Subplot 1: Input PWM
        ax1.plot(t, pwm, color=c, linewidth=1.5, label=f'{label_step}')

        # Subplot 2: Respon Motor Kiri
        ax2.plot(t, rl, color=c, alpha=0.8, linewidth=1.4,
                 label=f'{label_step} (K={res_l["K"]:.2f}, tau={res_l["tau"]:.2f}s)')

        # Subplot 3: Respon Motor Kanan
        ax3.plot(t, rr, color=c, alpha=0.8, linewidth=1.4,
                 label=f'{label_step} (K={res_r["K"]:.2f}, tau={res_r["tau"]:.2f}s)')

    print("=" * 78 + "\n")

    ax1.set_ylabel('Input PWM', fontweight='bold')
    ax1.set_title('Perbandingan Input Step PWM', fontsize=11, fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=8)

    ax2.set_ylabel('RPM Motor Kiri', fontweight='bold')
    ax2.set_title('Respon Kecepatan Motor Kiri (Raw Encoder)', fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', fontsize=8)

    ax3.set_xlabel('Waktu (detik)', fontweight='bold')
    ax3.set_ylabel('RPM Motor Kanan', fontweight='bold')
    ax3.set_title('Respon Kecepatan Motor Kanan (Raw Encoder)', fontsize=11, fontweight='bold')
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='lower right', fontsize=8)

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output_png)), exist_ok=True)
    plt.savefig(output_png, dpi=300)
    print(f"✅ Gambar grafik perbandingan multi-step tersimpan di:\n   {output_png}\n")

    if show_gui:
        try:
            plt.show()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Plot & Analisis Step Response Motor AMR")
    parser.add_argument('files', nargs='*', help="File CSV yang akan diplot")
    parser.add_argument('--compare', action='store_true', help="Bandingkan semua file yang diberikan dalam satu grafik")
    parser.add_argument('--show', action='store_true', help="Tampilkan jendela interaktif GUI")
    parser.add_argument('--output-img', type=str, default=None, help="Lokasi file output gambar perbandingan")
    args = parser.parse_args()

    files = args.files

    if not files:
        candidate_dirs = [
            os.path.join(os.getcwd(), 'data_step_response'),
            os.path.expanduser('~/deliver_amr/data_step_response'),
            os.path.expanduser('~/Documents/amr_ws/data_step_response')
        ]
        target_dir = None
        for folder in candidate_dirs:
            if os.path.exists(folder) and glob.glob(os.path.join(folder, "step_response_*.csv")):
                target_dir = folder
                break

        if target_dir:
            all_csvs = sorted(glob.glob(os.path.join(target_dir, "step_response_*.csv")),
                              key=lambda f: (extract_timestamp(f), os.path.getmtime(f)))
            if args.compare:
                # Mode compare: otomatis ambil semua pengujian base terbaru
                base_csvs = [f for f in all_csvs if 'step_response_base' in os.path.basename(f)]
                files = base_csvs if len(base_csvs) >= 2 else all_csvs
            else:
                latest = get_latest_csv(target_dir)
                if latest:
                    files = [latest]

    if not files:
        print("❌ Tidak ada file CSV Step Response yang ditemukan!")
        print("Jalankan pengujian terlebih dahulu dengan: python3 logger_step_respon.py")
        sys.exit(1)

    # Muat dataset
    datasets = []
    for f in files:
        if os.path.isdir(f):
            sub_files = sorted(glob.glob(os.path.join(f, "step_response_*.csv")),
                               key=lambda x: (extract_timestamp(x), os.path.getmtime(x)))
            for subf in sub_files:
                d = load_step_csv(subf)
                if d:
                    datasets.append(d)
        else:
            d = load_step_csv(f)
            if d:
                datasets.append(d)

    if not datasets:
        print("❌ Tidak ada data valid yang bisa dibaca dari file yang diberikan.")
        sys.exit(1)

    loaded_names = [d['filename'] for d in datasets]
    print(f"\n📂 File yang dimuat ({len(datasets)} file):")
    for name in loaded_names:
        print(f"   - {name}")

    if args.compare or len(datasets) > 1:
        out_img = args.output_img or os.path.join('data_step_response', 'step_response_comparison.png')
        plot_comparison(datasets, out_img, show_gui=args.show)
    else:
        plot_single(datasets[0], show_gui=args.show)


if __name__ == '__main__':
    main()
