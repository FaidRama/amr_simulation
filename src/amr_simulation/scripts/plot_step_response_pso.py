#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SCRIPT VISUALISASI & ANALISIS PERFORMA STEP RESPON (PI - OPTIMASI PSO)
SKRIPSI: DIVALA ZAHRA OKTAVIA (TEKNIK KOMPUTER UNNES 2026)
=============================================================================
Fitur:
  1. Otomatis membaca file CSV dari serial Arduino pengujian PI-PSO.
  2. Mendeteksi parameter Kp dan Ki PSO dari header file CSV.
  3. Menghitung 5 Parameter Performa Evaluasi Skripsi:
     - RMSE (Root Mean Square Error)
     - Overshoot (%)
     - Rise Time (Tr, 10% s.d 90%)
     - Settling Time (Ts, kriteria +-5%)
     - Steady-State Error (Ess)
  4. Menghasilkan gambar grafik publikasi (.png 300 DPI) lengkap dengan 
     timestamp waktu/jam pengujian agar tidak salah file.
=============================================================================
"""

import os
import sys
import glob
import csv
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt

def cari_file_csv_pso_terbaru():
    """Mencari file CSV hasil pengujian PSO terbaru jika argumen kosong."""
    pola_list = [
        "step_respon*pso*.csv",
        "hasil_pso*.csv",
        "data_step_response/*pso*.csv"
    ]
    kandidat = []
    for p in pola_list:
        kandidat.extend(glob.glob(p))
        kandidat.extend(glob.glob(os.path.join("..", p)))
        kandidat.extend(glob.glob(os.path.join(os.path.expanduser("~"), "deliver_amr", p)))
    
    if not kandidat:
        return None
    return max(kandidat, key=os.path.getmtime)

def baca_data_csv(path_file):
    """Membaca data streaming CSV pengujian Arduino beserta info gain di header."""
    if not os.path.exists(path_file):
        raise FileNotFoundError(f"File tidak ditemukan: {path_file}")
    
    with open(path_file, 'r', encoding='utf-8', errors='ignore') as f:
        sample = f.read(2048)
        delimiter = ';' if sample.count(';') > sample.count(',') else ','
    
    time_s, sp_l, actual_l, pwm_l, err_l = [], [], [], [], []
    sp_r, actual_r, pwm_r, err_r = [], [], [], []
    
    gain_info_l = "Kp=2.5238, Ki=23.5015"
    gain_info_r = "Kp=2.3640, Ki=24.2184"
    
    with open(path_file, 'r', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f, delimiter=delimiter)
        for row in reader:
            if not row:
                continue
            line_str = ";".join(row)
            if 'MOTOR_LEFT' in line_str:
                m_kp = re.search(r'Kp=([0-9\.]+)', line_str)
                m_ki = re.search(r'Ki=([0-9\.]+)', line_str)
                if m_kp and m_ki:
                    gain_info_l = f"Kp={float(m_kp.group(1)):.4f}, Ki={float(m_ki.group(1)):.4f}"
            if 'MOTOR_RIGHT' in line_str:
                m_kp = re.search(r'Kp=([0-9\.]+)', line_str)
                m_ki = re.search(r'Ki=([0-9\.]+)', line_str)
                if m_kp and m_ki:
                    gain_info_r = f"Kp={float(m_kp.group(1)):.4f}, Ki={float(m_ki.group(1)):.4f}"
                    
            if row[0].strip().startswith('#'):
                continue
            if 'time' in row[0].lower() or 'target' in row[1].lower():
                continue
            try:
                t_val = float(row[0])
                time_s.append(t_val)
                sp_l.append(float(row[1]))
                actual_l.append(float(row[2]))
                pwm_l.append(float(row[3]))
                err_l.append(float(row[4]))
                
                sp_r.append(float(row[5]))
                actual_r.append(float(row[6]))
                pwm_r.append(float(row[7]))
                err_r.append(float(row[8]))
            except (ValueError, IndexError):
                continue
                
    return {
        't': np.array(time_s),
        'sp_l': np.array(sp_l),
        'act_l': np.array(actual_l),
        'pwm_l': np.array(pwm_l),
        'err_l': np.array(err_l),
        'sp_r': np.array(sp_r),
        'act_r': np.array(actual_r),
        'pwm_r': np.array(pwm_r),
        'err_r': np.array(err_r),
        'gain_l': gain_info_l,
        'gain_r': gain_info_r
    }

def deteksi_tahap_step(t, sp):
    """Mendeteksi segmen step aktif (target > 5 RPM)."""
    tahap_list = []
    in_step = False
    start_idx = 0
    
    for i in range(len(sp)):
        if sp[i] > 5.0 and not in_step:
            in_step = True
            start_idx = i
        elif (sp[i] <= 5.0 or i == len(sp) - 1 or (in_step and abs(sp[i] - sp[start_idx]) > 5.0)) and in_step:
            end_idx = i - 1 if sp[i] <= 5.0 else i
            sp_val = sp[start_idx]
            tahap_list.append((t[start_idx], t[end_idx], sp_val))
            if sp[i] > 5.0 and abs(sp[i] - sp[start_idx]) > 5.0:
                start_idx = i
            else:
                in_step = False
                
    if not tahap_list and len(sp) > 0:
        sp_max = np.max(sp)
        if sp_max > 5.0:
            idx = np.where(sp >= 0.9 * sp_max)[0]
            if len(idx) > 0:
                tahap_list.append((t[idx[0]], t[idx[-1]], sp_max))
                
    return tahap_list

def hitung_parameter_performa(t_seg, act_seg, pwm_seg, target):
    """Menghitung 5 parameter performa transien & steady state."""
    t_rel = t_seg - t_seg[0]
    
    # 1. Overshoot & Peak Time
    ymax = float(np.max(act_seg))
    idx_peak = int(np.argmax(act_seg))
    tp = float(t_rel[idx_peak])
    overshoot = max(0.0, ((ymax - target) / target) * 100.0)
    
    # 2. Rise Time (10% s.d 90%)
    tr = 0.0
    idx_10 = np.where(act_seg >= 0.10 * target)[0]
    idx_90 = np.where(act_seg >= 0.90 * target)[0]
    if len(idx_10) > 0 and len(idx_90) > 0:
        t_10 = float(t_rel[idx_10[0]])
        t_90 = float(t_rel[idx_90[0]])
        tr = max(0.0, t_90 - t_10)
        
    # 3. Settling Time (Toleransi +- 5%)
    band = 0.05 * target
    ts = float(t_rel[-1])
    for i, y in enumerate(act_seg):
        if abs(y - target) <= band:
            sub = act_seg[i: min(i + 25, len(act_seg))]
            if np.mean(np.abs(sub - target) <= band) >= 0.85:
                ts = float(t_rel[i])
                break
                
    # 4 & 5. Steady State (data setelah 1.0 detik segmen aktif)
    idx_ss = np.where(t_rel >= min(1.0, 0.5 * t_rel[-1]))[0]
    if len(idx_ss) == 0:
        idx_ss = np.arange(int(0.7 * len(act_seg)), len(act_seg))
        
    act_ss = act_seg[idx_ss]
    pwm_ss = pwm_seg[idx_ss]
    
    rpm_steady = float(np.mean(act_ss))
    ess = abs(target - rpm_steady)
    ess_percent = (ess / target) * 100.0
    rmse = float(np.sqrt(np.mean((act_ss - target) ** 2)))
    pwm_steady = float(np.mean(pwm_ss))
    
    return {
        'target': target,
        'ymax': ymax,
        'tp': tp,
        'overshoot': overshoot,
        'tr': tr,
        'ts': ts,
        'rpm_steady': rpm_steady,
        'ess': ess,
        'ess_percent': ess_percent,
        'rmse': rmse,
        'pwm_steady': pwm_steady
    }

def main():
    parser = argparse.ArgumentParser(description="Plot & Evaluasi Step Response PI-PSO")
    parser.add_argument("csv_path", nargs="?", default=None, help="Path ke file CSV hasil tes PSO")
    parser.add_argument("--save", "-s", default=None, help="Nama file PNG output grafik")
    args = parser.parse_args()
    
    path_file = args.csv_path
    if path_file is None:
        path_file = cari_file_csv_pso_terbaru()
        
    if path_file is None:
        print("[!] File CSV pengujian PI-PSO tidak ditemukan.")
        print("[*] Contoh pemakaian: python3 plot_step_response_pso.py step_respon_60rpm_pso_20261109_182000.csv")
        sys.exit(1)
        
    print(f"[*] Memproses data PSO dari file: {path_file}")
    data = baca_data_csv(path_file)
    t = data['t']
    
    tahap_list = deteksi_tahap_step(t, data['sp_l'])
    
    # Deteksi timestamp jam dari nama file (jika ada)
    jam_match = re.search(r'(\d{8}_\d{6})', os.path.basename(path_file))
    waktu_label = ""
    if jam_match:
        ts_str = jam_match.group(1)
        waktu_label = f" | Waktu Uji: {ts_str[:4]}-{ts_str[4:6]}-{ts_str[6:8]} {ts_str[9:11]}:{ts_str[11:13]}:{ts_str[13:15]}"
    
    # Cetak Rangkuman Tabel di Konsol
    print("\n" + "=" * 82)
    print("        HASIL ANALISIS EVALUASI 5 PARAMETER PERFORMA (PI - PSO)")
    print(f"        Gain Kiri : {data['gain_l']}")
    print(f"        Gain Kanan: {data['gain_r']}")
    if waktu_label:
        print(f"       {waktu_label}")
    print("=" * 82)
    
    hasil_tahap = []
    for i, (t_start, t_end, sp_val) in enumerate(tahap_list):
        idx = np.where((t >= t_start) & (t <= t_end))[0]
        if len(idx) < 5:
            continue
            
        perf_l = hitung_parameter_performa(t[idx], data['act_l'][idx], data['pwm_l'][idx], sp_val)
        perf_r = hitung_parameter_performa(t[idx], data['act_r'][idx], data['pwm_r'][idx], sp_val)
        hasil_tahap.append((sp_val, perf_l, perf_r, t_start, t_end))
        
        print(f"\n>>> TAHAP SETPOINT: {sp_val:.1f} RPM (Durasi: {t_start:.1f}s s.d {t_end:.1f}s) <<<")
        print("-" * 82)
        print(f"{'Parameter Performa Evaluasi':<34} | {'Motor Kiri':<14} | {'Motor Kanan':<14} | {'Satuan'}")
        print("-" * 82)
        print(f"{'1. RMSE (Akurasi Steady-State)':<34} | {perf_l['rmse']:<14.3f} | {perf_r['rmse']:<14.3f} | RPM")
        print(f"{'2. Overshoot (Lonjakan Puncak)':<34} | {perf_l['overshoot']:<13.2f}% | {perf_r['overshoot']:<13.2f}% | %")
        print(f"{'3. Rise Time (Waktu Naik 10-90%)':<34} | {perf_l['tr']:<14.3f} | {perf_r['tr']:<14.3f} | detik")
        print(f"{'4. Settling Time (Waktu Mantap +-5%)':<34} | {perf_l['ts']:<14.3f} | {perf_r['ts']:<14.3f} | detik")
        print(f"{'5. Steady-State Error (Ess)':<34} | {perf_l['ess']:<14.3f} | {perf_r['ess']:<14.3f} | RPM")
        print(f"{'   Persentase Galat Ess':<34} | {perf_l['ess_percent']:<13.2f}% | {perf_r['ess_percent']:<13.2f}% | %")
        print(f"{'   Kecepatan Rata-rata Stabil':<34} | {perf_l['rpm_steady']:<14.2f} | {perf_r['rpm_steady']:<14.2f} | RPM")
        print(f"{'   Nilai PWM Rata-rata Stabil':<34} | {perf_l['pwm_steady']:<14.1f} | {perf_r['pwm_steady']:<14.1f} | PWM (0-255)")
        print("-" * 82)
        
    print("=" * 82)
    
    # -------------------------------------------------------------------------
    # PEMBUATAN GRAFIK MATPLOTLIB (PUBLICATION QUALITY)
    # -------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8.5), dpi=150, sharex=True)
    
    # SUBPLOT 1: KECEPATAN RPM & TARGET
    ax1.plot(t, data['sp_l'], 'k--', linewidth=2.0, label='Setpoint Target (RPM)')
    ax1.plot(t, data['act_l'], color='#1f77b4', linewidth=1.8, label='Roda Kiri (Actual RPM)')
    ax1.plot(t, data['act_r'], color='#2ca02c', linewidth=1.8, label='Roda Kanan (Actual RPM)')
    
    # Gambar area pita toleransi +-5%
    for sp_val, pl, pr, t_s, t_e in hasil_tahap:
        band_atas = sp_val * 1.05
        band_bawah = sp_val * 0.95
        ax1.hlines([band_atas, band_bawah], t_s, t_e, colors='gray', linestyles=':', alpha=0.6)
        ax1.fill_between([t_s, t_e], [band_bawah, band_bawah], [band_atas, band_atas], 
                         color='gray', alpha=0.12, label='Toleransi +-5%' if sp_val == hasil_tahap[0][0] else "")
        
    ax1.set_title("Grafik Step Response Kecepatan Roda AMR (Kendali PI - Optimasi PSO)\n"
                  f"Gain: Kiri ({data['gain_l']}) | Kanan ({data['gain_r']}){waktu_label}", 
                  fontsize=12, fontweight='bold', pad=12)
    ax1.set_ylabel("Kecepatan Roda (RPM)", fontsize=11, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='lower right', framealpha=0.92, fontsize=10)
    
    max_rpm = max(np.max(data['act_l']), np.max(data['act_r']), np.max(data['sp_l']))
    ax1.set_ylim(-5, max_rpm + 15)
    
    # SUBPLOT 2: SINYAL KENDALI PWM
    ax2.plot(t, data['pwm_l'], color='#1f77b4', linewidth=1.6, label='PWM Motor Kiri')
    ax2.plot(t, data['pwm_r'], color='#2ca02c', linewidth=1.6, label='PWM Motor Kanan')
    ax2.axhline(255, color='red', linestyle='--', alpha=0.4, label='Batas Saturasi PWM (255)')
    
    ax2.set_title("Grafik Sinyal Kendali PWM BTS7960 (PI-PSO)", fontsize=11, fontweight='bold', pad=8)
    ax2.set_xlabel("Waktu (detik)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("PWM (0 - 255)", fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower right', framealpha=0.92, fontsize=10)
    ax2.set_ylim(-10, 270)
    ax2.set_xlim(0, max(t))
    
    plt.tight_layout()
    
    # Simpan Gambar dengan Timestamp
    nama_png = args.save
    if nama_png is None:
        base = os.path.splitext(os.path.basename(path_file))[0]
        # Pastikan awalan grafik_
        if not base.startswith("grafik_"):
            nama_png = f"grafik_{base}.png"
        else:
            nama_png = f"{base}.png"
            
        output_dir = os.path.dirname(path_file)
        if output_dir:
            nama_png = os.path.join(output_dir, os.path.basename(nama_png))
        
    plt.savefig(nama_png, dpi=300, bbox_inches='tight')
    print(f"\n[SUKSES] Grafik step response PSO berhasil disimpan ke file:\n>> {nama_png}")

if __name__ == "__main__":
    main()
