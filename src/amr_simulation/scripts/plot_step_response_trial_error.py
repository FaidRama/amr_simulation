#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SCRIPT VISUALISASI & ANALISIS PERFORMA STEP RESPON (TRIAL & ERROR)
SKRIPSI: DIVALA ZAHRA OKTAVIA (TEKNIK KOMPUTER UNNES 2026)
=============================================================================
Fitur:
  1. Otomatis membaca file CSV dari serial Arduino (mendukung multi-step & single step).
  2. Menghitung 5 Parameter Performa Evaluasi Skripsi:
     - RMSE (Root Mean Square Error)
     - Overshoot (%)
     - Rise Time (Tr, 10% s.d 90%)
     - Settling Time (Ts, kriteria +-5%)
     - Steady-State Error (Ess)
  3. Menghasilkan dan menyimpan gambar grafik berkualitas tinggi (.png 300 DPI)
     lengkap dengan batas toleransi +-5%, target, respon roda kiri & kanan, serta PWM.
=============================================================================
"""

import os
import sys
import glob
import csv
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt

def cari_file_csv_terbaru():
    """Mencari file CSV hasil pengujian trial error terbaru jika argumen kosong."""
    pola_list = [
        "hasil_trial_error*.csv",
        "step_respon*trial_error*.csv",
        "step_respon*.csv",
        "data_step_response/*.csv"
    ]
    kandidat = []
    for p in pola_list:
        kandidat.extend(glob.glob(p))
        kandidat.extend(glob.glob(os.path.join("..", p)))
        kandidat.extend(glob.glob(os.path.join(os.path.expanduser("~"), "deliver_amr", p)))
    
    if not kandidat:
        return None
    # Pilih file dengan waktu modifikasi paling mutakhir
    return max(kandidat, key=os.path.getmtime)

def baca_data_csv(path_file):
    """Membaca data streaming CSV pengujian Arduino."""
    if not os.path.exists(path_file):
        raise FileNotFoundError(f"File tidak ditemukan: {path_file}")
    
    with open(path_file, 'r', encoding='utf-8', errors='ignore') as f:
        sample = f.read(2048)
        delimiter = ';' if sample.count(';') > sample.count(',') else ','
    
    time_s, sp_l, actual_l, pwm_l, err_l = [], [], [], [], []
    sp_r, actual_r, pwm_r, err_r = [], [], [], []
    
    with open(path_file, 'r', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f, delimiter=delimiter)
        for row in reader:
            if not row or len(row) < 5 or row[0].strip().startswith('#'):
                continue
            # Lewati header baris
            if 'time' in row[0].lower() or 'target' in row[1].lower():
                continue
            try:
                t = float(row[0].strip())
                target_l = float(row[1].strip())
                act_l = float(row[2].strip())
                p_l = float(row[3].strip())
                e_l = float(row[4].strip())
                
                # Format 9 kolom (L dan R lengkap)
                if len(row) >= 9:
                    target_r = float(row[5].strip())
                    act_r = float(row[6].strip())
                    p_r = float(row[7].strip())
                    e_r = float(row[8].strip())
                else:
                    target_r = target_l
                    act_r = act_l
                    p_r = p_l
                    e_r = e_l

                time_s.append(t)
                sp_l.append(target_l)
                actual_l.append(act_l)
                pwm_l.append(p_l)
                err_l.append(e_l)
                
                sp_r.append(target_r)
                actual_r.append(act_r)
                pwm_r.append(p_r)
                err_r.append(e_r)
            except (ValueError, IndexError):
                continue
                
    if not time_s:
        raise ValueError(f"Tidak ada baris data valid yang terbaca dari {path_file}")
        
    return {
        't': np.array(time_s),
        'sp_l': np.array(sp_l),
        'act_l': np.array(actual_l),
        'pwm_l': np.array(pwm_l),
        'sp_r': np.array(sp_r),
        'act_r': np.array(actual_r),
        'pwm_r': np.array(pwm_r)
    }

def deteksi_tahap_step(t, sp):
    """Mendeteksi segmen step kecepatan (misal 60, 80, 100 RPM)."""
    # Cari nilai setpoint unik di atas ambang batas 5 RPM
    sp_unik = []
    for val in sp:
        val_round = round(val, -1) if val > 10 else round(val)
        if val > 5.0 and (not sp_unik or abs(val_round - sp_unik[-1]) > 5.0):
            sp_unik.append(val_round)
            
    tahap_list = []
    if not sp_unik:
        # Fallback jika target statis
        target_default = float(np.max(sp)) if np.max(sp) > 5 else 60.0
        tahap_list.append((0.0, float(t[-1]), target_default))
        return tahap_list

    for target in sp_unik:
        idx = np.where(np.abs(sp - target) <= 5.0)[0]
        if len(idx) > 10:
            t_mulai = float(t[idx[0]])
            t_selesai = float(t[idx[-1]])
            tahap_list.append((t_mulai, t_selesai, target))
            
    return tahap_list

def hitung_parameter_performa(t_seg, act_seg, pwm_seg, target):
    """Menghitung 5 parameter evaluasi kontroler."""
    t_rel = t_seg - t_seg[0]
    
    # 1. Puncak & Overshoot
    ymax = float(np.max(act_seg))
    idx_puncak = int(np.argmax(act_seg))
    tp = float(t_rel[idx_puncak])
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
                
    # 4 & 5. Steady State (20% data terakhir dari segmen atau t_rel >= 1.0s)
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
    parser = argparse.ArgumentParser(description="Plot & Evaluasi Step Response Trial & Error")
    parser.add_argument("csv_path", nargs="?", default=None, help="Path ke file CSV hasil tes")
    parser.add_argument("--save", "-s", default=None, help="Nama file PNG output grafik")
    args = parser.parse_args()
    
    path_file = args.csv_path
    if path_file is None:
        path_file = cari_file_csv_terbaru()
        
    if path_file is None:
        print("[!] File CSV pengujian tidak ditemukan.")
        print("[*] Contoh pemakaian: python plot_step_response_trial_error.py hasil_trial_error.csv")
        sys.exit(1)
        
    print(f"[*] Memproses data dari file: {path_file}")
    data = baca_data_csv(path_file)
    t = data['t']
    
    tahap_list = deteksi_tahap_step(t, data['sp_l'])
    
    # Cetak Rangkuman Tabel di Konsol
    print("\n" + "=" * 82)
    print("      HASIL ANALISIS EVALUASI 5 PARAMETER PERFORMA (TRIAL & ERROR)")
    print("      Gain: Left (Kp=2.5, Ki=18) | Right (Kp=2.8, Ki=18)")
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
    ax1.plot(t, data['act_r'], color='#ff7f0e', linewidth=1.8, label='Roda Kanan (Actual RPM)')
    
    # Gambar area pita toleransi +-5% untuk masing-masing tahap
    for sp_val, pl, pr, t_s, t_e in hasil_tahap:
        band_atas = sp_val * 1.05
        band_bawah = sp_val * 0.95
        ax1.hlines([band_atas, band_bawah], t_s, t_e, colors='gray', linestyles=':', alpha=0.6)
        ax1.fill_between([t_s, t_e], [band_bawah, band_bawah], [band_atas, band_atas], 
                         color='gray', alpha=0.12, label='Toleransi +-5%' if sp_val == hasil_tahap[0][0] else "")
        
    ax1.set_title("Grafik Step Response Kecepatan Roda AMR (Kendali PI - Trial & Error)\n"
                  f"Parameter: Kiri (Kp=2.50, Ki=18.00) | Kanan (Kp=2.80, Ki=18.00)", 
                  fontsize=12, fontweight='bold', pad=12)
    ax1.set_ylabel("Kecepatan Roda (RPM)", fontsize=11, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='lower right', framealpha=0.92, fontsize=10)
    
    max_rpm = max(np.max(data['act_l']), np.max(data['act_r']), np.max(data['sp_l']))
    ax1.set_ylim(-5, max_rpm + 15)
    
    # SUBPLOT 2: SINYAL KENDALI PWM
    ax2.plot(t, data['pwm_l'], color='#1f77b4', linewidth=1.6, label='PWM Motor Kiri')
    ax2.plot(t, data['pwm_r'], color='#ff7f0e', linewidth=1.6, label='PWM Motor Kanan')
    ax2.axhline(255, color='red', linestyle='--', alpha=0.4, label='Batas Saturasi PWM (255)')
    
    ax2.set_title("Grafik Sinyal Kendali PWM BTS7960", fontsize=11, fontweight='bold', pad=8)
    ax2.set_xlabel("Waktu (detik)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("PWM (0 - 255)", fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='lower right', framealpha=0.92, fontsize=10)
    ax2.set_ylim(-10, 270)
    ax2.set_xlim(0, max(t))
    
    plt.tight_layout()
    
    # Simpan Gambar
    nama_png = args.save
    if nama_png is None:
        base = os.path.splitext(os.path.basename(path_file))[0]
        nama_png = f"grafik_{base}.png"
        
    plt.savefig(nama_png, dpi=300, bbox_inches='tight')
    print(f"\n[SUKSES] Grafik step response berhasil disimpan ke file: {nama_png}")

if __name__ == "__main__":
    main()
