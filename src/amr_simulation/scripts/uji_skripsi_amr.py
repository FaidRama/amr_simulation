#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
PROGRAM PENGUJIAN SKRIPSI AMR-01 (INTERAKTIF & ALL-IN-ONE)
Penyusun : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
=============================================================================
Fitur:
  1. Pilih Metode Kontroler: PI-PSO atau PI Trial & Error
  2. Pilih Target Kecepatan: 60, 80, 100 RPM
  3. Merekam streaming 9 kolom data dari Arduino Uno
  4. Menampilkan 5 Metrik Transien Tertulis: Peak RPM, Overshoot (Mp),
     Rise Time (Tr), Settling Time (Ts), Steady-State Error (Ess), dan RMSE
  5. Otomatis menyimpan file CSV (lengkap dengan data 9 kolom & tabel rangkuman 5 metrik)
  6. Menghasilkan GAMBAR GRAFIK (.PNG) dengan judul RPM, metode, dan cap tanggal-jam.
=============================================================================
"""

import os
import sys
import time
import math
import datetime
import numpy as np
import matplotlib.pyplot as plt

try:
    import serial
except ImportError:
    print("[!] Modul pyserial belum terinstall. Jalankan: pip3 install pyserial")
    sys.exit(1)


def get_timestamp():
    now = datetime.datetime.now()
    file_ts = now.strftime("%Y%m%d_%H%M%S")
    view_ts = now.strftime("%d/%m/%Y Jam %H:%M:%S WIB")
    return file_ts, view_ts


class AMRBenchmark:
    def __init__(self):
        self.metode = "PSO"           # "PSO" atau "TE"
        self.kondisi = "tanpa_beban"  # "tanpa_beban" atau "beban_3kg"
        self.sesi = "uji_1"           # "uji_1" atau "uji_2"
        self.output_dir = os.path.expanduser("~/deliver_amr")
        if not os.path.exists(self.output_dir):
            self.output_dir = "."

    def cari_port(self):
        for p in ['/dev/arduino', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', 'COM3', 'COM4']:
            if os.path.exists(p):
                return p
        return None

    def menu_skenario(self):
        print("\n" + "=" * 65)
        print("          PENGATURAN LABEL SKENARIO PENGUJIAN")
        print("=" * 65)
        print("1. Pilih Metode Kontroler:")
        print("   [1] PI-PSO (Optimasi Particle Swarm)")
        print("   [2] PI Trial & Error (Manual)")
        p1 = input("   Pilihan (1/2): ").strip()
        self.metode = "PSO" if p1 == '1' else "TE"

        print("\n2. Pilih Kondisi Beban Robot:")
        print("   [1] Tanpa Beban (0 kg)")
        print("   [2] Beban 3 kg")
        p2 = input("   Pilihan (1/2): ").strip()
        self.kondisi = "tanpa_beban" if p2 == '1' else "beban_3kg"

        print("\n3. Pilih Pengulangan:")
        print("   [1] Pengujian 1 (Uji 1)")
        print("   [2] Pengujian 2 (Uji 2)")
        p3 = input("   Pilihan (1/2): ").strip()
        self.sesi = "uji_1" if p3 == '1' else "uji_2"

        print(f"\n[✓] Skenario Aktif: [{self.metode}] - [{self.kondisi}] - [{self.sesi}]")
        print("=" * 65)

    def run_step_test(self):
        print("\n" + "=" * 65)
        print("          PENGUJIAN STEP RESPON KECEPATAN MOTOR")
        print("=" * 65)
        print(f"Skenario: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
        print("Pilih Target Kecepatan:")
        print("  [1] 60 RPM")
        print("  [2] 80 RPM")
        print("  [3] 100 RPM")
        print("  [4] Input Manual")
        p = input("Pilihan (1-4): ").strip()
        
        if p == '1': sp = 60.0
        elif p == '2': sp = 80.0
        elif p == '3': sp = 100.0
        elif p == '4': sp = float(input("Masukkan Target RPM: ").strip())
        else: sp = 60.0

        target_str = f"rpm{int(sp)}"
        file_ts, view_ts = get_timestamp()

        port = self.cari_port()
        if not port:
            print("[!] Port Arduino tidak ditemukan di /dev/arduino maupun /dev/ttyACM0.")
            return

        print(f"[*] Menghubungkan ke Arduino di {port}...")
        try:
            ser = serial.Serial(port, 115200, timeout=1)
        except Exception as e:
            print(f"[!] Gagal membuka port: {e}")
            return

        time.sleep(2.0)
        ser.reset_input_buffer()

        # Kirim perintah: "STEP,<METODE>,<RPM>\n"
        cmd = f"STEP,{self.metode},{int(sp)}\n"
        print(f"[*] Mengirim perintah ke Arduino: {cmd.strip()}")
        ser.write(cmd.encode())
        ser.flush()

        raw_lines = []
        t0 = time.time()
        print("[*] Merekam streaming data dari Arduino (~7 detik)...")
        while time.time() - t0 < 8.0:
            l = ser.readline().decode('utf-8', errors='ignore')
            if l:
                raw_lines.append(l)
        ser.close()

        # Parsing 9 Kolom: Time_s;Target_L;Act_L;PWM_L;Err_L;Target_R;Act_R;PWM_R;Err_R
        times, act_l, act_r, pwm_l, pwm_r = [], [], [], [], []
        valid_csv_lines = ["Time_s,Target_RPM_L,Actual_RPM_L,PWM_L,Error_L,Target_RPM_R,Actual_RPM_R,PWM_R,Error_R\n"]

        for l in raw_lines:
            if l.startswith('#') or 'Time' in l or 'Target' in l:
                continue
            parts = l.strip().replace(';', ',').split(',')
            if len(parts) >= 9:
                try:
                    t = float(parts[0])
                    al = float(parts[2])
                    pl = float(parts[3])
                    ar = float(parts[6])
                    pr = float(parts[7])
                    times.append(t)
                    act_l.append(al)
                    act_r.append(ar)
                    pwm_l.append(pl)
                    pwm_r.append(pr)
                    valid_csv_lines.append(','.join(parts) + '\n')
                except ValueError:
                    continue

        if len(times) < 20:
            print("[!] Data rekaman serial terlalu sedikit atau koneksi terputus.")
            return

        # 1. Hitung 5 Metrik Evaluasi Transien DULU
        t_arr = np.array(times)
        al_arr = np.array(act_l)
        ar_arr = np.array(act_r)

        res_L = self.evaluasi_metrik(t_arr, al_arr, sp)
        res_R = self.evaluasi_metrik(t_arr, ar_arr, sp)
        metode_nama = "PI-PSO (Optimasi)" if self.metode == "PSO" else "PI Trial & Error"

        # 2. Simpan CSV lengkap dengan Data 9 Kolom + Tabel Rangkuman 5 Parameter di Bawah
        filename_base = f"step_respon_{target_str}_{self.metode.lower()}_{self.kondisi}_{self.sesi}_{file_ts}"
        csv_path = os.path.join(self.output_dir, f"{filename_base}.csv")
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.writelines(valid_csv_lines)
            f.write("\n")
            f.write("# ========================================================================\n")
            f.write(f"# HASIL EVALUASI RESPON TRANSIEN MOTOR ({sp:.0f} RPM)\n")
            f.write(f"# Metode  : {metode_nama}\n")
            f.write(f"# Kondisi : {self.kondisi.upper().replace('_', ' ')} | {self.sesi.upper()}\n")
            f.write(f"# Waktu   : {view_ts}\n")
            f.write("# ========================================================================\n")
            f.write("# Parameter Evaluasi;Motor Kiri;Motor Kanan;Satuan\n")
            f.write(f"# 1. Nilai Puncak (Peak RPM);{res_L['peak']:.2f};{res_R['peak']:.2f};RPM\n")
            f.write(f"# 2. Overshoot (Mp);{res_L['mp']:.2f}%;{res_R['mp']:.2f}%;%\n")
            f.write(f"# 3. Rise Time (Tr 10%-90%);{res_L['tr']:.3f};{res_R['tr']:.3f};detik\n")
            f.write(f"# 4. Settling Time (Ts +-5%);{res_L['ts']:.3f};{res_R['ts']:.3f};detik\n")
            f.write(f"# 5. Steady-State Error (Ess);{res_L['ess']:.2f};{res_R['ess']:.2f};RPM\n")
            f.write(f"#    - Persentase Galat Ess;{res_L['ess_pct']:.2f}%;{res_R['ess_pct']:.2f}%;%\n")
            f.write(f"#    - RMSE Respon;{res_L['rmse']:.2f};{res_R['rmse']:.2f};RPM\n")
            f.write("# ========================================================================\n")

        # 3. Cetak Hasil Tertulis di Layar Terminal
        print("\n" + "=" * 72)
        print(f"      HASIL EVALUASI RESPON TRANSIEN MOTOR ({sp:.0f} RPM)")
        print(f"  Metode  : {metode_nama}")
        print(f"  Kondisi : {self.kondisi.upper().replace('_', ' ')} | {self.sesi.upper()}")
        print(f"  Waktu   : {view_ts}")
        print("=" * 72)
        print(f"{'Parameter Evaluasi':<32} | {'Motor Kiri':<14} | {'Motor Kanan':<14} | {'Satuan'}")
        print("-" * 72)
        print(f"{'1. Nilai Puncak (Peak RPM)':<32} | {res_L['peak']:<14.2f} | {res_R['peak']:<14.2f} | RPM")
        print(f"{'2. Overshoot (Mp)':<32} | {res_L['mp']:<13.2f}% | {res_R['mp']:<13.2f}% | %")
        print(f"{'3. Rise Time (Tr 10%-90%)':<32} | {res_L['tr']:<14.3f} | {res_R['tr']:<14.3f} | detik")
        print(f"{'4. Settling Time (Ts +-5%)':<32} | {res_L['ts']:<14.3f} | {res_R['ts']:<14.3f} | detik")
        print(f"{'5. Steady-State Error (Ess)':<32} | {res_L['ess']:<14.2f} | {res_R['ess']:<14.2f} | RPM")
        print(f"{'   - Persentase Galat Ess':<32} | {res_L['ess_pct']:<13.2f}% | {res_R['ess_pct']:<13.2f}% | %")
        print(f"{'   - RMSE Respon':<32} | {res_L['rmse']:<14.2f} | {res_R['rmse']:<14.2f} | RPM")
        print("=" * 72)

        # 4. Buat Gambar Grafik PNG
        png_path = os.path.join(self.output_dir, f"{filename_base}.png")
        self.simpan_grafik(t_arr, al_arr, ar_arr, sp, res_L, res_R, view_ts, png_path)

        print(f"\n[✓] Data Berhasil Disimpan:")
        print(f"    1. File CSV   : {csv_path}")
        print(f"    2. Gambar PNG : {png_path}\n")

    def evaluasi_metrik(self, t, act, sp):
        peak = float(np.max(act))
        mp = max(0.0, ((peak - sp) / sp) * 100.0)

        # Rise time (10% - 90%)
        idx_10 = np.where(act >= 0.10 * sp)[0]
        idx_90 = np.where(act >= 0.90 * sp)[0]
        tr = (t[idx_90[0]] - t[idx_10[0]]) if (len(idx_10) > 0 and len(idx_90) > 0) else 0.0

        # Settling time (+- 5%)
        band = 0.05 * sp
        ts = 0.0
        for i in range(len(t) - 1, -1, -1):
            if abs(act[i] - sp) > band:
                ts = t[i]
                break

        # Steady-State Error (20% akhir pengujian)
        n_steady = max(5, int(len(act) * 0.20))
        mean_steady = float(np.mean(act[-n_steady:]))
        ess = abs(sp - mean_steady)
        ess_pct = (ess / sp) * 100.0
        rmse = float(np.sqrt(np.mean((act[-n_steady:] - sp)**2)))

        return {'peak': peak, 'mp': mp, 'tr': tr, 'ts': ts, 'ess': ess, 'ess_pct': ess_pct, 'rmse': rmse}

    def simpan_grafik(self, t, act_l, act_r, sp, res_l, res_r, view_ts, path_out):
        plt.figure(figsize=(10, 5.5), dpi=300)
        band = 0.05 * sp

        plt.axhline(y=sp, color='black', linestyle='--', linewidth=1.5, label=f'Target ({sp:.0f} RPM)')
        plt.axhline(y=sp + band, color='gray', linestyle=':', alpha=0.7, label='Toleransi ±5%')
        plt.axhline(y=sp - band, color='gray', linestyle=':', alpha=0.7)

        plt.plot(t, act_l, 'b-', linewidth=1.8, label=f"Motor Kiri (Mp={res_l['mp']:.1f}%, Ess={res_l['ess']:.2f})")
        plt.plot(t, act_r, 'r-', linewidth=1.8, label=f"Motor Kanan (Mp={res_r['mp']:.1f}%, Ess={res_r['ess']:.2f})")

        metode_nama = "PI-PSO" if self.metode == "PSO" else "Trial & Error"
        plt.title(f"Grafik Respon Kecepatan {sp:.0f} RPM [{metode_nama}] - {self.kondisi.upper()}\nWaktu: {view_ts}", fontsize=11, fontweight='bold')
        plt.xlabel("Waktu (detik)", fontsize=10)
        plt.ylabel("Kecepatan Putar (RPM)", fontsize=10)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='lower right', framealpha=0.9)
        plt.tight_layout()
        plt.savefig(path_out)
        plt.close()

    def main_menu(self):
        while True:
            print("\n" + "=" * 65)
            print("       SISTEM BENCHMARK PENGUJIAN SKRIPSI AMR-01")
            print("=" * 65)
            print(f"Status Aktif: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
            print("-" * 65)
            print(" [K] Ganti Skenario (Metode PI, Beban, Pengulangan)")
            print(" [1] Uji Step Response Motor (60, 80, 100 RPM) + Buat Grafik")
            print(" [0] Keluar")
            print("=" * 65)

            p = input("Pilih Menu: ").strip().upper()
            if p == 'K': self.menu_skenario()
            elif p == '1': self.run_step_test()
            elif p == '0': break


if __name__ == '__main__':
    AMRBenchmark().main_menu()