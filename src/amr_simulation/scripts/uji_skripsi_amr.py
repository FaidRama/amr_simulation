#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
PROGRAM PENGUJIAN SKRIPSI AMR-01 (INTERAKTIF & ALL-IN-ONE)
Penyusun : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
=============================================================================
Fitur Lengkap (Semua Menghasilkan File CSV Otomatis):
  [1] Uji Step Response Motor (60, 80, 100 RPM) + Grafik PNG + CSV
  [2] Uji Odometri: Gerak Lurus 2.0 Meter (Input Δx cm & Δy cm) + CSV
  [3] Uji Odometri: Rotasi di Tempat 90°, 180°, 360° + CSV
  [4] Uji Odometri: Lintasan Persegi Tertutup (4 x 2m = 8m / UMBmark) + CSV
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

    # =========================================================================
    # MENU 1: STEP RESPONSE MOTOR (60, 80, 100 RPM)
    # =========================================================================
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

        t_arr = np.array(times)
        al_arr = np.array(act_l)
        ar_arr = np.array(act_r)

        res_L = self.evaluasi_metrik(t_arr, al_arr, sp)
        res_R = self.evaluasi_metrik(t_arr, ar_arr, sp)
        metode_nama = "PI-PSO (Optimasi)" if self.metode == "PSO" else "PI Trial & Error"

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

        png_path = os.path.join(self.output_dir, f"{filename_base}.png")
        self.simpan_grafik(t_arr, al_arr, ar_arr, sp, res_L, res_R, view_ts, png_path)

        print(f"\n[✓] Data Berhasil Disimpan:")
        print(f"    1. File CSV   : {csv_path}")
        print(f"    2. Gambar PNG : {png_path}\n")

    def evaluasi_metrik(self, t, act, sp):
        peak = float(np.max(act))
        mp = max(0.0, ((peak - sp) / sp) * 100.0)
        idx_10 = np.where(act >= 0.10 * sp)[0]
        idx_90 = np.where(act >= 0.90 * sp)[0]
        tr = (t[idx_90[0]] - t[idx_10[0]]) if (len(idx_10) > 0 and len(idx_90) > 0) else 0.0

        band = 0.05 * sp
        ts = 0.0
        for i in range(len(t) - 1, -1, -1):
            if abs(act[i] - sp) > band:
                ts = t[i]
                break

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

    # =========================================================================
    # MENU 2: ODOMETRI GERAK LURUS 2.0 METER (INPUT PRAKTIS CM)
    # =========================================================================
    def run_odom_lurus(self):
        print("\n" + "=" * 65)
        print("          PENGUJIAN ODOMETRI: GERAK LURUS 2.0 METER")
        print("=" * 65)
        print(f"Skenario: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
        file_ts, view_ts = get_timestamp()
        target_dist = 2.0

        port = self.cari_port()
        if not port:
            print("[!] Port Arduino tidak ditemukan.")
            return

        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2.0)
        ser.reset_input_buffer()

        input("[*] Posisikan as roda robot tepat di garis start lantai, lalu tekan ENTER...")
        ser.write(f"MODE,{self.metode}\n".encode())
        time.sleep(0.1)
        ser.write(b"1\n")
        ser.flush()

        print("[*] 🚀 Robot bergerak maju 2.0 meter...")
        lines = ["Time_s,RPM_L,RPM_R,PWM_L,PWM_R,X_odom,Y_odom,Theta_deg\n"]
        last_x, last_y, last_th = 0.0, 0.0, 0.0
        t0 = time.time()

        while time.time() - t0 < 12.0:
            l = ser.readline().decode('utf-8', errors='ignore').strip()
            if l.startswith("LOG:"):
                parts = l.replace("LOG:", "").split(",")
                if len(parts) >= 8:
                    lines.append(",".join(parts) + "\n")
                    last_x = float(parts[5])
                    last_y = float(parts[6])
                    last_th = float(parts[7])
                    d_cur = math.hypot(last_x, last_y)
                    sys.stdout.write(f"\r[*] Jarak Sensor: {d_cur:5.4f} m / {target_dist:.2f} m | Yaw={last_th:+5.2f}°")
                    sys.stdout.flush()
                    if d_cur >= target_dist:
                        break
        ser.close()

        dist_sensor = math.hypot(last_x, last_y)
        print(f"\n\n[✓] Pembacaan Odometri Sensor:")
        print(f"    • X_odom : {last_x:+.4f} m | Y_odom : {last_y:+.4f} m | Yaw : {last_th:+.2f}°")
        print(f"    • Total Jarak Sensor : {dist_sensor:.4f} meter")

        print("-" * 65)
        print("Pengukuran Fisik di Lantai (Lihat Gambar Acuan Nat Keramik):")
        delta_x_cm = float(input("  >> Masukkan Δx (Meleset Maju/Mundur dalam cm) [default 0.0]: ") or "0.0")
        delta_y_cm = float(input("  >> Masukkan Δy (Meleset Samping dalam cm)       [default 0.0]: ") or "0.0")

        # Hitung E_pos (Total Penyimpangan Posisi Diagonal)
        e_pos_cm = math.hypot(delta_x_cm, delta_y_cm)
        galat_pct = (e_pos_cm / 200.0) * 100.0 # 200 cm = 2.0 meter

        print("\n" + "=" * 65)
        print("       LAPORAN EVALUASI ODOMETRI LURUS 2.0 M")
        print(f"  Metode: {self.metode} | Kondisi: {self.kondisi} | {self.sesi}")
        print("=" * 65)
        print(f"• Pembacaan Sensor Odometri  : X={last_x:+.4f} m, Y={last_y:+.4f} m, Yaw={last_th:+.2f}°")
        print(f"• Meleset Maju/Mundur (Δx)   : {delta_x_cm:+.2f} cm")
        print(f"• Meleset Samping (Δy)       : {delta_y_cm:+.2f} cm")
        print(f"• Total Error Posisi (E_pos) : {e_pos_cm:.2f} cm")
        print(f"• Persentase Galat Odometri  : {galat_pct:.2f} %")
        print("=" * 65)

        # Simpan CSV Lengkap
        filename = f"odom_lurus_2m_{self.metode.lower()}_{self.kondisi}_{self.sesi}_{file_ts}.csv"
        csv_path = os.path.join(self.output_dir, filename)
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.writelines(lines)
            f.write("\n")
            f.write("# ========================================================================\n")
            f.write("# RANGKUMAN EVALUASI ODOMETRI GERAK LURUS 2.0 METER\n")
            f.write(f"# Metode               : {self.metode}\n")
            f.write(f"# Kondisi              : {self.kondisi} | {self.sesi}\n")
            f.write(f"# Waktu                : {view_ts}\n")
            f.write("# ========================================================================\n")
            f.write(f"# X_odom_m;{last_x:.4f}\n")
            f.write(f"# Y_odom_m;{last_y:.4f}\n")
            f.write(f"# Yaw_odom_deg;{last_th:.2f}\n")
            f.write(f"# Delta_X_cm;{delta_x_cm:.2f}\n")
            f.write(f"# Delta_Y_cm;{delta_y_cm:.2f}\n")
            f.write(f"# E_pos_cm;{e_pos_cm:.2f}\n")
            f.write(f"# Galat_Relatif_pct;{galat_pct:.2f}\n")
            f.write("# ========================================================================\n")

        print(f"[✓] Data Telemetri & Evaluasi Tersimpan ke: {csv_path}\n")

    # =========================================================================
    # MENU 3: ODOMETRI ROTASI DI TEMPAT (90°, 180°, 360°)
    # =========================================================================
    def run_odom_rotasi(self):
        print("\n" + "=" * 65)
        print("        PENGUJIAN ODOMETRI: ROTASI DI TEMPAT")
        print("=" * 65)
        print(f"Skenario: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
        print("Pilih Sudut Rotasi:")
        print("  [1] 90 Derajat")
        print("  [2] 180 Derajat")
        print("  [3] 360 Derajat (1 Putaran Penuh)")
        opt = input("Pilihan (1-3): ").strip()

        if opt == '1': cmd_char, target_deg = b"2\n", 90.0
        elif opt == '2': cmd_char, target_deg = b"3\n", 180.0
        else: cmd_char, target_deg = b"4\n", 360.0

        file_ts, view_ts = get_timestamp()
        port = self.cari_port()
        if not port:
            print("[!] Port Arduino tidak ditemukan.")
            return

        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2.0)
        ser.reset_input_buffer()

        input(f"[*] Tekan ENTER untuk memulai rotasi robot {target_deg:.0f}°...")
        ser.write(f"MODE,{self.metode}\n".encode())
        time.sleep(0.1)
        ser.write(cmd_char)
        ser.flush()

        print(f"[*] 🔄 Robot berputar di tempat sebesar {target_deg:.0f}°...")
        lines = ["Time_s,RPM_L,RPM_R,PWM_L,PWM_R,X_odom,Y_odom,Theta_deg\n"]
        last_th = 0.0
        t0 = time.time()

        while time.time() - t0 < 15.0:
            l = ser.readline().decode('utf-8', errors='ignore').strip()
            if l.startswith("LOG:"):
                parts = l.replace("LOG:", "").split(",")
                if len(parts) >= 8:
                    lines.append(",".join(parts) + "\n")
                    last_th = abs(float(parts[7]))
                    sys.stdout.write(f"\r[*] Sudut Sensor: {last_th:5.1f}° / {target_deg:.1f}°")
                    sys.stdout.flush()
                    if last_th >= (target_deg - 0.5):
                        break
        ser.close()

        print(f"\n\n[✓] Pembacaan Sudut Sensor: {last_th:.2f}°")
        print("-" * 65)
        th_riil = float(input(f"  >> Masukkan sudut aktual dari busur derajat (derajat) [default {target_deg:.0f}]: ") or str(target_deg))

        err_deg = abs(last_th - th_riil)
        err_pct = (err_deg / target_deg) * 100.0

        print("\n" + "=" * 65)
        print(f"       LAPORAN EVALUASI ROTASI {target_deg:.0f}°")
        print(f"  Metode: {self.metode} | Kondisi: {self.kondisi} | {self.sesi}")
        print("=" * 65)
        print(f"• Target Rotasi            : {target_deg:.1f}°")
        print(f"• Sudut Sensor Odometri    : {last_th:.2f}°")
        print(f"• Sudut Riil Busur Derajat : {th_riil:.2f}°")
        print(f"• Kesalahan Orientasi (Δθ) : {err_deg:.2f}°")
        print(f"• Persentase Galat Sudut   : {err_pct:.2f} %")
        print("=" * 65)

        filename = f"odom_rotasi_{int(target_deg)}deg_{self.metode.lower()}_{self.kondisi}_{self.sesi}_{file_ts}.csv"
        csv_path = os.path.join(self.output_dir, filename)
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.writelines(lines)
            f.write("\n")
            f.write("# ========================================================================\n")
            f.write(f"# RANGKUMAN EVALUASI ROTASI DI TEMPAT {target_deg:.0f} DERAJAT\n")
            f.write(f"# Metode               : {self.metode}\n")
            f.write(f"# Kondisi              : {self.kondisi} | {self.sesi}\n")
            f.write(f"# Waktu                : {view_ts}\n")
            f.write("# ========================================================================\n")
            f.write(f"# Target_deg;{target_deg:.1f}\n")
            f.write(f"# Theta_sensor_deg;{last_th:.2f}\n")
            f.write(f"# Theta_riil_deg;{th_riil:.2f}\n")
            f.write(f"# Kesalahan_Sudut_deg;{err_deg:.2f}\n")
            f.write(f"# Galat_Sudut_pct;{err_pct:.2f}\n")
            f.write("# ========================================================================\n")

        print(f"[✓] Data Telemetri & Evaluasi Tersimpan ke: {csv_path}\n")

    # =========================================================================
    # MENU 4: ODOMETRI PERSEGI TERTUTUP (4 x 2m = 8m / UMBmark)
    # =========================================================================
    def run_odom_persegi(self):
        print("\n" + "=" * 65)
        print("   PENGUJIAN ODOMETRI PERSEGI TERTUTUP (4 x 2.0 M = 8.0 M)")
        print("=" * 65)
        print(f"Skenario: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
        print("Robot akan menempuh 4 sisi: Maju 2m -> Belok 90° (diulang 4 kali).")
        file_ts, view_ts = get_timestamp()

        port = self.cari_port()
        if not port:
            print("[!] Port Arduino tidak ditemukan.")
            return

        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2.0)
        ser.reset_input_buffer()

        input("[*] Posisikan as roda robot tepat di titik start (0,0). Tekan ENTER...")
        ser.write(f"MODE,{self.metode}\n".encode())
        time.sleep(0.1)
        ser.write(b"5\n")
        ser.flush()

        print("[*] 🔄 Robot menjalankan lintasan persegi (durasi ~35-45 detik)...")
        lines = ["Time_s,RPM_L,RPM_R,PWM_L,PWM_R,X_odom,Y_odom,Theta_deg\n"]
        last_x, last_y, last_th = 0.0, 0.0, 0.0
        t0 = time.time()

        while time.time() - t0 < 60.0:
            l = ser.readline().decode('utf-8', errors='ignore').strip()
            if l.startswith("LOG:"):
                parts = l.replace("LOG:", "").split(",")
                if len(parts) >= 8:
                    lines.append(",".join(parts) + "\n")
                    last_x = float(parts[5])
                    last_y = float(parts[6])
                    last_th = float(parts[7])
                    sys.stdout.write(f"\r[*] X={last_x:+5.2f}m | Y={last_y:+5.2f}m | Yaw={last_th:+5.1f}°")
                    sys.stdout.flush()
        ser.close()

        print(f"\n\n[✓] Posisi Akhir Odometri Sensor:")
        print(f"    • Koordinat Sensor : ({last_x:+.4f} m, {last_y:+.4f} m)")
        print(f"    • Sudut Akhir Sensor : {last_th:+.2f}°")

        print("-" * 65)
        print("Pengukuran Fisik di Garis Finish (Lihat Gambar Segitiga A-B-C):")
        delta_x_cm = float(input("  >> Masukkan Δx (Meleset Maju/Mundur dalam cm) [default 0.0]: ") or "0.0")
        delta_y_cm = float(input("  >> Masukkan Δy (Meleset Samping dalam cm)       [default 0.0]: ") or "0.0")
        th_riil_err = float(input("  >> Masukkan Meleset Sudut Hadap / Heading (derajat) [default 0.0]: ") or "0.0")

        total_lintasan = 8.0 # meter
        e_return_cm = math.hypot(delta_x_cm, delta_y_cm)
        drift_rate = e_return_cm / total_lintasan # cm per meter

        print("\n" + "=" * 65)
        print("   LAPORAN EVALUASI LINTASAN PERSEGI (4x2m = 8m)")
        print(f"  Metode: {self.metode} | Kondisi: {self.kondisi} | {self.sesi}")
        print("=" * 65)
        print(f"• Total Jarak Tempuh Lintasan    : {total_lintasan:.1f} meter")
        print(f"• Posisi Akhir Sensor Odometri   : ({last_x:+.3f} m, {last_y:+.3f} m, {last_th:+.1f}°)")
        print(f"• Meleset Maju/Mundur (Δx)       : {delta_x_cm:+.2f} cm")
        print(f"• Meleset Samping (Δy)           : {delta_y_cm:+.2f} cm")
        print(f"• Return-to-Origin Error (E_ret) : {e_return_cm:.2f} cm")
        print(f"• Tingkat Penyimpangan (Drift)   : {drift_rate:.2f} cm/meter")
        print(f"• Kesalahan Sudut Hadap Akhir    : {th_riil_err:.2f}°")
        print("=" * 65)

        filename = f"odom_persegi_8m_{self.metode.lower()}_{self.kondisi}_{self.sesi}_{file_ts}.csv"
        csv_path = os.path.join(self.output_dir, filename)
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.writelines(lines)
            f.write("\n")
            f.write("# ========================================================================\n")
            f.write("# RANGKUMAN EVALUASI ODOMETRI LINTASAN PERSEGI (4x2m = 8m)\n")
            f.write(f"# Metode               : {self.metode}\n")
            f.write(f"# Kondisi              : {self.kondisi} | {self.sesi}\n")
            f.write(f"# Waktu                : {view_ts}\n")
            f.write("# ========================================================================\n")
            f.write(f"# Total_Lintasan_m;{total_lintasan:.1f}\n")
            f.write(f"# X_sensor_m;{last_x:.4f}\n")
            f.write(f"# Y_sensor_m;{last_y:.4f}\n")
            f.write(f"# Theta_sensor_deg;{last_th:.2f}\n")
            f.write(f"# Delta_X_cm;{delta_x_cm:.2f}\n")
            f.write(f"# Delta_Y_cm;{delta_y_cm:.2f}\n")
            f.write(f"# Return_To_Origin_Error_cm;{e_return_cm:.2f}\n")
            f.write(f"# Drift_Rate_cm_per_m;{drift_rate:.2f}\n")
            f.write(f"# Heading_Error_deg;{th_riil_err:.2f}\n")
            f.write("# ========================================================================\n")

        print(f"[✓] Data Telemetri & Evaluasi Tersimpan ke: {csv_path}\n")

    # =========================================================================
    # MENU UTAMA
    # =========================================================================
    def main_menu(self):
        while True:
            print("\n" + "=" * 65)
            print("       SISTEM BENCHMARK PENGUJIAN SKRIPSI AMR-01")
            print("=" * 65)
            print(f"Status Aktif: [{self.metode}] | [{self.kondisi}] | [{self.sesi}]")
            print("-" * 65)
            print(" [K] Ganti Skenario (Metode PI, Beban, Pengulangan)")
            print(" [1] Uji Step Response Motor (60, 80, 100 RPM) + Buat Grafik")
            print(" [2] Uji Odometri: Gerak Lurus 2.0 Meter")
            print(" [3] Uji Odometri: Rotasi di Tempat (90°, 180°, 360°)")
            print(" [4] Uji Odometri: Lintasan Persegi Tertutup (4 x 2m = 8m)")
            print(" [0] Keluar")
            print("=" * 65)

            p = input("Pilih Menu: ").strip().upper()
            if p == 'K': self.menu_skenario()
            elif p == '1': self.run_step_test()
            elif p == '2': self.run_odom_lurus()
            elif p == '3': self.run_odom_rotasi()
            elif p == '4': self.run_odom_persegi()
            elif p == '0': break


if __name__ == '__main__':
    AMRBenchmark().main_menu()