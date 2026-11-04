#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SCRIPT PEREKAM DATA RESMI PENGUJIAN STEP PWM (40 -> 160)
SKRIPSI TEKNIK KOMPUTER - DIVALA ZAHRA OKTAVIA (UNNES 2026)
=============================================================================
Fungsi:
  - Membuka port serial Arduino (/dev/arduino, /dev/ttyACM0, atau /dev/ttyUSB0)
  - Otomatis membuat file CSV ber-timestamp: data_step_pwm_40_160_YYYYMMDD_HHMMSS.csv
  - Merekam data presisi: t_sec, pwm_input, ticks_L, ticks_R, rpm_L, rpm_R
  - Otomatis berhenti ketika pengujian di Arduino selesai (# PENGUJIAN SELESAI)
=============================================================================
"""

import os
import sys
import time
import datetime
import serial

def cari_port():
    daftar_port = ['/dev/arduino', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    for p in daftar_port:
        if os.path.exists(p):
            return p
    return None

def main():
    print("=" * 70)
    print("  🚀 PEREKAM DATA OTOMATIS STEP RESPONSE PWM 40 -> 160")
    print("=====================================================================")

    # 1. Pastikan tidak ada proses lain yang mengunci port serial
    os.system("pkill -9 -f arduino_bridge.py > /dev/null 2>&1")

    port = cari_port()
    if not port:
        print("[!] Port Arduino tidak ditemukan! Periksa sambungan kabel USB.")
        sys.exit(1)

    print(f"[*] Terhubung ke port: {port} pada 115200 baud")

    # 2. Siapkan file CSV output dengan timestamp unik
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.expanduser("~/deliver_amr/data_step_response")
    os.makedirs(output_dir, exist_ok=True)
    
    nama_file_csv = os.path.join(output_dir, f"step_pwm_40_160_{timestamp}.csv")
    nama_file_link = os.path.expanduser("~/deliver_amr/data_step_pwm_40_160_terbaru.csv")

    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(1.0) # Tunggu koneksi stabil
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[!] Gagal membuka port serial: {e}")
        sys.exit(1)

    print(f"[*] Merekam data ke file: {nama_file_csv}")
    print("[*] Menunggu hitung mundur Arduino... (Standby)")
    print("-" * 70)

    jumlah_baris = 0
    header_ditemukan = False

    with open(nama_file_csv, 'w', encoding='utf-8') as f_csv:
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Tampilkan ke layar agar user tahu proses sedang berlangsung
                print(line)

                # Simpan baris komentar / info
                if line.startswith('#'):
                    if "PENGUJIAN SELESAI" in line:
                        print("-" * 70)
                        print("✅ Sinyal pengujian selesai terdeteksi!")
                        break
                    continue

                # Cek Header CSV
                if "t_sec" in line:
                    f_csv.write(line + "\n")
                    f_csv.flush()
                    header_ditemukan = True
                    continue

                # Simpan Baris Data CSV
                if ';' in line:
                    parts = line.split(';')
                    if len(parts) >= 6:
                        f_csv.write(line + "\n")
                        f_csv.flush()
                        jumlah_baris += 1

        except KeyboardInterrupt:
            print("\n[*] Perekaman dihentikan manual oleh user.")
        finally:
            ser.close()

    # Buat salinan / shortcut ke file terbaru
    os.system(f"cp '{nama_file_csv}' '{nama_file_link}'")

    print("=" * 70)
    print(f"🎉 SUKSES! Total {jumlah_baris} baris data berhasil disimpan.")
    print(f"📁 Lokasi File Resmi : {nama_file_csv}")
    print(f"📁 Shortcut Mudah    : {nama_file_link}")
    print("=" * 70)

if __name__ == '__main__':
    main()
