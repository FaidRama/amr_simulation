#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
pso_step_test.py
Runner Khusus untuk Memicu, Merekam, dan Mengevaluasi Pengujian Step Response PI-PSO.
Dilengkapi penamaan otomatis dengan TIMESTAMP (TANGGAL & JAM) agar tidak salah file.
Cukup jalankan: python3 pso_step_test.py
=============================================================================
"""

import os
import sys
import time
import shutil
import datetime

try:
    import serial
except ImportError:
    print("[!] Modul pyserial belum terinstall. Jalankan: pip3 install pyserial")
    sys.exit(1)

def main():
    # 1. Deteksi Port Otomatis
    port_candidates = ['/dev/ttyACM1', '/dev/ttyACM0', '/dev/arduino', '/dev/ttyUSB0']
    port = None
    for p in port_candidates:
        if os.path.exists(p):
            port = p
            break
            
    if not port:
        print("[!] Port Arduino tidak ditemukan di /dev/ttyACM1, /dev/ttyACM0, maupun /dev/arduino.")
        print("[*] Pastikan kabel USB Arduino terhubung dengan baik.")
        sys.exit(1)

    # 2. Buka Port Serial
    print(f"[*] Terhubung ke Arduino di {port} (115200 baud)...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"[!] Gagal membuka port {port}: {e}")
        print(f"[*] Coba jalankan: sudo chmod 666 {port}")
        sys.exit(1)

    time.sleep(2.0)
    ser.reset_input_buffer()

    # 3. Kirim Trigger '1'
    print("[*] Mengirim trigger '1' untuk memulai pengujian motor PI-PSO...")
    ser.write(b"1\n")
    ser.flush()

    # 4. Rekam Stream Data
    lines = []
    t0 = time.time()
    target_str = "60rpm" # default

    print("[*] Merekam data pengujian PI-PSO (durasi 8 detik)...")
    print("-" * 75)

    while time.time() - t0 < 8.0:
        line = ser.readline().decode('utf-8', errors='ignore')
        if line:
            print(line, end="")
            lines.append(line)
            if "TARGET_RPM" in line or "Target_RPM" in line:
                if "60" in line:
                    target_str = "60rpm"
                elif "80" in line:
                    target_str = "80rpm"
                elif "100" in line:
                    target_str = "100rpm"

    ser.close()
    print("-" * 75)

    # 5. Format Tanggal & Jam Pengujian
    now = datetime.datetime.now()
    timestamp_str = now.strftime("%Y%m%d_%H%M%S") # contoh: 20261109_182530
    jam_readable  = now.strftime("%d/%m/%Y Jam %H:%M:%S")

    # Direktori penyimpanan di Jetson
    output_dir = os.path.expanduser("~/deliver_amr")
    if not os.path.exists(output_dir):
        output_dir = "."

    # Simpan dengan TIMESTAMP (Lengkap dengan Tanggal & Jam)
    csv_timestamped = os.path.join(output_dir, f"step_respon_{target_str}_pso_{timestamp_str}.csv")
    csv_latest      = os.path.join(output_dir, f"step_respon_{target_str}_pso.csv")

    with open(csv_timestamped, "w", encoding="utf-8") as f:
        f.writelines(lines)

    # Buat juga salinan file latest agar mudah diakses
    shutil.copyfile(csv_timestamped, csv_latest)

    print(f"\n[OK] Data CSV dengan Jam berhasil disimpan ke:")
    print(f"     >> {csv_timestamped}")
    print(f"     (Waktu Pengujian: {jam_readable})")

    # 6. Buat Gambar Grafik Otomatis jika script plot tersedia
    script_dirs = [
        os.path.expanduser("~/deliver_amr/src/amr_simulation/scripts/plot_step_response_pso.py"),
        "plot_step_response_pso.py",
        os.path.join(os.path.dirname(__file__), "plot_step_response_pso.py")
    ]
    plot_script = next((s for s in script_dirs if os.path.exists(s)), None)
    
    if plot_script:
        print(f"\n[*] Membuat grafik otomatis menggunakan {plot_script}...")
        png_timestamped = os.path.join(output_dir, f"grafik_step_respon_{target_str}_pso_{timestamp_str}.png")
        os.system(f"python3 \"{plot_script}\" \"{csv_timestamped}\" --save \"{png_timestamped}\"")
        
        # Buat juga salinan grafik latest
        png_latest = os.path.join(output_dir, f"grafik_step_respon_{target_str}_pso.png")
        if os.path.exists(png_timestamped):
            shutil.copyfile(png_timestamped, png_latest)
            
        print("\n" + "=" * 75)
        print(f"[SELESAI] File pengujian {target_str.upper()} PSO tersimpan rapi:")
        print(f"1. CSV : {os.path.basename(csv_timestamped)}")
        print(f"2. PNG : {os.path.basename(png_timestamped)}")
        print(f"Download ke Laptop dengan PowerShell:")
        print(f"scp jetson@100.84.182.79:/home/jetson/deliver_amr/*{target_str}*pso* .")
        print("=" * 75)

if __name__ == "__main__":
    main()
