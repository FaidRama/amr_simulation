#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trial_error_step_test.py
Runner simpel untuk memicu, merekam, dan mengevaluasi pengujian step response robot AMR.
Cukup jalankan: python3 trial_error_step_test.py
"""

import os
import sys
import time

try:
    import serial
except ImportError:
    print("[!] Modul pyserial belum terinstall. Jalankan: pip3 install pyserial")
    sys.exit(1)

def main():
    # 1. Deteksi Port Otomatis
    port_candidates = ['/dev/ttyACM0', '/dev/arduino', '/dev/ttyACM1', '/dev/ttyUSB0']
    port = None
    for p in port_candidates:
        if os.path.exists(p):
            port = p
            break
            
    if not port:
        print("[!] Port Arduino tidak ditemukan di /dev/ttyACM0 maupun /dev/arduino.")
        print("[*] Pastikan kabel USB Arduino terhubung.")
        sys.exit(1)

    # 2. Buka Port Serial
    print(f"[*] Terhubung ke Arduino di {port} (115200 baud)...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"[!] Gagal membuka port {port}: {e}")
        print(f"[*] Jalankan: sudo chmod 666 {port}")
        sys.exit(1)

    time.sleep(2.0)
    ser.reset_input_buffer()

    # 3. Kirim Trigger '1'
    print("[*] Mengirim trigger '1' untuk memulai pengujian motor...")
    ser.write(b"1\n")
    ser.flush()

    # 4. Rekam Stream Data
    lines = []
    t0 = time.time()
    target_str = "80rpm"

    print("[*] Merekam data pengujian (durasi 8 detik)...")
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

    # 5. Simpan ke File CSV
    output_dir = os.path.expanduser("~/deliver_amr")
    if not os.path.exists(output_dir):
        output_dir = "."
    output_csv = os.path.join(output_dir, f"step_respon_{target_str}_trial_error.csv")

    with open(output_csv, "w", encoding="utf-8") as f:
        f.writelines(lines)

    print(f"\n[OK] Data berhasil disimpan ke: {output_csv}")

    # 6. Buat Gambar Grafik Otomatis jika script plot tersedia
    script_dirs = [
        os.path.expanduser("~/deliver_amr/src/amr_simulation/scripts/plot_step_response_trial_error.py"),
        "plot_step_response_trial_error.py",
        os.path.join(os.path.dirname(__file__), "plot_step_response_trial_error.py")
    ]
    plot_script = next((s for s in script_dirs if os.path.exists(s)), None)
    if plot_script:
        print(f"[*] Membuat grafik otomatis menggunakan {plot_script}...")
        os.system(f"python3 {plot_script} {output_csv}")

if __name__ == "__main__":
    main()
