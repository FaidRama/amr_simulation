#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rekam_uji_60rpm.py
Script pemicu & perekam otomatis pengujian Step Response 60 RPM.
"""

import sys
import time
import os

try:
    import serial
except ImportError:
    print("[!] Modul pyserial belum terinstall. Jalankan: pip3 install pyserial")
    sys.exit(1)

PORT = "/dev/arduino"
BAUD = 115200
DURASI = 8.0  # detik
OUTPUT_CSV = os.path.expanduser("~/deliver_amr/step_respon_60rpm_trial_error.csv")

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = PORT

    print(f"[*] Menghubungkan ke {port} pada {BAUD} baud...")
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except Exception as e:
        print(f"[!] Gagal membuka port {port}: {e}")
        print("[*] Pastikan bridge sudah dimatikan: pkill -9 -f arduino_bridge.py")
        sys.exit(1)

    print("[*] Menunggu inisialisasi Arduino (2 detik)...")
    time.sleep(2.0)
    ser.reset_input_buffer()

    print("[*] Mengirim sinyal trigger '1' untuk memulai pengujian 60 RPM...")
    ser.write(b"1\n")
    ser.flush()

    print(f"[*] Merekam data ke: {OUTPUT_CSV}")
    print("-" * 75)

    t_start = time.time()
    baris_tersimpan = 0

    with open(OUTPUT_CSV, "w", encoding="utf-8") as f:
        while time.time() - t_start < DURASI:
            line = ser.readline().decode("utf-8", errors="ignore")
            if line:
                print(line, end="")
                f.write(line)
                f.flush()
                baris_tersimpan += 1

    ser.close()
    print("-" * 75)
    print(f"[OK] Pengujian selesai! {baris_tersimpan} baris berhasil disimpan di {OUTPUT_CSV}")

if __name__ == "__main__":
    main()
