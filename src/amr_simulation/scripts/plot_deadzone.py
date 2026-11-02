#!/usr/bin/env python3
"""
plot_deadzone.py
Script visualisasi grafik kurva Deadzone Motor DC PG36 dari file CSV hasil pengujian.
Menghasilkan grafik publikasi (PNG) dan menampilkan jendela plot interaktif.

Penggunaan:
  python3 plot_deadzone.py                    # Otomatis mengambil file CSV terbaru
  python3 plot_deadzone.py path/ke/file.csv   # Membuka file tertentu
"""

import os
import sys
import glob
import csv
import matplotlib.pyplot as plt


def get_latest_csv(folder):
    files = glob.glob(os.path.join(folder, "deadzone_test_*.csv"))
    if not files:
        return None
    return max(files, key=os.path.getctime)


def main():
    candidate_dirs = [
        os.path.join(os.getcwd(), 'data_deadzone'),
        os.path.expanduser('~/deliver_amr/data_deadzone'),
        os.path.expanduser('~/Documents/amr_ws/data_deadzone')
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
        print("❌ File CSV tidak ditemukan di direktori data_deadzone!")
        print("Jalankan pengujian terlebih dahulu dengan deadzone_monitor_node.py.")
        return

    print(f"📊 Membaca data dari: {csv_path}")

    pwms = []
    rpm_l = []
    rpm_r = []

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 6:
                continue
            try:
                p = int(row[1])
                rl = float(row[4])
                rr = float(row[5])
                pwms.append(p)
                rpm_l.append(rl)
                rpm_r.append(rr)
            except ValueError:
                continue

    if not pwms:
        print("⚠️ File CSV tidak memiliki data yang valid.")
        return

    # Cari Deadzone (RPM > 1.5 pertama kali)
    deadzone_l = None
    deadzone_r = None

    for p, rl in zip(pwms, rpm_l):
        if deadzone_l is None and rl > 1.5:
            deadzone_l = p
            break

    for p, rr in zip(pwms, rpm_r):
        if deadzone_r is None and rr > 1.5:
            deadzone_r = p
            break

    # Setup Plot
    plt.figure(figsize=(10, 6))
    plt.plot(pwms, rpm_l, 'b.-', label='RPM Motor Kiri', linewidth=1.5, markersize=4)
    plt.plot(pwms, rpm_r, 'r.-', label='RPM Motor Kanan', linewidth=1.5, markersize=4)

    # Anotasi Deadzone Kiri
    if deadzone_l:
        plt.axvline(x=deadzone_l, color='blue', linestyle='--', alpha=0.7, label=f'Deadzone Kiri (PWM={deadzone_l})')
        plt.scatter([deadzone_l], [1.5], color='blue', s=80, zorder=5)

    # Anotasi Deadzone Kanan
    if deadzone_r:
        plt.axvline(x=deadzone_r, color='red', linestyle='--', alpha=0.7, label=f'Deadzone Kanan (PWM={deadzone_r})')
        plt.scatter([deadzone_r], [1.5], color='red', s=80, zorder=5)

    plt.title('Kurva Karakteristik Deadzone Motor PG36 + BTS7960 (Beban Lantai)', fontsize=13, fontweight='bold', pad=12)
    plt.xlabel('Nilai Input PWM (0 - 120)', fontsize=11)
    plt.ylabel('Kecepatan Putar Aktual (RPM)', fontsize=11)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(fontsize=10, loc='upper left')

    # Simpan Gambar PNG untuk Laporan / Skripsi
    output_png = csv_path.replace('.csv', '.png')
    plt.savefig(output_png, dpi=300, bbox_inches='tight')
    print(f"✅ Gambar grafik berhasil disimpan: {output_png}")

    # Tampilkan di Layar
    try:
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
