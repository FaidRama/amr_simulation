#!/usr/bin/env python3
"""
logger_step_respon.py
Runner & Logger Serial untuk Uji Step Response Open-Loop Motor PG36.
Mendukung pengujian multi-step: 40 -> 100, 120, 140, 160, 180 PWM
serta input kustom base_pwm & step_pwm. Data disimpan dalam format CSV mentah (raw data).
"""

import os
import sys
import time
import glob
import argparse
import datetime
import subprocess

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


def find_arduino_port():
    """Mencari port serial Arduino yang tersedia secara otomatis."""
    # Prioritas symlink udev
    if os.path.exists('/dev/arduino'):
        return '/dev/arduino'

    # Cari port ACM atau USB
    candidate_ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if candidate_ports:
        return candidate_ports[0]

    # Menggunakan pyserial list_ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'arduino' in p.description.lower() or 'ttyacm' in p.device.lower() or 'ttyusb' in p.device.lower():
            return p.device

    if ports:
        return ports[0].device

    return '/dev/ttyACM0'


def run_single_step_test(ser, base_pwm, step_pwm, output_dir):
    """Menjalankan satu sesi uji step response dan menyimpan data ke CSV."""
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"step_response_base{base_pwm}_step{step_pwm}_{timestamp}.csv"
    filepath = os.path.join(output_dir, filename)

    print("\n" + "=" * 65)
    print(f"🚀 MEMULAI UJI STEP RESPONSE: Base={base_pwm} PWM -> Step={step_pwm} PWM")
    print(f"📁 Output File: {filepath}")
    print("=" * 65)

    # Bersihkan buffer serial
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Kirim perintah start dengan parameter ke Arduino
    cmd = f"{base_pwm},{step_pwm}\n"
    ser.write(cmd.encode('ascii'))
    ser.flush()

    data_count = 0
    test_finished = False

    with open(filepath, 'w', encoding='utf-8') as f:
        # Tulis header standar CSV
        f.write("t_sec,pwm_input,raw_rpm_L,raw_rpm_R,ticks_L,ticks_R\n")
        f.flush()

        start_time = time.time()
        timeout_sec = 10.0  # Uji normal berlangsung 6 detik

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Cetak info/komentar dari Arduino
                if line.startswith('#'):
                    print(f"\033[96m{line}\033[0m")
                    if "PENGUJIAN SELESAI" in line or "STANDBY" in line and data_count > 50:
                        test_finished = True
                        break
                    continue

                # Baris data CSV: t_sec,pwm,rpm_L,rpm_R,ticks_L,ticks_R
                parts = line.split(',')
                if len(parts) >= 4:
                    f.write(line + "\n")
                    f.flush()
                    data_count += 1
                    try:
                        t = float(parts[0])
                        pwm = int(parts[1])
                        rl = float(parts[2])
                        rr = float(parts[3])
                        # Print progress setiap 0.5 detik (setiap 25 baris)
                        if data_count % 25 == 0 or t >= 5.9:
                            print(f"  ⏱️ t={t:5.2f}s | PWM={pwm:3d} | RPM_L={rl:6.2f} | RPM_R={rr:6.2f}")
                    except ValueError:
                        pass

            if time.time() - start_time > timeout_sec:
                print("⚠️ Timeout: Tidak ada data baru dari Arduino.")
                break

    print(f"\n✅ Perekaman selesai: {data_count} baris data tersimpan di {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser(description="Runner & Logger Uji Step Response Motor AMR")
    parser.add_argument('--port', type=str, default=None, help="Port serial Arduino (contoh: /dev/ttyACM0)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--base', type=int, default=None, help="Baseline PWM (default: 40)")
    parser.add_argument('--step', type=int, default=None, help="Target Step PWM (contoh: 100, 120, 140, 160, 180)")
    parser.add_argument('--all', action='store_true', help="Uji semua step berturut-turut (100, 120, 140, 160, 180)")
    parser.add_argument('--output-dir', type=str, default='data_step_response', help="Direktori penyimpanan CSV")
    parser.add_argument('--no-plot', action='store_true', help="Jangan langsung jalankan plot setelah uji")
    args = parser.parse_args()

    if not HAS_SERIAL:
        print("❌ Paket pyserial belum terinstall!")
        print("💡 Install dengan: pip3 install pyserial atau sudo apt install python3-serial")
        sys.exit(1)

    port = args.port or find_arduino_port()
    baud = args.baud

    print("=" * 65)
    print("🤖 LOGGER STEP RESPONSE OPEN-LOOP (RAW DATA - MOTOR PG36)")
    print(f"🔌 Port Serial : {port} ({baud} baud)")
    print(f"📁 Folder Data : {os.path.abspath(args.output_dir)}")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)  # Menunggu reset Arduino selesai
        ser.reset_input_buffer()
    except Exception as e:
        print(f"❌ Gagal membuka port serial {port}: {e}")
        print("💡 Tips: Pastikan kabel USB terpasang dan berikan izin: sudo chmod 666 /dev/ttyACM*")
        sys.exit(1)

    recorded_files = []

    try:
        if args.all:
            # Mode uji berurutan untuk semua step: 100, 120, 140, 160, 180
            steps = [100, 120, 140, 160, 180]
            base = args.base if args.base is not None else 40
            print(f"\n🚀 Menjalankan Uji Multi-Step Berurutan: Baseline {base} -> {steps}")
            for s in steps:
                input(f"\n👉 Posisikan robot pada lintasan bebas, lalu tekan [ENTER] untuk uji Step {base} -> {s} PWM...")
                fpath = run_single_step_test(ser, base, s, args.output_dir)
                recorded_files.append(fpath)
                time.sleep(1.0)
        elif args.step is not None:
            base = args.base if args.base is not None else 40
            fpath = run_single_step_test(ser, base, args.step, args.output_dir)
            recorded_files.append(fpath)
        else:
            # Menu Interaktif
            while True:
                print("\n" + "-" * 55)
                print("📋 PILIH SKENARIO PENGUJIAN STEP RESPONSE:")
                print("-" * 55)
                print("  [1] Step 40 -> 100 PWM (Delta = +60 PWM)")
                print("  [2] Step 40 -> 120 PWM (Delta = +80 PWM)")
                print("  [3] Step 40 -> 140 PWM (Delta = +100 PWM - Standar)")
                print("  [4] Step 40 -> 160 PWM (Delta = +120 PWM)")
                print("  [5] Step 40 -> 180 PWM (Delta = +140 PWM)")
                print("  [6] Uji SEMUA step berurutan (100 -> 120 -> 140 -> 160 -> 180)")
                print("  [7] Custom Input (Tentukan sendiri Base & Step PWM)")
                print("  [0] Keluar")
                print("-" * 55)

                pilihan = input("Pilihan Anda [0-7]: ").strip()
                if pilihan == '0':
                    break
                elif pilihan == '1':
                    fpath = run_single_step_test(ser, 40, 100, args.output_dir)
                    recorded_files.append(fpath)
                elif pilihan == '2':
                    fpath = run_single_step_test(ser, 40, 120, args.output_dir)
                    recorded_files.append(fpath)
                elif pilihan == '3':
                    fpath = run_single_step_test(ser, 40, 140, args.output_dir)
                    recorded_files.append(fpath)
                elif pilihan == '4':
                    fpath = run_single_step_test(ser, 40, 160, args.output_dir)
                    recorded_files.append(fpath)
                elif pilihan == '5':
                    fpath = run_single_step_test(ser, 40, 180, args.output_dir)
                    recorded_files.append(fpath)
                elif pilihan == '6':
                    steps = [100, 120, 140, 160, 180]
                    for s in steps:
                        input(f"\n👉 Posisikan robot, lalu tekan [ENTER] untuk uji Step 40 -> {s} PWM...")
                        fpath = run_single_step_test(ser, 40, s, args.output_dir)
                        recorded_files.append(fpath)
                        time.sleep(1.0)
                elif pilihan == '7':
                    try:
                        b = int(input("Masukkan Base PWM (default 40): ").strip() or "40")
                        s = int(input("Masukkan Target Step PWM (contoh 150): ").strip())
                        fpath = run_single_step_test(ser, b, s, args.output_dir)
                        recorded_files.append(fpath)
                    except ValueError:
                        print("❌ Input angka tidak valid.")
                else:
                    print("⚠️ Pilihan tidak valid.")

                lanjut = input("\nIngin melakukan pengujian lagi? (y/n): ").strip().lower()
                if lanjut not in ['y', 'yes', 'ya']:
                    break

    except KeyboardInterrupt:
        print("\n🛑 Perekaman dihentikan oleh pengguna.")
        ser.write(b"0\n")
    finally:
        ser.close()

    # Opsi panggil visualisasi plot otomatis
    if recorded_files and not args.no_plot:
        plot_script = os.path.join(os.path.dirname(__file__), 'src/amr_simulation/scripts/plot_step_response.py')
        if not os.path.exists(plot_script):
            plot_script = 'src/amr_simulation/scripts/plot_step_response.py'

        if os.path.exists(plot_script):
            print("\n" + "=" * 65)
            print("📈 Menjalankan analisis dan plotting otomatis...")
            if len(recorded_files) == 1:
                cmd = [sys.executable, plot_script, recorded_files[0]]
            else:
                cmd = [sys.executable, plot_script, "--compare"] + recorded_files
            subprocess.run(cmd)


if __name__ == '__main__':
    main()
