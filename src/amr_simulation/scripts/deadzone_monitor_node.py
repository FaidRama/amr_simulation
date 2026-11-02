#!/usr/bin/env python3
"""
deadzone_monitor_node.py
ROS 2 Node untuk memantau, mengontrol, dan MENYIMPAN OTOMATIS data pengujian
Deadzone Motor DC PG36 + BTS7960 ke file CSV.

Fitur:
  - Otomatis membuat file CSV ber-timestamp saat pengujian dimulai
  - Deteksi otomatis titik awal putaran (Deadzone Breakaway: RPM > 1.5)
  - Menyimpan data: Waktu_ms, PWM, Ticks_L, Ticks_R, RPM_L, RPM_R
  - Kontrol: Tekan [ENTER] di terminal untuk MULAI, ketik '0' untuk STOP
  - ROS 2 Topics: /motor_test/pwm, /motor_test/rpm_left, /motor_test/rpm_right
"""

import os
import sys
import time
import datetime
import threading
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Empty


class DeadzoneMonitorNode(Node):
    def __init__(self):
        super().__init__('deadzone_monitor_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('output_dir', os.path.expanduser('~/Documents/amr_ws/data_deadzone'))

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.output_dir = self.get_parameter('output_dir').value

        # Pastikan folder penyimpanan CSV ada
        os.makedirs(self.output_dir, exist_ok=True)

        # Publishers
        self.pub_pwm = self.create_publisher(Int32, '/motor_test/pwm', 10)
        self.pub_rpm_l = self.create_publisher(Float32, '/motor_test/rpm_left', 10)
        self.pub_rpm_r = self.create_publisher(Float32, '/motor_test/rpm_right', 10)
        self.pub_telemetry = self.create_publisher(String, '/motor_test/telemetry', 10)

        # Subscribers untuk trigger remote
        self.sub_start = self.create_subscription(
            Empty, '/motor_test/start', self.start_callback, 10
        )
        self.sub_stop = self.create_subscription(
            Empty, '/motor_test/stop', self.stop_callback, 10
        )

        # Status File CSV & Deteksi Deadzone
        self.csv_file = None
        self.csv_filepath = None
        self.deadzone_l = None
        self.deadzone_r = None
        self.test_active = False

        self.ser = None
        self.rx_buffer = ""
        self.connect_serial()

        # Timer loop pembacaan serial 50 Hz (20ms)
        self.timer = self.create_timer(0.02, self.read_serial_loop)

        # Thread untuk membaca input keyboard (Enter untuk start)
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info(
            f"✅ Deadzone Monitor aktif di {self.port} ({self.baud} baud)\n"
            f"   📁 Folder Simpan CSV : {self.output_dir}\n"
            "   [STANDBY] Motor tidak akan berputar sebelum Anda beri perintah.\n"
            "   >>> Tekan [ENTER] di terminal ini untuk MEMULAI pengujian & rekam CSV! <<<\n"
            "   >>> Atau kirim topik: ros2 topic pub --once /motor_test/start std_msgs/msg/Empty <<<\n"
        )

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            time.sleep(1.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Terhubung ke Arduino pada {self.port}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Menunggu koneksi port serial {self.port} ({e})")
            self.ser = None

    def start_new_csv(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"deadzone_test_{timestamp}.csv"
        self.csv_filepath = os.path.join(self.output_dir, filename)

        try:
            self.csv_file = open(self.csv_filepath, 'w', encoding='utf-8')
            # Header CSV
            self.csv_file.write("Waktu_ms,PWM,Ticks_L,Ticks_R,RPM_L,RPM_R\n")
            self.csv_file.flush()
            self.deadzone_l = None
            self.deadzone_r = None
            self.test_active = True
            self.get_logger().info(f"📄 Rekam CSV Baru Dimulai: {self.csv_filepath}")
        except Exception as e:
            self.get_logger().error(f"Gagal membuat file CSV: {e}")
            self.csv_file = None

    def close_csv(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.test_active = False

            # Cetak Rangkuman Indah
            print("\n" + "=" * 60)
            print("🎉 PENGUJIAN SELESAI & DATA TERSIMPAN DI CSV!")
            print(f"📁 Lokasi File : {self.csv_filepath}")
            print(f"👉 Deadzone Motor Kiri  : PWM = {self.deadzone_l if self.deadzone_l else 'Belum gerak'}")
            print(f"👉 Deadzone Motor Kanan : PWM = {self.deadzone_r if self.deadzone_r else 'Belum gerak'}")
            print("=" * 60 + "\n")

    def send_cmd(self, cmd_str: str):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f"Gagal mengirim perintah ke serial: {e}")

    def start_callback(self, msg: Empty):
        self.get_logger().info("🚀 Menerima perintah START via ROS 2! Mengirim '1' ke Arduino...")
        self.start_new_csv()
        self.send_cmd("1\n")

    def stop_callback(self, msg: Empty):
        self.get_logger().warn("🛑 Menerima perintah STOP via ROS 2! Mengirim '0' ke Arduino...")
        self.send_cmd("0\n")
        self.close_csv()

    def keyboard_loop(self):
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd in ['0', 'stop', 's']:
                    self.get_logger().warn("🛑 Mengirim STOP ke Arduino...")
                    self.send_cmd("0\n")
                    self.close_csv()
                else:
                    self.get_logger().info("🚀 Tombol ENTER ditekan! Mengirim MULAI ('1') ke Arduino...")
                    self.start_new_csv()
                    self.send_cmd("1\n")
            except Exception:
                break

    def read_serial_loop(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw_bytes = self.ser.read(waiting)
                self.rx_buffer += raw_bytes.decode('ascii', errors='ignore')

                while '\n' in self.rx_buffer:
                    line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    # Menampilkan pesan teks / log info dari Arduino
                    if line.startswith('#'):
                        print(f"\033[96m{line}\033[0m")
                        if "SELESAI" in line or "STOP" in line:
                            self.close_csv()
                        continue

                    # Parsing format CSV: Waktu_ms,PWM,Ticks_L,Ticks_R,RPM_L,RPM_R
                    parts = line.split(',')
                    if len(parts) == 6:
                        try:
                            pwm_val = int(parts[1])
                            ticks_l = int(parts[2])
                            ticks_r = int(parts[3])
                            rpm_l = float(parts[4])
                            rpm_r = float(parts[5])

                            # Deteksi titik Deadzone (RPM > 1.5 pertama kali)
                            if self.deadzone_l is None and rpm_l > 1.5:
                                self.deadzone_l = pwm_val
                                print(f"\033[92m🎯 [DEADZONE DETECTED] Motor Kiri mulai bergerak pada PWM = {pwm_val} (RPM={rpm_l:.1f})\033[0m")

                            if self.deadzone_r is None and rpm_r > 1.5:
                                self.deadzone_r = pwm_val
                                print(f"\033[92m🎯 [DEADZONE DETECTED] Motor Kanan mulai bergerak pada PWM = {pwm_val} (RPM={rpm_r:.1f})\033[0m")

                            # Tulis langsung ke file CSV jika pengujian aktif
                            if self.csv_file and not self.csv_file.closed:
                                self.csv_file.write(f"{line}\n")
                                self.csv_file.flush()

                            # Publish ke topik ROS 2
                            msg_pwm = Int32()
                            msg_pwm.data = pwm_val
                            self.pub_pwm.publish(msg_pwm)

                            msg_l = Float32()
                            msg_l.data = rpm_l
                            self.pub_rpm_l.publish(msg_l)

                            msg_r = Float32()
                            msg_r.data = rpm_r
                            self.pub_rpm_r.publish(msg_r)

                            msg_telem = String()
                            msg_telem.data = line
                            self.pub_telemetry.publish(msg_telem)

                        except ValueError:
                            pass
        except Exception as e:
            self.get_logger().error(f"Error membaca serial: {e}")
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = DeadzoneMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
