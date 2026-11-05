#!/usr/bin/env python3
"""
step_response_monitor_node.py
ROS 2 Node untuk mengontrol dan menyimpan data pengujian Step Response Open-Loop
Motor DC PG36 + BTS7960 secara otomatis ke file CSV (Raw Encoder Data - Tanpa Filter EMA).

Skenario Pengujian:
  - 0-1s : Diam (PWM 0)
  - 1-3s : Baseline konstan (PWM Base, default 40)
  - 3-6s : Step Jump (PWM Step, default 140 atau pilihan: 100, 120, 140, 160, 180)
  - > 6s : Selesai & Motor Stop Otomatis

Output File:
  - data_step_response/step_response_base{base}_step{step}_YYYYMMDD_HHMMSS.csv
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


class StepResponseMonitorNode(Node):
    def __init__(self):
        super().__init__('step_response_monitor_node')

        default_output = os.path.join(os.getcwd(), 'data_step_response')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('base_pwm', 40)
        self.declare_parameter('step_pwm', 140)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.output_dir = self.get_parameter('output_dir').value
        self.base_pwm = int(self.get_parameter('base_pwm').value)
        self.step_pwm = int(self.get_parameter('step_pwm').value)

        # Buat folder jika belum ada
        os.makedirs(self.output_dir, exist_ok=True)

        # Publishers
        self.pub_pwm = self.create_publisher(Int32, '/step_test/pwm', 10)
        self.pub_rpm_l = self.create_publisher(Float32, '/step_test/rpm_left', 10)
        self.pub_rpm_r = self.create_publisher(Float32, '/step_test/rpm_right', 10)
        self.pub_telem = self.create_publisher(String, '/step_test/telemetry', 10)

        # Subscribers untuk remote trigger
        self.sub_start = self.create_subscription(
            Empty, '/motor_test/start', self.start_callback, 10
        )
        self.sub_stop = self.create_subscription(
            Empty, '/motor_test/stop', self.stop_callback, 10
        )

        self.csv_file = None
        self.csv_filepath = None
        self.test_active = False

        self.ser = None
        self.rx_buffer = ""
        self.connect_serial()

        # Timer pembacaan serial 50 Hz
        self.timer = self.create_timer(0.01, self.read_serial_loop)

        # Thread keyboard input
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info(
            f"✅ Step Response Monitor aktif di {self.port} ({self.baud} baud)\n"
            f"   📁 Folder Simpan CSV : {self.output_dir}\n"
            f"   ⚙️ Setting Default   : Base={self.base_pwm} PWM, Step={self.step_pwm} PWM\n"
            "   [STANDBY] Motor diam aman. Siapkan lintasan bebas!\n"
            "   >>> Tekan [ENTER] untuk mulai dengan setting saat ini <<<\n"
            "   >>> Atau ketik nilai Step (100, 120, 140, 160, 180) lalu [ENTER] <<<\n"
            "   >>> Atau ketik 'base,step' misal '40,160' lalu [ENTER] <<<\n"
            "   >>> Ketik '0' atau 's' untuk STOP DARURAT <<<\n"
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

    def start_new_csv(self, base_val=None, step_val=None):
        if base_val is not None:
            self.base_pwm = base_val
        if step_val is not None:
            self.step_pwm = step_val

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"step_response_base{self.base_pwm}_step{self.step_pwm}_{timestamp}.csv"
        self.csv_filepath = os.path.join(self.output_dir, filename)

        try:
            self.csv_file = open(self.csv_filepath, 'w', encoding='utf-8')
            self.csv_file.write("t_sec,pwm_input,raw_rpm_L,raw_rpm_R,ticks_L,ticks_R\n")
            self.csv_file.flush()
            self.test_active = True
            self.get_logger().info(f"📄 Rekam CSV Baru Dimulai: {self.csv_filepath} (Base={self.base_pwm}, Step={self.step_pwm})")
        except Exception as e:
            self.get_logger().error(f"Gagal membuat file CSV: {e}")
            self.csv_file = None

    def close_csv(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.test_active = False

            print("\n" + "=" * 65)
            print("🎉 PENGUJIAN STEP RESPONSE SELESAI & TERSIMPAN DI CSV!")
            print(f"📁 Lokasi File : {self.csv_filepath}")
            print("🚀 Siap dihitung parameter K dan tau untuk Fungsi Alih G(s)!")
            print("=" * 65 + "\n")

    def send_cmd(self, cmd_str: str):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f"Gagal mengirim perintah: {e}")

    def start_callback(self, msg: Empty):
        self.get_logger().info(f"🚀 Menerima START via ROS 2! Base={self.base_pwm}, Step={self.step_pwm}...")
        self.start_new_csv()
        self.send_cmd(f"{self.base_pwm},{self.step_pwm}\n")

    def stop_callback(self, msg: Empty):
        self.get_logger().warn("🛑 Menerima STOP via ROS 2! Mengirim '0' ke Arduino...")
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
                    # Parsing input user jika memasukkan angka step tertentu
                    target_base = self.base_pwm
                    target_step = self.step_pwm

                    # Cek jika format "base,step" atau "base step"
                    clean_cmd = cmd.replace(',', ' ').replace(';', ' ')
                    parts = [p for p in clean_cmd.split() if p.isdigit()]

                    if len(parts) >= 2:
                        target_base = int(parts[0])
                        target_step = int(parts[1])
                    elif len(parts) == 1:
                        val = int(parts[0])
                        presets = {2: 100, 3: 120, 4: 140, 5: 160, 6: 180}
                        if val in presets:
                            target_step = presets[val]
                        elif val >= 50:
                            target_step = val
                        elif val == 1:
                            target_step = self.step_pwm

                    self.get_logger().info(f"🚀 Memulai Uji: Base={target_base} PWM -> Step={target_step} PWM...")
                    self.start_new_csv(target_base, target_step)
                    self.send_cmd(f"{target_base},{target_step}\n")
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

                    # Tangani komentar info dari Arduino
                    if line.startswith('#'):
                        print(f"\033[96m{line}\033[0m")
                        if "PENGUJIAN SELESAI" in line or "[STOP]" in line:
                            self.close_csv()
                        continue

                    # Format data: t_sec,pwm_input,raw_rpm_L,raw_rpm_R,ticks_L,ticks_R
                    parts = line.split(',')
                    if len(parts) >= 4:
                        try:
                            t_sec = float(parts[0])
                            pwm_val = int(parts[1])
                            rpm_l = float(parts[2])
                            rpm_r = float(parts[3])

                            # Tulis langsung ke file CSV
                            if self.csv_file and not self.csv_file.closed:
                                self.csv_file.write(f"{line}\n")
                                self.csv_file.flush()

                            # Publish ke ROS 2
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
                            self.pub_telem.publish(msg_telem)

                        except ValueError:
                            pass
        except Exception as e:
            self.get_logger().error(f"Error membaca serial: {e}")
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = StepResponseMonitorNode()
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
