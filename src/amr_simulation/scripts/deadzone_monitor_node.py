#!/usr/bin/env python3
"""
deadzone_monitor_node.py
ROS 2 Node untuk memantau & mengontrol pengujian Deadzone Motor dari Arduino Uno via Serial.

Fitur Kontrol:
  - Tekan [ENTER] di terminal node ini untuk MEMULAI pengujian.
  - Ketik '0' atau 's' lalu [ENTER] untuk STOP darurat.
  - Bisa juga di-trigger dari Laptop via topik:
      ros2 topic pub --once /motor_test/start std_msgs/msg/Empty
      ros2 topic pub --once /motor_test/stop std_msgs/msg/Empty

Topik yang di-publish:
  - /motor_test/pwm         (std_msgs/Int32)   : Nilai PWM yang sedang diuji (0 - 120)
  - /motor_test/rpm_left    (std_msgs/Float32) : RPM aktual motor kiri
  - /motor_test/rpm_right   (std_msgs/Float32) : RPM aktual motor kanan
  - /motor_test/telemetry   (std_msgs/String)  : String CSV lengkap untuk logging

Visualisasi di Laptop:
  ros2 run rqt_plot rqt_plot /motor_test/pwm /motor_test/rpm_left /motor_test/rpm_right
"""

import sys
import time
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

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value

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
            "   [STANDBY] Motor tidak akan berputar sebelum Anda beri perintah.\n"
            "   >>> Tekan [ENTER] di terminal ini untuk MEMULAI pengujian! <<<\n"
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

    def send_cmd(self, cmd_str: str):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f"Gagal mengirim perintah ke serial: {e}")

    def start_callback(self, msg: Empty):
        self.get_logger().info("🚀 Menerima perintah START via ROS 2! Mengirim '1' ke Arduino...")
        self.send_cmd("1\n")

    def stop_callback(self, msg: Empty):
        self.get_logger().warn("🛑 Menerima perintah STOP via ROS 2! Mengirim '0' ke Arduino...")
        self.send_cmd("0\n")

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
                else:
                    self.get_logger().info("🚀 Tombol ENTER ditekan! Mengirim MULAI ('1') ke Arduino...")
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
                        continue

                    # Parsing format CSV: Waktu_ms,PWM,Ticks_L,Ticks_R,RPM_L,RPM_R
                    parts = line.split(',')
                    if len(parts) == 6:
                        try:
                            pwm_val = int(parts[1])
                            rpm_l = float(parts[4])
                            rpm_r = float(parts[5])

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
