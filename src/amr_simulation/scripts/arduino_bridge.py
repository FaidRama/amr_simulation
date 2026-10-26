#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge Node (Jetson AGX Xavier <-> Arduino Uno)
- Menerima /cmd_vel (Twist) dari Teleop / Nav2
- Mengirim perintah kecepatan linear dan angular "V,<v>,<w>\n" ke Arduino Uno
  (Arduino melakukan Inverse Kinematics & Kendali PID-PSO kecepatan roda secara internal)
- Watchdog & Heartbeat 20Hz (tiap 50ms) agar pergerakan motor halus dan aman
- Membaca stream "ENC:<left>,<right>" dari Arduino dan mem-publish /odom + TF (odom -> base_link)
- Auto-reconnect otomatis jika koneksi USB serial terputus / izin port baru diberikan
- Siar TF kontinu pada 50Hz agar SLAM & RViz tidak pernah drop laser message
"""

import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw: float):
    """Konversi sudut yaw (rad) ke quaternion (x, y, z, w)."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # --- PARAMETER KONFIGURASI ROBOT ---
        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_diameter', 0.127)      # 5 inci = 0.127 meter
        self.declare_parameter('wheel_separation', 0.458)    # 45.8 cm = 0.458 meter
        self.declare_parameter('ticks_per_rev', 350.7)       # 7 PPR * 50.1 gearbox = 350.7 ticks/rev
        self.declare_parameter('cmd_vel_timeout', 1.0)       # Timeout 1.0s (halus, tidak getar saat Nav2 menghitung)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- STATE ODOMETRY ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_odom_time = self.get_clock().now()
        self.rx_buffer = ""

        # --- STATE TARGET KECEPATAN (v: m/s, w: rad/s) ---
        self.target_v = 0.0
        self.target_w = 0.0
        self.last_sent_v = None
        self.last_sent_w = None
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_tx_time = self.get_clock().now()
        self.last_reconnect_time = self.get_clock().now()

        # --- INISIALISASI SERIAL ARDUINO ---
        self.ser = None
        self.connect_serial()

        # --- ROS 2 PUB / SUB / TF ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pid_left_pub = self.create_publisher(Point, '/pid/motor_left', 10)
        self.pid_right_pub = self.create_publisher(Point, '/pid/motor_right', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Telemetri PID (Setpoint vs Aktual RPM)
        self.setpoint_rpm_l = 0.0
        self.setpoint_rpm_r = 0.0
        self.actual_rpm_l = 0.0
        self.actual_rpm_r = 0.0
        self.pwm_l = 0.0
        self.pwm_r = 0.0

        # Timer 50 Hz (tiap 20ms) untuk pembacaan serial, TF broadcast & heartbeat
        self.timer = self.create_timer(0.02, self.update_loop)
        self.get_logger().info(
            f"Arduino Bridge PID aktif di {self.port} ({self.baud} baud) | "
            f"D={self.wheel_diameter}m, L={self.wheel_separation}m, CPR={self.ticks_per_rev}"
        )

    def compute_setpoint_rpm(self, v: float, w: float):
        """Hitung target RPM motor kiri dan kanan dari linear v dan angular w."""
        v_l = v - (self.wheel_separation / 2.0) * w
        v_r = v + (self.wheel_separation / 2.0) * w
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0
        return rpm_l, rpm_r

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            # 1. Hardware auto-reset via DTR/RTS pulse (persis seperti saat kabel dicopot-pasang)
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = True
            self.ser.rts = True
            # 2. Beri waktu bootloader Optiboot Arduino Uno selesai (~1.6 detik)
            time.sleep(1.6)
            # 3. Bersihkan buffer sisa bootloader & kirim heartbeat nol
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b"V,0.0,0.0\n")
            self.get_logger().info(f"✅ Berhasil terhubung ke Arduino di {self.port} (DTR reset OK)")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Menunggu koneksi port Arduino {self.port} ({e})")
            self.ser = None

    def send_cmd(self, v: float, w: float, now):
        """Kirim perintah kecepatan 'V,<v>,<w>\n' ke Arduino."""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            cmd_str = f"V,{v:.3f},{w:.3f}\n"
            self.ser.write(cmd_str.encode('ascii'))
            self.last_sent_v = v
            self.last_sent_w = w
            self.last_tx_time = now
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        now = self.get_clock().now()

        self.last_cmd_vel_time = now
        self.target_v = v
        self.target_w = w
        self.setpoint_rpm_l, self.setpoint_rpm_r = self.compute_setpoint_rpm(v, w)

        # Kirim langsung jika ada perubahan nilai kecepatan agar respon instan
        if self.last_sent_v is None or abs(v - self.last_sent_v) > 0.005 or abs(w - self.last_sent_w) > 0.005:
            self.send_cmd(self.target_v, self.target_w, now)

    def update_loop(self):
        now = self.get_clock().now()

        # 0. Auto-reconnect jika serial belum terbuka / terputus
        if self.ser is None or not self.ser.is_open:
            dt_reconnect = (now - self.last_reconnect_time).nanoseconds / 1e9
            if dt_reconnect >= 2.0:
                self.last_reconnect_time = now
                self.connect_serial()
            # Tetap broadcast TF saat disconnected agar TF tree tidak broken
            self.broadcast_tf_and_odom(now)
            return

        # 1. Timeout /cmd_vel: jika > cmd_vel_timeout tidak ada perintah baru, otomatis kirim stop
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        dt_tx = (now - self.last_tx_time).nanoseconds / 1e9

        if dt_cmd > self.cmd_vel_timeout:
            if self.target_v != 0.0 or self.target_w != 0.0:
                self.target_v = 0.0
                self.target_w = 0.0
                self.setpoint_rpm_l = 0.0
                self.setpoint_rpm_r = 0.0
                self.send_cmd(0.0, 0.0, now)
            elif dt_tx >= 0.2:
                # Kirim heartbeat stop secara berkala tiap 200ms agar Arduino selalu yakin kecepatan = 0
                self.send_cmd(0.0, 0.0, now)

        # 2. Heartbeat: selama robot bergerak, kirim ulang perintah kecepatan tiap 50ms (20 Hz)
        elif (self.target_v != 0.0 or self.target_w != 0.0) and dt_tx >= 0.05:
            self.send_cmd(self.target_v, self.target_w, now)

        # 3. Baca serial non-blocking dari Arduino
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw_bytes = self.ser.read(waiting)
                self.rx_buffer += raw_bytes.decode('ascii', errors='ignore')

                while '\n' in self.rx_buffer:
                    line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                    line = line.strip()
                    if line.startswith("ENC:"):
                        parts = line[4:].split(",")
                        if len(parts) == 2:
                            left_ticks = int(parts[0])
                            right_ticks = int(parts[1])
                            self.process_encoder_ticks(left_ticks, right_ticks, now)
                    elif line.startswith("LOG:"):
                        parts = line[4:].split(",")
                        if len(parts) >= 5:
                            try:
                                self.actual_rpm_l = float(parts[1])
                                self.actual_rpm_r = float(parts[2])
                                self.pwm_l = float(parts[3])
                                self.pwm_r = float(parts[4])

                                # Publish Telemetri PID: x=Setpoint RPM, y=Actual RPM, z=PWM
                                pt_l = Point(x=float(self.setpoint_rpm_l), y=self.actual_rpm_l, z=self.pwm_l)
                                pt_r = Point(x=float(self.setpoint_rpm_r), y=self.actual_rpm_r, z=self.pwm_r)
                                self.pid_left_pub.publish(pt_l)
                                self.pid_right_pub.publish(pt_r)
                            except ValueError:
                                pass
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            self.ser = None

        # 4. Kontinu broadcast TF & Odometry pada 50Hz (mencegah SLAM/RViz drop message)
        self.broadcast_tf_and_odom(now)

    def process_encoder_ticks(self, left_ticks: int, right_ticks: int, now):
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now
            self.get_logger().info(
                f"Paket Encoder pertama diterima (L={left_ticks}, R={right_ticks}) -> Topik /odom aktif!"
            )
            return

        dt = (now - self.prev_odom_time).nanoseconds / 1e9
        if dt > 0.0001:
            d_left = (left_ticks - self.prev_left_ticks) * self.meters_per_tick
            d_right = (right_ticks - self.prev_right_ticks) * self.meters_per_tick
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now

            # Jarak rata-rata dan perubahan sudut orientasi (yaw)
            d_center = (d_right + d_left) / 2.0
            d_yaw = (d_right - d_left) / self.wheel_separation

            # Integrasi Runge-Kutta orde 2 (Midpoint)
            mid_yaw = self.yaw + (d_yaw / 2.0)
            self.x += d_center * math.cos(mid_yaw)
            self.y += d_center * math.sin(mid_yaw)
            self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

            self.vx = d_center / dt
            self.wz = d_yaw / dt

    def broadcast_tf_and_odom(self, now):
        qx, qy, qz, qw = euler_to_quaternion(self.yaw)
        stamp = now.to_msg()

        # Broadcast TF: odom -> base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # Publish /odom message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)


import signal

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    def emergency_stop_handler(sig, frame):
        if node.ser and node.ser.is_open:
            try:
                for _ in range(3):
                    node.ser.write(b"V,0.000,0.000\n")
                    node.ser.flush()
                    time.sleep(0.02)
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGINT, emergency_stop_handler)
    signal.signal(signal.SIGTERM, emergency_stop_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.ser and node.ser.is_open:
            try:
                for _ in range(3):
                    node.ser.write(b"V,0.000,0.000\n")
                    node.ser.flush()
                node.ser.close()
            except Exception:
                pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
