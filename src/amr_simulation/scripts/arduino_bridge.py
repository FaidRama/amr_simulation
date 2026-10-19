#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge Node (Jetson AGX Xavier <-> Arduino Uno)
- Mengubah /cmd_vel (Twist) menjadi perintah PWM "<PWM_L>,<PWM_R>\n" ke Arduino
- Dilengkapi Smooth Acceleration Ramp & Heartbeat 10Hz agar gerakan halus (anti patah-patah)
- Membaca "ENC:<left>,<right>" dari Arduino dan mem-publish /odom + TF (odom -> base_link)
"""

import math
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
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
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_diameter', 0.133)      # meter (sesuai URDF: 13.3 cm)
        self.declare_parameter('wheel_separation', 0.442)    # meter (sesuai URDF: 44.2 cm)
        self.declare_parameter('ticks_per_rev', 537.0)       # PG36 rata-rata ~500-600 ticks/putaran roda
        self.declare_parameter('max_speed_mps', 0.5)         # Kecepatan pada PWM maksimal (m/s)
        self.declare_parameter('min_pwm', 45)                # Deadzone minimum PWM agar motor mulai berputar
        self.declare_parameter('max_pwm', 200)               # Batas aman PWM maksimal
        self.declare_parameter('cmd_vel_timeout', 2.5)       # detik (agar teleop keyboard tidak putus-putus)
        self.declare_parameter('pwm_ramp_step', 30)          # Akselerasi halus per 50ms
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.pwm_ramp_step = int(self.get_parameter('pwm_ramp_step').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- STATE ODOMETRY ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_odom_time = self.get_clock().now()
        self.rx_buffer = ""

        # --- STATE TARGET & CURRENT PWM ---
        self.target_pwm_l = 0
        self.target_pwm_r = 0
        self.current_pwm_l = 0
        self.current_pwm_r = 0
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_tx_time = self.get_clock().now()

        # --- INISIALISASI SERIAL ARDUINO ---
        self.ser = None
        self.connect_serial()

        # --- ROS 2 PUB / SUB / TF ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer 50 Hz untuk baca serial & menjaga heartbeat motor
        self.timer = self.create_timer(0.02, self.update_loop)
        self.get_logger().info(
            f"Arduino Bridge aktif di {self.port} ({self.baud} baud) | "
            f"D={self.wheel_diameter}m, L={self.wheel_separation}m, Timeout={self.cmd_vel_timeout}s"
        )

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            time.sleep(2.0)  # Tunggu bootloader Arduino Uno selesai auto-reset
            self.ser.reset_input_buffer()
            self.ser.write(b"0,0\n")
            self.get_logger().info(f"Berhasil terhubung ke Arduino di {self.port}")
        except Exception as e:
            self.get_logger().error(f"Gagal membuka port Arduino {self.port}: {e}")
            self.ser = None

    def speed_to_pwm(self, speed_mps: float) -> int:
        """Konversi kecepatan roda (m/s) ke nilai PWM (-255 s/d 255) dengan kompensasi deadzone."""
        if abs(speed_mps) < 0.01:
            return 0
        ratio = min(abs(speed_mps) / self.max_speed_mps, 1.0)
        pwm = int(self.min_pwm + ratio * (self.max_pwm - self.min_pwm))
        return pwm if speed_mps > 0 else -pwm

    @staticmethod
    def ramp_value(current: int, target: int, step: int, min_pwm: int) -> int:
        """Akselerasi halus menuju target, namun langsung berhenti instan jika target == 0."""
        if target == 0:
            return 0
        # Lompati deadzone saat baru mulai bergerak
        if current == 0 and abs(target) >= min_pwm:
            current = min_pwm if target > 0 else -min_pwm
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + step if diff > 0 else current - step

    def send_pwm(self, pwm_l: int, pwm_r: int, now):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            cmd_str = f"{pwm_l},{pwm_r}\n"
            self.ser.write(cmd_str.encode('ascii'))
            self.last_tx_time = now
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Kinematika Differential Drive
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        new_l = self.speed_to_pwm(v_left)
        new_r = self.speed_to_pwm(v_right)
        now = self.get_clock().now()
        self.last_cmd_vel_time = now

        # Jika berubah (terutama perintah STOP 'k'), langsung kirim instan
        if new_l != self.target_pwm_l or new_r != self.target_pwm_r:
            self.target_pwm_l = new_l
            self.target_pwm_r = new_r
            self.current_pwm_l = self.ramp_value(self.current_pwm_l, self.target_pwm_l, self.pwm_ramp_step, self.min_pwm)
            self.current_pwm_r = self.ramp_value(self.current_pwm_r, self.target_pwm_r, self.pwm_ramp_step, self.min_pwm)
            self.send_pwm(self.current_pwm_l, self.current_pwm_r, now)

    def update_loop(self):
        if self.ser is None or not self.ser.is_open:
            return

        now = self.get_clock().now()

        # 1. Timeout /cmd_vel: hentikan roda jika melebihi cmd_vel_timeout (default 2.5 detik)
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_vel_timeout:
            if self.target_pwm_l != 0 or self.target_pwm_r != 0:
                self.target_pwm_l = 0
                self.target_pwm_r = 0
                self.current_pwm_l = 0
                self.current_pwm_r = 0
                self.send_pwm(0, 0, now)

        # 2. Kirim Heartbeat PWM tiap 80ms (12.5 Hz) selama roda bergerak agar Watchdog Arduino (500ms) tidak memutus motor
        dt_tx = (now - self.last_tx_time).nanoseconds / 1e9
        if (self.target_pwm_l != 0 or self.target_pwm_r != 0) and dt_tx >= 0.08:
            self.current_pwm_l = self.ramp_value(self.current_pwm_l, self.target_pwm_l, self.pwm_ramp_step, self.min_pwm)
            self.current_pwm_r = self.ramp_value(self.current_pwm_r, self.target_pwm_r, self.pwm_ramp_step, self.min_pwm)
            self.send_pwm(self.current_pwm_l, self.current_pwm_r, now)

        # 3. Baca data encoder dari Arduino secara Non-Blocking menggunakan buffer
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
                            self.process_odometry(left_ticks, right_ticks, now)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def process_odometry(self, left_ticks: int, right_ticks: int, now):
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now
            self.get_logger().info(
                f"Paket Encoder pertama diterima (L={left_ticks}, R={right_ticks}) -> Topik /odom mulai dipublish!"
            )

        dt = (now - self.prev_odom_time).nanoseconds / 1e9
        if dt <= 0.0001:
            d_left = 0.0
            d_right = 0.0
            vx = 0.0
            wz = 0.0
        else:
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

            vx = d_center / dt
            wz = d_yaw / dt

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
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b"0,0\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
