#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge Node (Jetson AGX Xavier <-> Arduino Uno)
- Mengubah /cmd_vel (Twist) menjadi perintah PWM "<PWM_L>,<PWM_R>\n" ke Arduino
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
        self.declare_parameter('max_speed_mps', 0.5)         # Kecepatan pada PWM 255 (m/s)
        self.declare_parameter('min_pwm', 45)                # Deadzone minimum PWM agar motor mulai berputar
        self.declare_parameter('max_pwm', 220)               # Batas aman PWM maksimal
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

        # --- STATE TARGET KECEPATAN ---
        self.target_pwm_l = 0
        self.target_pwm_r = 0
        self.last_cmd_vel_time = self.get_clock().now()

        # --- INISIALISASI SERIAL ARDUINO ---
        self.ser = None
        self.connect_serial()

        # --- ROS 2 PUB / SUB / TF ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer 50 Hz untuk baca serial & kirim perintah motor
        self.timer = self.create_timer(0.02, self.update_loop)
        self.get_logger().info(
            f"Arduino Bridge aktif di {self.port} ({self.baud} baud) | "
            f"D={self.wheel_diameter}m, L={self.wheel_separation}m, Ticks/Rev={self.ticks_per_rev}"
        )

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            time.sleep(2.0)  # Tunggu bootloader Arduino Uno selesai auto-reset
            self.ser.reset_input_buffer()
            self.ser.write(b"RST\n")
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

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Kinematika Differential Drive
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        self.target_pwm_l = self.speed_to_pwm(v_left)
        self.target_pwm_r = self.speed_to_pwm(v_right)
        self.last_cmd_vel_time = self.get_clock().now()

    def update_loop(self):
        if self.ser is None or not self.ser.is_open:
            return

        now = self.get_clock().now()

        # 1. Timeout /cmd_vel: jika > 0.5 detik tidak ada perintah baru, hentikan roda
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt_cmd > 0.5:
            self.target_pwm_l = 0
            self.target_pwm_r = 0

        # 2. Kirim PWM ke Arduino
        try:
            cmd_str = f"{self.target_pwm_l},{self.target_pwm_r}\n"
            self.ser.write(cmd_str.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

        # 3. Baca data encoder dari Arduino ("ENC:<left>,<right>")
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
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
            return

        dt = (now - self.prev_odom_time).nanoseconds / 1e9
        if dt <= 0.0001:
            return

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
