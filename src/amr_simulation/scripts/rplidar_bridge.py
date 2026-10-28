#!/usr/bin/env python3
"""
Node RPLiDAR Bridge & Diagnostik Mandiri (AMR Delivery)
-------------------------------------------------------
Fungsi:
1. Memeriksa keberadaan port fisik /dev/rplidar atau /dev/ttyUSB0
2. Menjalankan driver rplidar_composition (C++) dan memantau koneksinya
3. Memberikan status real-time persis seperti arduino_bridge:
   - "✅ Berhasil terhubung ke RPLiDAR"
   - "📡 Paket LaserScan pertama diterima (x titik) -> Topik /scan aktif!"
   - Telemetri berkala (Hz, jarak rintangan terdekat depan/kiri/kanan)
   - Peringatan jika motor LiDAR macet atau kabel terputus
4. Cleanup bersih saat Ctrl+C (anti-hang)
"""

import os
import sys
import time
import math
import signal
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RPLidarBridgeNode(Node):
    def __init__(self):
        super().__init__('rplidar_bridge')

        # Parameter
        default_port = '/dev/rplidar' if os.path.exists('/dev/rplidar') else '/dev/ttyUSB0'
        self.declare_parameter('serial_port', default_port)
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('auto_launch_driver', True)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('serial_baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.auto_launch = bool(self.get_parameter('auto_launch_driver').value)

        self.get_logger().info(f"🔍 Memeriksa perangkat RPLiDAR di {self.port} ({self.baud} baud)...")

        # Cek ketersediaan port fisik
        self.check_hardware_port()

        # State Telemetri
        self.has_first_scan = False
        self.last_scan_time = None
        self.scan_count = 0
        self.last_diag_time = time.time()
        self.driver_proc = None

        # Jalankan driver rplidar_composition jika diminta
        if self.auto_launch:
            self.launch_rplidar_driver()

        # Subscriber ke topik /scan untuk monitoring kesehatan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        # Timer watchdog 1 Hz untuk memantau liveness data
        self.timer = self.create_timer(1.0, self.check_liveness)

    def check_hardware_port(self):
        if not os.path.exists(self.port):
            # Cek alternatif
            alt = '/dev/ttyUSB0' if self.port != '/dev/ttyUSB0' else '/dev/rplidar'
            if os.path.exists(alt):
                self.get_logger().warn(f"⚠️ Port {self.port} tidak ditemukan, beralih ke {alt}")
                self.port = alt
            else:
                self.get_logger().error(
                    f"❌ Port RPLiDAR ({self.port}) TIDAK DITEMUKAN! Pastikan kabel USB LiDAR tercolok."
                )
                return False

        # Cek izin akses
        if not os.access(self.port, os.R_OK | os.W_OK):
            self.get_logger().warn(
                f"⚠️ Tidak punya izin read/write ke {self.port}! Coba jalankan: sudo chmod 666 {self.port}"
            )
        else:
            self.get_logger().info(f"✅ Port serial {self.port} terdeteksi dan memiliki izin akses OK.")
        return True

    def reset_lidar_hardware(self):
        """Kirim frame STOP & RESET resmi Slamtec agar sisa stream biner dibersihkan."""
        try:
            import serial
            s = serial.Serial(self.port, self.baud, timeout=0.3)
            # 1. Hentikan pemindaian aktif (STOP: 0xA5 0x25)
            s.write(b'\xA5\x25')
            time.sleep(0.05)
            # 2. Reset mikroprosesor internal RPLiDAR (RESET: 0xA5 0x40)
            s.write(b'\xA5\x40')
            time.sleep(0.1)
            # 3. Kuras buffer serial
            s.reset_input_buffer()
            s.reset_output_buffer()
            s.close()
            time.sleep(0.4)
            self.get_logger().info("🔄 Pre-flight reset RPLiDAR OK (STOP + RESET frame terkirim, buffer bersih).")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Pre-flight reset RPLiDAR dilewati: {e}")

    def launch_rplidar_driver(self):
        """Jalankan rplidar_composition sebagai subprocess terkelola."""
        # Bersihkan hardware & buffer terlebih dahulu agar driver tidak timeout
        self.reset_lidar_hardware()

        cmd = [
            'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
            '--ros-args',
            '-p', f'serial_port:={self.port}',
            '-p', f'serial_baudrate:={self.baud}',
            '-p', f'frame_id:={self.frame_id}',
            '-p', 'inverted:=false',
            '-p', 'angle_compensate:=true'
        ]

        try:
            self.get_logger().info(f"🚀 Menjalankan driver rplidar_composition...")
            # Buat process group terpisah dan teruskan output ke terminal agar error/diagnostik terlihat jelas
            self.driver_proc = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid
            )
        except Exception as e:
            self.get_logger().error(f"❌ Gagal menjalankan rplidar_composition: {e}")
            self.driver_proc = None

    def on_scan(self, msg: LaserScan):
        now = time.time()
        self.scan_count += 1
        num_points = len(msg.ranges)

        if not self.has_first_scan:
            self.has_first_scan = True
            self.get_logger().info(
                f"📡 Paket LaserScan pertama diterima! ({num_points} titik, range: {msg.range_min:.2f}m - {msg.range_max:.2f}m) -> Topik /scan AKTIF!"
            )

        self.last_scan_time = now

        # Telemetri diagnostik setiap 2.5 detik
        if now - self.last_diag_time >= 2.5:
            dt = now - self.last_diag_time
            hz = self.scan_count / dt
            self.scan_count = 0
            self.last_diag_time = now

            # Hitung jarak rintangan di sektor Depan, Kiri, Kanan
            front_dist = self.get_sector_min(msg, -15, 15)
            left_dist = self.get_sector_min(msg, 75, 105)
            right_dist = self.get_sector_min(msg, -105, -75)

            f_str = f"{front_dist:.2f}m" if front_dist < 99.0 else ">10m"
            l_str = f"{left_dist:.2f}m" if left_dist < 99.0 else ">10m"
            r_str = f"{right_dist:.2f}m" if right_dist < 99.0 else ">10m"

            self.get_logger().info(
                f"🟢 RPLiDAR Sehat: {hz:.1f} Hz ({num_points} pts) | Depan: {f_str} | Kiri: {l_str} | Kanan: {r_str}"
            )

    def get_sector_min(self, msg: LaserScan, deg_min: float, deg_max: float) -> float:
        """Cari jarak rintangan terdekat dalam rentang sudut tertentu (derajat)."""
        rad_min = math.radians(deg_min)
        rad_max = math.radians(deg_max)

        min_val = 999.0
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Normalisasi sudut ke rentang -pi .. pi
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if rad_min <= angle <= rad_max:
                if r < min_val:
                    min_val = r
        return min_val

    def check_liveness(self):
        now = time.time()
        # Jika driver proses mati tak terduga
        if self.driver_proc and self.driver_proc.poll() is not None:
            err = self.driver_proc.stderr.read().decode('utf-8', errors='ignore') if self.driver_proc.stderr else ''
            self.get_logger().error(f"❌ Driver rplidar_composition berhenti (exit code {self.driver_proc.returncode})! Error: {err.strip()[:200]}")
            self.driver_proc = None

        # Jika sudah pernah dapat scan tapi terhenti > 2.5 detik
        if self.has_first_scan and self.last_scan_time and (now - self.last_scan_time > 2.5):
            self.get_logger().warn(
                f"⚠️ Tidak ada data /scan selama {(now - self.last_scan_time):.1f} detik! Cek putaran motor LiDAR."
            )

    def shutdown(self):
        if self.driver_proc and self.driver_proc.poll() is None:
            self.get_logger().info("🛑 Menghentikan driver RPLiDAR...")
            try:
                os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGTERM)
                self.driver_proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        try:
            import serial
            s = serial.Serial(self.port, self.baud, timeout=0.2)
            s.write(b'\xA5\x25')
            time.sleep(0.02)
            s.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
