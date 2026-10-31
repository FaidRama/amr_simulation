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
        self.declare_parameter('quiet', False)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('serial_baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.auto_launch = bool(self.get_parameter('auto_launch_driver').value)
        self.quiet = bool(self.get_parameter('quiet').value)

        self.get_logger().info(f"🔍 Memeriksa perangkat RPLiDAR di {self.port} ({self.baud} baud)...")

        # Cek ketersediaan port fisik
        self.check_hardware_port()

        # State Telemetri
        self.has_first_scan = False
        self.last_scan_time = None
        self.scan_count = 0
        self.last_diag_time = time.time()
        self.driver_proc = None
        self.driver_start_time = 0.0
        self.restart_count = 0

        # Bersihkan proses rplidar_composition yatim (zombie) sebelum mulai agar port serial bebas
        if self.auto_launch:
            subprocess.run(['pkill', '-9', '-f', 'rplidar_composition'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.3)
            self.launch_rplidar_driver()

        # Subscriber ke topik /scan untuk monitoring kesehatan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        # Timer watchdog 1 Hz untuk memantau liveness data & auto-restart
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

    def launch_rplidar_driver(self):
        """Jalankan rplidar_composition sebagai subprocess terkelola."""
        self.shutdown_driver_only()
        if not self.check_hardware_port():
            self.driver_start_time = time.time()
            return

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
            self.get_logger().info("🚀 Menjalankan driver rplidar_composition...")
            self.driver_start_time = time.time()
            # Buat process group terpisah agar Ctrl+C bisa membunuh driver dengan rapi
            self.driver_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
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
            self.restart_count = 0
            self.get_logger().info(
                f"📡 Paket LaserScan pertama diterima! ({num_points} titik, range: {msg.range_min:.2f}m - {msg.range_max:.2f}m) -> Topik /scan AKTIF!"
            )

        self.last_scan_time = now

        # Telemetri diagnostik setiap 2.5 detik (hanya ditampilkan jika mode quiet=False)
        if not self.quiet and (now - self.last_diag_time >= 2.5):
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
        if not self.auto_launch:
            return

        # 1. Jika driver proses mati tak terduga (misal RESULT_OPERATION_TIMEOUT saat motor baru mulai berputar)
        if self.driver_proc is None or self.driver_proc.poll() is not None:
            if self.driver_proc is not None:
                err = self.driver_proc.stderr.read().decode('utf-8', errors='ignore') if self.driver_proc.stderr else ''
                self.get_logger().warn(
                    f"⚠️ Driver rplidar_composition berhenti (code {self.driver_proc.returncode}): {err.strip()[:120]} -> Auto-restart..."
                )
                self.driver_proc = None
            if now - self.driver_start_time >= 1.5:
                self.restart_count += 1
                self.has_first_scan = False
                self.launch_rplidar_driver()
            return

        # 2. Jika sejak diluncurkan > 6.0 detik belum ada paket /scan pertama sama sekali
        if not self.has_first_scan and (now - self.driver_start_time > 6.0):
            self.get_logger().warn("⚠️ Belum ada /scan setelah 6 detik -> Me-restart driver RPLiDAR...")
            self.restart_count += 1
            self.launch_rplidar_driver()
            return

        # 3. Jika sudah pernah dapat scan tapi terhenti > 4.0 detik
        if self.has_first_scan and self.last_scan_time and (now - self.last_scan_time > 4.0):
            self.get_logger().warn(
                f"⚠️ Tidak ada data /scan selama {(now - self.last_scan_time):.1f} detik! Me-restart driver RPLiDAR..."
            )
            self.has_first_scan = False
            self.restart_count += 1
            self.launch_rplidar_driver()

    def shutdown_driver_only(self):
        if self.driver_proc and self.driver_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGTERM)
                self.driver_proc.wait(timeout=1.5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.driver_proc = None

    def shutdown(self):
        if self.driver_proc and self.driver_proc.poll() is None:
            self.get_logger().info("🛑 Menghentikan driver RPLiDAR...")
        self.shutdown_driver_only()


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarBridgeNode()

    def _sigterm_handler(signum, frame):
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
