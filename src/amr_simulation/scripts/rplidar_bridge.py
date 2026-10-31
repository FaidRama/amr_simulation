#!/usr/bin/env python3
"""
Node RPLiDAR Bridge & Diagnostik Mandiri (AMR Delivery)
-------------------------------------------------------
Fungsi:
1. Memeriksa keberadaan port fisik /dev/rplidar atau /dev/ttyUSB0
2. Melakukan Auto-Reset Hardware & USB (Setara Cabut-Colok Kabel secara Software):
   - Mengirim command protokol Slamtec STOP (\xA5\x25) & RESET (\xA5\x40)
   - Toggle pin DTR/RTS pada chip CP2102 untuk membebaskan buffer UART yang macet
   - Melakukan Kernel USB Reset (ioctl USBDEVFS_RESET) jika driver macet berulang
3. Menjalankan driver rplidar_composition (C++) langsung dan memantau koneksinya
4. Memberikan status real-time & cleanup bersih saat Ctrl+C / SIGTERM (anti-hang)
"""

import os
import sys
import glob
import time
import math
import fcntl
import signal
import serial
import threading
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Konstanta ioctl Linux untuk me-reset perangkat USB (setara cabut-colok fisik)
USBDEVFS_RESET = 21780  # 0x5514 ('U' << 8 | 20)


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
        self.last_driver_err = ""

        # Bersihkan proses rplidar_composition yatim (zombie) sebelum mulai agar port serial bebas
        if self.auto_launch:
            subprocess.run(['pkill', '-2', '-f', 'rplidar_composition'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.3)
            subprocess.run(['pkill', '-9', '-f', 'rplidar_composition'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.2)
            self.launch_rplidar_driver(force_usb_reset=False)

        # Subscriber ke topik /scan untuk monitoring kesehatan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        # Timer watchdog 1 Hz untuk memantau liveness data & auto-restart
        self.timer = self.create_timer(1.0, self.check_liveness)

    def check_hardware_port(self):
        if not os.path.exists(self.port):
            for cand in ['/dev/rplidar', '/dev/ttyUSB0', '/dev/ttyUSB1']:
                if os.path.exists(cand):
                    # Pastikan bukan port yang sama dengan /dev/arduino
                    try:
                        if os.path.exists('/dev/arduino') and os.path.realpath(cand) == os.path.realpath('/dev/arduino'):
                            continue
                    except Exception:
                        pass
                    self.get_logger().warn(f"⚠️ Port {self.port} tidak ditemukan, beralih ke {cand}")
                    self.port = cand
                    break
            else:
                self.get_logger().error(
                    f"❌ Port RPLiDAR ({self.port}) TIDAK DITEMUKAN! Pastikan kabel USB LiDAR tercolok."
                )
                return False

        if not os.access(self.port, os.R_OK | os.W_OK):
            self.get_logger().warn(
                f"⚠️ Tidak punya izin read/write ke {self.port}! Coba jalankan: sudo chmod 666 {self.port}"
            )
        else:
            self.get_logger().info(f"✅ Port serial {self.port} terdeteksi dan memiliki izin akses OK.")
        return True

    def reset_usb_device(self):
        """
        Reset port USB di level Kernel Linux (USBDEVFS_RESET).
        Efeknya 100% sama dengan mencabut dan mencolok kembali kabel USB LiDAR secara fisik.
        """
        try:
            real_tty = os.path.realpath(self.port)  # misal /dev/ttyUSB0
            tty_name = os.path.basename(real_tty)   # ttyUSB0
            sys_dev_path = f"/sys/class/tty/{tty_name}/device"
            if not os.path.exists(sys_dev_path):
                return False

            # Naik ke direktori parent USB device yang memiliki busnum & devnum
            cur = os.path.realpath(sys_dev_path)
            busnum_file = None
            devnum_file = None
            for _ in range(5):
                b_cand = os.path.join(cur, "busnum")
                d_cand = os.path.join(cur, "devnum")
                if os.path.exists(b_cand) and os.path.exists(d_cand):
                    busnum_file = b_cand
                    devnum_file = d_cand
                    break
                cur = os.path.dirname(cur)

            if not busnum_file or not devnum_file:
                return False

            with open(busnum_file, 'r') as f:
                busnum = int(f.read().strip())
            with open(devnum_file, 'r') as f:
                devnum = int(f.read().strip())

            usb_dev_path = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
            if not os.path.exists(usb_dev_path):
                return False

            self.get_logger().info(f"🔌 Melakukan Auto-Reset USB Kernel ({usb_dev_path}) seperti cabut-colok fisik...")
            fd = os.open(usb_dev_path, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            finally:
                os.close(fd)
            time.sleep(1.2)  # Tunggu enumerasi ulang /dev/ttyUSBx & symlink /dev/rplidar
            self.get_logger().info("✅ Auto-Reset USB RPLiDAR berhasil!")
            return True
        except Exception as e:
            self.get_logger().debug(f"Info USB reset: {e}")
            return False

    def hardware_reset_rplidar(self, force_usb_reset=False):
        """
        Reset hardware & protokol serial RPLiDAR A1/A2 agar tidak pernah hang/timeout:
        1. Jika diminta (setelah gagal koneksi), lakukan USBDEVFS_RESET (simulasi cabut-colok USB).
        2. Buka serial, kirim command STOP (\xA5\x25) dan RESET MCU (\xA5\x40), toggle DTR/RTS,
           dan kuras seluruh sampah byte scan lama dari buffer UART.
        """
        if force_usb_reset:
            self.reset_usb_device()

        if not self.check_hardware_port():
            return False

        try:
            with serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.2) as ser:
                # 1. Pastikan DTR=False agar motor RPLiDAR A1 diizinkan berputar oleh rangkaian kontrol CP2102
                ser.dtr = True
                ser.rts = True
                time.sleep(0.05)
                # 2. Kirim perintah STOP (\xA5\x25) dua kali untuk menghentikan stream liar
                ser.write(b'\xA5\x25')
                ser.flush()
                time.sleep(0.05)
                ser.write(b'\xA5\x25')
                ser.flush()
                time.sleep(0.05)
                # 3. Jika sedang recovery (restart_count > 0), kirim perintah REBOOT MCU LiDAR (\xA5\x40)
                if self.restart_count > 0 or force_usb_reset:
                    self.get_logger().info("🔄 Mengirim perintah Hardware Reboot (0xA5 0x40) ke MCU RPLiDAR...")
                    ser.write(b'\xA5\x40')
                    ser.flush()
                    time.sleep(0.6)
                # 4. Set DTR=False (kondisi normal operasi motor RPLiDAR A1) & bersihkan buffer
                ser.dtr = False
                ser.rts = False
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            time.sleep(0.25)
            return True
        except Exception as e:
            self.get_logger().warn(f"⚠️ Peringatan saat pre-reset serial RPLiDAR: {e}")
            return True

    def _consume_driver_stderr(self, proc):
        """Baca stderr driver secara asinkron agar pipe tidak pernah penuh dan tangkap pesan error."""
        try:
            for line in iter(proc.stderr.readline, b''):
                if not line:
                    break
                text = line.decode('utf-8', errors='ignore').strip()
                if text:
                    self.last_driver_err = text
                    if 'Error' in text or 'time out' in text.lower() or 'cannot' in text.lower():
                        self.get_logger().warn(f"[rplidar_node] {text}")
        except Exception:
            pass

    def launch_rplidar_driver(self, force_usb_reset=False):
        """Jalankan rplidar_composition sebagai subprocess terkelola setelah port di-reset bersih."""
        self.shutdown_driver_only()

        # Lakukan reset serial & (jika retry) reset USB otomatis
        if not self.hardware_reset_rplidar(force_usb_reset=force_usb_reset):
            self.driver_start_time = time.time()
            return

        # Gunakan executable langsung jika ada agar tidak terkena overhead 2 detik dari wrapper 'ros2 run'
        direct_bin = '/opt/ros/foxy/lib/rplidar_ros/rplidar_composition'
        if os.path.exists(direct_bin):
            cmd = [direct_bin]
        else:
            cmd = ['ros2', 'run', 'rplidar_ros', 'rplidar_composition']

        cmd += [
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
            self.last_driver_err = ""
            self.driver_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            t = threading.Thread(target=self._consume_driver_stderr, args=(self.driver_proc,), daemon=True)
            t.start()
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
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if rad_min <= angle <= rad_max:
                if r < min_val:
                    min_val = r
        return min_val

    def check_liveness(self):
        now = time.time()
        if not self.auto_launch:
            return

        # 1. Jika driver proses berhenti sendiri (misal RESULT_OPERATION_TIMEOUT)
        if self.driver_proc is None or self.driver_proc.poll() is not None:
            if self.driver_proc is not None:
                rc = self.driver_proc.returncode
                self.get_logger().warn(
                    f"⚠️ Driver rplidar_composition berhenti (code {rc}): {self.last_driver_err[:120]} -> Auto-recovery (Reset Serial & USB)..."
                )
                self.driver_proc = None
            if now - self.driver_start_time >= 1.5:
                self.restart_count += 1
                self.has_first_scan = False
                self.launch_rplidar_driver(force_usb_reset=True)
            return

        # 2. Jika sejak diluncurkan > 14.0 detik belum ada paket /scan pertama sama sekali
        # (Beri waktu 14 detik agar motor RPLiDAR sempat mencapai RPM stabil saat sistem sibuk)
        if not self.has_first_scan and (now - self.driver_start_time > 14.0):
            self.get_logger().warn("⚠️ Belum ada /scan setelah 14 detik -> Auto-recovery (Reset Serial & USB RPLiDAR)...")
            self.restart_count += 1
            self.launch_rplidar_driver(force_usb_reset=True)
            return

        # 3. Jika sudah pernah dapat scan tapi tiba-tiba terhenti > 5.0 detik
        if self.has_first_scan and self.last_scan_time and (now - self.last_scan_time > 5.0):
            self.get_logger().warn(
                f"⚠️ Aliran /scan terhenti selama {(now - self.last_scan_time):.1f} detik! Me-reset & menjalankan ulang RPLiDAR..."
            )
            self.has_first_scan = False
            self.restart_count += 1
            self.launch_rplidar_driver(force_usb_reset=True)

    def shutdown_driver_only(self):
        if self.driver_proc and self.driver_proc.poll() is None:
            try:
                # Kirim SIGINT terlebih dahulu agar rplidar_composition mengirim command STOP motor ke LiDAR
                os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGINT)
                self.driver_proc.wait(timeout=1.2)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGTERM)
                    self.driver_proc.wait(timeout=0.8)
                except Exception:
                    try:
                        os.killpg(os.getpgid(self.driver_proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
        self.driver_proc = None

    def shutdown(self):
        if self.driver_proc and self.driver_proc.poll() is None:
            self.get_logger().info("🛑 Menghentikan driver RPLiDAR dengan aman...")
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
