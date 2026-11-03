#!/usr/bin/env python3
"""
ROS 2 Arduino Serial Bridge Node (Jetson AGX Xavier <-> Arduino Uno)
- Menerima /cmd_vel (Twist) dari Teleop / Nav2
- Mengirim perintah kecepatan linear dan angular "V,<v>,<w>\n" ke Arduino Uno
  (Arduino melakukan Inverse Kinematics & Kendali PID-PSO kecepatan roda secara internal)
- Watchdog & Heartbeat 20Hz (tiap 50ms) agar pergerakan motor halus dan aman
- Membaca stream "ENC:<left>,<right>" dari Arduino dan mem-publish /odom + TF (odom -> base_link)
- Auto-reconnect otomatis jika koneksi USB serial terputus / dicabut-colok manual
- Siar TF kontinu pada 50Hz agar SLAM & RViz tidak pernah drop laser message
"""

import os
import sys
import math
import time
import fcntl
import signal
import subprocess
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

# Konstanta ioctl Linux untuk me-reset perangkat USB
USBDEVFS_RESET = 21780  # 0x5514 ('U' << 8 | 20)


def kill_other_instances(process_pattern: str):
    """Bunuh proses lain yang cocok dengan pattern kecuali PID proses ini sendiri dan parent-nya."""
    my_pid = os.getpid()
    my_ppid = os.getppid()
    try:
        res = subprocess.run(['pgrep', '-f', process_pattern], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        for line in res.stdout.splitlines():
            line = line.strip()
            if not line.isdigit():
                continue
            pid = int(line)
            if pid not in (my_pid, my_ppid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
    except Exception:
        pass


def euler_to_quaternion(yaw: float):
    """Konversi sudut yaw (rad) ke quaternion (x, y, z, w)."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # 0. Bersihkan proses arduino_bridge.py yatim/zombie dari sesi sebelumnya agar port serial bebas
        kill_other_instances('arduino_bridge.py')

        # --- PARAMETER KONFIGURASI ROBOT ---
        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_diameter', 0.127)      # 5 inci = 0.127 meter
        self.declare_parameter('wheel_separation', 0.458)     # 45.8 cm = 0.458 meter
        self.declare_parameter('ticks_per_rev', 350.7)       # 7 PPR * 50.1 gearbox = 350.7 ticks/rev
        self.declare_parameter('cmd_vel_timeout', 1.0)       # Timeout 1.0s (halus, tidak getar/tersendat saat jitter)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.preferred_port = self.get_parameter('serial_port').value
        self.port = self.preferred_port
        self.baud = self.get_parameter('baudrate').value
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # --- VERIFIKASI ENCODER ---
        # Encoder dianggap SIAP setelah N paket ENC valid berturut-turut diterima.
        self.declare_parameter('ready_packet_count', 5)        # 5 paket @50Hz = 0.1 detik
        self.declare_parameter('enc_stream_timeout', 0.8)      # >0.8s tanpa ENC -> stream hilang
        self.declare_parameter('enc_startup_timeout', 7.0)     # >7.0s tanpa ENC -> auto-recovery
        self.declare_parameter('max_ticks_per_packet', 500)    # lompatan tick tak wajar -> resync baseline
        self.ready_packet_count = int(self.get_parameter('ready_packet_count').value)
        self.enc_stream_timeout = float(self.get_parameter('enc_stream_timeout').value)
        self.enc_startup_timeout = float(self.get_parameter('enc_startup_timeout').value)
        self.max_ticks_per_packet = int(self.get_parameter('max_ticks_per_packet').value)

        self.encoder_ready = False
        self.valid_enc_count = 0
        self.bad_line_count = 0
        self.last_enc_wall = 0.0
        self.connect_wall = 0.0
        self.last_warn_wall = 0.0
        self.last_ready_pub_wall = 0.0
        self.restart_count = 0
        self.rx_bytes_since_connect = 0
        self.last_other_line = ""
        self.warned_wrong_firmware = False
        self.port_missing_logged = False

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
        self.last_safe_cmd_wall = 0.0
        self.last_tx_time = self.get_clock().now()
        self.last_reconnect_time = self.get_clock().now()

        # Publisher status kesiapan (latched / transient local) agar node lain bisa cek kapan saja
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.ready_pub = self.create_publisher(Bool, '/arduino/ready', ready_qos)

        # --- ROS 2 PUB / SUB / TF ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pid_left_pub = self.create_publisher(Point, '/pid/motor_left', 10)
        self.pid_right_pub = self.create_publisher(Point, '/pid/motor_right', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe ke cmd_vel_topic utama; jika diset ke /cmd_vel_safe, dengarkan juga /cmd_vel
        # sebagai fallback langsung apabila adaptive_velocity_node sedang tidak berjalan.
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.cmd_fallback_sub = None
        if self.cmd_vel_topic != '/cmd_vel':
            self.cmd_fallback_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_fallback_callback, 10
            )

        # Telemetri PID (Setpoint vs Aktual RPM)
        self.setpoint_rpm_l = 0.0
        self.setpoint_rpm_r = 0.0
        self.actual_rpm_l = 0.0
        self.actual_rpm_r = 0.0
        self.pwm_l = 0.0
        self.pwm_r = 0.0

        self.publish_ready(False, force=True)

        # --- INISIALISASI SERIAL ARDUINO ---
        self.ser = None
        self.get_logger().info(f"🔍 Menghubungkan ke Arduino ({self.preferred_port}, {self.baud} baud)...")
        self.connect_serial(force_usb_reset=False)

        # Timer 50 Hz (tiap 20ms) untuk pembacaan serial, TF broadcast & heartbeat
        self.timer = self.create_timer(0.02, self.update_loop)
        self.get_logger().info(
            f"Arduino Bridge PID aktif | Topic: {self.cmd_vel_topic} | "
            f"D={self.wheel_diameter}m, L={self.wheel_separation}m, CPR={self.ticks_per_rev}"
        )

    def publish_ready(self, value: bool, force: bool = False):
        wall = time.time()
        if force or (wall - self.last_ready_pub_wall) >= 1.0:
            self.last_ready_pub_wall = wall
            self.ready_pub.publish(Bool(data=value))

    def set_encoder_ready(self, value: bool, reason: str = ""):
        if self.encoder_ready == value:
            return
        self.encoder_ready = value
        self.publish_ready(value, force=True)
        if value:
            self.restart_count = 0
            self.get_logger().info("============================================================")
            self.get_logger().info(
                f"🟢 [ENCODER SIAP] {self.valid_enc_count} paket ENC diterima dari {self.port} -> /odom & motor AKTIF!"
            )
            self.get_logger().info("============================================================")
        else:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(b"V,0.000,0.000\n")
                except Exception:
                    pass
            self.target_v = 0.0
            self.target_w = 0.0
            self.last_sent_v = 0.0
            self.last_sent_w = 0.0
            self.vx = 0.0
            self.wz = 0.0
            self.valid_enc_count = 0
            self.prev_left_ticks = None
            self.prev_right_ticks = None
            self.get_logger().error(f"🔴 [ENCODER TIDAK SIAP] {reason} -> motor dihentikan.")

    def compute_setpoint_rpm(self, v: float, w: float):
        """Hitung target RPM motor kiri dan kanan dari linear v dan angular w."""
        v_l = v - (self.wheel_separation / 2.0) * w
        v_r = v + (self.wheel_separation / 2.0) * w
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0
        return rpm_l, rpm_r

    def _is_lidar_port(self, dev_path: str) -> bool:
        """Pastikan kita TIDAK PERNAH membuka port milik RPLiDAR (CP2102 idVendor=10c4 atau /dev/rplidar)."""
        try:
            real_dev = os.path.realpath(dev_path)
            if os.path.exists('/dev/rplidar') and real_dev == os.path.realpath('/dev/rplidar'):
                return True
            tty_name = os.path.basename(real_dev)
            sys_dev = f"/sys/class/tty/{tty_name}/device"
            if os.path.exists(sys_dev):
                cur = os.path.realpath(sys_dev)
                for _ in range(6):
                    vid_file = os.path.join(cur, "idVendor")
                    if os.path.exists(vid_file):
                        with open(vid_file, "r") as f:
                            vid = f.read().strip().lower()
                        # 10c4 = Silicon Labs CP210x (RPLiDAR A1/A2)
                        if vid == "10c4":
                            return True
                        break
                    cur = os.path.dirname(cur)
        except Exception:
            pass
        return False

    def resolve_arduino_port(self) -> bool:
        """
        Cari port serial Arduino yang sedang tercolok (/dev/arduino, /dev/ttyACM0, /dev/ttyACM1).
        TIDAK menimpa self.preferred_port secara permanen saat kabel dicabut, dan TIDAK PERNAH
        mengambil port RPLiDAR (/dev/ttyUSB0 dengan idVendor 10c4).
        """
        # Bangun daftar kandidat dengan urutan prioritas: preferred_port -> /dev/arduino -> /dev/ttyACM*
        candidates = []
        for p in [self.preferred_port, '/dev/arduino', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if p and p not in candidates:
                candidates.append(p)

        found = None
        for cand in candidates:
            if os.path.exists(cand):
                if self._is_lidar_port(cand):
                    continue
                found = cand
                break

        if not found:
            if not self.port_missing_logged:
                self.port_missing_logged = True
                self.get_logger().warn(
                    f"⚠️ Port Arduino ({self.preferred_port} / /dev/ttyACM*) belum terdeteksi. Menunggu kabel USB Arduino..."
                )
            # Saat kabel dicabut fisik, reset counter recovery agar begitu dicolok langsung connect normal
            self.restart_count = 0
            return False

        self.port_missing_logged = False
        if found != self.port:
            self.get_logger().info(f"🔌 Port Arduino terdeteksi di: {found} (-> {os.path.realpath(found)})")
            self.port = found

        if not os.access(self.port, os.R_OK | os.W_OK):
            self.get_logger().warn(f"🔒 Membuka izin akses baca-tulis untuk {self.port}...")
            try:
                real_port = os.path.realpath(self.port)
                subprocess.run(['sudo', '-n', 'chmod', '666', real_port], check=False)
            except Exception:
                pass

        if not os.access(self.port, os.R_OK | os.W_OK):
            self.get_logger().error(
                f"❌ Permission denied pada {self.port}. Jalankan: sudo chmod 666 {os.path.realpath(self.port)}"
            )
            return False

        return True

    def find_usb_device_path(self):
        """Cari path perangkat USB (/dev/bus/usb/BBB/DDD) dari port serial Arduino saat ini."""
        try:
            real_tty = os.path.realpath(self.port)
            tty_name = os.path.basename(real_tty)
            sys_dev_path = f"/sys/class/tty/{tty_name}/device"
            if not os.path.exists(sys_dev_path):
                return None

            cur = os.path.realpath(sys_dev_path)
            for _ in range(7):
                b_cand = os.path.join(cur, "busnum")
                d_cand = os.path.join(cur, "devnum")
                if os.path.exists(b_cand) and os.path.exists(d_cand):
                    with open(b_cand, 'r') as f:
                        busnum = int(f.read().strip())
                    with open(d_cand, 'r') as f:
                        devnum = int(f.read().strip())
                    usb_dev_path = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
                    if os.path.exists(usb_dev_path):
                        return usb_dev_path
                    break
                cur = os.path.dirname(cur)
        except Exception:
            pass
        return None

    def reset_usb_device(self) -> bool:
        """Reset perangkat USB Arduino di level kernel menggunakan ioctl USBDEVFS_RESET."""
        usb_path = self.find_usb_device_path()
        self.close_serial()

        if not usb_path or not os.path.exists(usb_path):
            return False

        try:
            self.get_logger().info(f"🔌 Melakukan USB Reset ({usb_path}) untuk {self.port}...")
            fd = None
            try:
                fd = os.open(usb_path, os.O_WRONLY)
            except PermissionError:
                subprocess.run(
                    ['sudo', '-n', 'chmod', '666', usb_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                fd = os.open(usb_path, os.O_WRONLY)

            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            finally:
                if fd is not None:
                    os.close(fd)

            time.sleep(1.5)
            return True
        except Exception as e:
            self.get_logger().warn(f"⚠️ USB Reset {usb_path} dilewati ({e})")
            return False

    def reset_encoder_session(self):
        """Dipanggil tiap (re)connect agar tidak terjadi lompatan odom."""
        if self.encoder_ready:
            self.set_encoder_ready(False, "Koneksi serial di-reset")
        self.encoder_ready = False
        self.valid_enc_count = 0
        self.bad_line_count = 0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.vx = 0.0
        self.wz = 0.0
        self.rx_buffer = ""
        self.last_enc_wall = 0.0
        self.rx_bytes_since_connect = 0
        self.last_other_line = ""
        self.warned_wrong_firmware = False
        self.last_sent_v = None
        self.last_sent_w = None
        self.publish_ready(False, force=True)

    def close_serial(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def connect_serial(self, force_usb_reset: bool = False):
        """
        Buka koneksi serial ke Arduino dalam SATU kali pembukaan bersih:
        1. Cari port Arduino yang benar (/dev/arduino atau /dev/ttyACM0).
        2. Buka serial.Serial satu kali saja.
        3. Lakukan pulse DTR singkat (100ms) pada handle yang sama agar Arduino reset bersih.
        4. Kosongkan buffer input dan tunggu stream ENC: di update_loop (tanpa kirim byte apapun selama bootloader).
        """
        self.close_serial()

        if force_usb_reset:
            self.reset_usb_device()

        if not self.resolve_arduino_port():
            self.connect_wall = time.time()
            self.last_reconnect_time = self.get_clock().now()
            return

        self.reset_encoder_session()
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=0.2
            )
            # Pulse DTR/RTS 1x pada objek serial yang sama (tanpa menutup port!)
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = True
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.connect_wall = time.time()
            self.last_reconnect_time = self.get_clock().now()
            self.get_logger().info(
                f"✅ Terhubung ke Arduino di {self.port} ({os.path.realpath(self.port)}) -> menunggu bootloader (~1.5s) & stream ENC..."
            )
        except Exception as e:
            self.get_logger().warn(f"⚠️ Gagal membuka port Arduino {self.port}: {e}")
            self.ser = None

    def pulse_dtr_in_place(self):
        """Pulse DTR langsung pada self.ser yang sudah terbuka tanpa menutup/membuka ulang port."""
        if self.ser is None or not self.ser.is_open:
            self.connect_serial(force_usb_reset=False)
            return
        try:
            self.get_logger().info(f"🔄 Mengirim pulsa DTR reset ke Arduino di {self.port}...")
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.2)
            self.ser.dtr = True
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.reset_encoder_session()
            self.connect_wall = time.time()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Gagal pulse DTR pada {self.port}: {e}")
            self.close_serial()

    def send_cmd(self, v: float, w: float, now):
        """Kirim perintah kecepatan 'V,<v>,<w>\n' ke Arduino (HANYA jika encoder sudah siap)."""
        if self.ser is None or not self.ser.is_open or not self.encoder_ready:
            return
        try:
            cmd_str = f"V,{v:.3f},{w:.3f}\n"
            self.ser.write(cmd_str.encode('ascii'))
            self.last_sent_v = v
            self.last_sent_w = w
            self.last_tx_time = now
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self.close_serial()

    def _apply_cmd_vel(self, v: float, w: float):
        now = self.get_clock().now()

        if not self.encoder_ready:
            if (abs(v) > 1e-3 or abs(w) > 1e-3):
                wall = time.time()
                if wall - self.last_warn_wall >= 2.0:
                    self.last_warn_wall = wall
                    self.get_logger().warn("⛔ /cmd_vel diabaikan: encoder Arduino belum SIAP (odom belum valid).")
            self.target_v = 0.0
            self.target_w = 0.0
            self.setpoint_rpm_l = 0.0
            self.setpoint_rpm_r = 0.0
            return

        self.last_cmd_vel_time = now
        self.target_v = v
        self.target_w = w
        self.setpoint_rpm_l, self.setpoint_rpm_r = self.compute_setpoint_rpm(v, w)

        # Kirim langsung jika ada perubahan nilai kecepatan agar respon instan
        if self.last_sent_v is None or abs(v - self.last_sent_v) > 0.005 or abs(w - self.last_sent_w) > 0.005:
            self.send_cmd(self.target_v, self.target_w, now)

    def cmd_vel_callback(self, msg: Twist):
        if self.cmd_vel_topic != '/cmd_vel':
            self.last_safe_cmd_wall = time.time()
        self._apply_cmd_vel(msg.linear.x, msg.angular.z)

    def cmd_vel_fallback_callback(self, msg: Twist):
        """Jika cmd_vel_topic adalah /cmd_vel_safe tetapi adaptive_velocity_node tidak aktif (>1.0s),
        terima langsung perintah dari /cmd_vel agar teleop & evaluasi_pid selalu jalan."""
        if (time.time() - self.last_safe_cmd_wall) > 1.0:
            self._apply_cmd_vel(msg.linear.x, msg.angular.z)

    def check_encoder_health(self):
        """Watchdog stream ENC."""
        wall = time.time()
        if self.last_enc_wall == 0.0:
            since_connect = wall - self.connect_wall
            # Jika Arduino mengirim teks lain (bukan ENC:), beri peringatan instan bahwa sketch salah!
            if self.rx_bytes_since_connect > 0 and self.last_other_line and not self.warned_wrong_firmware:
                if since_connect > 2.5:
                    self.warned_wrong_firmware = True
                    self.get_logger().error(
                        f"⚠️ Arduino di {self.port} mengirim: \"{self.last_other_line[:60]}\" (BUKAN 'ENC:')! "
                        f"Pastikan firmware yang ter-upload di Arduino adalah 'amr_firmware.ino' (bukan uji_deadzone / uji_step_response)!"
                    )

            if since_connect > self.enc_startup_timeout:
                # Jika sudah ada trafik serial tapi bukan ENC:, jangan reset USB terus-menerus karena firmware-nya yang beda
                if self.rx_bytes_since_connect > 20 and self.last_other_line:
                    self.connect_wall = wall
                    self.get_logger().warn(
                        f"⏳ Menunggu 'ENC:' dari {self.port} (Data saat ini: '{self.last_other_line[:45]}'). "
                        f"Silakan upload amr_firmware.ino jika belum."
                    )
                    return

                self.restart_count += 1
                if self.restart_count == 1:
                    self.get_logger().warn(
                        f"⚠️ Belum ada paket ENC setelah {since_connect:.1f}s di {self.port} (percobaan #1) -> Pulse DTR Reset..."
                    )
                    self.pulse_dtr_in_place()
                else:
                    self.get_logger().warn(
                        f"⚠️ Belum ada paket ENC setelah {since_connect:.1f}s di {self.port} (percobaan #{self.restart_count}) -> Reconnect & USB Reset..."
                    )
                    self.connect_serial(force_usb_reset=(self.restart_count % 2 == 0))
            return

        gap = wall - self.last_enc_wall
        if self.encoder_ready and gap > self.enc_stream_timeout:
            self.set_encoder_ready(False, f"Stream ENC terputus {gap:.2f}s")
        if gap > self.enc_startup_timeout:
            self.restart_count += 1
            self.get_logger().warn(
                f"🔌 Stream ENC hilang {gap:.1f}s (percobaan #{self.restart_count}) -> Reconnect Arduino..."
            )
            self.connect_serial(force_usb_reset=(self.restart_count >= 2))

    def update_loop(self):
        now = self.get_clock().now()

        # 0. Auto-reconnect jika serial belum terbuka / terputus (misal habis dicabut-colok manual)
        if self.ser is None or not self.ser.is_open:
            dt_reconnect = (now - self.last_reconnect_time).nanoseconds / 1e9
            if dt_reconnect >= 1.0:
                self.last_reconnect_time = now
                self.connect_serial(force_usb_reset=False)
            self.publish_ready(False)
            self.broadcast_tf_and_odom(now)
            return

        # 1. Baca serial non-blocking dari Arduino TERLEBIH DAHULU
        try:
            waiting = self.ser.in_waiting
            raw_bytes = self.ser.read(waiting) if waiting > 0 else b""
        except Exception as e:
            self.get_logger().warn(f"🔌 Koneksi serial {self.port} terputus ({e}) -> menunggu dicolok kembali...")
            self.close_serial()
            self.restart_count = 0
            self.broadcast_tf_and_odom(now)
            return

        if raw_bytes:
            self.rx_bytes_since_connect += len(raw_bytes)
            self.rx_buffer += raw_bytes.decode('ascii', errors='ignore')
            if len(self.rx_buffer) > 4096:
                self.rx_buffer = ""

            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith("ENC:"):
                    self.handle_enc_line(line, now)
                elif line.startswith("LOG:"):
                    self.handle_log_line(line)
                else:
                    self.last_other_line = line

        # 2. Kirim perintah kecepatan & heartbeat HANYA JIKA ENCODER SUDAH SIAP!
        if self.encoder_ready:
            dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
            dt_tx = (now - self.last_tx_time).nanoseconds / 1e9

            if dt_cmd > self.cmd_vel_timeout:
                if self.last_sent_v != 0.0 or self.last_sent_w != 0.0:
                    self.target_v = 0.0
                    self.target_w = 0.0
                    self.setpoint_rpm_l = 0.0
                    self.setpoint_rpm_r = 0.0
                    self.send_cmd(0.0, 0.0, now)
                elif dt_tx >= 0.2:
                    self.send_cmd(0.0, 0.0, now)
            elif (self.target_v != 0.0 or self.target_w != 0.0) and dt_tx >= 0.05:
                self.send_cmd(self.target_v, self.target_w, now)
        else:
            self.target_v = 0.0
            self.target_w = 0.0

        # 3. Watchdog kesehatan stream encoder
        self.check_encoder_health()
        self.publish_ready(self.encoder_ready)

        # 4. Kontinu broadcast TF & Odometry pada 50Hz
        self.broadcast_tf_and_odom(now)

    def handle_enc_line(self, line: str, now):
        """Parse 'ENC:<L>,<R>'."""
        parts = line[4:].split(",")
        try:
            if len(parts) != 2:
                raise ValueError("jumlah field salah")
            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
        except ValueError:
            self.bad_line_count += 1
            self.valid_enc_count = 0
            return

        self.last_enc_wall = time.time()
        if self.process_encoder_ticks(left_ticks, right_ticks, now):
            if not self.encoder_ready:
                self.valid_enc_count += 1
                if self.valid_enc_count >= self.ready_packet_count:
                    self.set_encoder_ready(True)
        else:
            self.valid_enc_count = 0

    def handle_log_line(self, line: str):
        parts = line[4:].split(",")
        if len(parts) >= 5:
            try:
                self.actual_rpm_l = float(parts[1])
                self.actual_rpm_r = float(parts[2])
                self.pwm_l = float(parts[3])
                self.pwm_r = float(parts[4])

                pt_l = Point(x=float(self.setpoint_rpm_l), y=self.actual_rpm_l, z=self.pwm_l)
                pt_r = Point(x=float(self.setpoint_rpm_r), y=self.actual_rpm_r, z=self.pwm_r)
                self.pid_left_pub.publish(pt_l)
                self.pid_right_pub.publish(pt_r)
            except ValueError:
                pass

    def process_encoder_ticks(self, left_ticks: int, right_ticks: int, now) -> bool:
        """Return True jika paket valid (baseline atau delta wajar)."""
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now
            self.get_logger().info(
                f"Paket Encoder pertama diterima (L={left_ticks}, R={right_ticks}) -> baseline odom diset, verifikasi stream..."
            )
            return True

        jump_l = abs(left_ticks - self.prev_left_ticks)
        jump_r = abs(right_ticks - self.prev_right_ticks)
        if jump_l > self.max_ticks_per_packet or jump_r > self.max_ticks_per_packet:
            self.get_logger().warn(
                f"⚠️ Lompatan tick tak wajar (dL={jump_l}, dR={jump_r}) -> resync baseline, paket diabaikan."
            )
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now
            return False

        dt = (now - self.prev_odom_time).nanoseconds / 1e9
        if dt > 0.0001:
            d_left = (left_ticks - self.prev_left_ticks) * self.meters_per_tick
            d_right = (right_ticks - self.prev_right_ticks) * self.meters_per_tick
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_odom_time = now

            d_center = (d_right + d_left) / 2.0
            d_yaw = (d_right - d_left) / self.wheel_separation

            mid_yaw = self.yaw + (d_yaw / 2.0)
            self.x += d_center * math.cos(mid_yaw)
            self.y += d_center * math.sin(mid_yaw)
            self.yaw = math.atan2(math.sin(self.yaw + d_yaw), math.cos(self.yaw + d_yaw))

            self.vx = d_center / dt
            self.wz = d_yaw / dt
        return True

    def broadcast_tf_and_odom(self, now):
        qx, qy, qz, qw = euler_to_quaternion(self.yaw)
        stamp = now.to_msg()
        future_stamp = (now + rclpy.duration.Duration(seconds=0.05)).to_msg()

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = future_stamp
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


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    def emergency_stop_handler(sig, frame):
        if node.ser and node.ser.is_open:
            try:
                for _ in range(3):
                    node.ser.write(b"V,0.000,0.000\n")
                    node.ser.flush()
                    time.sleep(0.01)
                node.ser.close()
            except Exception:
                pass
        os._exit(0)

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
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
