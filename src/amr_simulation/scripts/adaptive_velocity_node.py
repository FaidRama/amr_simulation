#!/usr/bin/env python3
from collections import deque
import json
import math
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

try:
    from std_srvs.srv import Empty
    HAS_STD_SRVS = True
except ImportError:
    HAS_STD_SRVS = False


class AdaptiveVelocityNode(Node):
    def __init__(self):
        super().__init__('adaptive_velocity_node')

        self.declare_parameter('normal_max_linear', 0.45)
        self.declare_parameter('reduced_max_linear', 0.20)
        self.declare_parameter('max_accel', 0.4)
        self.declare_parameter('max_decel', 0.8)
        self.declare_parameter('state_hold_time', 1.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('safe_stop_wait_duration', 15.0)
        self.declare_parameter('bypass_clear_duration', 8.0)
        self.declare_parameter('human_state_timeout', 3.0)

        # Parameter Auto-Alignment Orientasi & Posisi AMCL saat Startup / Pra-Jalan
        self.declare_parameter('auto_align_on_startup', True)
        self.declare_parameter('startup_align_spin_enabled', True)
        self.declare_parameter('align_max_sigma_xy', 0.18)
        self.declare_parameter('align_max_sigma_yaw', 0.14)
        self.declare_parameter('align_spin_speed', 0.40)
        self.declare_parameter('align_spin_timeout', 16.0)
        self.declare_parameter('align_min_clearance', 0.34)
        self.declare_parameter('initial_cov_xy', 0.64)
        self.declare_parameter('initial_cov_yaw', math.pi ** 2)

        self.effective_state = "NORMAL"
        self.last_state_change_time = self.get_clock().now()
        self.last_high_sev_time = self.get_clock().now()
        self.last_human_msg_wall = 0.0
        self.safe_stop_enter_time = None
        self.non_safe_stop_since = None
        self.safe_stop_avoid_logged = False
        self.bypass_active = False
        self.bypass_clear_since = None
        self.last_human_nav_mode = None
        self.last_human_nav_mode_pub_wall = 0.0

        self.in_v = 0.0
        self.in_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.last_linear_v = 0.0
        self.last_time = self.get_clock().now()

        # --- NAV2 BOOTSTRAP TF & CONTINUITY KEEPER (map -> odom) ---
        # Menjamin frame 'map' selalu ada sejak detik ke-0 DAN tidak pernah kedaluwarsa
        # di TF buffer saat robot diam lama / menunggu antar-pengantaran (ROS 2 Foxy AMCL).
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_amcl_tf_wall = 0.0
        self.last_amcl_pose_wall = 0.0
        self.amcl_valid_until_sec = 0.0
        self.own_tf_stamps = deque(maxlen=30)
        self.own_initialpose_stamps = deque(maxlen=20)
        self.initialpose_pause_until = 0.0
        self.last_keepalive_pub_wall = 0.0
        self.last_amcl_alive_check_wall = 0.0
        self.amcl_crash_warned = False
        self.start_wall_time = time.time()
        self.amcl_initialized = False
        self.map_received = False
        self.local_costmap_active = False
        self.global_costmap_active = False
        self.nav_ready_announced = False
        self.last_ready_check_time = 0.0
        self.last_scan_wall_time = 0.0
        self.first_scan_wall_time = 0.0
        self.last_min_scan_range = 99.0
        self.last_initpose_sent_time = 0.0
        self.initpose_sent_count = 0
        self.last_map_odom_x = 0.0
        self.last_map_odom_y = 0.0
        self.last_map_odom_qz = 0.0
        self.last_map_odom_qw = 1.0

        # --- STATE KALIBRASI & PENGUNCIAN ORIENTASI AMCL ---
        self.amcl_sigma_x = 99.0
        self.amcl_sigma_y = 99.0
        self.amcl_sigma_yaw = 99.0
        self.amcl_pose_x = 0.0
        self.amcl_pose_y = 0.0
        self.manual_initialpose_received = False
        self.skip_auto_spin_once = False
        self.startup_align_phase = 'WAIT_READY'  # WAIT_READY -> SETTLE_POSE -> NOMOTION_UPDATE -> ALIGN_SPIN -> LOCKED
        self.align_phase_since_wall = 0.0
        self.align_spin_elapsed = 0.0
        self.align_spin_last_tick_wall = 0.0
        self.align_nomotion_count = 0
        self.align_nomotion_max = 6
        self.align_last_nomotion_wall = 0.0
        self.nomotion_future = None
        self.align_warned_clearance = False
        self.last_align_state_pub_wall = 0.0
        self.last_align_state_payload = None

        # --- VERIFIKASI ENCODER ARDUINO (setara cek LiDAR) ---
        # require_encoder=False dipakai saat use_arduino:=False (odom->base_link static)
        self.declare_parameter('require_encoder', True)
        _req = self.get_parameter('require_encoder').value
        self.require_encoder = (str(_req).strip().lower() in ('true', '1', 'yes')) if isinstance(_req, str) else bool(_req)
        self.arduino_ready = False
        self.last_odom_wall_time = 0.0
        self.encoder_was_ok = False
        self.last_wait_log_time = 0.0

        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, tl_qos)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, 10)
        self.create_subscription(Bool, '/arduino/ready', self.on_arduino_ready, tl_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, tl_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, tl_qos)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self.on_local_footprint, 5)
        self.create_subscription(PolygonStamped, '/global_costmap/published_footprint', self.on_global_footprint, 5)
        self.create_subscription(String, '/amr/request_align', self.on_request_align, 10)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', tl_qos)
        self.initialpose_pub_vol = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.delivery_log_pub = self.create_publisher(String, '/amr/delivery_log', 10)
        self.human_nav_mode_pub = self.create_publisher(String, '/human_nav_mode', 10)
        self.align_state_pub = self.create_publisher(String, '/amr/amcl_alignment_state', 10)

        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update') if HAS_STD_SRVS else None

        self.create_subscription(String, '/human_state', self.state_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.cmd_callback, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        # Loop kontrol tetap 20 Hz, independen dari Nav2
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Adaptive Velocity, Auto-Align AMCL & Nav2 Bootstrap TF aktif")

    def on_map(self, msg: OccupancyGrid):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info("🗺️ Peta (/map) terdeteksi oleh Nav2 Bootstrap.")

    def on_scan(self, msg: LaserScan):
        now = time.time()
        if self.first_scan_wall_time == 0.0:
            self.first_scan_wall_time = now
        self.last_scan_wall_time = now
        if msg.ranges:
            valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
            self.last_min_scan_range = min(valid) if valid else 99.0

    def on_arduino_ready(self, msg: Bool):
        self.arduino_ready = bool(msg.data)

    def on_odom(self, msg: Odometry):
        self.last_odom_wall_time = time.time()

    def encoder_ok(self, wall_now: float) -> bool:
        if not self.require_encoder:
            return True
        return self.arduino_ready and (wall_now - self.last_odom_wall_time) < 2.0

    def on_tf(self, msg: TFMessage):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                stamp_ns = int(t.header.stamp.sec) * 1_000_000_000 + int(t.header.stamp.nanosec)
                if stamp_ns in self.own_tf_stamps:
                    continue
                self.last_map_odom_x = t.transform.translation.x
                self.last_map_odom_y = t.transform.translation.y
                self.last_map_odom_qz = t.transform.rotation.z
                self.last_map_odom_qw = t.transform.rotation.w
                self.last_amcl_tf_wall = time.time()
                # AMCL menstempel map->odom dengan now + transform_tolerance (1.5s)
                self.amcl_valid_until_sec = max(stamp_ns / 1e9, now_sec + 1.0)
                if not self.amcl_initialized:
                    self.amcl_initialized = True
                    self.get_logger().info("✅ AMCL telah aktif (TF map->odom diterima) -> AMCL + Continuity Keeper aktif.")

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        is_own = stamp_key in self.own_initialpose_stamps
        now_wall = time.time()

        if self.amcl_initialized:
            should_log = now_wall + 2.0 > self.initialpose_pause_until
            self.initialpose_pause_until = now_wall + 2.5
            if should_log:
                self.get_logger().info("📍 /initialpose diterima -> menjeda TF keepalive 2.5s agar AMCL memperbarui map->odom.")

        if not is_own:
            self.manual_initialpose_received = True
            self.skip_auto_spin_once = True
            if self.startup_align_phase == 'ALIGN_SPIN':
                self.startup_align_phase = 'LOCKED'
                self.get_logger().info("📍 Manual /initialpose diterima -> menghentikan putaran kalibrasi otomatis.")

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        now_wall = time.time()
        self.last_amcl_tf_wall = now_wall
        self.last_amcl_pose_wall = now_wall
        pos = msg.pose.pose.position
        self.amcl_pose_x = float(pos.x)
        self.amcl_pose_y = float(pos.y)
        cov = msg.pose.covariance
        if len(cov) >= 36:
            self.amcl_sigma_x = math.sqrt(max(0.0, float(cov[0])))
            self.amcl_sigma_y = math.sqrt(max(0.0, float(cov[7])))
            self.amcl_sigma_yaw = math.sqrt(max(0.0, float(cov[35])))
        if not self.amcl_initialized:
            self.amcl_initialized = True
            self.get_logger().info("✅ AMCL telah aktif (/amcl_pose diterima) -> AMCL + Continuity Keeper aktif.")

    def on_local_footprint(self, msg: PolygonStamped):
        self.local_costmap_active = True

    def on_global_footprint(self, msg: PolygonStamped):
        self.global_costmap_active = True

    def on_request_align(self, msg: String):
        """Terima permintaan relokalisasi pra-jalan dari waypoint_manager/dashboard."""
        req = msg.data.strip().lower()
        if not req:
            return
        if self.startup_align_phase == 'LOCKED':
            self.skip_auto_spin_once = False
            self.manual_initialpose_received = False
            now_wall = time.time()
            if 'wide' in req or self.amcl_sigma_yaw > 0.45:
                cov_xy = float(self.get_parameter('initial_cov_xy').value)
                cov_yaw = float(self.get_parameter('initial_cov_yaw').value)
                self.send_auto_initialpose(
                    self.get_clock().now().to_msg(),
                    x=self.amcl_pose_x, y=self.amcl_pose_y, yaw=0.0,
                    cov_xy=cov_xy, cov_yaw=cov_yaw
                )
            self.startup_align_phase = 'SETTLE_POSE'
            self.align_phase_since_wall = now_wall
            self.align_nomotion_count = 0
            self.align_last_nomotion_wall = 0.0
            self.align_spin_elapsed = 0.0
            self.get_logger().info("🧭 [AUTO-ALIGN] Permintaan penguncian orientasi AMCL diterima sebelum jalan...")

    def _load_home_pose(self):
        """Muat home_position aktual dari waypoints.yaml (profil real jika ada)."""
        candidates = [
            os.path.join(os.path.expanduser('~'), 'deliver_amr', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.expanduser('~'), 'Documents', 'amr_ws', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'amr_dashboard', 'config', 'waypoints.yaml'),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(get_package_share_directory('amr_dashboard'), 'config', 'waypoints.yaml'))
        except Exception:
            pass

        for path in candidates:
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        cfg = yaml.safe_load(f) or {}
                    mode = str(cfg.get('operation_mode', 'real')).strip().lower()
                    profiles = cfg.get('profiles', {})
                    mode_hp = profiles.get(mode, {}).get('home_position') if isinstance(profiles.get(mode), dict) else None
                    if isinstance(mode_hp, dict) and 'x' in mode_hp and 'y' in mode_hp:
                        hp = mode_hp
                    elif isinstance(profiles.get('real'), dict) and isinstance(profiles['real'].get('home_position'), dict):
                        hp = profiles['real']['home_position']
                    else:
                        hp = cfg.get('home_position', {})
                    if isinstance(hp, dict) and 'x' in hp and 'y' in hp:
                        return float(hp.get('x', 0.0)), float(hp.get('y', 0.0)), float(hp.get('yaw', 0.0))
                except Exception:
                    pass
        return 0.0, 0.0, 0.0

    def _is_amcl_converged(self) -> bool:
        max_xy = float(self.get_parameter('align_max_sigma_xy').value)
        max_yaw = float(self.get_parameter('align_max_sigma_yaw').value)
        return (
            self.amcl_initialized
            and max(self.amcl_sigma_x, self.amcl_sigma_y) <= max_xy
            and self.amcl_sigma_yaw <= max_yaw
        )

    def _call_nomotion_async(self):
        if self.nomotion_client is None:
            return False
        if self.nomotion_future is not None and not self.nomotion_future.done():
            return False
        if not self.nomotion_client.service_is_ready():
            return False
        self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
        return True

    def _publish_align_state(self, wall_now: float, force: bool = False):
        payload_dict = {
            'phase': self.startup_align_phase,
            'localized': self._is_amcl_converged(),
            'sigma_xy': round(self.amcl_sigma_x, 3),
            'sigma_yaw_deg': round(math.degrees(self.amcl_sigma_yaw), 1),
        }
        payload = json.dumps(payload_dict, sort_keys=True)
        if force or payload != self.last_align_state_payload or (wall_now - self.last_align_state_pub_wall) >= 0.5:
            self.last_align_state_payload = payload
            self.last_align_state_pub_wall = wall_now
            m = String()
            m.data = payload
            self.align_state_pub.publish(m)

    def _step_startup_alignment(self, now_msg, wall_now: float, scan_stable: bool, enc_ok: bool, dt: float):
        """
        Kalibrasi otomatis posisi & hadap awal AMCL:
        1. Sebar partikel 360 derajat di sekitar Home (kecuali /initialpose manual sudah ada).
        2. Coba kunci diam-diam via /request_nomotion_update.
        3. Jika sigma_yaw masih lebar -> putar pelan di tempat sampai terkunci (jeda bila SAFE_STOP/terlalu rapat).
        """
        if not bool(self.get_parameter('auto_align_on_startup').value):
            if self.startup_align_phase != 'LOCKED':
                self.startup_align_phase = 'LOCKED'
            self._publish_align_state(wall_now)
            return False, 0.0, 0.0

        override = False
        spin_w = 0.0

        if self.startup_align_phase == 'WAIT_READY':
            nav_nodes_ready = (
                self.map_received
                and scan_stable
                and enc_ok
                and self.amcl_initialized
                and self.local_costmap_active
                and self.global_costmap_active
            )
            if nav_nodes_ready:
                if self.manual_initialpose_received:
                    self.align_phase_since_wall = wall_now
                    self.align_nomotion_count = 0
                    self.align_last_nomotion_wall = 0.0
                    self.startup_align_phase = 'NOMOTION_UPDATE'
                    self.get_logger().info("🧭 [AUTO-ALIGN] Pose manual terdeteksi -> verifikasi konvergensi AMCL tanpa menimpa pose...")
                else:
                    hx, hy, hyaw = self._load_home_pose()
                    cov_xy = float(self.get_parameter('initial_cov_xy').value)
                    cov_yaw = float(self.get_parameter('initial_cov_yaw').value)
                    self.initialpose_pause_until = wall_now + 2.5
                    self.send_auto_initialpose(now_msg, x=hx, y=hy, yaw=hyaw, cov_xy=cov_xy, cov_yaw=cov_yaw)
                    self.align_phase_since_wall = wall_now
                    self.align_nomotion_count = 0
                    self.align_last_nomotion_wall = 0.0
                    self.startup_align_phase = 'SETTLE_POSE'
                    self.get_logger().info(
                        f"🧭 [AUTO-ALIGN] Mengirim initialpose Home ({hx:.2f}, {hy:.2f}) sebaran hadap 360° untuk kunci arah AMCL..."
                    )

        elif self.startup_align_phase == 'SETTLE_POSE':
            if (wall_now - self.align_phase_since_wall) >= 0.45:
                self.startup_align_phase = 'NOMOTION_UPDATE'
                self.align_phase_since_wall = wall_now

        elif self.startup_align_phase == 'NOMOTION_UPDATE':
            if (wall_now - self.align_last_nomotion_wall) >= 0.35 and self.align_nomotion_count < self.align_nomotion_max:
                if self._call_nomotion_async():
                    self.align_nomotion_count += 1
                self.align_last_nomotion_wall = wall_now

            fresh_pose = self.last_amcl_pose_wall > (self.align_phase_since_wall + 0.15)
            if self.align_nomotion_count >= 2 and fresh_pose and self._is_amcl_converged():
                self.startup_align_phase = 'LOCKED'
                msg_txt = (
                    f"✅ [AUTO-ALIGN] Posisi & hadap AMCL terkunci dari pindaian diam! "
                    f"(sigma_xy={max(self.amcl_sigma_x, self.amcl_sigma_y):.2f}m, sigma_yaw={math.degrees(self.amcl_sigma_yaw):.1f}°)"
                )
                self.get_logger().info(msg_txt)
                log_m = String(); log_m.data = msg_txt
                self.delivery_log_pub.publish(log_m)
            elif (
                (self.align_nomotion_count >= self.align_nomotion_max and (wall_now - self.align_last_nomotion_wall) >= 0.4)
                or (wall_now - self.align_phase_since_wall) >= 3.2
            ):
                if self.skip_auto_spin_once or not bool(self.get_parameter('startup_align_spin_enabled').value):
                    self.startup_align_phase = 'LOCKED'
                else:
                    self.startup_align_phase = 'ALIGN_SPIN'
                    self.align_phase_since_wall = wall_now
                    self.align_spin_elapsed = 0.0
                    self.align_spin_last_tick_wall = wall_now
                    self.align_warned_clearance = False
                    msg_txt = (
                        f"🧭 [AUTO-ALIGN] Hadap awal belum yakin (sigma_yaw={math.degrees(self.amcl_sigma_yaw):.1f}°): "
                        "memutar pelan di tempat sampai AMCL mengunci hadap..."
                    )
                    self.get_logger().info(msg_txt)
                    log_m = String(); log_m.data = msg_txt
                    self.delivery_log_pub.publish(log_m)

        elif self.startup_align_phase == 'ALIGN_SPIN':
            override = True
            min_clr = float(self.get_parameter('align_min_clearance').value)
            too_close = self.last_min_scan_range < min_clr
            human_near = (self.effective_state == "SAFE_STOP")

            if human_near or too_close:
                spin_w = 0.0
                if too_close and not self.align_warned_clearance:
                    self.align_warned_clearance = True
                    self.get_logger().warn(
                        f"⚠️ [AUTO-ALIGN] Objek sangat dekat ({self.last_min_scan_range:.2f}m < {min_clr:.2f}m): menjeda putaran kalibrasi."
                    )
            else:
                self.align_warned_clearance = False
                spin_w = float(self.get_parameter('align_spin_speed').value)
                self.align_spin_elapsed += dt
                if (wall_now - self.align_last_nomotion_wall) >= 0.9:
                    self._call_nomotion_async()
                    self.align_last_nomotion_wall = wall_now

            fresh_spin_pose = self.last_amcl_pose_wall > (self.align_phase_since_wall + 0.6)
            if self.align_spin_elapsed >= 1.2 and fresh_spin_pose and self._is_amcl_converged():
                self.startup_align_phase = 'LOCKED'
                override = False
                spin_w = 0.0
                msg_txt = (
                    f"✅ [AUTO-ALIGN] Arah hadap AMCL terkunci ({self.align_spin_elapsed:.1f}s putaran)! "
                    f"(sigma_xy={max(self.amcl_sigma_x, self.amcl_sigma_y):.2f}m, sigma_yaw={math.degrees(self.amcl_sigma_yaw):.1f}°)"
                )
                self.get_logger().info(msg_txt)
                log_m = String(); log_m.data = msg_txt
                self.delivery_log_pub.publish(log_m)
            elif (
                self.align_spin_elapsed >= float(self.get_parameter('align_spin_timeout').value)
                or (wall_now - self.align_phase_since_wall) >= (float(self.get_parameter('align_spin_timeout').value) + 10.0)
            ):
                self.startup_align_phase = 'LOCKED'
                override = False
                spin_w = 0.0
                msg_txt = (
                    f"⚠️ [AUTO-ALIGN] Batas putaran selesai (sigma_yaw={math.degrees(self.amcl_sigma_yaw):.1f}°). "
                    "Bila area simetris, cek hadap robot di peta Dashboard sebelum berangkat."
                )
                self.get_logger().warn(msg_txt)
                log_m = String(); log_m.data = msg_txt
                self.delivery_log_pub.publish(log_m)

        self._publish_align_state(wall_now)
        return override, 0.0, spin_w

    def publish_bootstrap_tf(self, now_msg):
        stamp_ns = int(now_msg.sec) * 1_000_000_000 + int(now_msg.nanosec)
        self.own_tf_stamps.append(stamp_ns)
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.last_map_odom_x
        t.transform.translation.y = self.last_map_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = self.last_map_odom_qz
        t.transform.rotation.w = self.last_map_odom_qw
        self.tf_broadcaster.sendTransform(t)

    def send_auto_initialpose(self, now_msg, x=0.0, y=0.0, yaw=0.0, cov_xy=None, cov_yaw=None):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now_msg
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        msg.pose.covariance[0] = float(cov_xy if cov_xy is not None else self.get_parameter('initial_cov_xy').value)
        msg.pose.covariance[7] = float(cov_xy if cov_xy is not None else self.get_parameter('initial_cov_xy').value)
        msg.pose.covariance[35] = float(cov_yaw if cov_yaw is not None else self.get_parameter('initial_cov_yaw').value)
        self.own_initialpose_stamps.append((int(now_msg.sec), int(now_msg.nanosec)))
        self.initialpose_pub.publish(msg)
        self.initialpose_pub_vol.publish(msg)

    def state_callback(self, msg: String):
        severity = {"NORMAL": 0, "REDUCED_SPEED": 1, "SAFE_STOP": 2}
        new_state = msg.data.strip().upper()
        if new_state not in severity:
            new_state = "NORMAL"
        now = self.get_clock().now()
        self.last_human_msg_wall = time.time()
        new_sev = severity[new_state]
        curr_sev = severity.get(self.effective_state, 0)

        if new_sev >= curr_sev:
            # Catat timestamp frame high-severity terbaru agar dt_hold hanya mulai menghitung
            # setelah state tinggi benar-benar hilang, bukan dari detik ke-0 saat masuk.
            self.last_high_sev_time = now
            if self.effective_state != new_state:
                self.get_logger().info(f"[ADAPTIF] {self.effective_state} -> {new_state}")
                self.effective_state = new_state
                self.last_state_change_time = now
                if new_state == "SAFE_STOP" and (not self.bypass_active) and (self.safe_stop_enter_time is None):
                    self.safe_stop_enter_time = now
                    self.non_safe_stop_since = None
                    self.safe_stop_avoid_logged = False
        else:
            dt_hold = (now - self.last_high_sev_time).nanoseconds / 1e9
            if dt_hold >= float(self.get_parameter('state_hold_time').value):
                self.get_logger().info(f"[ADAPTIF] Relaksasi {self.effective_state} -> {new_state}")
                self.effective_state = new_state
                self.last_state_change_time = now
                self.last_high_sev_time = now

    def _update_human_safety_mode(self, now, wall_now):
        """Kelola mode henti 15 detik (WAIT_HUMAN) -> bypass berkelanjutan (BYPASS_HUMAN) -> NORMAL."""
        human_timeout = float(self.get_parameter('human_state_timeout').value)
        if (
            self.last_human_msg_wall > 0.0
            and self.effective_state != "NORMAL"
            and (wall_now - self.last_human_msg_wall) > human_timeout
        ):
            self.get_logger().warn(f"[ADAPTIF] /human_state timeout (>{human_timeout:.1f}s) -> relaksasi ke NORMAL.")
            self.effective_state = "NORMAL"
            self.last_state_change_time = now
            self.last_high_sev_time = now

        safe_stop_wait = float(self.get_parameter('safe_stop_wait_duration').value)
        bypass_clear_sec = float(self.get_parameter('bypass_clear_duration').value)
        reduced_v = float(self.get_parameter('reduced_max_linear').value)
        normal_v = float(self.get_parameter('normal_max_linear').value)

        if self.bypass_active:
            if self.effective_state == "NORMAL":
                if self.bypass_clear_since is None:
                    self.bypass_clear_since = now
                elif (now - self.bypass_clear_since).nanoseconds / 1e9 >= bypass_clear_sec:
                    self.bypass_active = False
                    self.bypass_clear_since = None
                    self.safe_stop_enter_time = None
                    self.non_safe_stop_since = None
                    self.safe_stop_avoid_logged = False
                    self.get_logger().info(
                        f"✅ [ADAPTIF] Manusia sudah terlewati (NORMAL >= {bypass_clear_sec:.1f}s): Kembali ke mode NORMAL."
                    )
            else:
                self.bypass_clear_since = None

        is_safe_stop_active = False
        if self.bypass_active:
            ceiling = reduced_v
            nav_mode = "BYPASS_HUMAN"
        elif self.effective_state == "SAFE_STOP":
            self.non_safe_stop_since = None
            if self.safe_stop_enter_time is None:
                self.safe_stop_enter_time = now
            safe_stop_duration = (now - self.safe_stop_enter_time).nanoseconds / 1e9
            if safe_stop_duration < safe_stop_wait:
                ceiling = 0.0
                is_safe_stop_active = True
                nav_mode = "WAIT_HUMAN"
            else:
                self.bypass_active = True
                self.bypass_clear_since = None
                if not self.safe_stop_avoid_logged:
                    self.get_logger().info(
                        f"🚶 [ADAPTIF] Manusia tetap di jalur > {safe_stop_wait:.1f}s: Mengaktifkan mode BYPASS (kecepatan pelan & setir aktif via costmap)..."
                    )
                    self.safe_stop_avoid_logged = True
                ceiling = reduced_v
                nav_mode = "BYPASS_HUMAN"
        elif self.effective_state == "REDUCED_SPEED":
            if self.safe_stop_enter_time is not None:
                if self.non_safe_stop_since is None:
                    self.non_safe_stop_since = now
                elif (now - self.non_safe_stop_since).nanoseconds / 1e9 >= 4.0:
                    self.safe_stop_enter_time = None
                    self.non_safe_stop_since = None
                    self.safe_stop_avoid_logged = False
            ceiling = reduced_v
            nav_mode = "REDUCED_SPEED"
        else:
            self.safe_stop_enter_time = None
            self.non_safe_stop_since = None
            self.safe_stop_avoid_logged = False
            ceiling = normal_v
            nav_mode = "NORMAL"

        if nav_mode != self.last_human_nav_mode or (wall_now - self.last_human_nav_mode_pub_wall) >= 0.5:
            self.last_human_nav_mode = nav_mode
            self.last_human_nav_mode_pub_wall = wall_now
            mode_msg = String()
            mode_msg.data = nav_mode
            self.human_nav_mode_pub.publish(mode_msg)

        return ceiling, is_safe_stop_active, nav_mode

    def cmd_callback(self, msg: Twist):
        self.in_v = msg.linear.x
        self.in_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()
        wall_now = time.time()

        # 1. Bootstrap TF map -> odom & Continuity Keeper (tanpa menimpa AMCL / initialpose):
        # - Saat startup sebelum AMCL init: siarkan bootstrap TF (0,0) pada 20 Hz.
        # - Setelah AMCL aktif: serahkan ke AMCL selama transform AMCL masih berlaku.
        # - Saat robot diam/jeda dan transform AMCL hampir kedaluwarsa (sisa < 0.35s)
        #   serta AMCL hening > 0.8s (perilaku Nav2 Foxy saat diam), siarkan ulang pada 5 Hz.
        scan_active = (wall_now - self.last_scan_wall_time) < 2.0
        if not scan_active:
            self.first_scan_wall_time = 0.0
        enc_ok = self.encoder_ok(wall_now)

        if not self.amcl_initialized:
            future_stamp = (now + rclpy.duration.Duration(seconds=0.1)).to_msg()
            self.publish_bootstrap_tf(future_stamp)
        else:
            now_sec = now.nanoseconds / 1e9
            amcl_expiring = now_sec >= (self.amcl_valid_until_sec - 0.35)
            amcl_silent = (wall_now - self.last_amcl_tf_wall) > 0.8
            if (
                wall_now >= self.initialpose_pause_until
                and amcl_expiring
                and amcl_silent
                and (wall_now - self.last_keepalive_pub_wall) >= 0.2
            ):
                self.last_keepalive_pub_wall = wall_now
                keep_stamp = (now + rclpy.duration.Duration(seconds=0.4)).to_msg()
                self.publish_bootstrap_tf(keep_stamp)

            # Deteksi dini jika node 'amcl' crash di tengah jalan
            if not self.amcl_crash_warned and (wall_now - self.last_amcl_alive_check_wall) >= 5.0:
                self.last_amcl_alive_check_wall = wall_now
                if 'amcl' not in self.get_node_names():
                    self.amcl_crash_warned = True
                    self.get_logger().error("🔴 [AMCL CRASH] Node 'amcl' terdeteksi mati di tengah navigasi!")
                    log_msg = String()
                    log_msg.data = "🔴 [AMCL CRASH] Node 'amcl' mati di tengah jalan - harap STOP & START ulang Navigasi."
                    self.delivery_log_pub.publish(log_msg)

        # Deteksi encoder hilang/kembali setelah sempat siap
        if self.encoder_was_ok and not enc_ok:
            self.get_logger().error("🔴 [ENCODER HILANG] Odometry Arduino tidak valid -> robot DIHENTIKAN sampai encoder pulih.")
            log_msg = String()
            log_msg.data = "🔴 ENCODER ARDUINO HILANG - robot dihentikan"
            self.delivery_log_pub.publish(log_msg)
        elif (not self.encoder_was_ok) and enc_ok and self.require_encoder:
            self.get_logger().info("✅ Encoder Arduino terverifikasi (/arduino/ready=True & /odom mengalir).")
        self.encoder_was_ok = enc_ok

        # 2. Kirim otomatis /initialpose sebaran 360° di Home saat /map, /scan_filtered & ENCODER siap tapi AMCL belum aktif.
        #    Beri jeda minimal 2.5 detik sejak paket /scan pertama agar AMCL selesai menginisialisasi
        #    createLaserObject dan likelihood field (mencegah Issue #2711 pf_update_resample).
        scan_stable = scan_active and (self.first_scan_wall_time > 0.0) and ((wall_now - self.first_scan_wall_time) >= 2.5)
        if (not self.amcl_initialized) and self.map_received and scan_stable and enc_ok and (not self.manual_initialpose_received):
            interval = 2.5 if self.initpose_sent_count < 5 else 4.0
            if (wall_now - self.start_wall_time > 4.0) and (wall_now - self.last_initpose_sent_time > interval):
                self.last_initpose_sent_time = wall_now
                self.initpose_sent_count += 1
                hx, hy, hyaw = self._load_home_pose()
                self.get_logger().info(
                    f"📍 Mengirim auto-initialpose Home ({hx:.2f}, {hy:.2f}) sebaran hadap 360° ke AMCL (percobaan #{self.initpose_sent_count})..."
                )
                self.send_auto_initialpose(now_msg, x=hx, y=hy, yaw=hyaw)

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # 2B. Jalankan mesin kalibrasi posisi & orientasi AMCL (nomotion -> align spin jika masih belum konvergen)
        align_override, align_v, align_w = self._step_startup_alignment(now_msg, wall_now, scan_stable, enc_ok, dt)

        # 3. Verifikasi kesiapan penuh (Peta + LiDAR + Encoder + AMCL + Auto-Align LOCKED + Costmap + Nav2 Stack Aktif)
        if not self.nav_ready_announced and (wall_now - self.last_ready_check_time >= 1.0):
            self.last_ready_check_time = wall_now
            if (
                self.map_received
                and scan_active
                and enc_ok
                and self.amcl_initialized
                and self.startup_align_phase == 'LOCKED'
                and self.local_costmap_active
                and self.global_costmap_active
            ):
                node_names = self.get_node_names()
                if 'bt_navigator' in node_names and 'planner_server' in node_names and 'controller_server' in node_names:
                    self.nav_ready_announced = True
                    self.get_logger().info("============================================================")
                    self.get_logger().info("🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR!")
                    self.get_logger().info("   ✔ Peta (/map)          : AKTIF")
                    self.get_logger().info("   ✔ Sensor LiDAR (/scan) : AKTIF")
                    if self.require_encoder:
                        self.get_logger().info("   ✔ Encoder Arduino(/odom): AKTIF & TERVERIFIKASI")
                    self.get_logger().info(
                        f"   ✔ Lokalisasi (AMCL)    : TERLOKALISASI (sigma_yaw={math.degrees(self.amcl_sigma_yaw):.1f}°)"
                    )
                    self.get_logger().info("   ✔ Local/Global Costmap : AKTIF")
                    self.get_logger().info("   ✔ Nav2 Stack           : SIAP (bt_navigator, planner, controller)")
                    self.get_logger().info("============================================================")
                    log_msg = String()
                    log_msg.data = "🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR! (Peta + LiDAR + Encoder + AMCL Terkunci + Costmap + Nav2 Aktif)"
                    self.delivery_log_pub.publish(log_msg)
            elif wall_now - self.last_wait_log_time >= 5.0:
                # Tampilkan apa saja yang masih ditunggu agar mudah didiagnosis
                self.last_wait_log_time = wall_now
                def mark(ok):
                    return "✔" if ok else "⏳"
                self.get_logger().info(
                    f"[MENUNGGU] Peta {mark(self.map_received)} | LiDAR {mark(scan_active)} | "
                    f"Encoder {mark(enc_ok)} | AMCL {mark(self.amcl_initialized)} | "
                    f"Align {self.startup_align_phase} | Costmap {mark(self.local_costmap_active and self.global_costmap_active)}"
                )
                if not self.amcl_initialized and self.initpose_sent_count >= 3:
                    node_names = self.get_node_names()
                    if 'amcl' not in node_names:
                        self.get_logger().error(
                            "🔴 [AMCL CRASH] Node 'amcl' terdeteksi mati! Harap matikan (STOP) dan nyalakan (START) ulang Navigasi di Dashboard."
                        )

        # Watchdog: Nav2 diam -> anggap perintah nol. Encoder tidak valid -> paksa nol juga.
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.get_parameter('cmd_timeout').value or not enc_ok:
            orig_v, orig_w = 0.0, 0.0
        else:
            orig_v, orig_w = self.in_v, self.in_w

        ceiling, is_safe_stop_active, _ = self._update_human_safety_mode(now, wall_now)

        # Saat sedang mengunci posisi/orientasi awal AMCL, jangan jalankan gerak maju Nav2 dulu
        if align_override:
            self.last_linear_v = 0.0
            out = Twist()
            out.linear.x = float(align_v)
            out.angular.z = float(align_w)
            self.pub.publish(out)
            return
        if self.startup_align_phase in ('SETTLE_POSE', 'NOMOTION_UPDATE'):
            orig_v, orig_w = 0.0, 0.0

        target_v = max(-ceiling, min(ceiling, orig_v))

        # Ramping dua arah (berlaku juga untuk kecepatan mundur)
        max_accel = self.get_parameter('max_accel').value
        max_decel = self.get_parameter('max_decel').value
        limit = (max_accel if abs(target_v) > abs(self.last_linear_v) else max_decel) * dt
        diff = target_v - self.last_linear_v
        actual_v = self.last_linear_v + max(-limit, min(limit, diff))
        self.last_linear_v = actual_v

        out = Twist()
        out.linear.x = actual_v

        # Teruskan kecepatan setir (angular.z) penuh dari Nav2 tanpa dipotong (mencegah understeer / manuver melingkar melebar).
        # Hanya matikan setir jika sedang dalam masa jeda diam penuh safe stop.
        if is_safe_stop_active:
            out.angular.z = 0.0
        else:
            out.angular.z = orig_w

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()