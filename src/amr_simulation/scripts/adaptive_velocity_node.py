#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped, PolygonStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


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

        self.effective_state = "NORMAL"
        self.last_state_change_time = self.get_clock().now()

        self.in_v = 0.0
        self.in_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.last_linear_v = 0.0
        self.last_time = self.get_clock().now()

        # --- NAV2 BOOTSTRAP TF & CONTINUITY KEEPER (map -> odom) ---
        # Menjamin frame 'map' selalu ada sejak detik ke-0 DAN tidak pernah kedaluwarsa
        # di TF buffer saat robot diam lama / standby antar-pengantaran.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_amcl_tf_wall = 0.0
        self.start_wall_time = time.time()
        self.amcl_initialized = False
        self.map_received = False
        self.local_costmap_active = False
        self.global_costmap_active = False
        self.nav_ready_announced = False
        self.last_ready_check_time = 0.0
        self.last_scan_wall_time = 0.0
        self.first_scan_wall_time = 0.0
        self.last_initpose_sent_time = 0.0
        self.initpose_sent_count = 0
        self.last_map_odom_x = 0.0
        self.last_map_odom_y = 0.0
        self.last_map_odom_qz = 0.0
        self.last_map_odom_qw = 1.0

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
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, tl_qos)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self.on_local_footprint, 5)
        self.create_subscription(PolygonStamped, '/global_costmap/published_footprint', self.on_global_footprint, 5)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', tl_qos)
        self.initialpose_pub_vol = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.delivery_log_pub = self.create_publisher(String, '/amr/delivery_log', 10)

        self.create_subscription(String, '/human_state', self.state_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.cmd_callback, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        # Loop kontrol tetap 20 Hz, independen dari Nav2
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Adaptive Velocity & Nav2 Bootstrap TF aktif")

    def on_map(self, msg: OccupancyGrid):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info("🗺️ Peta (/map) terdeteksi oleh Nav2 Bootstrap.")

    def on_scan(self, msg: LaserScan):
        now = time.time()
        if self.first_scan_wall_time == 0.0:
            self.first_scan_wall_time = now
        self.last_scan_wall_time = now

    def on_arduino_ready(self, msg: Bool):
        self.arduino_ready = bool(msg.data)

    def on_odom(self, msg: Odometry):
        self.last_odom_wall_time = time.time()

    def encoder_ok(self, wall_now: float) -> bool:
        if not self.require_encoder:
            return True
        return self.arduino_ready and (wall_now - self.last_odom_wall_time) < 1.0

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_tf_wall = time.time()
        if not self.amcl_initialized:
            self.amcl_initialized = True
            self.get_logger().info("✅ AMCL telah aktif (/amcl_pose diterima) -> TF map->odom dikelola penuh oleh AMCL.")

    def on_local_footprint(self, msg: PolygonStamped):
        self.local_costmap_active = True

    def on_global_footprint(self, msg: PolygonStamped):
        self.global_costmap_active = True

    def publish_bootstrap_tf(self, now_msg):
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

    def send_auto_initialpose(self, now_msg):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now_msg
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initialpose_pub.publish(msg)
        self.initialpose_pub_vol.publish(msg)

    def state_callback(self, msg: String):
        severity = {"NORMAL": 0, "REDUCED_SPEED": 1, "SAFE_STOP": 2}
        new_state = msg.data
        now = self.get_clock().now()
        new_sev = severity.get(new_state, 0)
        curr_sev = severity.get(self.effective_state, 0)

        if new_sev >= curr_sev:
            if self.effective_state != new_state:
                self.get_logger().info(f"[ADAPTIF] {self.effective_state} -> {new_state}")
            self.effective_state = new_state
            self.last_state_change_time = now
        else:
            dt_hold = (now - self.last_state_change_time).nanoseconds / 1e9
            if dt_hold >= self.get_parameter('state_hold_time').value:
                self.get_logger().info(f"[ADAPTIF] Relaksasi {self.effective_state} -> {new_state}")
                self.effective_state = new_state
                self.last_state_change_time = now

    def cmd_callback(self, msg: Twist):
        self.in_v = msg.linear.x
        self.in_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        now_msg = now.to_msg()
        wall_now = time.time()

        # Pantau transform map -> odom dari AMCL yang ada di TF buffer
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.last_map_odom_x = t.transform.translation.x
            self.last_map_odom_y = t.transform.translation.y
            self.last_map_odom_qz = t.transform.rotation.z
            self.last_map_odom_qw = t.transform.rotation.w

            t_sec = t.header.stamp.sec + (t.header.stamp.nanosec / 1e9)
            now_sec = now.nanoseconds / 1e9
            if abs(now_sec - t_sec) < 0.4:
                self.last_amcl_tf_wall = wall_now
        except Exception:
            pass

        # 1. Bootstrap TF map -> odom & TF Continuity Keeper:
        # Menjamin frame 'map' selalu terhubung ke 'odom' dan TIDAK PERNAH KEDALUWARSA di TF buffer!
        # - Saat startup sebelum AMCL init: siarkan bootstrap TF (0,0)
        # - Saat AMCL sedang hening > 0.4s (misal robot diam lama saat standby / menunggu dispatch baru):
        #   siarkan ulang transform map->odom terakhir yang valid dengan stempel waktu sekarang (now + 0.1s).
        scan_active = (wall_now - self.last_scan_wall_time) < 2.0
        if not scan_active:
            self.first_scan_wall_time = 0.0
        enc_ok = self.encoder_ok(wall_now)

        amcl_silent = (wall_now - self.last_amcl_tf_wall) > 0.4
        if (not self.amcl_initialized) or amcl_silent:
            future_stamp = (now + rclpy.duration.Duration(seconds=0.1)).to_msg()
            self.publish_bootstrap_tf(future_stamp)

        # Deteksi encoder hilang/kembali setelah sempat siap
        if self.encoder_was_ok and not enc_ok:
            self.get_logger().error("🔴 [ENCODER HILANG] Odometry Arduino tidak valid -> robot DIHENTIKAN sampai encoder pulih.")
            log_msg = String()
            log_msg.data = "🔴 ENCODER ARDUINO HILANG - robot dihentikan"
            self.delivery_log_pub.publish(log_msg)
        elif (not self.encoder_was_ok) and enc_ok and self.require_encoder:
            self.get_logger().info("✅ Encoder Arduino terverifikasi (/arduino/ready=True & /odom mengalir).")
        self.encoder_was_ok = enc_ok

        # 2. Kirim otomatis /initialpose (0,0,0) saat /map, /scan_filtered & ENCODER sudah siap tapi AMCL belum init.
        #    Beri jeda minimal 2.5 detik sejak paket /scan pertama agar AMCL selesai menginisialisasi
        #    createLaserObject dan likelihood field. Ini mencegah bug race-condition Nav2 pf_update_resample
        #    assertion failed (Issue #2711) yang mematikan proses AMCL.
        scan_stable = scan_active and (self.first_scan_wall_time > 0.0) and ((wall_now - self.first_scan_wall_time) >= 2.5)
        if (not self.amcl_initialized) and self.map_received and scan_stable and enc_ok:
            interval = 2.5 if self.initpose_sent_count < 5 else 4.0
            if (wall_now - self.start_wall_time > 4.0) and (wall_now - self.last_initpose_sent_time > interval):
                self.last_initpose_sent_time = wall_now
                self.initpose_sent_count += 1
                self.get_logger().info(f"📍 Mengirim auto-initialpose ke AMCL (percobaan #{self.initpose_sent_count})...")
                self.send_auto_initialpose(now_msg)

        # 3. Verifikasi kesiapan penuh (Peta + LiDAR + Encoder + AMCL + Local/Global Costmap + Nav2 Stack Aktif)
        if not self.nav_ready_announced and (wall_now - self.last_ready_check_time >= 1.0):
            self.last_ready_check_time = wall_now
            if (
                self.map_received
                and scan_active
                and enc_ok
                and self.amcl_initialized
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
                    self.get_logger().info("   ✔ Lokalisasi (AMCL)    : TERLOKALISASI")
                    self.get_logger().info("   ✔ Local/Global Costmap : AKTIF")
                    self.get_logger().info("   ✔ Nav2 Stack           : SIAP (bt_navigator, planner, controller)")
                    self.get_logger().info("============================================================")
                    log_msg = String()
                    log_msg.data = "🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR! (Peta + LiDAR + Encoder + AMCL + Costmap + Nav2 Aktif)"
                    self.delivery_log_pub.publish(log_msg)
            elif wall_now - self.last_wait_log_time >= 5.0:
                # Tampilkan apa saja yang masih ditunggu agar mudah didiagnosis
                self.last_wait_log_time = wall_now
                def mark(ok):
                    return "✔" if ok else "⏳"
                self.get_logger().info(
                    f"[MENUNGGU] Peta {mark(self.map_received)} | LiDAR {mark(scan_active)} | "
                    f"Encoder {mark(enc_ok)} | AMCL {mark(self.amcl_initialized)} | "
                    f"Costmap {mark(self.local_costmap_active and self.global_costmap_active)}"
                )
                if not self.amcl_initialized and self.initpose_sent_count >= 3:
                    node_names = self.get_node_names()
                    if 'amcl' not in node_names:
                        self.get_logger().error(
                            "🔴 [AMCL CRASH] Node 'amcl' terdeteksi mati! Harap matikan (STOP) dan nyalakan (START) ulang Navigasi di Dashboard."
                        )

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # Watchdog: Nav2 diam -> anggap perintah nol. Encoder tidak valid -> paksa nol juga.
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.get_parameter('cmd_timeout').value or not enc_ok:
            orig_v, orig_w = 0.0, 0.0
        else:
            orig_v, orig_w = self.in_v, self.in_w

        # Cek durasi jika sedang berada di status SAFE_STOP
        safe_stop_duration = (now - self.last_state_change_time).nanoseconds / 1e9 if self.effective_state == "SAFE_STOP" else 0.0

        # Logika Jeda 5 Detik:
        # 1. Jika SAFE_STOP < 5 detik: robot berhenti penuh (beri waktu orang lewat).
        # 2. Jika SAFE_STOP >= 5 detik tapi orang tidak pergi: buka kecepatan pelan (REDUCED)
        #    dan aktifkan setir (angular) agar Nav2 DWB membelok menghindar!
        is_safe_stop_active = False
        if self.effective_state == "SAFE_STOP":
            if safe_stop_duration < 5.0:
                ceiling = 0.0
                is_safe_stop_active = True
            else:
                # Setelah 5 detik orang tidak pergi: perlakukan sebagai rintangan biasa (REDUCED SPEED)
                ceiling = self.get_parameter('reduced_max_linear').value
        elif self.effective_state == "REDUCED_SPEED":
            ceiling = self.get_parameter('reduced_max_linear').value
        else:
            ceiling = self.get_parameter('normal_max_linear').value

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

        # Hanya matikan setir jika masih dalam masa jeda 5 detik pertama
        if is_safe_stop_active:
            out.angular.z = 0.0
        elif abs(orig_v) > 0.02:
            scale = max(0.4, min(1.0, actual_v / orig_v))
            out.angular.z = orig_w * scale
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