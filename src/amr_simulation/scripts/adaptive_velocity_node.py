#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2_ACTION = True
except ImportError:
    HAS_NAV2_ACTION = False


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

        # --- NAV2 BOOTSTRAP TF (map -> odom -> base_link -> laser/base_footprint) ---
        # Menjamin seluruh pohon TF selalu utuh 100% sejak detik ke-0 agar AMCL,
        # global_costmap, local_costmap, dan RViz tidak pernah kehilangan frame 'laser' atau 'map'.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.start_wall_time = time.time()
        self.amcl_initialized = False
        self.map_received = False
        self.nav_ready_announced = False
        self.last_ready_check_time = 0.0
        self.last_scan_wall_time = 0.0
        self.last_odom_wall_time = 0.0
        self.last_initpose_sent_time = 0.0
        self.initpose_sent_count = 0
        self.last_map_odom_x = 0.0
        self.last_map_odom_y = 0.0
        self.last_map_odom_qz = 0.0
        self.last_map_odom_qw = 1.0

        # Siarkan Static TF (base_link -> laser & base_link -> base_footprint) secara permanen
        self.publish_static_robot_transforms()

        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, tl_qos)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, tl_qos)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.delivery_log_pub = self.create_publisher(String, '/amr/delivery_log', 10)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') if HAS_NAV2_ACTION else None

        self.create_subscription(String, '/human_state', self.state_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.cmd_callback, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        # Loop kontrol tetap 20 Hz, independen dari Nav2
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Adaptive Velocity & Nav2 Bootstrap TF aktif")

    def publish_static_robot_transforms(self):
        """Pastikan base_link -> laser dan base_link -> base_footprint selalu tersedia di /tf_static."""
        now_msg = self.get_clock().now().to_msg()

        # 1. base_link -> laser (yaw = 180 deg / pi rad)
        t_laser = TransformStamped()
        t_laser.header.stamp = now_msg
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.0
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = math.sin(3.14159 / 2.0)
        t_laser.transform.rotation.w = math.cos(3.14159 / 2.0)

        # 2. base_link -> base_footprint (identitas)
        t_fp = TransformStamped()
        t_fp.header.stamp = now_msg
        t_fp.header.frame_id = 'base_link'
        t_fp.child_frame_id = 'base_footprint'
        t_fp.transform.translation.x = 0.0
        t_fp.transform.translation.y = 0.0
        t_fp.transform.translation.z = 0.0
        t_fp.transform.rotation.x = 0.0
        t_fp.transform.rotation.y = 0.0
        t_fp.transform.rotation.z = 0.0
        t_fp.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([t_laser, t_fp])

    def on_map(self, msg: OccupancyGrid):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info("🗺️ Peta (/map) terdeteksi oleh Nav2 Bootstrap.")

    def on_scan(self, msg: LaserScan):
        self.last_scan_wall_time = time.time()

    def on_odom(self, msg: Odometry):
        self.last_odom_wall_time = time.time()

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        if not self.amcl_initialized:
            self.amcl_initialized = True
            self.get_logger().info("✅ AMCL telah aktif (/amcl_pose diterima) -> TF map->odom dikelola penuh oleh AMCL.")

    def publish_bootstrap_tf(self, future_msg, publish_odom_fallback=False):
        transforms = []

        # 1. Bootstrap map -> odom (sebelum AMCL aktif)
        if not self.amcl_initialized:
            t_map = TransformStamped()
            t_map.header.stamp = future_msg
            t_map.header.frame_id = 'map'
            t_map.child_frame_id = 'odom'
            t_map.transform.translation.x = self.last_map_odom_x
            t_map.transform.translation.y = self.last_map_odom_y
            t_map.transform.translation.z = 0.0
            t_map.transform.rotation.x = 0.0
            t_map.transform.rotation.y = 0.0
            t_map.transform.rotation.z = self.last_map_odom_qz
            t_map.transform.rotation.w = self.last_map_odom_qw
            transforms.append(t_map)

        # 2. Fallback odom -> base_link (hanya saat Arduino sedang booting/reset dan belum publish /odom)
        if publish_odom_fallback:
            t_odom = TransformStamped()
            t_odom.header.stamp = future_msg
            t_odom.header.frame_id = 'odom'
            t_odom.child_frame_id = 'base_link'
            t_odom.transform.translation.x = 0.0
            t_odom.transform.translation.y = 0.0
            t_odom.transform.translation.z = 0.0
            t_odom.transform.rotation.x = 0.0
            t_odom.transform.rotation.y = 0.0
            t_odom.transform.rotation.z = 0.0
            t_odom.transform.rotation.w = 1.0
            transforms.append(t_odom)

        # 3. Pastikan base_link -> laser & base_footprint juga disiarkan di /tf agar AMCL MessageFilter tidak pernah drop
        t_laser = TransformStamped()
        t_laser.header.stamp = future_msg
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.0
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = math.sin(3.14159 / 2.0)
        t_laser.transform.rotation.w = math.cos(3.14159 / 2.0)
        transforms.append(t_laser)

        t_fp = TransformStamped()
        t_fp.header.stamp = future_msg
        t_fp.header.frame_id = 'base_link'
        t_fp.child_frame_id = 'base_footprint'
        t_fp.transform.translation.x = 0.0
        t_fp.transform.translation.y = 0.0
        t_fp.transform.translation.z = 0.0
        t_fp.transform.rotation.x = 0.0
        t_fp.transform.rotation.y = 0.0
        t_fp.transform.rotation.z = 0.0
        t_fp.transform.rotation.w = 1.0
        transforms.append(t_fp)

        self.tf_broadcaster.sendTransform(transforms)

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
        future_msg = (now + rclpy.duration.Duration(seconds=0.10)).to_msg()
        wall_now = time.time()

        # 1. Bootstrap & pengaman rantai TF (map -> odom -> base_link -> laser/base_footprint)
        scan_active = (wall_now - self.last_scan_wall_time) < 2.0
        odom_active = (wall_now - self.last_odom_wall_time) < 1.0
        self.publish_bootstrap_tf(future_msg, publish_odom_fallback=(not odom_active))

        # 2. Kirim otomatis /initialpose (0,0,0) maksimal 2 kali saat /map & /scan_filtered sudah siap tapi AMCL belum init
        if (not self.amcl_initialized) and self.map_received and scan_active:
            if (wall_now - self.start_wall_time > 3.5) and (wall_now - self.last_initpose_sent_time > 3.0) and (self.initpose_sent_count < 2):
                self.last_initpose_sent_time = wall_now
                self.initpose_sent_count += 1
                self.send_auto_initialpose(now_msg)

        # 3. Verifikasi kesiapan penuh (Peta + LiDAR + AMCL + Nav2 Action Server /navigate_to_pose)
        if not self.nav_ready_announced and (wall_now - self.last_ready_check_time >= 0.8):
            self.last_ready_check_time = wall_now
            if self.map_received and scan_active and self.amcl_initialized:
                nav_srv_ok = self.nav_action_client.server_is_ready() if self.nav_action_client is not None else (wall_now - self.start_wall_time > 7.0)
                if nav_srv_ok:
                    self.nav_ready_announced = True
                    self.get_logger().info("============================================================")
                    self.get_logger().info("🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR!")
                    self.get_logger().info("   ✔ Peta (/map)          : AKTIF")
                    self.get_logger().info("   ✔ Sensor LiDAR (/scan) : AKTIF")
                    self.get_logger().info("   ✔ Lokalisasi (AMCL)    : TERLOKALISASI")
                    self.get_logger().info("   ✔ Nav2 Action Server   : SIAP (/navigate_to_pose)")
                    self.get_logger().info("============================================================")
                    log_msg = String()
                    log_msg.data = "🟢 [SIAP NAVIGASI] AMR SIAP UNTUK NAVIGASI & MENGANTAR! (Peta + LiDAR + AMCL + Nav2 Action Server Aktif)"
                    self.delivery_log_pub.publish(log_msg)

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        # Watchdog: Nav2 diam -> anggap perintah nol
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.get_parameter('cmd_timeout').value:
            orig_v, orig_w = 0.0, 0.0
        else:
            orig_v, orig_w = self.in_v, self.in_w

        if self.effective_state == "SAFE_STOP":
            ceiling = 0.0
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

        if self.effective_state == "SAFE_STOP":
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