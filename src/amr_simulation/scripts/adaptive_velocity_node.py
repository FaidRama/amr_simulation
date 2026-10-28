#!/usr/bin/env python3
"""
adaptive_velocity_node.py
Local Path Planning Adaptif - Adaptive Velocity Constraint
Menerima perintah kecepatan dari Nav2 (/cmd_vel_nav), memodifikasi kecepatan
berdasarkan state deteksi manusia (/human_state) secara proporsional (menjaga kurva belok DWA),
dilengkapi dengan Hysteresis Hold-Time & Smooth Ramping agar tidak terjadi bentrokan osilasi.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AdaptiveVelocityNode(Node):
    def __init__(self):
        super().__init__('adaptive_velocity_node')

        # 1. Parameter Kecepatan Adaptif (Sinkron dengan nav2_params.yaml)
        self.declare_parameter('normal_max_linear', 0.45)   # Sesuai max_vel_x Nav2
        self.declare_parameter('reduced_max_linear', 0.20)  # Kecepatan aman saat ada manusia
        self.declare_parameter('max_accel', 0.4)            # Akselerasi naik lembut (m/s^2)
        self.declare_parameter('max_decel', 0.8)            # Deselerasi turun aman (m/s^2)
        self.declare_parameter('state_hold_time', 1.5)      # Tahan minimal 1.5 detik agar kamera tidak flicker

        self.current_state = "NORMAL"
        self.effective_state = "NORMAL"
        self.last_state_change_time = self.get_clock().now()

        self.last_linear_v = 0.0
        self.last_time = self.get_clock().now()

        # Subscription ke state kamera/FSM
        self.sub_state = self.create_subscription(
            String,
            '/human_state',
            self.state_callback,
            10
        )

        # Subscription ke output DWB Local Planner Nav2
        self.sub_nav_cmd = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_callback,
            10
        )

        # Publisher akhir ke Arduino Driver
        self.pub_final_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info("=== Node Adaptive Velocity (Anti-Bentrokan & Proportional Curvature) Aktif ===")

    def state_callback(self, msg: String):
        new_state = msg.data
        now = self.get_clock().now()

        # Prioritas keselamatan: Jika bahaya meningkat (masuk REDUCED_SPEED atau SAFE_STOP), eksekusi INSTAN
        severity = {"NORMAL": 0, "REDUCED_SPEED": 1, "SAFE_STOP": 2}
        new_sev = severity.get(new_state, 0)
        curr_sev = severity.get(self.effective_state, 0)

        if new_sev >= curr_sev:
            if self.effective_state != new_state:
                self.get_logger().info(f"[ADAPTIF] Eskalasi State Aman: {self.effective_state} -> {new_state}")
            self.effective_state = new_state
            self.last_state_change_time = now
        else:
            # Jika bahaya mereda (misal SAFE_STOP -> NORMAL karena robot berbelok dan orang lepas dari kamera),
            # TAHAN dulu selama state_hold_time agar robot tidak menyentak bolak-balik
            dt_hold = (now - self.last_state_change_time).nanoseconds / 1e9
            hold_limit = self.get_parameter('state_hold_time').value
            if dt_hold >= hold_limit:
                if self.effective_state != new_state:
                    self.get_logger().info(f"[ADAPTIF] Relaksasi State: {self.effective_state} -> {new_state}")
                self.effective_state = new_state
                self.last_state_change_time = now

        self.current_state = new_state

    def cmd_callback(self, in_cmd: Twist):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1  # Fallback nilai dt jika loop tersendat

        normal_max_v = self.get_parameter('normal_max_linear').value
        reduced_max_v = self.get_parameter('reduced_max_linear').value
        max_accel = self.get_parameter('max_accel').value
        max_decel = self.get_parameter('max_decel').value

        orig_v = in_cmd.linear.x
        orig_w = in_cmd.angular.z

        # 1. Tentukan target batas kecepatan berdasarkan effective_state
        if self.effective_state == "NORMAL":
            target_ceiling = normal_max_v
        elif self.effective_state == "REDUCED_SPEED":
            target_ceiling = reduced_max_v
        elif self.effective_state == "SAFE_STOP":
            target_ceiling = 0.0
        else:
            target_ceiling = normal_max_v

        # Pangkas target v linear sesuai ceiling
        target_v = orig_v
        if target_v > target_ceiling:
            target_v = target_ceiling
        elif self.effective_state == "SAFE_STOP":
            target_v = 0.0

        # 2. RAMPING HALUS DUA ARAH (Mencegah hentakan akselerasi maupun pengereman)
        if target_v > self.last_linear_v:
            # Akselerasi naik dibatasi max_accel
            actual_v = min(target_v, self.last_linear_v + (max_accel * dt))
        else:
            # Deselerasi turun dibatasi max_decel
            actual_v = max(target_v, self.last_linear_v - (max_decel * dt))

        self.last_linear_v = actual_v

        # 3. KOREKSI PROPORSI BELOKAN (Menjaga Radius Kurva Nav2 DWA)
        # Jika v diperlambat, w juga diskalakan proporsional agar robot tidak berputar di tempat (pivot)
        out_cmd = Twist()
        out_cmd.linear.x = actual_v

        if self.effective_state == "SAFE_STOP":
            out_cmd.angular.z = 0.0
        elif abs(orig_v) > 0.02:
            scale_ratio = actual_v / orig_v
            # Batasi scaling antara 0.4 sampai 1.0 agar belokan tetap mulus dan tidak ekstrem
            effective_scale = max(0.4, min(1.0, scale_ratio))
            out_cmd.angular.z = orig_w * effective_scale
        else:
            # Jika memang DWA merencanakan putar di tempat murni (orig_v ~ 0), biarkan orig_w asli
            out_cmd.angular.z = orig_w

        self.pub_final_cmd.publish(out_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
