#!/usr/bin/env python3
"""
adaptive_velocity_node.py
Local Path Planning Adaptif - Adaptive Velocity Constraint & Dynamic Evade
Fitur:
1. Menunggu manusia di zona bahaya (SAFE_STOP) selama batas waktu tertentu (default: 15 detik).
2. Jika manusia tidak kunjung pergi setelah timeout, robot TIDAK MOGOK/ABORT,
   melainkan mengizinkan Nav2 bermanuver pelan (EVADE) mencari celah memutar atau mundur.
3. Menjaga proporsi kurva belokan DWA (Proportional Curvature) agar tidak membanting stir.
4. Ramping halus dua arah (akselerasi & deselerasi aman untuk motor/odometri).
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
        self.declare_parameter('reduced_max_linear', 0.20)  # Kecepatan aman saat mendekati manusia
        self.declare_parameter('evade_max_linear', 0.12)    # Kecepatan merayap pelan saat mencari rute elak
        self.declare_parameter('safe_stop_timeout', 15.0)   # Menunggu orang max 15 detik sebelum mulai menghindar
        self.declare_parameter('max_accel', 0.4)            # Akselerasi naik lembut (m/s^2)
        self.declare_parameter('max_decel', 0.8)            # Deselerasi turun aman (m/s^2)
        self.declare_parameter('state_hold_time', 1.5)      # Tahan minimal 1.5 detik peredam flicker kamera

        self.current_state = "NORMAL"
        self.effective_state = "NORMAL"
        self.last_state_change_time = self.get_clock().now()
        
        # Timer pelacak durasi berhenti di SAFE_STOP
        self.safe_stop_start_time = None
        self.evade_logged = False

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

        self.get_logger().info("=== Node Adaptive Velocity (Dynamic Evade & Curvature Scaling) Siap ===")

    def state_callback(self, msg: String):
        new_state = msg.data
        now = self.get_clock().now()

        severity = {"NORMAL": 0, "REDUCED_SPEED": 1, "SAFE_STOP": 2}
        new_sev = severity.get(new_state, 0)
        curr_sev = severity.get(self.effective_state, 0)

        # Jika bahaya meningkat: eksekusi langsung demi keselamatan
        if new_sev >= curr_sev:
            if self.effective_state != new_state:
                self.get_logger().info(f"[ADAPTIF] Eskalasi State: {self.effective_state} -> {new_state}")
                if new_state == "SAFE_STOP":
                    self.safe_stop_start_time = now
                    self.evade_logged = False

            self.effective_state = new_state
            self.last_state_change_time = now

        else:
            # Jika bahaya mereda: tahan dulu selama state_hold_time
            dt_hold = (now - self.last_state_change_time).nanoseconds / 1e9
            hold_limit = self.get_parameter('state_hold_time').value
            if dt_hold >= hold_limit:
                if self.effective_state != new_state:
                    self.get_logger().info(f"[ADAPTIF] Relaksasi State: {self.effective_state} -> {new_state}")
                self.effective_state = new_state
                self.last_state_change_time = now
                if new_state != "SAFE_STOP":
                    self.safe_stop_start_time = None
                    self.evade_logged = False

        self.current_state = new_state

    def cmd_callback(self, in_cmd: Twist):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1

        normal_max_v = self.get_parameter('normal_max_linear').value
        reduced_max_v = self.get_parameter('reduced_max_linear').value
        evade_max_v = self.get_parameter('evade_max_linear').value
        safe_timeout = self.get_parameter('safe_stop_timeout').value
        max_accel = self.get_parameter('max_accel').value
        max_decel = self.get_parameter('max_decel').value

        orig_v = in_cmd.linear.x
        orig_w = in_cmd.angular.z

        # -------------------------------------------------------------
        # 1. EVALUASI LOGIKA SAFE_STOP TIMEOUT (DYNAMIC EVADE)
        # -------------------------------------------------------------
        is_evading = False
        target_ceiling = normal_max_v

        if self.effective_state == "NORMAL":
            target_ceiling = normal_max_v

        elif self.effective_state == "REDUCED_SPEED":
            target_ceiling = reduced_max_v

        elif self.effective_state == "SAFE_STOP":
            if self.safe_stop_start_time is None:
                self.safe_stop_start_time = now

            dt_stopped = (now - self.safe_stop_start_time).nanoseconds / 1e9

            if dt_stopped < safe_timeout:
                # Masih dalam rentang waktu toleransi (0 - 15 detik): BERHENTI PENUH
                target_ceiling = 0.0
            else:
                # Sudah menunggu lebih dari 15 detik dan orang belum pergi:
                # BUKA MODE EVADE agar Nav2 bisa bermanuver memutar / mundur pelan
                is_evading = True
                target_ceiling = evade_max_v
                if not self.evade_logged:
                    self.get_logger().warn(
                        f"[ADAPTIF] Manusia diam > {safe_timeout:.0f} detik! Mengaktifkan mode EVADE untuk mencari jalan elak."
                    )
                    self.evade_logged = True

        # Tentukan target kecepatan maju (atau mundur jika Nav2 recovery)
        target_v = orig_v
        if target_v > target_ceiling:
            target_v = target_ceiling
        elif not is_evading and self.effective_state == "SAFE_STOP":
            target_v = 0.0

        # -------------------------------------------------------------
        # 2. RAMPING HALUS DUA ARAH (Smooth Acceleration & Deceleration)
        # -------------------------------------------------------------
        if target_v > self.last_linear_v:
            actual_v = min(target_v, self.last_linear_v + (max_accel * dt))
        else:
            actual_v = max(target_v, self.last_linear_v - (max_decel * dt))

        self.last_linear_v = actual_v

        # -------------------------------------------------------------
        # 3. KOREKSI BELOKAN (Memastikan Nav2 Bisa Bermanuver Menghindar)
        # -------------------------------------------------------------
        out_cmd = Twist()
        out_cmd.linear.x = actual_v

        if not is_evading and self.effective_state == "SAFE_STOP":
            # Berhenti darurat penuh saat awal deteksi
            out_cmd.angular.z = 0.0
        elif is_evading:
            # Mode Evade: Izinkan rotasi penuh dari Nav2 untuk mencari celah keluar
            out_cmd.angular.z = orig_w
        elif abs(orig_v) > 0.02:
            # Proporsional kurva DWA saat melambat biasa
            scale_ratio = actual_v / orig_v
            effective_scale = max(0.4, min(1.0, scale_ratio))
            out_cmd.angular.z = orig_w * effective_scale
        else:
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
