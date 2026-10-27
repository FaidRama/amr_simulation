#!/usr/bin/env python3
"""
adaptive_velocity_node.py
Local Path Planning Adaptif - Adaptive Velocity Constraint
Menerima perintah kecepatan dari Nav2 (/cmd_vel_nav), memodifikasi kecepatan maju (linear.x)
berdasarkan state deteksi manusia (/human_state), dengan transisi halus (ramping),
lalu mengirimkannya ke Arduino Driver (/cmd_vel).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AdaptiveVelocityNode(Node):
    def __init__(self):
        super().__init__('adaptive_velocity_node')

        # 1. Parameter Kecepatan Adaptif (Sinkron dengan Nav2)
        self.declare_parameter('normal_max_linear', 0.45)   # Sesuai max_vel_x di nav2_params.yaml
        self.declare_parameter('reduced_max_linear', 0.18)  # Batas aman saat ada manusia di zona waspada
        self.declare_parameter('max_deceleration', 0.8)     # Batas perlambatan halus (m/s^2)

        self.current_state = "NORMAL"
        self.last_linear_x = 0.0
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

        self.get_logger().info("=== Node DWB Adaptif (Speed Governor) Aktif & Sinkron ===")

    def state_callback(self, msg: String):
        if self.current_state != msg.data:
            self.current_state = msg.data
            self.get_logger().info(f"[ADAPTIF] Perubahan State: {self.current_state}")

    def cmd_callback(self, in_cmd: Twist):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.1  # Nilai default fallback

        normal_v = self.get_parameter('normal_max_linear').value
        reduced_v = self.get_parameter('reduced_max_linear').value
        max_decel = self.get_parameter('max_deceleration').value

        out_cmd = Twist()
        # PENTING: Jangan sunat angular.z! Biarkan DWA leluasa membelok menghindar
        out_cmd.angular.z = in_cmd.angular.z

        # Tentukan target ceiling kecepatan linear berdasarkan state
        target_v = in_cmd.linear.x

        if self.current_state == "NORMAL":
            target_v = min(target_v, normal_v)

        elif self.current_state == "REDUCED_SPEED":
            target_v = min(target_v, reduced_v)

        elif self.current_state == "SAFE_STOP":
            target_v = 0.0
            # Jika berhenti darurat, hentikan rotasi juga
            out_cmd.angular.z = 0.0

        # RAMPING / SMOOTHING: Turunkan kecepatan secara mulus (tidak patah/hentak)
        if target_v < self.last_linear_x:
            allowed_drop = max_decel * dt
            actual_v = max(target_v, self.last_linear_x - allowed_drop)
        else:
            actual_v = target_v

        out_cmd.linear.x = actual_v
        self.last_linear_x = actual_v

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