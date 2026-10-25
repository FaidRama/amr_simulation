#!/usr/bin/env python3
"""
adaptive_velocity_node.py
Local Path Planning Adaptif - Adaptive Velocity Constraint
Menerima perintah kecepatan dari Nav2 (/cmd_vel_nav), memodifikasi kecepatan
berdasarkan state deteksi manusia (/human_state), lalu mengirimkannya ke Arduino (/cmd_vel).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AdaptiveVelocityNode(Node):
    def __init__(self):
        super().__init__('adaptive_velocity_node')

        # Parameter Kecepatan Adaptif
        self.declare_parameter('normal_max_linear', 0.45)
        self.declare_parameter('reduced_max_linear', 0.18)
        self.declare_parameter('reduced_max_angular', 0.50)

        self.current_state = "NORMAL"

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

        self.get_logger().info("=== Node DWB Adaptif (Speed Governor) Aktif ===")

    def state_callback(self, msg: String):
        if self.current_state != msg.data:
            self.current_state = msg.data
            self.get_logger().info(f"[ADAPTIF] Perubahan State: {self.current_state}")

            # Jika masuk SAFE_STOP, langsung kirim rem darurat instan ke motor
            if self.current_state == "SAFE_STOP":
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.pub_final_cmd.publish(stop_cmd)

    def cmd_callback(self, in_cmd: Twist):
        out_cmd = Twist()
        out_cmd.linear.x = in_cmd.linear.x
        out_cmd.angular.z = in_cmd.angular.z

        normal_v = self.get_parameter('normal_max_linear').value
        reduced_v = self.get_parameter('reduced_max_linear').value
        reduced_w = self.get_parameter('reduced_max_angular').value

        # Logika Adaptive Velocity Constraint
        if self.current_state == "NORMAL":
            if out_cmd.linear.x > normal_v:
                # Jaga proporsi sudut belok jika linear dipotong
                scale = normal_v / out_cmd.linear.x
                out_cmd.linear.x = normal_v
                out_cmd.angular.z *= scale

        elif self.current_state == "REDUCED_SPEED":
            if out_cmd.linear.x > reduced_v:
                scale = reduced_v / out_cmd.linear.x
                out_cmd.linear.x = reduced_v
                out_cmd.angular.z *= scale
            if abs(out_cmd.angular.z) > reduced_w:
                direction = 1.0 if out_cmd.angular.z > 0 else -1.0
                out_cmd.angular.z = direction * reduced_w
            
        elif self.current_state == "SAFE_STOP":
            # State SAFE_STOP: Berhenti penuh
            out_cmd.linear.x = 0.0
            out_cmd.angular.z = 0.0

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