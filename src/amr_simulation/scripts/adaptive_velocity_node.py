#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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

        self.create_subscription(String, '/human_state', self.state_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.cmd_callback, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        # Loop kontrol tetap 20 Hz, independen dari Nav2
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Adaptive Velocity aktif")

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()