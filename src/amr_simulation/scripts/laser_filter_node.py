#!/usr/bin/env python3
"""
Node Filter Laser Sasis AMR
- Menyaring dan memotong pantulan 4 tiang aluminium sasis robot
- Mengubah jarak pantulan < min_range (default 0.32m) menjadi NaN
- Topik input: /scan -> Topik output: /scan_filtered
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter')

        self.declare_parameter('min_range', 0.32)
        self.declare_parameter('max_range', 12.0)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.get_logger().info(
            f"Laser Filter Sasis aktif: memotong pantulan < {self.min_range}m (4 tiang sasis)"
        )

    def scan_callback(self, msg: LaserScan):
        # Gandakan array ranges dan ganti objek di dalam sasis (< min_range) menjadi NaN
        filtered_ranges = []
        for r in msg.ranges:
            if math.isnan(r) or r < self.min_range or r > self.max_range:
                filtered_ranges.append(float('nan'))
            else:
                filtered_ranges.append(r)

        msg.ranges = filtered_ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
