#!/usr/bin/env python3
"""
Node Filter Laser Sasis AMR
- Menyaring dan memotong pantulan 4 tiang aluminium sasis robot
- Menggunakan penyaringan berbasis Geometri Persegi (Box Filter) sesuai bodi sasis (40x40 cm)
- Menghilangkan zona buta 12 cm di depan bemper yang sebelumnya terjadi akibat filter lingkaran (radius cut)
- Mengubah titik pantulan di dalam sasis menjadi NaN
- Topik input: /scan -> Topik output: /scan_filtered
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter')

        # Mode filter: 'box' (default rekomendasi) atau 'circle' (legacy)
        self.declare_parameter('filter_type', 'box')
        
        # Batas kotak sasis robot (dalam kerangka base_link, satuan meter)
        # Sasis fisik berukuran 40x40 cm (setengah sisi 20 cm = 0.20m).
        # Diberi margin aman 1.5 cm -> [-0.215, 0.215] meter.
        # Batas ini membungkus sempurna ke-4 tiang aluminium sasis di pojok (jarak sqrt(20^2+20^2) = 28.3 cm).
        self.declare_parameter('box_min_x', -0.215)
        self.declare_parameter('box_max_x', 0.215)
        self.declare_parameter('box_min_y', -0.215)
        self.declare_parameter('box_max_y', 0.215)

        # Offset sudut RPLidar terhadap base_link (RPLidar terpasang membelakangi base_link, yaw = pi rad)
        self.declare_parameter('laser_yaw_offset', 3.141592653589793)

        # Batas jarak fisik sensor (meter) - noise hardware RPLiDAR A1
        self.declare_parameter('sensor_min_range', 0.05)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('circle_min_range', 0.32)  # Hanya dipakai jika filter_type == 'circle'

        self.filter_type = str(self.get_parameter('filter_type').value).strip().lower()
        self.box_min_x = float(self.get_parameter('box_min_x').value)
        self.box_max_x = float(self.get_parameter('box_max_x').value)
        self.box_min_y = float(self.get_parameter('box_min_y').value)
        self.box_max_y = float(self.get_parameter('box_max_y').value)
        self.laser_yaw_offset = float(self.get_parameter('laser_yaw_offset').value)
        self.sensor_min_range = float(self.get_parameter('sensor_min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        # Jika launch file lama masih passing 'min_range' > 0.1, simpan ke circle_min_range untuk mode circle
        raw_min = float(self.get_parameter('min_range').value)
        self.circle_min_range = raw_min if raw_min > 0.1 else float(self.get_parameter('circle_min_range').value)

        # Cache tabel trigonometri untuk eksekusi vektor numpy (< 0.05 ms per scan)
        self.cached_len = 0
        self.cached_angle_min = None
        self.cached_angle_inc = None
        self.cos_angles = None
        self.sin_angles = None

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        if self.filter_type == 'box':
            self.get_logger().info(
                f"🛡️ Laser Filter Sasis [BOX FILTER] Aktif: Sasis 40x40 cm "
                f"(X: [{self.box_min_x:.3f}, {self.box_max_x:.3f}]m, "
                f"Y: [{self.box_min_y:.3f}, {self.box_max_y:.3f}]m, "
                f"Yaw Offset: {self.laser_yaw_offset:.3f} rad). "
                f"Sensor min range: {self.sensor_min_range}m. Zona buta depan dihilangkan!"
            )
        else:
            self.get_logger().info(
                f"🛡️ Laser Filter Sasis [CIRCLE FILTER] Aktif: radius < {self.circle_min_range}m dipotong."
            )

    def scan_callback(self, msg: LaserScan):
        # 1. Paksa sinkronisasi waktu laser dengan waktu Jetson
        msg.header.stamp = self.get_clock().now().to_msg()

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)

        if self.filter_type == 'box':
            # Di mode BOX: Set msg.range_min ke batas fisik sensor (0.05m) agar RViz & Costmap
            # TIDAK memotong titik di luar bodi secara lingkaran!
            msg.range_min = self.sensor_min_range

            # Precompute sinus dan cosinus sudut jika ukuran array scan belum tercache
            if (n != self.cached_len or 
                msg.angle_min != self.cached_angle_min or 
                msg.angle_increment != self.cached_angle_inc):
                self.cached_len = n
                self.cached_angle_min = msg.angle_min
                self.cached_angle_inc = msg.angle_increment
                angles_laser = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
                angles_base = angles_laser + np.float32(self.laser_yaw_offset)
                self.cos_angles = np.cos(angles_base).astype(np.float32)
                self.sin_angles = np.sin(angles_base).astype(np.float32)

            # Hitung posisi (X, Y) tiap berkas laser dalam kerangka robot (base_link)
            x_base = ranges * self.cos_angles
            y_base = ranges * self.sin_angles

            # Titik di dalam kotak sasis (tiang sasis & bodi dalam)
            inside_box = (
                (x_base >= self.box_min_x) & (x_base <= self.box_max_x) &
                (y_base >= self.box_min_y) & (y_base <= self.box_max_y)
            )

            # Titik tidak valid (NaN, Inf, noise hardware < sensor_min_range, atau out-of-range > max_range)
            # JANGAN gunakan circle min_range di sini!
            invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges < self.sensor_min_range) | (ranges > self.max_range)

            # Masking titik yang harus dibuang
            mask = inside_box | invalid
            ranges[mask] = np.nan
        else:
            # Mode circle legacy
            msg.range_min = self.circle_min_range
            invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges < self.circle_min_range) | (ranges > self.max_range)
            ranges[invalid] = np.nan

        msg.ranges = ranges.tolist()
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

