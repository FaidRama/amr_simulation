#!/usr/bin/env python3
"""
Emergency Map Saver (Anti-Timeout, Transient Local QoS)
Menyimpan topik /map langsung ke file PGM dan YAML tanpa resiko timeout.
"""

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid


class DirectMapSaver(Node):
    def __init__(self, filename="peta_lt1_dc"):
        super().__init__('direct_map_saver')
        self.filename = filename
        self.saved = False

        # QoS Transient Local + Reliable (Bisa membaca peta yang sudah ada di memori meski robot diam)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.get_logger().info("🔍 Mencari topik /map di memori ROS 2...")

    def map_callback(self, msg: OccupancyGrid):
        if self.saved:
            return
        self.saved = True

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info(f"✅ Peta diterima! Ukuran: {width} x {height} pixel, Resolusi: {res} m/px")

        # Buat folder jika belum ada
        output_dir = os.path.expanduser("~/deliver_amr/maps")
        if not os.path.exists(output_dir):
            output_dir = os.getcwd()

        pgm_path = os.path.join(output_dir, f"{self.filename}.pgm")
        yaml_path = os.path.join(output_dir, f"{self.filename}.yaml")

        # Konversi data (-1: unknown, 0: free, 100: occupied) ke Grayscale PGM (0-255)
        # Standar ROS: 254 (free/putih), 0 (occupied/hitam), 205 (unknown/abu-abu)
        raw_data = msg.data
        pixel_bytes = bytearray(width * height)

        for row in range(height):
            # Baris di PGM dibalik (dari atas ke bawah)
            src_row = (height - 1 - row) * width
            dst_row = row * width
            for col in range(width):
                val = raw_data[src_row + col]
                if val == 0:
                    pixel_bytes[dst_row + col] = 254
                elif val == 100:
                    pixel_bytes[dst_row + col] = 0
                else:
                    pixel_bytes[dst_row + col] = 205

        # 1. Tulis file PGM (Format P5 binary)
        with open(pgm_path, 'wb') as f:
            header = f"P5\n{width} {height}\n255\n".encode('ascii')
            f.write(header)
            f.write(pixel_bytes)

        # 2. Tulis file YAML
        yaml_content = f"""image: {self.filename}.pgm
mode: trinary
resolution: {res}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

        self.get_logger().info(f"🎉 SUKSES TERSIMPAN!")
        self.get_logger().info(f"   📁 PGM : {pgm_path}")
        self.get_logger().info(f"   📁 YAML: {yaml_path}")
        rclpy.shutdown()


def main():
    rclpy.init()
    filename = sys.argv[1] if len(sys.argv) > 1 else "peta_lt1_dc"
    node = DirectMapSaver(filename)
    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
