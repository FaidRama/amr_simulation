import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math

class PenyadapJarak(Node):
    def __init__(self):
        super().__init__('penyadap_jarak_astar')
        # Menyadap topik rute global dari Nav2 (biasanya /plan)
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.kalkulasi_jarak_callback,
            10)
        self.get_logger().info('Menunggu garis rute A* dari RViz2...')

    def kalkulasi_jarak_callback(self, msg):
        total_jarak = 0.0
        poses = msg.poses
        
        # Hitung jarak Euclidean antar titik-titik rute
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            total_jarak += math.hypot(p2.x - p1.x, p2.y - p1.y)
        
        # Cetak hasilnya dengan warna hijau biar gampang dilihat
        print(f"\n\033[92m[TARGET BARU DITERIMA] -> Jarak Estimasi A*: {total_jarak:.2f} meter\033[0m\n")

def main(args=None):
    rclpy.init(args=args)
    node = PenyadapJarak()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()