#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import time

def main():
    rclpy.init()
    node = Node('usb_camera_publisher')
    
    # Hanya kirim CompressedImage (18 KB per frame - 98% hemat Wi-Fi!)
    pub_comp = node.create_publisher(CompressedImage, '/camera/image_raw/compressed', 1)
    
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        node.get_logger().error("❌ Gagal membuka kamera /dev/video0!")
        return

    node.get_logger().info("✅ Kamera Fisik: Murni Mode JPEG Ringan (Anti-Lag Wi-Fi)!")
    
    # 12 FPS stabil
    frame_interval = 1.0 / 12.0
    count = 0
    last_log = time.time()
    
    try:
        while rclpy.ok():
            loop_start = time.time()
            
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            
            # 480x270 widescreen (Kualitas kompresi 60 -> cuma ~18 KB per frame!)
            frame_resized = cv2.resize(frame, (480, 270))
            _, jpeg_data = cv2.imencode('.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, 60])
            
            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.format = 'jpeg'
            msg.data = jpeg_data.tobytes()
            pub_comp.publish(msg)
            
            count += 1
            now = time.time()
            if now - last_log >= 2.0:
                fps = count / (now - last_log)
                node.get_logger().info(f"🎥 Video LIVE: {fps:.1f} FPS (Hanya 18 KB/frame)")
                count = 0
                last_log = now
            
            elapsed = time.time() - loop_start
            sleep_needed = frame_interval - elapsed
            if sleep_needed > 0:
                time.sleep(sleep_needed)
            else:
                time.sleep(0.005)
                
            rclpy.spin_once(node, timeout_sec=0.001)
            
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
