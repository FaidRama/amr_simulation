#!/usr/bin/env python3
"""
Camera Publisher Ultra-Low Latency (Zero-Buffer / Real-Time Teleoperation)
- Menggunakan Threaded Reader untuk terus-menerus mengosongkan kernel buffer V4L2
- Memastikan frame yang dikirim adalah frame TERBARU dari sensor (< 30-50ms)
- Resolusi 480x270 @ 20 FPS (JPEG Quality 55) -> Sangat ringan untuk Wi-Fi (~300 KB/s)
"""

import glob
import os
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def find_camera_device():
    """Cari device kamera yang valid:
    1. Cek symlink udev /dev/camera
    2. Cek symlink udev /dev/video_cam
    3. Auto-scan /dev/video* dan tes device mana yang bisa membaca frame (bukan metadata node)
    """
    if os.path.exists('/dev/camera'):
        return '/dev/camera'
    if os.path.exists('/dev/video_cam'):
        return '/dev/video_cam'

    # Auto-scan /dev/video* (misal video0, video1, video2, dst)
    video_devices = sorted(glob.glob('/dev/video*'))
    for dev in video_devices:
        num_str = dev.replace('/dev/video', '')
        if not num_str.isdigit():
            continue
        idx = int(num_str)
        try:
            test_cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if test_cap.isOpened():
                ret, _ = test_cap.read()
                test_cap.release()
                if ret:
                    return dev
        except Exception:
            continue

    return '/dev/video0'


class ZeroLatencyCamera:
    """Threaded camera reader yang membuang tumpukan buffer FIFO V4L2."""

    def __init__(self, device='/dev/camera', width=640, height=360):
        self.device = device
        self.cap = None

        # Resolve path fisik asli jika device berupa symlink (/dev/camera -> /dev/video1)
        actual_path = os.path.realpath(str(device)) if os.path.exists(str(device)) else str(device)
        self.actual_path = actual_path

        if actual_path.startswith('/dev/video'):
            try:
                dev_idx = int(actual_path.replace('/dev/video', ''))
                self.cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
            except ValueError:
                self.cap = cv2.VideoCapture(actual_path, cv2.CAP_V4L2)
        elif str(device).isdigit():
            self.cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if self.cap and self.cap.isOpened():
            # Minta format hardware MJPEG dari webcam (ringan di USB & CPU Jetson)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            # Paksa kernel V4L2 hanya simpan 1 frame buffer (anti-lag)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.ret = False
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

        if self.cap and self.cap.isOpened():
            self.ret, self.frame = self.cap.read()

    def start(self):
        if not self.cap or not self.cap.isOpened():
            return False
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        return True

    def _reader_loop(self):
        """Thread background yang terus-menerus menguras buffer kamera."""
        while self.running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.01)
                continue
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            else:
                time.sleep(0.005)

    def get_latest_frame(self):
        with self.lock:
            if not self.ret or self.frame is None:
                return False, None
            return True, self.frame

    def release(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()


def main():
    rclpy.init()
    node = Node('usb_camera_publisher')

    node.declare_parameter('device', '')
    node.declare_parameter('device_id', -1)
    node.declare_parameter('target_fps', 20.0)
    node.declare_parameter('jpeg_quality', 55)

    device_param = str(node.get_parameter('device').value).strip()
    device_id_param = int(node.get_parameter('device_id').value)
    target_fps = float(node.get_parameter('target_fps').value)
    jpeg_quality = int(node.get_parameter('jpeg_quality').value)

    if device_param:
        target_device = device_param
    elif device_id_param >= 0:
        target_device = f'/dev/video{device_id_param}'
    else:
        target_device = find_camera_device()

    # Hanya kirim CompressedImage (15 KB per frame - hemat Wi-Fi)
    pub_comp = node.create_publisher(CompressedImage, '/camera/image_raw/compressed', 1)

    node.get_logger().info(f"🔍 Membuka kamera pada device: {target_device}...")
    cam = ZeroLatencyCamera(device=target_device, width=640, height=360)
    if not cam.start():
        node.get_logger().error(f"❌ Gagal membuka kamera pada {target_device} (real: {cam.actual_path})!")
        return

    node.get_logger().info(
        f"✅ Kamera Real-time Aktif pada {target_device} -> {cam.actual_path} "
        f"(Zero Latency, {target_fps:.0f} FPS, Quality {jpeg_quality}%)"
    )

    frame_interval = 1.0 / target_fps
    count = 0
    last_log = time.time()

    try:
        while rclpy.ok():
            loop_start = time.time()

            ret, frame = cam.get_latest_frame()
            if ret and frame is not None:
                # Resize ke 480x270 widescreen (ideal untuk dashboard responsif)
                frame_resized = cv2.resize(frame, (480, 270))
                _, jpeg_data = cv2.imencode(
                    '.jpg', frame_resized, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
                )

                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                msg.format = 'jpeg'
                msg.data = jpeg_data.tobytes()
                pub_comp.publish(msg)

                count += 1
                now = time.time()
                if now - last_log >= 3.0:
                    fps = count / (now - last_log)
                    node.get_logger().info(f"🎥 Video LIVE: {fps:.1f} FPS (Real-time)")
                    count = 0
                    last_log = now

            elapsed = time.time() - loop_start
            sleep_needed = frame_interval - elapsed
            if sleep_needed > 0:
                time.sleep(sleep_needed)
            else:
                time.sleep(0.002)

            rclpy.spin_once(node, timeout_sec=0.001)

    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()