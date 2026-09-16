#!/usr/bin/env python3
"""
=================================================================
AMR DELIVERY DASHBOARD - Flask Server + ROS2 Integration
Server utama yang berjalan di Jetson Nano.
Menggabungkan Flask HTTP server dengan ROS2 node dalam satu proses.
=================================================================
"""
import os
import sys
import json
import time
import math
import sqlite3
import threading
import signal
from datetime import datetime
from io import BytesIO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

from flask import Flask, render_template, jsonify, request, Response, send_file

# Coba import cv_bridge & cv2 (untuk camera streaming)
try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CAMERA = True
except ImportError:
    HAS_CAMERA = False
    print('[WARN] cv_bridge atau OpenCV tidak tersedia. Camera streaming dinonaktifkan.')

try:
    from PIL import Image as PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print('[WARN] Pillow tidak tersedia. Map rendering terbatas.')

import yaml


# =====================================================================
# KONFIGURASI
# =====================================================================
# Cari path berdasarkan environment (dev vs installed)
def find_package_path():
    """Cari root directory package, baik saat development atau installed."""
    # Cek apakah dijalankan dari source (development)
    src_path = os.path.join(os.path.dirname(__file__), '..')
    if os.path.exists(os.path.join(src_path, 'templates')):
        return os.path.abspath(src_path)
    
    # Cek installed path via ament
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('amr_dashboard')
    except Exception:
        pass
    
    return os.path.abspath(src_path)


PKG_PATH = find_package_path()
TEMPLATE_DIR = os.path.join(PKG_PATH, 'templates')
STATIC_DIR = os.path.join(PKG_PATH, 'static')

# Cari path peta dari package amr_simulation
MAP_PGM_PATH = None
MAP_YAML_PATH = None
MAP_METADATA = {}

def find_map_files():
    """Cari file peta dari amr_simulation package."""
    global MAP_PGM_PATH, MAP_YAML_PATH, MAP_METADATA
    
    # Coba dari ament index
    try:
        from ament_index_python.packages import get_package_share_directory
        sim_path = get_package_share_directory('amr_simulation')
        MAP_PGM_PATH = os.path.join(sim_path, 'maps', 'peta_dc.pgm')
        MAP_YAML_PATH = os.path.join(sim_path, 'maps', 'peta_dc.yaml')
    except Exception:
        pass
    
    # Fallback: cari di workspace
    if not MAP_PGM_PATH or not os.path.exists(MAP_PGM_PATH):
        ws_root = os.environ.get('AMR_WS', os.path.expanduser('~/Documents/amr_ws'))
        MAP_PGM_PATH = os.path.join(ws_root, 'src', 'amr_simulation', 'maps', 'peta_dc.pgm')
        MAP_YAML_PATH = os.path.join(ws_root, 'src', 'amr_simulation', 'maps', 'peta_dc.yaml')
    
    # Parse YAML metadata
    if MAP_YAML_PATH and os.path.exists(MAP_YAML_PATH):
        with open(MAP_YAML_PATH, 'r') as f:
            MAP_METADATA = yaml.safe_load(f)
        print(f'[MAP] Loaded: {MAP_PGM_PATH}')
        print(f'[MAP] Resolution: {MAP_METADATA.get("resolution", 0.05)} m/px')
        print(f'[MAP] Origin: {MAP_METADATA.get("origin", [0, 0, 0])}')


# =====================================================================
# DATABASE (SQLite untuk riwayat delivery)
# =====================================================================
DB_PATH = os.path.join(os.path.expanduser('~'), '.amr_dashboard', 'history.db')

def init_database():
    """Inisialisasi database SQLite."""
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS delivery_history (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        mission_id TEXT,
        pickup_name TEXT,
        pickup_x REAL,
        pickup_y REAL,
        delivery_name TEXT,
        delivery_x REAL,
        delivery_y REAL,
        status TEXT,
        duration REAL,
        timestamp TEXT
    )''')
    conn.commit()
    conn.close()

def save_to_history(mission_data):
    """Simpan hasil misi ke database."""
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        for task in mission_data.get('tasks', []):
            c.execute('''INSERT INTO delivery_history 
                (mission_id, pickup_name, pickup_x, pickup_y, 
                 delivery_name, delivery_x, delivery_y, 
                 status, duration, timestamp)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
                (
                    mission_data.get('mission_id', ''),
                    task['pickup']['name'],
                    task['pickup']['x'],
                    task['pickup']['y'],
                    task['delivery']['name'],
                    task['delivery']['x'],
                    task['delivery']['y'],
                    task.get('status', 'unknown'),
                    mission_data.get('elapsed_time', 0),
                    datetime.now().isoformat()
                ))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f'[DB ERROR] {e}')


# =====================================================================
# ROS2 DASHBOARD NODE
# =====================================================================
class DashboardNode(Node):
    """Node ROS2 yang mengumpulkan semua data untuk dashboard."""
    
    def __init__(self):
        super().__init__('dashboard_server')
        
        # === DATA STORE (thread-safe) ===
        self.lock = threading.Lock()
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.delivery_status = {'status': 'idle', 'current_phase': 'idle'}
        self.human_status = 'NO_HUMAN'
        self.latest_frame = None  # JPEG bytes terakhir dari kamera
        self.cmd_vel_current = {'linear': 0.0, 'angular': 0.0}
        
        # === SUBSCRIBERS ===
        # Posisi AMR dari AMCL
        qos = QoSProfile(depth=10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose', 
            self.on_pose, qos)
        
        # Status delivery dari waypoint_manager
        self.status_sub = self.create_subscription(
            String, '/amr/delivery_status', self.on_delivery_status, 10)
        
        # Status human awareness
        self.human_sub = self.create_subscription(
            String, '/human_status', self.on_human_status, 10)
        
        # === PUBLISHERS ===
        # Task delivery ke waypoint_manager
        self.task_pub = self.create_publisher(String, '/amr/delivery_tasks', 10)
        # Kontrol delivery (cancel, pause, resume)
        self.control_pub = self.create_publisher(String, '/amr/delivery_control', 10)
        # Manual velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # === CAMERA SUBSCRIBER ===
        if HAS_CAMERA:
            self.bridge = CvBridge()
            self.cam_sub = self.create_subscription(
                Image, '/camera/image_raw', self.on_camera, 10)
            self.cam_sub_sensor = self.create_subscription(
                Image, '/camera_sensor/image_raw', self.on_camera, 10)
        
        self.get_logger().info('Dashboard Server Node AKTIF')
    
    def on_pose(self, msg):
        """Callback posisi AMR dari AMCL."""
        p = msg.pose.pose
        # Extract yaw dari quaternion
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        
        with self.lock:
            self.robot_pose = {
                'x': round(p.position.x, 3),
                'y': round(p.position.y, 3),
                'yaw': round(yaw, 3)
            }
    
    def on_delivery_status(self, msg):
        """Callback status delivery dari waypoint_manager."""
        try:
            data = json.loads(msg.data)
            with self.lock:
                if data.get('mission'):
                    self.delivery_status = data['mission']
                    # Simpan ke history jika misi selesai
                    if data['mission']['status'] in ('completed', 'cancelled'):
                        save_to_history(data['mission'])
                else:
                    self.delivery_status = {'status': 'idle', 'current_phase': 'idle'}
        except json.JSONDecodeError:
            pass
    
    def on_human_status(self, msg):
        """Callback status human awareness."""
        with self.lock:
            self.human_status = msg.data
    
    def on_camera(self, msg):
        """Callback kamera — convert ke JPEG untuk streaming."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize untuk performa (max 640px lebar)
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640.0 / w
                frame = cv2.resize(frame, (640, int(h * scale)))
            # Encode ke JPEG
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with self.lock:
                self.latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f'Camera error: {e}')
    
    def send_delivery_tasks(self, tasks):
        """Kirim daftar task ke waypoint_manager."""
        msg = String()
        msg.data = json.dumps({'tasks': tasks})
        self.task_pub.publish(msg)
    
    def send_control(self, command):
        """Kirim perintah kontrol ke waypoint_manager."""
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)
    
    def send_cmd_vel(self, linear, angular):
        """Kirim velocity manual (joystick)."""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)
        with self.lock:
            self.cmd_vel_current = {'linear': linear, 'angular': angular}


# =====================================================================
# FLASK APPLICATION
# =====================================================================
app = Flask(__name__,
            template_folder=TEMPLATE_DIR,
            static_folder=STATIC_DIR)

# Global reference ke ROS2 node
ros_node = None

# Load waypoints config
waypoints_config = {}
def load_waypoints_config():
    global waypoints_config
    config_paths = [
        os.path.join(PKG_PATH, 'config', 'waypoints.yaml'),
        os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'),
    ]
    for path in config_paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                waypoints_config = yaml.safe_load(f)
            print(f'[CONFIG] Loaded waypoints dari {path}')
            return
    print('[WARN] waypoints.yaml tidak ditemukan, menggunakan default.')
    waypoints_config = {
        'predefined_points': [],
        'home_position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'settings': {
            'max_linear_speed': 0.7,
            'max_angular_speed': 0.9,
            'waypoint_pause_duration': 3,
            'auto_return_home': True
        }
    }


# --- ROUTES ---

@app.route('/')
def index():
    """Halaman utama SPA."""
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    """Status AMR real-time (posisi, delivery status, human status)."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    
    with ros_node.lock:
        return jsonify({
            'pose': ros_node.robot_pose,
            'delivery': ros_node.delivery_status,
            'human_status': ros_node.human_status,
            'cmd_vel': ros_node.cmd_vel_current,
            'connected': True,
            'timestamp': time.time()
        })


@app.route('/api/map')
def api_map():
    """Serve gambar peta sebagai PNG."""
    if not MAP_PGM_PATH or not os.path.exists(MAP_PGM_PATH):
        return jsonify({'error': 'File peta tidak ditemukan'}), 404
    
    if HAS_PIL:
        img = PILImage.open(MAP_PGM_PATH)
        buf = BytesIO()
        img.save(buf, format='PNG')
        buf.seek(0)
        return send_file(buf, mimetype='image/png')
    else:
        return send_file(MAP_PGM_PATH, mimetype='image/x-portable-graymap')


@app.route('/api/map/metadata')
def api_map_metadata():
    """Metadata peta (resolution, origin, size)."""
    metadata = dict(MAP_METADATA)
    if MAP_PGM_PATH and os.path.exists(MAP_PGM_PATH) and HAS_PIL:
        img = PILImage.open(MAP_PGM_PATH)
        metadata['width'] = img.size[0]
        metadata['height'] = img.size[1]
    return jsonify(metadata)


@app.route('/api/delivery/start', methods=['POST'])
def api_delivery_start():
    """Mulai misi delivery dengan daftar task."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    
    data = request.get_json()
    tasks = data.get('tasks', [])
    
    if not tasks:
        return jsonify({'error': 'Daftar task kosong'}), 400
    
    ros_node.send_delivery_tasks(tasks)
    return jsonify({'message': f'Misi dikirim: {len(tasks)} task', 'success': True})


@app.route('/api/delivery/control', methods=['POST'])
def api_delivery_control():
    """Kontrol misi: cancel, pause, resume, skip."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    
    data = request.get_json()
    command = data.get('command', '')
    
    if command not in ('cancel', 'pause', 'resume', 'skip'):
        return jsonify({'error': f'Perintah tidak valid: {command}'}), 400
    
    ros_node.send_control(command)
    return jsonify({'message': f'Perintah [{command}] terkirim', 'success': True})


@app.route('/api/cmd_vel', methods=['POST'])
def api_cmd_vel():
    """Kirim velocity manual dari joystick."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    
    data = request.get_json()
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    
    # Clamp values
    max_lin = waypoints_config.get('settings', {}).get('max_linear_speed', 0.7)
    max_ang = waypoints_config.get('settings', {}).get('max_angular_speed', 0.9)
    linear = max(-max_lin, min(max_lin, linear))
    angular = max(-max_ang, min(max_ang, angular))
    
    ros_node.send_cmd_vel(linear, angular)
    return jsonify({'success': True})


@app.route('/api/history')
def api_history():
    """Riwayat delivery dari database."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        c = conn.cursor()
        c.execute('''SELECT * FROM delivery_history 
                      ORDER BY timestamp DESC LIMIT 100''')
        rows = [dict(r) for r in c.fetchall()]
        conn.close()
        return jsonify(rows)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/history/clear', methods=['POST'])
def api_history_clear():
    """Hapus semua riwayat delivery."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.execute('DELETE FROM delivery_history')
        conn.commit()
        conn.close()
        return jsonify({'success': True, 'message': 'Riwayat dihapus'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/saved_points')
def api_saved_points():
    """Daftar titik predefined + custom."""
    points = waypoints_config.get('predefined_points', [])
    home = waypoints_config.get('home_position', {'x': 0, 'y': 0, 'yaw': 0})
    return jsonify({'points': points, 'home': home})


@app.route('/api/saved_points', methods=['POST'])
def api_save_point():
    """Simpan titik baru ke config."""
    data = request.get_json()
    point = {
        'name': data.get('name', 'Unnamed'),
        'x': data.get('x', 0),
        'y': data.get('y', 0),
        'type': data.get('type', 'pickup'),
        'description': data.get('description', '')
    }
    
    if 'predefined_points' not in waypoints_config:
        waypoints_config['predefined_points'] = []
    waypoints_config['predefined_points'].append(point)
    
    # Simpan ke file
    config_path = os.path.join(PKG_PATH, 'config', 'waypoints.yaml')
    try:
        with open(config_path, 'w') as f:
            yaml.dump(waypoints_config, f, default_flow_style=False, allow_unicode=True)
    except Exception:
        pass
    
    return jsonify({'success': True, 'point': point})


@app.route('/api/saved_points/delete', methods=['POST'])
def api_delete_point():
    """Hapus titik dari config."""
    data = request.get_json()
    name = data.get('name', '')
    
    points = waypoints_config.get('predefined_points', [])
    waypoints_config['predefined_points'] = [p for p in points if p['name'] != name]
    
    config_path = os.path.join(PKG_PATH, 'config', 'waypoints.yaml')
    try:
        with open(config_path, 'w') as f:
            yaml.dump(waypoints_config, f, default_flow_style=False, allow_unicode=True)
    except Exception:
        pass
    
    return jsonify({'success': True})


@app.route('/api/settings')
def api_get_settings():
    """Get pengaturan."""
    return jsonify(waypoints_config.get('settings', {}))


@app.route('/api/settings', methods=['POST'])
def api_set_settings():
    """Update pengaturan."""
    data = request.get_json()
    if 'settings' not in waypoints_config:
        waypoints_config['settings'] = {}
    waypoints_config['settings'].update(data)
    
    config_path = os.path.join(PKG_PATH, 'config', 'waypoints.yaml')
    try:
        with open(config_path, 'w') as f:
            yaml.dump(waypoints_config, f, default_flow_style=False, allow_unicode=True)
    except Exception:
        pass
    
    return jsonify({'success': True, 'settings': waypoints_config['settings']})


_placeholder_jpeg_cache = None

def get_placeholder_frame():
    """Hasilkan gambar placeholder jika belum ada frame dari kamera ROS2."""
    global _placeholder_jpeg_cache
    if _placeholder_jpeg_cache is not None:
        return _placeholder_jpeg_cache
    
    if HAS_CAMERA:
        img = np.zeros((360, 640, 3), dtype=np.uint8)
        img[:] = (25, 20, 15)  # Dark navy background
        cv2.putText(img, "KAMERA AMR - STANDBY", (140, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
        cv2.putText(img, "Menunggu signal /camera/image_raw...", (140, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        _placeholder_jpeg_cache = jpeg.tobytes()
        return _placeholder_jpeg_cache
    return b''


@app.route('/api/camera/stream')
def api_camera_stream():
    """MJPEG stream dari kamera AMR."""
    if not HAS_CAMERA or ros_node is None:
        return jsonify({'error': 'Kamera tidak tersedia'}), 503
    
    def generate():
        while True:
            with ros_node.lock:
                frame = ros_node.latest_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                # Kirim placeholder JPEG valid
                placeholder = get_placeholder_frame()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + placeholder + b'\r\n')
            time.sleep(0.066)  # ~15 FPS
    
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/camera/snapshot')
def api_camera_snapshot():
    """Single snapshot dari kamera."""
    if not HAS_CAMERA or ros_node is None:
        return jsonify({'error': 'Kamera tidak tersedia'}), 503
    
    with ros_node.lock:
        frame = ros_node.latest_frame
    
    if frame:
        return Response(frame, mimetype='image/jpeg')
    return Response(get_placeholder_frame(), mimetype='image/jpeg')


# =====================================================================
# MAIN — Jalankan Flask + ROS2
# =====================================================================
def main(args=None):
    global ros_node
    
    print('='*60)
    print('  AMR DELIVERY DASHBOARD SERVER')
    print('  Memulai Flask + ROS2...')
    print('='*60)
    
    # Inisialisasi
    find_map_files()
    load_waypoints_config()
    init_database()
    
    # Inisialisasi ROS2
    rclpy.init(args=args)
    ros_node = DashboardNode()
    
    # Spin ROS2 di thread terpisah
    def ros_spin():
        try:
            rclpy.spin(ros_node)
        except Exception:
            pass
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    print(f'\n[SERVER] Templates: {TEMPLATE_DIR}')
    print(f'[SERVER] Static: {STATIC_DIR}')
    print(f'[SERVER] Database: {DB_PATH}')
    print(f'[SERVER] Kamera: {"AKTIF" if HAS_CAMERA else "NONAKTIF"}')
    print(f'\n{"="*60}')
    print(f'  Dashboard siap di: http://0.0.0.0:5000')
    print(f'  Buka dari HP: http://<IP_JETSON>:5000')
    print(f'{"="*60}\n')
    
    try:
        # Jalankan Flask (host 0.0.0.0 agar bisa diakses dari HP)
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
