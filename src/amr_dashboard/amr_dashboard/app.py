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
import subprocess
import re
import hashlib
import urllib.request
import urllib.parse
from datetime import datetime
from io import BytesIO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np

try:
    from tf2_ros import Buffer, TransformListener
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False
    print('[WARN] tf2_ros tidak tersedia. Menggunakan fallback /amcl_pose.')

from flask import Flask, render_template, jsonify, request, Response, send_file

# Coba import cv_bridge & cv2 (untuk camera streaming)
try:
    # pyrefly: ignore [missing-import]
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

HAS_CAMERA = HAS_CV2 or HAS_CV_BRIDGE
if not HAS_CAMERA:
    print('[WARN] OpenCV atau cv_bridge tidak tersedia. Camera streaming dinonaktifkan.')

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
AUDIO_CACHE_DIR = os.path.join(STATIC_DIR, 'audio_cache')
os.makedirs(AUDIO_CACHE_DIR, exist_ok=True)

# Cari path peta dari package amr_simulation
MAP_PGM_PATH = None
MAP_YAML_PATH = None
MAP_METADATA = {}
ACTIVE_MAP_FILE = 'peta_1b.yaml'


def get_workspace_roots():
    """Dapatkan daftar kemungkinan root workspace (Jetson ~/deliver_amr & Laptop ~/Documents/amr_ws)."""
    roots = []
    env_ws = os.environ.get('AMR_WS')
    if env_ws:
        roots.append(os.path.expanduser(env_ws))
    curr = os.path.abspath(os.path.dirname(__file__))
    for _ in range(6):
        if os.path.isdir(os.path.join(curr, 'src')):
            if curr not in roots:
                roots.append(curr)
            break
        curr = os.path.dirname(curr)
    for candidate in ('~/deliver_amr', '~/Documents/amr_ws'):
        exp = os.path.expanduser(candidate)
        if os.path.isdir(exp) and exp not in roots:
            roots.append(exp)
    if not roots:
        roots.append(os.path.expanduser('~/Documents/amr_ws'))
    return roots


def get_maps_directories():
    """Dapatkan daftar direktori maps/ dari source workspace dan installed share."""
    dirs = []
    for ws_root in get_workspace_roots():
        src_maps = os.path.join(ws_root, 'src', 'amr_simulation', 'maps')
        if os.path.isdir(src_maps) and src_maps not in dirs:
            dirs.append(src_maps)
    try:
        from ament_index_python.packages import get_package_share_directory
        sim_maps = os.path.join(get_package_share_directory('amr_simulation'), 'maps')
        if os.path.isdir(sim_maps) and sim_maps not in dirs:
            dirs.append(sim_maps)
    except Exception:
        pass
    return dirs


def list_available_maps():
    """Kembalikan daftar file peta (.yaml) yang tersedia + opsi live ROS2 /map."""
    found = {}
    labels = {
        'peta_l1_ddc.yaml': 'peta_l1_ddc.yaml (Peta Lantai 1 DDC)',
        'peta_1b.yaml': 'peta_1b.yaml (Real Navigation Default)',
        'peta_kampus.yaml': 'peta_kampus.yaml (Peta Kampus)',
        'peta_rumah.yaml': 'peta_rumah.yaml (Peta Rumah)',
        'peta_dc.yaml': 'peta_dc.yaml (Simulasi Gazebo Gudang)',
    }
    for d in get_maps_directories():
        try:
            for fname in sorted(os.listdir(d)):
                if fname.endswith('.yaml') and fname not in found:
                    found[fname] = {
                        'id': fname,
                        'name': labels.get(fname, fname),
                        'type': 'sim' if fname == 'peta_dc.yaml' else 'real'
                    }
        except Exception:
            pass
    res = list(found.values())
    res.append({
        'id': 'live',
        'name': 'Live ROS2 /map Topic (SLAM Real-Time / Map Server)',
        'type': 'live'
    })
    return res


def find_map_files(map_yaml_name=None):
    """Cari dan muat file peta dari amr_simulation package sesuai nama peta aktif."""
    global MAP_PGM_PATH, MAP_YAML_PATH, MAP_METADATA, ACTIVE_MAP_FILE

    if map_yaml_name:
        ACTIVE_MAP_FILE = map_yaml_name

    if ACTIVE_MAP_FILE == 'live':
        print('[MAP] Mode peta diset ke LIVE ROS2 /map topic')
        if MAP_PGM_PATH and os.path.exists(MAP_PGM_PATH):
            return
        target_yaml = 'peta_1b.yaml'
    else:
        target_yaml = ACTIVE_MAP_FILE or 'peta_1b.yaml'

    yaml_candidate = None
    for d in get_maps_directories():
        cand = os.path.join(d, target_yaml)
        if os.path.exists(cand):
            yaml_candidate = cand
            break

    if not yaml_candidate:
        for d in get_maps_directories():
            for fallback in ('peta_1b.yaml', 'peta_dc.yaml'):
                cand = os.path.join(d, fallback)
                if os.path.exists(cand):
                    yaml_candidate = cand
                    break
            if yaml_candidate:
                break

    if yaml_candidate and os.path.exists(yaml_candidate):
        MAP_YAML_PATH = yaml_candidate
        with open(MAP_YAML_PATH, 'r') as f:
            MAP_METADATA = yaml.safe_load(f) or {}

        img_field = MAP_METADATA.get('image', target_yaml.replace('.yaml', '.pgm'))
        map_dir = os.path.dirname(MAP_YAML_PATH)
        cand_pgm = os.path.join(map_dir, img_field)
        if not os.path.exists(cand_pgm):
            cand_pgm = os.path.join(map_dir, os.path.basename(img_field))
        MAP_PGM_PATH = cand_pgm
        print(f'[MAP] Loaded ({ACTIVE_MAP_FILE}): {MAP_PGM_PATH}')
        print(f'[MAP] Resolution: {MAP_METADATA.get("resolution", 0.05)} m/px | Origin: {MAP_METADATA.get("origin", [0, 0, 0])}')


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
        self.declare_parameter('initial_mode', '')
        
        # === DATA STORE (thread-safe) ===
        self.lock = threading.Lock()
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.pose_source = 'NONE'  # 'TF_MAP', 'AMCL', 'ODOM', atau 'NONE'
        self.last_amcl_time = 0.0
        self.last_tf_time = 0.0
        self.has_map_pose = False
        self.live_map_png = None
        self.live_map_meta = None
        self.latest_map_msg = None
        self.delivery_status = {'status': 'idle', 'current_phase': 'idle'}
        self.human_status = 'NO_HUMAN'
        self.human_state = None            # nilai dari /human_state (NORMAL, REDUCED_SPEED, SAFE_STOP)
        self.human_info = None            # dict JSON dari /human_info (detail deteksi)
        self.last_human_status_time = 0.0  # waktu terakhir /human_status diterima
        self.last_human_state_time = 0.0   # waktu terakhir /human_state diterima
        self.last_human_info_time = 0.0    # waktu terakhir /human_info diterima
        self.latest_frame = None  # JPEG bytes terakhir dari kamera
        self.cmd_vel_current = {'linear': 0.0, 'angular': 0.0}
        self.odom_speed = {'linear': 0.0, 'angular': 0.0}
        self.last_status_time = 0.0  # Waktu terakhir status diterima
        self._history_saved_ids = set()  # Track misi yang sudah disimpan ke history
        self.nav_path = []  # List titik jalur navigasi global dari /plan Nav2
        self.last_path_time = 0.0
        self.last_odom_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.odom_to_map_offset = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
        # === TF2 LISTENER (Real-time map -> base_link seperti RViz) ===
        if HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_pose_timer = self.create_timer(0.1, self.update_tf_pose)
        
        # === SUBSCRIBERS ===
        # Nav2 Global Planned Path (/plan)
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.on_plan, 10)

        # Posisi AMR dari AMCL (mendukung TRANSIENT_LOCAL latched & VOLATILE)
        amcl_qos_tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pose_sub_tl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_pose, amcl_qos_tl)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose', 
            self.on_pose, QoSProfile(depth=10))
        
        # Live OccupancyGrid dari /map (nav2_map_server / slam_toolbox)
        map_qos_tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub_tl = self.create_subscription(
            OccupancyGrid, '/map', self.on_map_grid, map_qos_tl)
        self.map_sub_vol = self.create_subscription(
            OccupancyGrid, '/map', self.on_map_grid, 1)
        
        # Status delivery dari waypoint_manager
        self.status_sub = self.create_subscription(
            String, '/amr/delivery_status', self.on_delivery_status, 10)
        
        # Status human awareness (/human_state, /human_status, dan /human_info)
        self.human_sub = self.create_subscription(
            String, '/human_status', self.on_human_status, 10)
        self.human_state_sub = self.create_subscription(
            String, '/human_state', self.on_human_state, 10)
        # Detail deteksi (JSON) untuk keterangan di bawah frame kamera
        self.human_info_sub = self.create_subscription(
            String, '/human_info', self.on_human_info, 10)

        # Monitor kecepatan cmd_vel & odometry
        self.cmd_vel_listen_sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel_listen, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        
        # === PUBLISHERS ===
        # Task delivery ke waypoint_manager
        self.task_pub = self.create_publisher(String, '/amr/delivery_tasks', 10)
        # Kontrol delivery (cancel, pause, resume, reload_config)
        self.control_pub = self.create_publisher(String, '/amr/delivery_control', 10)
        # Manual velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Initial pose AMCL (/initialpose)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # === CAMERA SUBSCRIBER ===
        if HAS_CAMERA:
            self.bridge = CvBridge() if HAS_CV_BRIDGE else None
            self.last_cam_time = 0
            self.last_annotated_time = 0
            self.cam_sub = self.create_subscription(
                Image, '/camera/image_raw', self.on_camera, 1)
            self.cam_sub_sensor = self.create_subscription(
                Image, '/camera_sensor/image_raw', self.on_camera, 1)
            # Frame ber-anotasi (bbox/HUD) sengaja TIDAK di-subscribe: dulu stream
            # berpindah-pindah antara frame annotated (~0.8 FPS) dan raw saat annotated
            # telat >1.5 dtk -> kedap-kedip. Hasil deteksi kini lewat /human_info (teks).
            self.cam_sub_comp = self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self.on_compressed_camera, 1)
        
        # Timer untuk cek stale status (jika waypoint_manager mati)
        self.stale_timer = self.create_timer(3.0, self.check_stale_status)
        
        self.get_logger().info('Dashboard Server Node AKTIF')
    
    def on_plan(self, msg):
        """Callback jalur navigasi global Nav2 (/plan)."""
        pts = []
        n = len(msg.poses)
        if n > 0:
            step = max(1, n // 100)
            for i in range(0, n, step):
                p = msg.poses[i].pose.position
                pts.append({'x': round(float(p.x), 3), 'y': round(float(p.y), 3)})
            last_p = msg.poses[-1].pose.position
            if not pts or pts[-1]['x'] != round(float(last_p.x), 3) or pts[-1]['y'] != round(float(last_p.y), 3):
                pts.append({'x': round(float(last_p.x), 3), 'y': round(float(last_p.y), 3)})

        with self.lock:
            self.nav_path = pts
            self.last_path_time = time.time()

    def update_tf_pose(self):
        """Ambil posisi robot real-time dari TF tree (map -> base_footprint / base_link) persis seperti RViz."""
        if not HAS_TF2:
            return
        for child_frame in ('base_footprint', 'base_link'):
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', child_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.04)
                )
                tr = t.transform.translation
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
                with self.lock:
                    self.last_tf_time = time.time()
                    self.has_map_pose = True
                    self.pose_source = 'TF_MAP'
                    self.robot_pose = {
                        'x': round(float(tr.x), 3),
                        'y': round(float(tr.y), 3),
                        'yaw': round(float(yaw), 3)
                    }
                    # Update offset odom -> map agar fallback odometry selalu tersinkronisasi
                    ox = self.last_odom_pose['x']
                    oy = self.last_odom_pose['y']
                    oyaw = self.last_odom_pose['yaw']
                    self.odom_to_map_offset = {
                        'x': float(tr.x) - ox,
                        'y': float(tr.y) - oy,
                        'yaw': yaw - oyaw
                    }
                return
            except Exception:
                pass

    def on_pose(self, msg):
        """Callback posisi AMR dari AMCL (/amcl_pose)."""
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        
        with self.lock:
            self.last_amcl_time = time.time()
            self.has_map_pose = True
            self.pose_source = 'AMCL'
            self.robot_pose = {
                'x': round(p.position.x, 3),
                'y': round(p.position.y, 3),
                'yaw': round(yaw, 3)
            }
            ox = self.last_odom_pose['x']
            oy = self.last_odom_pose['y']
            oyaw = self.last_odom_pose['yaw']
            self.odom_to_map_offset = {
                'x': float(p.position.x) - ox,
                'y': float(p.position.y) - oy,
                'yaw': yaw - oyaw
            }

    def on_cmd_vel_listen(self, msg):
        """Callback monitor kecepatan perintah robot."""
        with self.lock:
            self.cmd_vel_current = {
                'linear': round(msg.linear.x, 3),
                'angular': round(msg.angular.z, 3)
            }

    def on_odom(self, msg):
        """Callback monitor kecepatan aktual & fallback posisi odometry robot (50Hz)."""
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        now = time.time()
        with self.lock:
            lin = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            ang = msg.twist.twist.angular.z
            self.odom_speed = {
                'linear': round(lin, 3),
                'angular': round(ang, 3)
            }
            self.last_odom_pose = {
                'x': float(p.position.x),
                'y': float(p.position.y),
                'yaw': float(yaw)
            }
            # Jika TF_MAP dan AMCL tidak update selama > 0.6 detik (atau belum pernah ada map pose)
            # gunakan Odometry terfusi agar pergerakan robot pada peta dashboard TIDAK PERNAH DIAM!
            tf_stale = (now - self.last_tf_time > 0.6) and (now - self.last_amcl_time > 0.6)
            if not self.has_map_pose or tf_stale:
                off = getattr(self, 'odom_to_map_offset', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
                self.pose_source = 'ODOM_FUSED' if self.has_map_pose else 'ODOM'
                self.robot_pose = {
                    'x': round(p.position.x + off['x'], 3),
                    'y': round(p.position.y + off['y'], 3),
                    'yaw': round(yaw + off['yaw'], 3)
                }

    def on_map_grid(self, msg):
        """Callback OccupancyGrid dari /map untuk mode Live ROS Map (SLAM / Nav2)."""
        global ACTIVE_MAP_FILE
        try:
            w = msg.info.width
            h = msg.info.height
            res = round(float(msg.info.resolution), 5)
            ox = round(float(msg.info.origin.position.x), 4)
            oy = round(float(msg.info.origin.position.y), 4)
            if w == 0 or h == 0 or not HAS_PIL:
                return
            arr = np.array(msg.data, dtype=np.int16).reshape((h, w))
            # Standar PGM: 254 = free (0), 0 = occupied (100), 205 = unknown (-1)
            img_arr = np.full((h, w), 205, dtype=np.uint8)
            img_arr[arr == 0] = 254
            img_arr[arr >= 50] = 0
            mask_mid = (arr > 0) & (arr < 50)
            if np.any(mask_mid):
                img_arr[mask_mid] = (254 - (arr[mask_mid] * 254 // 100)).astype(np.uint8)
            # Flip vertikal agar baris 0 di atas (sesuai konvensi PGM & map.js)
            img_arr = np.flipud(img_arr)
            pil_img = PILImage.fromarray(img_arr, mode='L')
            buf = BytesIO()
            pil_img.save(buf, format='PNG')
            png_bytes = buf.getvalue()
            meta = {
                'resolution': res,
                'origin': [ox, oy, 0.0],
                'width': w,
                'height': h,
                'source': 'live_ros_topic'
            }
            with self.lock:
                self.latest_map_msg = msg
                self.live_map_png = png_bytes
                self.live_map_meta = meta

            # Auto-sinkronisasi file peta aktif jika origin /map berbeda dengan file statis saat ini
            if ACTIVE_MAP_FILE != 'live':
                cur_orig = MAP_METADATA.get('origin', [0.0, 0.0, 0.0])
                if abs(float(cur_orig[0]) - ox) > 0.05 or abs(float(cur_orig[1]) - oy) > 0.05:
                    matched_yaml = None
                    for d in get_maps_directories():
                        try:
                            for fname in sorted(os.listdir(d)):
                                if fname.endswith('.yaml'):
                                    ypath = os.path.join(d, fname)
                                    with open(ypath, 'r') as yf:
                                        ym = yaml.safe_load(yf) or {}
                                    yo = ym.get('origin', [0.0, 0.0, 0.0])
                                    if abs(float(yo[0]) - ox) <= 0.05 and abs(float(yo[1]) - oy) <= 0.05:
                                        matched_yaml = fname
                                        break
                        except Exception:
                            pass
                        if matched_yaml:
                            break
                    if matched_yaml and matched_yaml != ACTIVE_MAP_FILE:
                        self.get_logger().info(f'Auto-sync peta dashboard ke peta aktif ROS2: {matched_yaml}')
                        find_map_files(matched_yaml)
        except Exception as e:
            self.get_logger().warn(f'Gagal memproses /map OccupancyGrid: {e}')
    
    def on_delivery_status(self, msg):
        """Callback status delivery dari waypoint_manager."""
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.last_status_time = time.time()
                if data.get('mission'):
                    self.delivery_status = data['mission']
                    # Simpan ke history jika misi selesai (hanya sekali per misi)
                    mission_id = data['mission'].get('mission_id', '')
                    if (data['mission']['status'] in ('completed', 'cancelled')
                            and mission_id not in self._history_saved_ids):
                        self._history_saved_ids.add(mission_id)
                        save_to_history(data['mission'])
                else:
                    self.delivery_status = {'status': 'idle', 'current_phase': 'idle'}
        except json.JSONDecodeError:
            pass
    
    def check_stale_status(self):
        """Cek apakah status delivery beku (waypoint_manager mungkin mati)."""
        with self.lock:
            if self.delivery_status.get('status') in ('running', 'paused'):
                elapsed = time.time() - self.last_status_time
                if self.last_status_time > 0 and elapsed > 5.0:
                    self.get_logger().warn(
                        f'Status delivery beku {elapsed:.0f}s — '
                        f'waypoint_manager mungkin mati. Reset ke idle.')
                    self.delivery_status = {'status': 'idle', 'current_phase': 'idle'}
    
    def on_human_status(self, msg):
        """Callback status human awareness (/human_status)."""
        with self.lock:
            self.human_status = msg.data
            self.last_human_status_time = time.time()

    def on_human_state(self, msg):
        """Callback state human awareness (/human_state)."""
        with self.lock:
            self.human_state = msg.data
            self.last_human_state_time = time.time()

    def on_human_info(self, msg):
        """Callback detail deteksi human awareness (/human_info, JSON)."""
        try:
            info = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(info, dict):
            return
        with self.lock:
            self.human_info = info
            self.last_human_info_time = time.time()
    
    def on_compressed_annotated_camera(self, msg):
        """Callback kamera ber-anotasi AI terkompresi JPEG (YOLO bounding box & HUD)."""
        self.last_annotated_time = time.time()
        now = time.time()
        if now - self.last_cam_time < 0.03:
            return
        self.last_cam_time = now
        with self.lock:
            self.latest_frame = bytes(msg.data)

    def on_annotated_camera(self, msg):
        """Callback kamera ber-anotasi AI (YOLO pose & bounding box)."""
        self.last_annotated_time = time.time()
        self.process_camera_frame(msg)

    def on_compressed_camera(self, msg):
        """Callback kamera terkompresi JPEG langsung dari robot (ultra-ringan bandwidth)."""
        self.last_comp_time = time.time()
        if time.time() - getattr(self, 'last_annotated_time', 0) < 1.5:
            return
        now = time.time()
        if now - self.last_cam_time < 0.03:
            return
        self.last_cam_time = now
        with self.lock:
            self.latest_frame = bytes(msg.data)

    def on_camera(self, msg):
        """Callback kamera raw — abaikan jika stream compressed atau annotated aktif."""
        if time.time() - getattr(self, 'last_comp_time', 0) < 2.0:
            return
        if time.time() - getattr(self, 'last_annotated_time', 0) < 1.5:
            return
        self.process_camera_frame(msg)

    def process_camera_frame(self, msg):
        """Convert ROS Image ke JPEG untuk web streaming."""
        if self.bridge is None:
            return
        now = time.time()
        if now - self.last_cam_time < 0.03:  # Throttle ~30 FPS
            return
        self.last_cam_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize untuk performa tinggi (max 480px lebar)
            h, w = frame.shape[:2]
            if w > 480:
                scale = 480.0 / w
                frame = cv2.resize(frame, (480, int(h * scale)))
            # Encode ke JPEG (quality 60 untuk bandwidth ringan & 30 FPS mulus)
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
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

    def send_initial_pose(self, x, y, yaw=0.0):
        """Publish 2D Pose Estimate ke /initialpose untuk lokalisasi AMCL."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # Covariance standar AMCL (x, y, yaw)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)
        with self.lock:
            self.robot_pose = {
                'x': round(float(x), 3),
                'y': round(float(y), 3),
                'yaw': round(float(yaw), 3)
            }
        self.get_logger().info(f'Initial Pose dikirim ke /initialpose: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')


# =====================================================================
# FLASK APPLICATION
# =====================================================================
app = Flask(__name__,
            template_folder=TEMPLATE_DIR,
            static_folder=STATIC_DIR)

# Global reference ke ROS2 node
ros_node = None

# Load waypoints config
def load_env_file():
    """Load variabel lingkungan dari file .env lokal."""
    env_paths = [
        os.path.join(PKG_PATH, 'config', '.env'),
        os.path.join(os.path.dirname(__file__), '..', 'config', '.env'),
    ]
    for ws_root in get_workspace_roots():
        env_paths.extend([
            os.path.join(ws_root, 'src', 'amr_dashboard', 'config', '.env'),
            os.path.join(ws_root, 'build', 'amr_dashboard', 'config', '.env'),
            os.path.join(ws_root, '.env'),
        ])
    for path in env_paths:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#') and '=' in line:
                            k, v = line.split('=', 1)
                            os.environ[k.strip()] = v.strip().strip('"\'')
                print(f'[CONFIG] Loaded environment variables dari {path}')
                return
            except Exception as e:
                print(f'[WARN] Gagal membaca {path}: {e}')


waypoints_config = {}

def save_waypoints_config():
    """Simpan waypoints_config ke install share dan src agar persisten."""
    paths = [os.path.join(PKG_PATH, 'config', 'waypoints.yaml')]
    for ws_root in get_workspace_roots():
        cand = os.path.join(ws_root, 'src', 'amr_dashboard', 'config', 'waypoints.yaml')
        if cand not in paths:
            paths.append(cand)
    saved_any = False
    for cfg_path in paths:
        try:
            if os.path.exists(os.path.dirname(cfg_path)):
                with open(cfg_path, 'w') as f:
                    yaml.dump(waypoints_config, f, default_flow_style=False, allow_unicode=True)
                saved_any = True
        except Exception as e:
            print(f'[WARN] Gagal menyimpan waypoints.yaml ke {cfg_path}: {e}')
    return saved_any


def apply_mode_profile(mode, custom_map_file=None, save=True):
    """Aktifkan profil mode ('sim' atau 'real') dan muat peta yang sesuai."""
    global waypoints_config
    mode = 'sim' if str(mode).lower() == 'sim' else 'real'
    waypoints_config['operation_mode'] = mode

    if 'profiles' not in waypoints_config or not isinstance(waypoints_config['profiles'], dict):
        waypoints_config['profiles'] = {}

    default_map = 'peta_dc.yaml' if mode == 'sim' else 'peta_1b.yaml'
    if mode not in waypoints_config['profiles']:
        waypoints_config['profiles'][mode] = {
            'map_file': default_map,
            'home_position': dict(waypoints_config.get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})),
            'predefined_points': list(waypoints_config.get('predefined_points', [])),
            'settings': dict(waypoints_config.get('settings', {
                'max_linear_speed': 1.0 if mode == 'sim' else 0.35,
                'max_angular_speed': 1.2 if mode == 'sim' else 0.8,
                'waypoint_pause_duration': 3,
                'auto_return_home': True
            }))
        }

    prof = waypoints_config['profiles'][mode]
    if custom_map_file:
        prof['map_file'] = custom_map_file

    map_to_load = prof.get('map_file', default_map)
    waypoints_config['predefined_points'] = list(prof.get('predefined_points', []))
    waypoints_config['home_position'] = dict(prof.get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}))
    waypoints_config['settings'] = dict(prof.get('settings', {}))

    find_map_files(map_to_load)
    if save:
        save_waypoints_config()
    if ros_node is not None:
        ros_node.send_control('reload_config')
    print(f'[MODE] Aktif: {mode.upper()} | Peta: {ACTIVE_MAP_FILE} | Titik: {len(waypoints_config["predefined_points"])}')


def load_waypoints_config(override_mode=None):
    global waypoints_config
    load_env_file()
    config_paths = []
    for ws_root in get_workspace_roots():
        config_paths.append(os.path.join(ws_root, 'src', 'amr_dashboard', 'config', 'waypoints.yaml'))
    config_paths.extend([
        os.path.join(PKG_PATH, 'config', 'waypoints.yaml'),
        os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'),
    ])
    loaded = False
    for path in config_paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                waypoints_config = yaml.safe_load(f) or {}
            print(f'[CONFIG] Loaded waypoints dari {path}')
            loaded = True
            break
    if not loaded:
        print('[WARN] waypoints.yaml tidak ditemukan, menggunakan default.')
        waypoints_config = {
            'operation_mode': 'real',
            'predefined_points': [],
            'home_position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'settings': {
                'max_linear_speed': 0.35,
                'max_angular_speed': 0.8,
                'waypoint_pause_duration': 3,
                'auto_return_home': True
            }
        }
    active_mode = override_mode if override_mode in ('sim', 'real') else waypoints_config.get('operation_mode', 'real')
    apply_mode_profile(active_mode, save=bool(override_mode))


# =====================================================================
# SLAM MAPPING PROCESS MANAGER & MAP SAVER
# =====================================================================
MAPPING_PROC = None
MAPPING_LOCK = threading.Lock()
MAPPING_LOGS = []


def _append_mapping_log(line):
    global MAPPING_LOGS
    ts = datetime.now().strftime('%H:%M:%S')
    MAPPING_LOGS.append(f'[{ts}] {line}')
    if len(MAPPING_LOGS) > 80:
        MAPPING_LOGS = MAPPING_LOGS[-80:]


def _read_mapping_stdout(proc):
    try:
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            clean = line.strip()
            if clean:
                _append_mapping_log(clean)
    except Exception:
        pass


# =====================================================================
# ASYNC RUNTIME PROCESS STATE CACHE (ZERO-LATENCY FOR TELEOP)
# =====================================================================
RUNTIME_CACHE = {
    'slam_active': False,
    'arduino_active': False,
    'nav_active': False,
    'last_updated': 0
}

def _update_process_states_loop():
    """Background loop setiap 2.0s untuk cek proses di OS tanpa memblokir HTTP teleop."""
    while True:
        try:
            # 1. Cek SLAM Mapping
            slam_act = False
            with MAPPING_LOCK:
                if MAPPING_PROC is not None and MAPPING_PROC.poll() is None:
                    slam_act = True
            if not slam_act:
                try:
                    res = subprocess.run(['pgrep', '-f', '(async_slam_toolbox_node|real_mapping\\.launch\\.py)'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                    slam_act = (res.returncode == 0 and bool(res.stdout.strip()))
                except Exception:
                    pass

            # 2. Cek Arduino Bridge
            ard_act = False
            with ARDUINO_LOCK:
                if ARDUINO_PROC is not None and ARDUINO_PROC.poll() is None:
                    ard_act = True
            if not ard_act:
                try:
                    res = subprocess.run(['pgrep', '-f', 'arduino_bridge'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                    ard_act = (res.returncode == 0 and bool(res.stdout.strip()))
                except Exception:
                    pass

            # 3. Cek Navigation (Nav2)
            nav_act = False
            with NAVIGATION_LOCK:
                if NAVIGATION_PROC is not None and NAVIGATION_PROC.poll() is None:
                    nav_act = True
            if not nav_act:
                try:
                    res = subprocess.run(['pgrep', '-f', '(real_navigation\\.launch\\.py|bt_navigator)'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                    nav_act = (res.returncode == 0 and bool(res.stdout.strip()))
                except Exception:
                    pass

            RUNTIME_CACHE['slam_active'] = slam_act
            RUNTIME_CACHE['arduino_active'] = ard_act
            RUNTIME_CACHE['nav_active'] = nav_act
            RUNTIME_CACHE['last_updated'] = time.time()
        except Exception:
            pass
        time.sleep(2.0)

_proc_cache_thread = threading.Thread(target=_update_process_states_loop, daemon=True)
_proc_cache_thread.start()


def is_slam_active():
    """Cek apakah SLAM Mapping sedang berjalan (instant O(1))."""
    with MAPPING_LOCK:
        if MAPPING_PROC is not None and MAPPING_PROC.poll() is None:
            return True
    return bool(RUNTIME_CACHE.get('slam_active', False))


def stop_slam_process():
    """Hentikan proses real_mapping.launch.py dan bersihkan node SLAM."""
    global MAPPING_PROC
    with MAPPING_LOCK:
        proc = MAPPING_PROC
        MAPPING_PROC = None
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    subprocess.run(
        ['pkill', '-f', '(real_mapping\\.launch\\.py|async_slam_toolbox_node|rplidar_composition|arduino_bridge\\.py|laser_filter_node\\.py)'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    _append_mapping_log('🛑 Proses SLAM Mapping dihentikan.')


# =====================================================================
# ARDUINO BRIDGE PROCESS MANAGER
# =====================================================================
ARDUINO_PROC = None
ARDUINO_LOCK = threading.Lock()


def is_arduino_active():
    """Cek apakah Arduino Bridge sedang berjalan (instant O(1))."""
    with ARDUINO_LOCK:
        if ARDUINO_PROC is not None and ARDUINO_PROC.poll() is None:
            return True
    return bool(RUNTIME_CACHE.get('arduino_active', False))


def stop_arduino_process():
    """Hentikan proses arduino_bridge.py."""
    global ARDUINO_PROC
    with ARDUINO_LOCK:
        proc = ARDUINO_PROC
        ARDUINO_PROC = None
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    subprocess.run(
        ['pkill', '-f', 'arduino_bridge\\.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


# =====================================================================
# REAL NAVIGATION (NAV2) PROCESS MANAGER
# =====================================================================
NAVIGATION_PROC = None
NAVIGATION_LOCK = threading.Lock()
NAVIGATION_LOGS = []


def _append_navigation_log(line):
    global NAVIGATION_LOGS
    ts = datetime.now().strftime('%H:%M:%S')
    NAVIGATION_LOGS.append(f'[{ts}] {line}')
    if len(NAVIGATION_LOGS) > 80:
        NAVIGATION_LOGS = NAVIGATION_LOGS[-80:]


def _read_navigation_stdout(proc):
    try:
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            clean = line.strip()
            if clean:
                _append_navigation_log(clean)
    except Exception:
        pass


def is_navigation_active():
    """Cek apakah Nav2 / Real Navigation sedang berjalan (instant O(1))."""
    with NAVIGATION_LOCK:
        if NAVIGATION_PROC is not None and NAVIGATION_PROC.poll() is None:
            return True
    return bool(RUNTIME_CACHE.get('nav_active', False))


def stop_navigation_process():
    """Hentikan proses real_navigation.launch.py dan bersihkan node Nav2."""
    global NAVIGATION_PROC
    with NAVIGATION_LOCK:
        proc = NAVIGATION_PROC
        NAVIGATION_PROC = None
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    subprocess.run(
        ['pkill', '-f', '(real_navigation\\.launch\\.py|amcl|bt_navigator|controller_server|planner_server|recoveries_server|lifecycle_manager|adaptive_velocity_node)'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    _append_navigation_log('🛑 Proses Navigasi (Nav2) dihentikan.')


# --- ROUTES ---

@app.route('/')
def index():
    """Halaman utama SPA."""
    return render_template('index.html')


@app.route('/api/mapping/status')
def api_mapping_status():
    """Status real-time proses SLAM Mapping, metadata peta live, dan log."""
    active = is_slam_active()
    launched_by_dash = False
    with MAPPING_LOCK:
        launched_by_dash = (MAPPING_PROC is not None and MAPPING_PROC.poll() is None)
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    pose_source = 'NONE'
    has_live_map = False
    map_meta = None
    if ros_node is not None:
        with ros_node.lock:
            pose = dict(ros_node.robot_pose)
            pose_source = ros_node.pose_source
            has_live_map = ros_node.live_map_png is not None
            map_meta = dict(ros_node.live_map_meta) if ros_node.live_map_meta else None
    return jsonify({
        'active': active,
        'launched_by_dashboard': launched_by_dash,
        'has_live_map': has_live_map,
        'map_meta': map_meta,
        'pose': pose,
        'pose_source': pose_source,
        'active_map_file': ACTIVE_MAP_FILE,
        'available_maps': list_available_maps(),
        'logs': MAPPING_LOGS[-35:]
    })


@app.route('/api/mapping/start', methods=['POST'])
def api_mapping_start():
    """Jalankan ros2 launch amr_simulation real_mapping.launch.py di background."""
    global MAPPING_PROC, ACTIVE_MAP_FILE
    if is_slam_active():
        ACTIVE_MAP_FILE = 'live'
        return jsonify({
            'success': True,
            'message': 'SLAM Mapping sudah aktif! Mode peta dialihkan ke Live /map.'
        })

    # Matikan node navigasi (AMCL/Nav2) jika masih jalan agar tidak bentrok port serial LiDAR & Arduino
    subprocess.run(
        ['pkill', '-9', '-f', '(real_navigation\\.launch\\.py|real_mapping\\.launch\\.py|amcl|bt_navigator|controller_server|planner_server|recoveries_server|lifecycle_manager|async_slam_toolbox_node|rplidar_composition|arduino_bridge\\.py|laser_filter_node\\.py)'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(0.8)

    ws_roots = get_workspace_roots()
    domain_id = os.environ.get('ROS_DOMAIN_ID', '1')
    setup_parts = [
        f'export ROS_DOMAIN_ID={domain_id}',
        'source /opt/ros/foxy/setup.bash 2>/dev/null || true'
    ]
    for wr in reversed(ws_roots):
        setup_bash = os.path.join(wr, 'install', 'setup.bash')
        if os.path.exists(setup_bash):
            setup_parts.append(f'source "{setup_bash}"')
    setup_parts.append('exec ros2 launch amr_simulation real_mapping.launch.py')
    full_cmd = ' && '.join(setup_parts)

    try:
        with MAPPING_LOCK:
            MAPPING_LOGS.clear()
            _append_mapping_log('🚀 Meluncurkan: ros2 launch amr_simulation real_mapping.launch.py ...')
            MAPPING_PROC = subprocess.Popen(
                ['/bin/bash', '-c', full_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            t = threading.Thread(target=_read_mapping_stdout, args=(MAPPING_PROC,), daemon=True)
            t.start()

        ACTIVE_MAP_FILE = 'live'
        if ros_node is not None:
            with ros_node.lock:
                ros_node.has_map_pose = False
        return jsonify({
            'success': True,
            'message': 'SLAM Mapping dimulai! Tunggu ~3-5 detik hingga LiDAR & SLAM Toolbox menampilkan peta.'
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/mapping/stop', methods=['POST'])
def api_mapping_stop():
    """Hentikan proses SLAM Mapping."""
    if ros_node is not None:
        ros_node.send_cmd_vel(0.0, 0.0)
    stop_slam_process()
    return jsonify({
        'success': True,
        'message': 'Proses SLAM Mapping berhasil dihentikan.'
    })


@app.route('/api/arduino/status')
def api_arduino_status():
    """Status proses Arduino Bridge."""
    return jsonify({
        'active': is_arduino_active()
    })


@app.route('/api/arduino/start', methods=['POST'])
def api_arduino_start():
    """Jalankan arduino_bridge.py di background."""
    global ARDUINO_PROC
    if is_arduino_active():
        return jsonify({
            'success': True,
            'active': True,
            'message': 'Arduino Bridge sudah aktif!'
        })

    ws_roots = get_workspace_roots()
    domain_id = os.environ.get('ROS_DOMAIN_ID', '1')
    setup_parts = [
        f'export ROS_DOMAIN_ID={domain_id}',
        'source /opt/ros/foxy/setup.bash 2>/dev/null || true'
    ]
    for wr in reversed(ws_roots):
        setup_bash = os.path.join(wr, 'install', 'setup.bash')
        if os.path.exists(setup_bash):
            setup_parts.append(f'source "{setup_bash}"')
    setup_parts.append('exec ros2 run amr_simulation arduino_bridge.py')
    full_cmd = ' && '.join(setup_parts)

    try:
        with ARDUINO_LOCK:
            ARDUINO_PROC = subprocess.Popen(
                ['/bin/bash', '-c', full_cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
        time.sleep(0.5)
        return jsonify({
            'success': True,
            'active': True,
            'message': 'Arduino Bridge berhasil dinyalakan!'
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/arduino/stop', methods=['POST'])
def api_arduino_stop():
    """Hentikan proses Arduino Bridge."""
    if ros_node is not None:
        ros_node.send_cmd_vel(0.0, 0.0)
    stop_arduino_process()
    return jsonify({
        'success': True,
        'active': False,
        'message': 'Arduino Bridge berhasil dihentikan.'
    })


@app.route('/api/navigation/status')
def api_navigation_status():
    """Status real-time proses Nav2 Navigation."""
    return jsonify({
        'active': is_navigation_active(),
        'map_file': ACTIVE_MAP_FILE,
        'logs': NAVIGATION_LOGS[-35:]
    })


@app.route('/api/navigation/start', methods=['POST'])
def api_navigation_start():
    """Jalankan ros2 launch amr_simulation real_navigation.launch.py di background."""
    global NAVIGATION_PROC
    if is_navigation_active():
        return jsonify({
            'success': True,
            'active': True,
            'message': 'Navigasi Nav2 sudah aktif!'
        })

    # Hentikan mapping jika aktif agar port serial LiDAR & Arduino dilepas
    stop_slam_process()
    time.sleep(0.8)

    ws_roots = get_workspace_roots()
    domain_id = os.environ.get('ROS_DOMAIN_ID', '1')
    setup_parts = [
        f'export ROS_DOMAIN_ID={domain_id}',
        'source /opt/ros/foxy/setup.bash 2>/dev/null || true'
    ]
    for wr in reversed(ws_roots):
        setup_bash = os.path.join(wr, 'install', 'setup.bash')
        if os.path.exists(setup_bash):
            setup_parts.append(f'source "{setup_bash}"')

    # Cari file peta aktif
    map_to_use = None
    if MAP_YAML_PATH and os.path.exists(MAP_YAML_PATH):
        map_to_use = MAP_YAML_PATH
    else:
        for wr in ws_roots:
            cand = os.path.join(wr, 'src', 'amr_simulation', 'maps', ACTIVE_MAP_FILE if ACTIVE_MAP_FILE != 'live' else 'peta_1b.yaml')
            if os.path.exists(cand):
                map_to_use = cand
                break
        if not map_to_use:
            for wr in ws_roots:
                cand = os.path.join(wr, 'src', 'amr_simulation', 'maps', 'peta_1b.yaml')
                if os.path.exists(cand):
                    map_to_use = cand
                    break

    map_arg = f'map:="{map_to_use}"' if map_to_use else ''
    setup_parts.append(f'exec ros2 launch amr_simulation real_navigation.launch.py {map_arg} use_sim_time:=False')
    full_cmd = ' && '.join(setup_parts)

    try:
        with NAVIGATION_LOCK:
            NAVIGATION_LOGS.clear()
            _append_navigation_log(f'🚀 Meluncurkan Navigasi dengan peta: {os.path.basename(map_to_use or "default")} ...')
            NAVIGATION_PROC = subprocess.Popen(
                ['/bin/bash', '-c', full_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            t = threading.Thread(target=_read_navigation_stdout, args=(NAVIGATION_PROC,), daemon=True)
            t.start()

        return jsonify({
            'success': True,
            'active': True,
            'message': f'Navigasi Nav2 berhasil dijalankan! Tunggu ~5-8 detik hingga sensor & AMCL siap.'
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/navigation/stop', methods=['POST'])
def api_navigation_stop():
    """Hentikan proses Navigasi Nav2."""
    if ros_node is not None:
        ros_node.send_cmd_vel(0.0, 0.0)
    stop_navigation_process()
    return jsonify({
        'success': True,
        'active': False,
        'message': 'Proses Navigasi Nav2 berhasil dihentikan.'
    })


@app.route('/api/mapping/save', methods=['POST'])
def api_mapping_save():
    """Simpan peta OccupancyGrid dari topik /map langsung ke file .pgm dan .yaml (Anti-Timeout)."""
    global ACTIVE_MAP_FILE
    if ros_node is None:
        return jsonify({'success': False, 'error': 'ROS2 belum terhubung'}), 503

    data = request.get_json() or {}
    raw_name = str(data.get('map_name', '')).strip()
    set_active = bool(data.get('set_active', True))

    if raw_name.endswith('.yaml') or raw_name.endswith('.pgm'):
        raw_name = os.path.splitext(raw_name)[0]
    clean_name = re.sub(r'[^a-zA-Z0-9_\-]', '_', raw_name).strip('_')
    if not clean_name:
        clean_name = f'peta_slam_{datetime.now().strftime("%H%M%S")}'

    with ros_node.lock:
        msg = ros_node.latest_map_msg

    if msg is None or msg.info.width == 0 or msg.info.height == 0:
        return jsonify({
            'success': False,
            'error': 'Belum ada data peta dari topik /map. Pastikan SLAM Mapping sedang berjalan!'
        }), 400

    try:
        width = int(msg.info.width)
        height = int(msg.info.height)
        res = round(float(msg.info.resolution), 5)
        origin_x = round(float(msg.info.origin.position.x), 3)
        origin_y = round(float(msg.info.origin.position.y), 3)

        # Konversi data OccupancyGrid (-1: unknown, 0: free, 100: occupied) ke PGM P5 (0-255)
        arr = np.array(msg.data, dtype=np.int16).reshape((height, width))
        img_arr = np.full((height, width), 205, dtype=np.uint8)
        img_arr[arr == 0] = 254
        img_arr[arr >= 50] = 0
        mask_mid = (arr > 0) & (arr < 50)
        if np.any(mask_mid):
            img_arr[mask_mid] = (254 - (arr[mask_mid] * 254 // 100)).astype(np.uint8)
        # Flip vertikal agar baris 0 berada di atas sesuai standar PGM ROS map_server
        img_arr = np.flipud(img_arr)

        pgm_header = f"P5\n{width} {height}\n255\n".encode('ascii')
        pgm_bytes = pgm_header + img_arr.tobytes()

        yaml_content = (
            f"image: {clean_name}.pgm\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{origin_x}, {origin_y}, 0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )

        target_dirs = get_maps_directories()
        if not target_dirs:
            fallback_dir = os.path.join(get_workspace_roots()[0], 'src', 'amr_simulation', 'maps')
            os.makedirs(fallback_dir, exist_ok=True)
            target_dirs = [fallback_dir]

        saved_paths = []
        for d in target_dirs:
            try:
                os.makedirs(d, exist_ok=True)
                pgm_path = os.path.join(d, f"{clean_name}.pgm")
                yaml_path = os.path.join(d, f"{clean_name}.yaml")
                with open(pgm_path, 'wb') as pf:
                    pf.write(pgm_bytes)
                with open(yaml_path, 'w') as yf:
                    yf.write(yaml_content)
                saved_paths.append(yaml_path)
            except Exception as e:
                print(f'[WARN] Gagal menyimpan peta ke {d}: {e}')

        if not saved_paths:
            return jsonify({'success': False, 'error': 'Gagal menulis file peta ke disk'}), 500

        yaml_filename = f"{clean_name}.yaml"
        _append_mapping_log(f'💾 Peta berhasil disimpan: {yaml_filename} ({width}x{height} px)')

        if set_active:
            if 'profiles' in waypoints_config and 'real' in waypoints_config['profiles']:
                waypoints_config['profiles']['real']['map_file'] = yaml_filename
            save_waypoints_config()
            if not is_slam_active():
                find_map_files(yaml_filename)

        return jsonify({
            'success': True,
            'map_file': yaml_filename,
            'width': width,
            'height': height,
            'resolution': res,
            'origin': [origin_x, origin_y, 0.0],
            'saved_paths': saved_paths,
            'available_maps': list_available_maps(),
            'message': f'Peta "{yaml_filename}" ({width}×{height} px) berhasil disimpan!'
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/status')
def api_status():
    """Status AMR real-time (posisi, delivery status, human status, mode)."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    
    with ros_node.lock:
        odom = getattr(ros_node, 'odom_speed', {'linear': 0.0, 'angular': 0.0})
        cmd = ros_node.cmd_vel_current
        active_speed = odom if (abs(odom.get('linear', 0.0)) > 0.001 or abs(odom.get('angular', 0.0)) > 0.001) else cmd
        # --- Status human awareness (dengan deteksi node mati / data basi) ---
        HUMAN_STALE_SEC = 5.0
        now_h = time.time()
        info_fresh = (ros_node.human_info is not None and
                      now_h - ros_node.last_human_info_time <= HUMAN_STALE_SEC)
        status_fresh = (ros_node.last_human_status_time > 0 and
                        now_h - ros_node.last_human_status_time <= HUMAN_STALE_SEC)
        state_fresh = (ros_node.last_human_state_time > 0 and
                       now_h - ros_node.last_human_state_time <= HUMAN_STALE_SEC)
        if info_fresh:
            # Sumber utama: state FSM dari /human_info. NO_HUMAN hanya jika FSM sudah
            # NORMAL dan tidak ada orang (selama FSM masih REDUCED_SPEED/SAFE_STOP
            # karena debounce, itu yang ditampilkan).
            human_info = ros_node.human_info
            human_status = human_info.get('state', 'NORMAL')
            if human_status == 'NORMAL' and human_info.get('persons', 0) == 0:
                human_status = 'NO_HUMAN'
        elif status_fresh:
            human_info = None
            human_status = ros_node.human_status
        elif state_fresh:
            human_info = None
            st = ros_node.human_state
            human_status = 'NO_HUMAN' if st == 'NORMAL' else st
        else:
            human_info = None
            human_status = 'NODE_OFFLINE'

        return jsonify({
            'pose': ros_node.robot_pose,
            'pose_source': ros_node.pose_source,
            'delivery': ros_node.delivery_status,
            'human_status': human_status,
            'human_info': human_info,
            'cmd_vel': cmd,
            'odom_vel': odom,
            'speed': active_speed,
            'mode': waypoints_config.get('operation_mode', 'real'),
            'map_file': ACTIVE_MAP_FILE,
            'has_live_map': ros_node.live_map_png is not None,
            'nav_path': getattr(ros_node, 'nav_path', []),
            'arduino_active': is_arduino_active(),
            'nav_active': is_navigation_active(),
            'connected': True,
            'timestamp': time.time()
        })


@app.route('/api/map')
def api_map():
    """Serve gambar peta sebagai PNG (dari topik /map live atau file PGM statis)."""
    force_live = request.args.get('live') == '1'
    # 1. Jika mode peta adalah 'live' (atau diminta ?live=1) dan data /map tersedia, kirim langsung
    if (force_live or ACTIVE_MAP_FILE == 'live') and ros_node is not None:
        with ros_node.lock:
            live_png = ros_node.live_map_png
        if live_png:
            resp = send_file(BytesIO(live_png), mimetype='image/png')
            resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            return resp
        if force_live:
            return jsonify({'error': 'Topik /map live belum tersedia'}), 404

    # 2. Fallback ke file PGM statis
    if not MAP_PGM_PATH or not os.path.exists(MAP_PGM_PATH):
        # Jika file tidak ada tapi ada live map, gunakan live map
        if ros_node is not None:
            with ros_node.lock:
                live_png = ros_node.live_map_png
            if live_png:
                resp = send_file(BytesIO(live_png), mimetype='image/png')
                resp.headers['Cache-Control'] = 'no-store'
                return resp
        return jsonify({'error': 'File peta tidak ditemukan'}), 404
    
    if HAS_PIL:
        img = PILImage.open(MAP_PGM_PATH)
        buf = BytesIO()
        img.save(buf, format='PNG')
        buf.seek(0)
        resp = send_file(buf, mimetype='image/png')
    else:
        resp = send_file(MAP_PGM_PATH, mimetype='image/x-portable-graymap')
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    return resp


@app.route('/api/map/metadata')
def api_map_metadata():
    """Metadata peta (resolution, origin, size, mode, map_file)."""
    force_live = request.args.get('live') == '1'
    if (force_live or ACTIVE_MAP_FILE == 'live') and ros_node is not None:
        with ros_node.lock:
            live_meta = ros_node.live_map_meta
        if live_meta:
            meta = dict(live_meta)
            meta['map_file'] = 'live'
            meta['mode'] = waypoints_config.get('operation_mode', 'real')
            return jsonify(meta)
        if force_live:
            return jsonify({'error': 'Metadata /map live belum tersedia'}), 404

    metadata = dict(MAP_METADATA)
    if MAP_PGM_PATH and os.path.exists(MAP_PGM_PATH) and HAS_PIL:
        img = PILImage.open(MAP_PGM_PATH)
        metadata['width'] = img.size[0]
        metadata['height'] = img.size[1]
    metadata['map_file'] = ACTIVE_MAP_FILE
    metadata['mode'] = waypoints_config.get('operation_mode', 'real')
    return jsonify(metadata)


@app.route('/api/mode', methods=['GET', 'POST'])
def api_mode():
    """Get atau ganti mode operasi ('sim' vs 'real') dan file peta aktif."""
    if request.method == 'GET':
        mode = waypoints_config.get('operation_mode', 'real')
        has_live = False
        if ros_node is not None:
            with ros_node.lock:
                has_live = ros_node.live_map_png is not None
        return jsonify({
            'mode': mode,
            'map_file': ACTIVE_MAP_FILE,
            'available_maps': list_available_maps(),
            'has_live_map': has_live,
            'home': waypoints_config.get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
            'points': waypoints_config.get('predefined_points', []),
            'settings': waypoints_config.get('settings', {})
        })

    data = request.get_json() or {}
    new_mode = data.get('mode', waypoints_config.get('operation_mode', 'real'))
    new_map = data.get('map_file')
    apply_mode_profile(new_mode, custom_map_file=new_map, save=True)
    return jsonify({
        'success': True,
        'mode': waypoints_config.get('operation_mode', 'real'),
        'map_file': ACTIVE_MAP_FILE,
        'home': waypoints_config.get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
        'points': waypoints_config.get('predefined_points', []),
        'settings': waypoints_config.get('settings', {}),
        'message': f'Mode berhasil diubah ke {"SIMULASI (Gazebo)" if new_mode == "sim" else "REAL ROBOT"} ({ACTIVE_MAP_FILE})'
    })


@app.route('/api/initialpose', methods=['POST'])
def api_initialpose():
    """Set posisi awal robot (/initialpose untuk AMCL) dan opsional simpan sebagai Home."""
    if ros_node is None:
        return jsonify({'error': 'ROS2 belum terhubung'}), 503
    data = request.get_json() or {}
    x = float(data.get('x', 0.0))
    y = float(data.get('y', 0.0))
    yaw = float(data.get('yaw', 0.0))
    set_as_home = bool(data.get('set_as_home', False))

    ros_node.send_initial_pose(x, y, yaw)

    if set_as_home:
        home_obj = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        waypoints_config['home_position'] = home_obj
        active_mode = waypoints_config.get('operation_mode', 'real')
        if 'profiles' in waypoints_config and active_mode in waypoints_config['profiles']:
            waypoints_config['profiles'][active_mode]['home_position'] = dict(home_obj)
        save_waypoints_config()
        ros_node.send_control('reload_config')

    return jsonify({
        'success': True,
        'pose': {'x': x, 'y': y, 'yaw': yaw},
        'set_as_home': set_as_home,
        'message': f'Initial Pose ({x:.2f}, {y:.2f}) dikirim ke AMCL'
    })


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
    
    if command not in ('cancel', 'pause', 'resume', 'skip', 'confirm_pickup', 'confirm_delivery', 'reload_config'):
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
    """Daftar titik predefined + custom untuk mode aktif."""
    points = waypoints_config.get('predefined_points', [])
    home = waypoints_config.get('home_position', {'x': 0, 'y': 0, 'yaw': 0})
    return jsonify({
        'points': points,
        'home': home,
        'mode': waypoints_config.get('operation_mode', 'real'),
        'map_file': ACTIVE_MAP_FILE
    })


@app.route('/api/saved_points', methods=['POST'])
def api_save_point():
    """Simpan titik baru ke config mode aktif."""
    data = request.get_json()
    point = {
        'name': data.get('name', 'Unnamed'),
        'x': round(float(data.get('x', 0)), 3),
        'y': round(float(data.get('y', 0)), 3),
        'type': data.get('type', 'pickup'),
        'description': data.get('description', '')
    }
    
    if 'predefined_points' not in waypoints_config:
        waypoints_config['predefined_points'] = []
    # Update jika nama sama sudah ada, atau tambahkan baru
    existing_idx = next((i for i, p in enumerate(waypoints_config['predefined_points']) if p.get('name') == point['name']), None)
    if existing_idx is not None:
        waypoints_config['predefined_points'][existing_idx] = point
    else:
        waypoints_config['predefined_points'].append(point)
    
    active_mode = waypoints_config.get('operation_mode', 'real')
    if 'profiles' in waypoints_config and active_mode in waypoints_config['profiles']:
        waypoints_config['profiles'][active_mode]['predefined_points'] = list(waypoints_config['predefined_points'])
    
    save_waypoints_config()
    return jsonify({'success': True, 'point': point})


@app.route('/api/saved_points/delete', methods=['POST'])
def api_delete_point():
    """Hapus titik dari config mode aktif."""
    data = request.get_json()
    name = data.get('name', '')
    
    points = waypoints_config.get('predefined_points', [])
    waypoints_config['predefined_points'] = [p for p in points if p['name'] != name]
    
    active_mode = waypoints_config.get('operation_mode', 'real')
    if 'profiles' in waypoints_config and active_mode in waypoints_config['profiles']:
        waypoints_config['profiles'][active_mode]['predefined_points'] = list(waypoints_config['predefined_points'])
    
    save_waypoints_config()
    return jsonify({'success': True})


@app.route('/api/settings')
def api_get_settings():
    """Get pengaturan."""
    return jsonify(waypoints_config.get('settings', {}))


@app.route('/api/settings', methods=['POST'])
def api_set_settings():
    """Update pengaturan untuk mode aktif."""
    data = request.get_json()
    if 'settings' not in waypoints_config:
        waypoints_config['settings'] = {}
    waypoints_config['settings'].update(data)
    
    active_mode = waypoints_config.get('operation_mode', 'real')
    if 'profiles' in waypoints_config and active_mode in waypoints_config['profiles']:
        if 'settings' not in waypoints_config['profiles'][active_mode]:
            waypoints_config['profiles'][active_mode]['settings'] = {}
        waypoints_config['profiles'][active_mode]['settings'].update(data)
    
    save_waypoints_config()
    if ros_node is not None:
        ros_node.send_control('reload_config')
    return jsonify({'success': True, 'settings': waypoints_config['settings']})


@app.route('/api/tts')
def api_tts():
    """Endpoint Google Translate Text-To-Speech (suara alami) dengan caching lokal offline."""
    text = request.args.get('text', '').strip()
    if not text:
        return jsonify({'error': 'Parameter text tidak boleh kosong'}), 400

    if len(text) > 300:
        text = text[:300]

    # Hash nama file untuk caching lokal
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    cache_file = os.path.join(AUDIO_CACHE_DIR, f'{text_hash}.mp3')

    # Jika file sudah ada di cache lokal, kirim langsung
    if os.path.exists(cache_file) and os.path.getsize(cache_file) > 0:
        return send_file(cache_file, mimetype='audio/mpeg')

    # Unduh dari Google Translate TTS jika belum ada di cache
    try:
        url = 'https://translate.google.com/translate_tts?ie=UTF-8&tl=id&client=tw-ob&q=' + urllib.parse.quote(text)
        req = urllib.request.Request(
            url,
            headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            audio_bytes = resp.read()
            with open(cache_file, 'wb') as f:
                f.write(audio_bytes)
            return Response(audio_bytes, mimetype='audio/mpeg')
    except Exception as e:
        print(f'[WARN] Gagal mengunduh audio Google TTS: {e}')
        return jsonify({'error': str(e)}), 500


# =====================================================================
# GEMINI AI & VOICE COMMAND INTEGRATION
# =====================================================================
def get_gemini_api_key():
    """Dapatkan API Key Gemini dari env, file .env, atau settings."""
    key = os.environ.get('GEMINI_API_KEY', '').strip()
    if key:
        return key
    
    load_env_file()
    key = os.environ.get('GEMINI_API_KEY', '').strip()
    if key:
        return key

    return waypoints_config.get('settings', {}).get('gemini_api_key', '').strip()


def save_gemini_api_key(key):
    """Simpan API Key ke file .env lokal."""
    key = key.strip()
    ws_root = os.environ.get('AMR_WS', os.path.expanduser('~/Documents/amr_ws'))
    env_paths = [
        os.path.join(PKG_PATH, 'config', '.env'),
        os.path.join(ws_root, 'src', 'amr_dashboard', 'config', '.env'),
        os.path.join(ws_root, '.env'),
    ]
    for env_path in env_paths:
        try:
            os.makedirs(os.path.dirname(env_path), exist_ok=True)
            with open(env_path, 'w') as f:
                f.write(f'GEMINI_API_KEY="{key}"\n')
            print(f'[CONFIG] Gemini API Key disimpan ke {env_path}')
        except Exception as e:
            print(f'[WARN] Gagal menyimpan API Key ke {env_path}: {e}')
    os.environ['GEMINI_API_KEY'] = key


def call_gemini_api(user_text, system_instruction, api_key):
    """Panggil Gemini Flash API via HTTP POST (urllib) dengan fallback model cepat."""
    models = ["gemini-3.5-flash-lite", "gemini-3.5-flash", "gemini-3.8-flash"]
    last_err = None

    for model in models:
        try:
            url = f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key={api_key}"
            payload = {
                "contents": [{"role": "user", "parts": [{"text": user_text}]}],
                "systemInstruction": {"parts": [{"text": system_instruction}]},
                "generationConfig": {
                    "temperature": 0.2,
                    "responseMimeType": "application/json"
                }
            }
            data = json.dumps(payload).encode('utf-8')
            req = urllib.request.Request(
                url,
                data=data,
                headers={
                    'Content-Type': 'application/json',
                    'User-Agent': 'AMR-Delivery-Bot/1.0'
                }
            )
            with urllib.request.urlopen(req, timeout=8) as resp:
                res_data = json.loads(resp.read().decode('utf-8'))
                candidates = res_data.get('candidates', [])
                if candidates and 'content' in candidates[0]:
                    parts = candidates[0]['content'].get('parts', [])
                    for part in reversed(parts):
                        if 'text' in part and not part.get('thought'):
                            raw_text = part['text'].strip()
                            if raw_text.startswith('```json'):
                                raw_text = raw_text[7:]
                            if raw_text.startswith('```'):
                                raw_text = raw_text[3:]
                            if raw_text.endswith('```'):
                                raw_text = raw_text[:-3]
                            return json.loads(raw_text.strip())
        except Exception as e:
            print(f'[GEMINI WARN] Model {model} gagal: {e}')
            last_err = e
            continue
    raise last_err or ValueError("Gagal menghubungi model Gemini")


def find_point_by_name(name, points, point_type=None):
    """Cari titik di list waypoints secara fleksibel (exact atau substring)."""
    if not name:
        return None
    name_clean = name.strip().lower()
    for p in points:
        if p.get('name', '').strip().lower() == name_clean:
            return p
    for p in points:
        p_name = p.get('name', '').strip().lower()
        if name_clean in p_name or p_name in name_clean:
            return p
    if point_type:
        for p in points:
            if p.get('type') == point_type:
                return p
    return None


@app.route('/api/gemini/status')
def api_gemini_status():
    """Cek status konfigurasi Gemini API Key."""
    key = get_gemini_api_key()
    configured = bool(key)
    masked = ''
    if configured:
        masked = key[:6] + '...' + key[-4:] if len(key) > 10 else '******'
    return jsonify({'configured': configured, 'masked_key': masked})


@app.route('/api/gemini/save_key', methods=['POST'])
def api_gemini_save_key():
    """Simpan API Key ke .env lokal."""
    data = request.get_json() or {}
    key = data.get('api_key', '').strip()
    if not key:
        return jsonify({'success': False, 'error': 'API Key kosong'}), 400
    save_gemini_api_key(key)
    return jsonify({'success': True, 'message': 'API Key berhasil disimpan di konfigurasi lokal!'})


@app.route('/api/gemini/test', methods=['POST'])
def api_gemini_test():
    """Uji koneksi Gemini API."""
    data = request.get_json() or {}
    key = data.get('api_key', '').strip() or get_gemini_api_key()
    if not key:
        return jsonify({'success': False, 'error': 'API Key belum diisi'}), 400
    try:
        res = call_gemini_api(
            "Uji koneksi robot AMR.",
            "Jawab dalam JSON: {\"status\": \"ok\", \"reply\": \"Koneksi Gemini AI Berhasil! AMR siap menerima perintah suara.\"}",
            key
        )
        return jsonify({'success': True, 'message': res.get('reply', 'Koneksi Gemini AI Berhasil!')})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/voice_command', methods=['POST'])
def api_voice_command():
    """Memproses perintah suara operator menggunakan Gemini AI."""
    data = request.get_json() or {}
    text = data.get('text', '').strip()
    if not text:
        return jsonify({'success': False, 'error': 'Teks perintah kosong'}), 400

    print(f'[VOICE] Menerima voice command: "{text}"')
    text_lower = text.lower()

    # Rule-based offline fast path untuk perintah stop / darurat
    if any(k in text_lower for k in ['stop', 'berhenti', 'batal', 'emergency']):
        if ros_node:
            ros_node.send_control('cancel')
        return jsonify({
            'success': True,
            'action': 'cancel',
            'spoken_response': 'Misi dibatalkan dan robot segera dihentikan.',
            'text': text
        })

    # Cek API Key
    api_key = get_gemini_api_key()
    if not api_key:
        msg = 'API Key Gemini belum dikonfigurasi. Silakan masukkan API Key di menu Setting.'
        return jsonify({
            'success': False,
            'error': msg,
            'spoken_response': msg,
            'text': text
        }), 400

    points = waypoints_config.get('predefined_points', [])
    home = waypoints_config.get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

    points_desc = "\n".join([
        f"- {p.get('name')} (tipe: {p.get('type')}, x={p.get('x')}, y={p.get('y')})"
        for p in points
    ])
    if not points_desc:
        points_desc = "(Belum ada titik tersimpan di peta)"

    system_instruction = f"""
Kamu adalah asisten AI pengendali Autonomous Mobile Robot (AMR) Delivery.
Tugasmu adalah menganalisis ucapan perintah operator manusia dan mengubahnya menjadi keputusan tindakan robot.

DAFTAR TITIK LOKASI DI PETA:
{points_desc}
Titik Home/Docking: (x={home.get('x')}, y={home.get('y')})

FORMAT OUTPUT WAJIB BERUPA JSON VALID dengan struktur:
{{
  "action": "delivery" | "return_home" | "cancel" | "status" | "chat",
  "pickup": "<nama titik pickup persis dari daftar, atau null>",
  "delivery": "<nama titik delivery persis dari daftar, atau null>",
  "spoken_response": "<kalimat balasan singkat, ramah, dan jelas dalam Bahasa Indonesia untuk diucapkan robot>"
}}

PANDUAN:
1. Jika operator meminta mengambil barang, mengantar barang, atau mengantar ke lokasi tertentu:
   - action: "delivery"
   - Pilih nama titik pickup dan delivery yang paling sesuai dari daftar lokasi.
   - Jika titik pickup tidak disebutkan jelas, gunakan titik pickup pertama yang tersedia.
   - Contoh spoken_response: "Baik, saya segera mengambil barang di [pickup] dan mengantarkannya ke [delivery]."
2. Jika operator meminta robot pulang, kembali ke home, atau docking:
   - action: "return_home"
   - spoken_response: "Baik, saya segera kembali ke titik awal."
3. Jika operator meminta robot berhenti / batalkan:
   - action: "cancel"
   - spoken_response: "Perintah diterima, pergerakan dihentikan."
4. Jika operator menanyakan posisi, kondisi, atau sedang apa:
   - action: "status"
   - spoken_response: "Saat ini saya standby siap menerima tugas."
5. Jika sapaan atau percakapan umum:
   - action: "chat"
   - spoken_response: respon ramah dan tawarkan bantuan pengantaran barang.
"""

    try:
        decision = call_gemini_api(text, system_instruction, api_key)
        action = decision.get('action', 'chat')
        spoken_response = decision.get('spoken_response', 'Perintah diterima.')

        execution_info = {}

        if action in ['delivery', 'pick_and_deliver', 'pickup', 'deliver'] or 'deliver' in action or 'pick' in action:
            pickup_name = decision.get('pickup')
            delivery_name = decision.get('delivery')

            pickup_pt = find_point_by_name(pickup_name, points, 'pickup')
            delivery_pt = find_point_by_name(delivery_name, points, 'delivery')

            if not pickup_pt:
                pickup_pt = next((p for p in points if p.get('type') == 'pickup'), None)
            if not delivery_pt:
                delivery_pt = next((p for p in points if p.get('type') == 'delivery'), None)

            if pickup_pt and delivery_pt and ros_node:
                task = {
                    'pickup': {
                        'name': pickup_pt.get('name'),
                        'x': float(pickup_pt.get('x', 0)),
                        'y': float(pickup_pt.get('y', 0)),
                        'yaw': float(pickup_pt.get('yaw', 0.0))
                    },
                    'delivery': {
                        'name': delivery_pt.get('name'),
                        'x': float(delivery_pt.get('x', 0)),
                        'y': float(delivery_pt.get('y', 0)),
                        'yaw': float(delivery_pt.get('yaw', 0.0))
                    }
                }
                ros_node.send_delivery_tasks([task])
                execution_info = {'task': task, 'executed': True}
            else:
                if not points:
                    spoken_response = "Maaf, belum ada titik lokasi yang tersimpan di peta. Silakan tambahkan titik di menu Setting."
                else:
                    spoken_response = "Maaf, saya tidak dapat menemukan lokasi yang dimaksud di dalam daftar titik peta."

        elif action == 'return_home':
            if ros_node:
                ros_node.send_control('cancel')
            execution_info = {'action': 'return_home', 'executed': True}

        elif action == 'cancel':
            if ros_node:
                ros_node.send_control('cancel')
            execution_info = {'action': 'cancel', 'executed': True}

        elif action == 'status':
            if ros_node:
                with ros_node.lock:
                    st = ros_node.delivery_status.get('current_phase', 'idle')
                    pose = ros_node.robot_pose
                    spoken_response = f"Status robot saat ini {st}, berada di koordinat x {pose.get('x', 0):.1f} dan y {pose.get('y', 0):.1f}."

        return jsonify({
            'success': True,
            'decision': decision,
            'execution': execution_info,
            'spoken_response': spoken_response,
            'text': text
        })

    except Exception as e:
        print(f'[ERROR] voice_command error: {e}')
        return jsonify({
            'success': False,
            'error': str(e),
            'spoken_response': 'Maaf, terjadi gangguan saat memproses perintah ke AI Gemini.',
            'text': text
        }), 500


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
        last_sent = None
        while True:
            with ros_node.lock:
                frame = ros_node.latest_frame
            if frame:
                if frame != last_sent:
                    last_sent = frame
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                placeholder = get_placeholder_frame()
                if placeholder != last_sent:
                    last_sent = placeholder
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + placeholder + b'\r\n')
            time.sleep(0.015)
    
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
    
    # Inisialisasi ROS2 terlebih dahulu agar bisa membaca parameter initial_mode
    rclpy.init(args=args)
    ros_node = DashboardNode()
    init_mode = ros_node.get_parameter('initial_mode').get_parameter_value().string_value.strip().lower()
    
    # Inisialisasi config & database
    load_waypoints_config(override_mode=init_mode if init_mode in ('sim', 'real') else None)
    init_database()
    
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