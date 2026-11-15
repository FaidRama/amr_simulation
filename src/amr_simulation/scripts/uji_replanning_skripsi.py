#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SKRIPSI BAB 4 - SKENARIO 4: EVALUASI KEANDALAN INTEGRASI & PERENCANAAN ULANG
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Robot   : Autonomous Mobile Robot Differential Drive (AMR-01)
Tujuan  :
  Memvalidasi keandalan integrasi subsistem navigasi global (Nav2 planner_server)
  saat menghadapi rintangan fisik baru (boks tiba-tiba) yang tidak terpetakan
  pada peta statis (Obstacle Layer -> Replanning Event A*).

Metrik Evaluasi Sesuai Bab 3.5.4 Proposal:
  1. Tingkat Keberhasilan (Success Rate, SR):
     Persentase keberhasilan perencana rute A* menemukan rute alternatif yang valid
     tanpa status abort (Target kelayakan: 100%).
  2. Waktu Komputasi Ulang (Replanning Time, T_replan):
     Durasi waktu yang dibutuhkan modul A* untuk mempublikasikan jalur baru (/plan)
     sejak rintangan terdeteksi (Target kelayakan: <= 200 ms).
  3. Pengulangan Pengujian:
     Sebanyak 10 kali percobaan rintangan tiba-tiba.

Fitur Skrip (Tanpa Default Titik di Luar Gedung):
  - Menu Interaktif (Default saat dijalankan tanpa argumen), konsisten dengan
    skrip pengujian AMCL Dinamis:
      [1] Mode Standby / Monitor Pasif (Kirim Goal bebas dari Web Dashboard / RViz)
      [2] Pilih Waypoint dari waypoints.yaml atau Input Koordinat Manual (X, Y, Yaw)
      [3] Rekapitulasi Tabel & Statistik Skenario 4
      [4] Hapus 1 Baris Data Terakhir CSV
      [5] Reset / Hapus Seluruh File CSV
  - Mendukung TF map->base_footprint & transformasi LiDAR ke frame map agar
    deteksi boks di jalur awal akurat di orientasi koridor mana saja.
  - Ekspor otomatis ke: <workspace>/hasil_data_pengujian_skripsi/pengujian_4_replanning/skenario4_evaluasi_replanning.csv
=============================================================================
"""

import os
import sys
import time
import math
import csv
import json
import yaml
import select
import argparse
import threading
from collections import deque
from datetime import datetime
from typing import Optional, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def euler_from_quaternion(q) -> float:
    """Konversi quaternion ke sudut yaw (radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(yaw_deg: float) -> Quaternion:
    """Konversi sudut yaw (derajat) ke Quaternion geometry_msgs."""
    yaw_rad = math.radians(yaw_deg)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def calculate_path_length(poses) -> float:
    """Hitung total panjang lintasan (Euclidean distance) dari daftar PoseStamped."""
    if not poses or len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Hitung jarak minimum titik P(px, py) ke segmen garis AB."""
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def min_distance_to_path(px: float, py: float, path_pts: List[Tuple[float, float]]) -> float:
    """Hitung jarak minimum titik (px, py) terhadap polyline jalur."""
    if not path_pts:
        return 999.0
    if len(path_pts) == 1:
        return math.hypot(px - path_pts[0][0], py - path_pts[0][1])
    best = 999.0
    for i in range(1, len(path_pts)):
        d = point_to_segment_distance(
            px, py,
            path_pts[i - 1][0], path_pts[i - 1][1],
            path_pts[i][0], path_pts[i][1]
        )
        if d < best:
            best = d
    return best


def remaining_path_length_from_pose(rx: float, ry: float, poses) -> float:
    """Hitung sisa panjang jalur dari proyeksi posisi robot terdekat hingga titik akhir."""
    if not poses or len(poses) < 2:
        return 0.0
    best_idx = 0
    best_d = 999.0
    for i, ps in enumerate(poses):
        d = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
        if d < best_d:
            best_d = d
            best_idx = i
    return calculate_path_length(poses[best_idx:])


def timed_input(prompt: str, default_val: str = "", timeout_sec: float = 8.0, cancel_check_fn=None) -> str:
    """Membaca input terminal dengan timeout otomatis agar monitor tidak macet saat trip beruntun."""
    sys.stdout.write(prompt)
    sys.stdout.flush()

    if not sys.stdin.isatty():
        sys.stdout.write(f"{default_val}\n")
        sys.stdout.flush()
        return default_val

    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        if cancel_check_fn and cancel_check_fn():
            sys.stdout.write(f"\n⏱️ [AUTO-RESUME] Navigasi berikutnya terdeteksi! Pakai default ({default_val}).\n")
            sys.stdout.flush()
            return default_val
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                line = sys.stdin.readline().strip()
                return line if line else default_val
        except Exception:
            return default_val

    sys.stdout.write(f"\n⏱️ [TIMEOUT {timeout_sec:.0f}s] Pakai default ({default_val}).\n")
    sys.stdout.flush()
    return default_val


def resolve_thesis_output_dir(subfolder: str) -> str:
    """Simpan hasil ke <workspace_root>/hasil_data_pengujian_skripsi/<subfolder>."""
    for base in (os.path.dirname(os.path.abspath(__file__)), os.getcwd()):
        cur = os.path.abspath(base)
        while True:
            if os.path.isdir(os.path.join(cur, '.git')) or (
                os.path.basename(cur) != 'src'
                and os.path.isdir(os.path.join(cur, 'src', 'amr_simulation'))
                and os.path.isdir(os.path.join(cur, 'src', 'amr_dashboard'))
            ):
                return os.path.join(cur, 'hasil_data_pengujian_skripsi', subfolder)
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    for ws in (os.path.expanduser('~/deliver_amr'), os.path.expanduser('~/Documents/amr_ws')):
        if os.path.isdir(ws):
            return os.path.join(ws, 'hasil_data_pengujian_skripsi', subfolder)
    return os.path.join(os.getcwd(), 'hasil_data_pengujian_skripsi', subfolder)


class EvaluasiReplanningNode(Node):
    def __init__(self, args):
        super().__init__('evaluasi_replanning_node')
        self.args = args

        self.out_dir = os.path.expanduser(args.out_dir) if args.out_dir else resolve_thesis_output_dir('pengujian_4_replanning')
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_file = os.path.join(self.out_dir, 'skenario4_evaluasi_replanning.csv')

        # Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # TF2 Listener (sinkron dengan pose real-time map->base_footprint dan map->laser)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Lock & Events
        self.lock = threading.Lock()
        self.nav_started_event = threading.Event()
        self.replan_detected_event = threading.Event()
        self.goal_finished_event = threading.Event()

        # Telemetri Pose & Sensor
        self.current_amcl_pose: Optional[Dict[str, float]] = None
        self.current_speed = 0.0
        self.scan_filtered_active = False

        # State Navigasi & Replanning
        self.is_navigating = False
        self.last_goal_id: Optional[bytes] = None
        self.active_goal: Optional[Dict] = None
        self.start_pose: Optional[Dict[str, float]] = None
        self.nav_start_time = 0.0
        self.last_log_time = 0.0

        self.initial_path: Optional[Path] = None
        self.initial_path_pts: List[Tuple[float, float]] = []
        self.initial_path_len = 0.0
        self.remaining_len_before_replan = 0.0
        self.replanned_path_len = 0.0
        self.max_detour_shift_m = 0.0

        self.obstacle_detected_time: Optional[float] = None
        self.obstacle_detect_dist: Optional[float] = None
        self.detect_pose_stamped: Optional[PoseStamped] = None
        self.new_path_published_time: Optional[float] = None
        self.has_replanned = False
        self.recent_on_path_hits = deque(maxlen=40)

        self.astar_compute_ms: Optional[float] = None
        self.astar_query_done = threading.Event()

        self.finished_run_data: Optional[Dict] = None
        self.waypoints = self.load_waypoints_config()

        # Subscriptions
        self.sub_amcl_tl = self.create_subscription(
            self._get_amcl_msg_type(), '/amcl_pose', self.on_amcl_pose, tl_qos)
        self.sub_amcl_rel = self.create_subscription(
            self._get_amcl_msg_type(), '/amcl_pose', self.on_amcl_pose, reliable_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, reliable_qos)
        self.sub_scan_filtered = self.create_subscription(
            LaserScan, '/scan_filtered', self.on_scan_filtered, sensor_qos)
        self.sub_scan_raw = self.create_subscription(
            LaserScan, '/scan', self.on_scan_raw, sensor_qos)
        self.sub_plan = self.create_subscription(Path, '/plan', self.on_plan, reliable_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, reliable_qos)
        self.sub_delivery = self.create_subscription(
            String, '/amr/delivery_status', self.on_delivery_status, 10)
        self.sub_status = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav_status, reliable_qos)

        self._init_csv()

    def _get_amcl_msg_type(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        return PoseWithCovarianceStamped

    def load_waypoints_config(self) -> List[Dict]:
        """Memuat daftar waypoint tersimpan dari waypoints.yaml sesuai profil aktif (real/sim)."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.abspath(os.path.join(script_dir, '..', '..', 'amr_dashboard', 'config', 'waypoints.yaml')),
            os.path.join(os.path.expanduser('~'), 'Documents', 'amr_ws', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.expanduser('~'), 'deliver_amr', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
        ]
        for p in candidates:
            if os.path.exists(p):
                try:
                    with open(p, 'r', encoding='utf-8') as f:
                        cfg = yaml.safe_load(f) or {}
                    mode = cfg.get('operation_mode', 'real')
                    prof = cfg.get('profiles', {}).get(mode, {})
                    pts = prof.get('predefined_points', []) or cfg.get('predefined_points', [])
                    home = prof.get('home_position') or cfg.get('home_position') or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                    res = [{
                        'name': 'Home / Stasiun Pangkalan',
                        'x': float(home.get('x', 0.0)),
                        'y': float(home.get('y', 0.0)),
                        'yaw': float(home.get('yaw', 0.0)),
                        'source_file': p,
                        'mode': mode,
                    }]
                    for pt in pts:
                        res.append({
                            'name': pt.get('name', f"Titik {len(res)}"),
                            'x': float(pt.get('x', 0.0)),
                            'y': float(pt.get('y', 0.0)),
                            'yaw': float(pt.get('yaw', 0.0)),
                            'source_file': p,
                            'mode': mode,
                        })
                    return res
                except Exception as e:
                    self.get_logger().warn(f"Gagal membaca waypoints.yaml ({p}): {e}")
        return []

    def _init_csv(self):
        """Buat header CSV jika belum ada."""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Percobaan",
                    "Timestamp",
                    "Nama_Target",
                    "Start_X", "Start_Y",
                    "Goal_X", "Goal_Y",
                    "Jarak_Obstacle_Deteksi_m",
                    "Astar_Compute_Time_ms",
                    "Replanning_Latency_ms",
                    "Panjang_Rute_Awal_m",
                    "Sisa_Rute_Sebelum_Replan_m",
                    "Panjang_Rute_Replan_m",
                    "Delta_Panjang_m",
                    "Deviasi_Lateral_Max_m",
                    "Event_Replan",
                    "Status_Navigasi",
                    "Keterangan_Kelayakan"
                ])

    def get_total_csv_rows(self) -> int:
        if not os.path.exists(self.csv_file):
            return 0
        try:
            with open(self.csv_file, 'r', encoding='utf-8') as f:
                return sum(1 for _ in csv.DictReader(f))
        except Exception:
            return 0

    def get_live_pose(self) -> Dict[str, float]:
        """Membaca pose robot real-time di frame map (TF map->base_footprint, fallback /amcl_pose)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            return {
                'x': float(tr.x),
                'y': float(tr.y),
                'yaw': euler_from_quaternion(rot),
            }
        except Exception:
            if self.current_amcl_pose:
                return dict(self.current_amcl_pose)
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def get_laser_map_transform(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        """Ambil posisi & orientasi sensor LiDAR langsung di frame map."""
        target_frame = frame_id.lstrip('/') if frame_id else 'laser'
        try:
            t = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            return float(tr.x), float(tr.y), euler_from_quaternion(rot)
        except Exception:
            live = self.get_live_pose()
            if live:
                return live['x'], live['y'], live['yaw']
            return None

    def on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_amcl_pose = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw': euler_from_quaternion(q),
        }

    def on_odom(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

    def on_scan_filtered(self, msg: LaserScan):
        self.scan_filtered_active = True
        self._process_scan(msg)

    def on_scan_raw(self, msg: LaserScan):
        if not self.scan_filtered_active:
            self._process_scan(msg)

    def _process_scan(self, msg: LaserScan):
        """
        Deteksi rintangan baru di depan robot yang benar-benar memblokir jalur awal (/plan),
        bukan dinding ruangan statis yang memang sudah dihindari sejak awal.
        """
        if not self.is_navigating or self.has_replanned:
            return
        if not self.initial_path_pts:
            return

        tf_laser = self.get_laser_map_transform(msg.header.frame_id)
        if tf_laser is None:
            return
        lx, ly, lyaw = tf_laser

        live = self.get_live_pose()
        rx, ry, ryaw = live['x'], live['y'], live['yaw']

        min_d_on_path = 999.0
        angle = msg.angle_min
        step = max(1, len(msg.ranges) // 180)

        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            ang = angle + i * msg.angle_increment
            if not math.isinf(r) and not math.isnan(r):
                if self.args.obstacle_dist_min <= r <= self.args.obstacle_dist_max:
                    # Proyeksi titik pantul laser ke frame map
                    mx = lx + r * math.cos(lyaw + ang)
                    my = ly + r * math.sin(lyaw + ang)

                    # Pastikan titik berada di area depan robot (+-50 deg terhadap arah robot)
                    bearing_to_robot = math.atan2(my - ry, mx - rx) - ryaw
                    while bearing_to_robot > math.pi:
                        bearing_to_robot -= 2.0 * math.pi
                    while bearing_to_robot < -math.pi:
                        bearing_to_robot += 2.0 * math.pi

                    if abs(bearing_to_robot) <= math.radians(50.0):
                        # Cek apakah titik pantul laser tersebut memotong jalur awal robot
                        d_to_path = min_distance_to_path(mx, my, self.initial_path_pts)
                        if d_to_path <= 0.45:
                            if r < min_d_on_path:
                                min_d_on_path = r

        now_perf = time.perf_counter()
        if min_d_on_path < 900.0:
            self.recent_on_path_hits.append((now_perf, min_d_on_path, dict(live)))
            if self.obstacle_detected_time is None:
                self.obstacle_detected_time = now_perf
                self.obstacle_detect_dist = min_d_on_path
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = live['x']
                ps.pose.position.y = live['y']
                ps.pose.orientation = euler_to_quaternion(math.degrees(live['yaw']))
                self.detect_pose_stamped = ps
                self.get_logger().info(
                    f"⚠️  [DETEKSI RINTANGAN DI JALUR] Boks memblokir jalur awal! Jarak = {min_d_on_path:.2f} m"
                )

    def _match_waypoint_name(self, x: float, y: float) -> str:
        for i, wp in enumerate(self.waypoints):
            if math.hypot(wp['x'] - x, wp['y'] - y) < 0.35:
                prefix = "[HOME] " if i == 0 else ""
                return prefix + wp['name']
        return f"Custom Goal ({x:.2f}, {y:.2f})"

    def on_goal_pose(self, msg: PoseStamped):
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        gyaw = euler_from_quaternion(msg.pose.orientation)
        with self.lock:
            self.active_goal = {
                'name': self._match_waypoint_name(gx, gy),
                'x': gx,
                'y': gy,
                'yaw': gyaw,
                'source': 'topic'
            }

    def on_delivery_status(self, msg: String):
        """Membaca misi aktif dari Web Dashboard waypoint_manager."""
        try:
            data = json.loads(msg.data)
            mission = data.get('mission')
            if not mission or mission.get('status') != 'running':
                return
            phase = mission.get('current_phase', 'idle')
            tasks = mission.get('tasks', [])
            idx = mission.get('current_task_index', 0)

            target_info = None
            if phase == 'returning_home':
                home = self.waypoints[0] if self.waypoints else {'name': 'Home', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                target_info = {
                    'name': "[HOME] Stasiun Pangkalan",
                    'x': float(home['x']),
                    'y': float(home['y']),
                    'yaw': float(home['yaw']),
                    'source': 'dashboard_home'
                }
            elif 0 <= idx < len(tasks):
                task = tasks[idx]
                if phase == 'navigating_pickup':
                    pk = task.get('pickup') or {}
                    target_info = {
                        'name': f"[PICKUP] {pk.get('name', f'Titik Pickup {idx + 1}')}",
                        'x': float(pk.get('x', 0.0)),
                        'y': float(pk.get('y', 0.0)),
                        'yaw': float(pk.get('yaw', 0.0)),
                        'source': 'dashboard_pickup'
                    }
                elif phase == 'navigating_delivery':
                    dl = task.get('delivery') or task.get('destination') or {}
                    prefix = "[DELIVERY]" if task.get('pickup') else "[SEKALI ANTAR]"
                    target_info = {
                        'name': f"{prefix} {dl.get('name', f'Titik Tujuan {idx + 1}')}",
                        'x': float(dl.get('x', 0.0)),
                        'y': float(dl.get('y', 0.0)),
                        'yaw': float(dl.get('yaw', 0.0)),
                        'source': 'dashboard_delivery'
                    }
            if target_info:
                with self.lock:
                    if not self.active_goal or self.active_goal.get('name') != target_info['name']:
                        self.active_goal = target_info
        except Exception:
            pass

    def _reset_trial_state(self):
        self.initial_path = None
        self.initial_path_pts = []
        self.initial_path_len = 0.0
        self.remaining_len_before_replan = 0.0
        self.replanned_path_len = 0.0
        self.max_detour_shift_m = 0.0

        self.obstacle_detected_time = None
        self.obstacle_detect_dist = None
        self.detect_pose_stamped = None
        self.new_path_published_time = None
        self.has_replanned = False
        self.recent_on_path_hits.clear()

        self.astar_compute_ms = None
        self.astar_query_done.clear()
        self.replan_detected_event.clear()

    def on_plan(self, msg: Path):
        """Menerima jalur /plan dari Nav2 planner_server dan mendeteksi manuver detour nyata."""
        if not self.is_navigating or not msg.poses or len(msg.poses) < 2:
            return

        # Lengkapi active_goal jika belum terisi dari dashboard/topic
        lp = msg.poses[-1].pose
        dest_x = float(lp.position.x)
        dest_y = float(lp.position.y)
        dest_yaw = euler_from_quaternion(lp.orientation)
        with self.lock:
            if not self.active_goal or math.hypot(self.active_goal['x'] - dest_x, self.active_goal['y'] - dest_y) > 0.40:
                self.active_goal = {
                    'name': self._match_waypoint_name(dest_x, dest_y),
                    'x': dest_x,
                    'y': dest_y,
                    'yaw': dest_yaw,
                    'source': 'nav_plan'
                }

        # Abaikan jika /plan ujungnya bukan menuju active_goal saat ini
        if self.active_goal and math.hypot(dest_x - self.active_goal['x'], dest_y - self.active_goal['y']) > 0.60:
            return

        path_len = calculate_path_length(msg.poses)

        # Simpan jalur awal pertama sebelum ada halangan
        if self.initial_path is None:
            self.initial_path = msg
            step = max(1, len(msg.poses) // 120)
            self.initial_path_pts = [
                (ps.pose.position.x, ps.pose.position.y)
                for ps in msg.poses[::step]
            ]
            last_p = msg.poses[-1].pose.position
            self.initial_path_pts.append((last_p.x, last_p.y))
            self.initial_path_len = path_len
            self.get_logger().info(
                f"📍 Jalur awal /plan dikunci: {len(msg.poses)} titik, Panjang = {path_len:.2f} m"
            )
            return

        if self.has_replanned:
            return

        # Cek deviasi lateral jalur baru terhadap polyline jalur awal
        max_shift = self._compute_max_lateral_detour(msg.poses)
        if max_shift >= self.args.detour_threshold:
            now_perf = time.perf_counter()

            # Jika detour terbit sebelum callback scan terakhir sempat menandai deteksi, cari di buffer 3 detik terakhir
            if self.obstacle_detected_time is None:
                for t_hit, d_hit, pose_hit in reversed(self.recent_on_path_hits):
                    if now_perf - t_hit <= 3.0:
                        self.obstacle_detected_time = t_hit
                        self.obstacle_detect_dist = d_hit
                        ps = PoseStamped()
                        ps.header.frame_id = 'map'
                        ps.pose.position.x = pose_hit['x']
                        ps.pose.position.y = pose_hit['y']
                        ps.pose.orientation = euler_to_quaternion(math.degrees(pose_hit['yaw']))
                        self.detect_pose_stamped = ps
                        break

            # Hanya catat event replanning jika benar ada halangan terdeteksi dekat jalur (atau deviasi kuat >= detour_threshold)
            self.new_path_published_time = now_perf
            self.has_replanned = True
            self.max_detour_shift_m = max_shift
            self.replanned_path_len = path_len

            start_ref = msg.poses[0].pose.position
            self.remaining_len_before_replan = remaining_path_length_from_pose(
                start_ref.x, start_ref.y, self.initial_path.poses
            )

            latency_str = "N/A"
            if self.obstacle_detected_time is not None:
                lat_ms = (self.new_path_published_time - self.obstacle_detected_time) * 1000.0
                latency_str = f"{lat_ms:.1f} ms"

            self.get_logger().info(
                f"✅ [REPLANNING EVENT] Jalur memutar terbit! Deviasi = {max_shift:.2f} m | "
                f"Sisa Awal = {self.remaining_len_before_replan:.2f} m -> Replan = {path_len:.2f} m | Latensi = {latency_str}"
            )
            self.replan_detected_event.set()

            # Jalankan pengukuran waktu komputasi A* di thread terpisah selagi boks masih ada di costmap
            threading.Thread(target=self._measure_astar_worker, daemon=True).start()

    def _compute_max_lateral_detour(self, new_poses) -> float:
        """Hitung jarak geser lateral terbesar antara jalur baru dengan jalur awal."""
        if not self.initial_path_pts or len(new_poses) < 3:
            return 0.0
        max_d = 0.0
        step = max(1, len(new_poses) // 80)
        for ps in new_poses[::step]:
            d = min_distance_to_path(ps.pose.position.x, ps.pose.position.y, self.initial_path_pts)
            if d > max_d:
                max_d = d
        return max_d

    def _measure_astar_worker(self):
        """Mengukur waktu komputasi murni A* via action /compute_path_to_pose tanpa memblokir spin thread."""
        try:
            if not self.planner_client.wait_for_server(timeout_sec=1.5):
                self.astar_query_done.set()
                return

            start_ps = self.detect_pose_stamped
            if start_ps is None:
                live = self.get_live_pose()
                start_ps = PoseStamped()
                start_ps.header.frame_id = 'map'
                start_ps.pose.position.x = live['x']
                start_ps.pose.position.y = live['y']
                start_ps.pose.orientation = euler_to_quaternion(math.degrees(live['yaw']))

            with self.lock:
                g = dict(self.active_goal) if self.active_goal else None
            if not g:
                self.astar_query_done.set()
                return

            goal_ps = PoseStamped()
            goal_ps.header.frame_id = 'map'
            goal_ps.pose.position.x = float(g['x'])
            goal_ps.pose.position.y = float(g['y'])
            goal_ps.pose.orientation = euler_to_quaternion(math.degrees(g['yaw']))

            req = ComputePathToPose.Goal()
            req.start = start_ps
            req.goal = goal_ps
            req.planner_id = "GridBased"
            req.use_start = True

            t0 = time.perf_counter()
            gh_event = threading.Event()
            gh_holder = {}
            send_fut = self.planner_client.send_goal_async(req)

            def _on_gh(f):
                try:
                    gh_holder['gh'] = f.result()
                except Exception:
                    pass
                gh_event.set()

            send_fut.add_done_callback(_on_gh)
            if not gh_event.wait(timeout=2.0) or not gh_holder.get('gh') or not gh_holder['gh'].accepted:
                self.astar_query_done.set()
                return

            res_event = threading.Event()
            res_holder = {}
            res_fut = gh_holder['gh'].get_result_async()

            def _on_res(f):
                try:
                    res_holder['res'] = f.result().result
                except Exception:
                    pass
                res_event.set()

            res_fut.add_done_callback(_on_res)
            if not res_event.wait(timeout=2.5) or 'res' not in res_holder:
                self.astar_query_done.set()
                return

            t1 = time.perf_counter()
            res_msg = res_holder['res']
            if hasattr(res_msg, 'planning_time') and (res_msg.planning_time.sec > 0 or res_msg.planning_time.nanosec > 0):
                self.astar_compute_ms = (res_msg.planning_time.sec * 1000.0) + (res_msg.planning_time.nanosec / 1e6)
            else:
                self.astar_compute_ms = (t1 - t0) * 1000.0
        except Exception:
            pass
        finally:
            self.astar_query_done.set()

    def on_nav_status(self, msg: GoalStatusArray):
        """Mendeteksi siklus hidup NavigateToPose (mulai, tiba, batal/abort) seperti uji_amcl_dinamis."""
        if not msg.status_list:
            return

        with self.lock:
            target_status = None
            if self.last_goal_id is not None:
                for gs in msg.status_list:
                    if bytes(gs.goal_info.goal_id.uuid) == self.last_goal_id:
                        target_status = gs
                        break

            if target_status is None:
                for gs in reversed(msg.status_list):
                    if gs.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                        target_status = gs
                        self.last_goal_id = bytes(gs.goal_info.goal_id.uuid)
                        break

            if target_status is None:
                target_status = msg.status_list[-1]

            st = target_status.status

            if st in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if not self.is_navigating:
                    self._reset_trial_state()
                    self.is_navigating = True
                    self.nav_start_time = time.time()
                    self.start_pose = self.get_live_pose()
                    self.goal_finished_event.clear()
                    self.nav_started_event.set()

            elif st in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                if self.is_navigating:
                    self.is_navigating = False
                    self.last_goal_id = None
                    final_status = (
                        "SUCCEEDED" if st == GoalStatus.STATUS_SUCCEEDED
                        else ("ABORTED" if st == GoalStatus.STATUS_ABORTED else "CANCELED")
                    )

                    goal = dict(self.active_goal) if self.active_goal else {
                        'name': 'Target Navigasi',
                        'x': self.get_live_pose()['x'],
                        'y': self.get_live_pose()['y'],
                        'yaw': 0.0
                    }

                    latency_ms: Optional[float] = None
                    if self.obstacle_detected_time is not None and self.new_path_published_time is not None:
                        latency_ms = max(0.0, (self.new_path_published_time - self.obstacle_detected_time) * 1000.0)

                    self.finished_run_data = {
                        'goal': goal,
                        'start_pose': dict(self.start_pose) if self.start_pose else {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                        'final_status': final_status,
                        'travel_time': time.time() - (self.nav_start_time or time.time()),
                        'has_replanned': self.has_replanned,
                        'obstacle_dist': self.obstacle_detect_dist,
                        'latency_ms': latency_ms,
                        'initial_path_len': self.initial_path_len,
                        'remaining_before_replan': self.remaining_len_before_replan,
                        'replanned_path_len': self.replanned_path_len,
                        'max_detour_shift': self.max_detour_shift_m,
                    }
                    self.active_goal = None
                    self.goal_finished_event.set()

    def finalize_and_save_run(self, run: Dict, interactive_confirm: bool = True) -> Optional[Dict]:
        """Tampilkan hasil lengkap & simpan ke CSV (hanya nilai terukur nyata, tanpa angka rekaan)."""
        # Tunggu sebentar jika worker A* masih menyelesaikan query
        if run['has_replanned'] and not self.astar_query_done.is_set():
            self.astar_query_done.wait(timeout=1.5)

        astar_ms = self.astar_compute_ms
        latency_ms = run['latency_ms']
        obs_dist = run['obstacle_dist']
        l_init = run['initial_path_len']
        l_rem = run['remaining_before_replan']
        l_rep = run['replanned_path_len']
        delta_l = (l_rep - l_rem) if (run['has_replanned'] and l_rep > 0 and l_rem > 0) else None
        final_status = run['final_status']
        goal = run['goal']
        sp = run['start_pose']

        # Evaluasi waktu replan terhadap ambang <= 200 ms (pakai T_A* jika ada, atau latency sistem)
        t_eval_ms = astar_ms if astar_ms is not None else latency_ms
        is_success = (final_status == "SUCCEEDED")

        if not run['has_replanned']:
            event_str = "TIDAK_TERDETEKSI"
            ket_str = "TANPA_EVENT_REPLAN"
        elif is_success and (t_eval_ms is None or t_eval_ms <= float(self.args.pass_max_replan_time)):
            event_str = "TERDETEKSI"
            ket_str = "MEMENUHI KELAYAKAN (LULUS)"
        else:
            event_str = "TERDETEKSI"
            ket_str = "TIDAK MEMENUHI"

        obs_txt = f"{obs_dist:.2f} m" if obs_dist is not None else "Tidak terdeteksi di depan jalur"
        astar_txt = f"{astar_ms:.2f} ms" if astar_ms is not None else "N/A"
        lat_txt = f"{latency_ms:.1f} ms" if latency_ms is not None else "N/A"
        rep_txt = f"{l_rep:.2f} m (Sisa awal: {l_rem:.2f} m, Delta: {delta_l:+.2f} m)" if delta_l is not None else "N/A"

        print("\n" + "=" * 75)
        print(f"🏁 NAVIGASI SELESAI: '{goal['name']}' | Status: {final_status}")
        print("=" * 75)
        print(f"  • Titik Start -> Goal      : ({sp['x']:.2f}, {sp['y']:.2f}) -> ({goal['x']:.2f}, {goal['y']:.2f})")
        print(f"  • Durasi Navigasi          : {run['travel_time']:.2f} detik")
        print(f"  • Event Replanning A*      : {event_str} (Deviasi lateral max: {run['max_detour_shift']:.2f} m)")
        print(f"  • Jarak Deteksi Boks       : {obs_txt}")
        print(f"  • Waktu Komputasi A*       : {astar_txt} (Batas <= {self.args.pass_max_replan_time:.0f} ms)")
        print(f"  • Latensi Replan Sistem    : {lat_txt}")
        print(f"  • Panjang Rute Awal        : {l_init:.2f} m")
        print(f"  • Rute Saat Replan         : {rep_txt}")
        print(f"  • Keterangan Evaluasi      : {ket_str}")
        print("=" * 75)

        save_to_csv = True
        if interactive_confirm and not run['has_replanned']:
            ans = timed_input(
                "ℹ️  Tidak ada manuver detour/replan terdeteksi (mis. trip pulang atau koridor kosong).\n"
                "   Simpan baris ini ke CSV Skenario 4? (y/n) [Default: n]: ",
                default_val="n",
                timeout_sec=8.0,
                cancel_check_fn=lambda: self.is_navigating
            ).lower()
            save_to_csv = (ans == 'y')

        if save_to_csv:
            trial_num = self.get_total_csv_rows() + 1
            row = [
                trial_num,
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                goal['name'],
                f"{sp['x']:.2f}", f"{sp['y']:.2f}",
                f"{goal['x']:.2f}", f"{goal['y']:.2f}",
                f"{obs_dist:.2f}" if obs_dist is not None else "-",
                f"{astar_ms:.2f}" if astar_ms is not None else "-",
                f"{latency_ms:.1f}" if latency_ms is not None else "-",
                f"{l_init:.2f}",
                f"{l_rem:.2f}" if l_rem > 0 else "-",
                f"{l_rep:.2f}" if l_rep > 0 else "-",
                f"{delta_l:+.2f}" if delta_l is not None else "-",
                f"{run['max_detour_shift']:.2f}" if run['max_detour_shift'] > 0 else "-",
                event_str,
                final_status,
                ket_str
            ]
            with open(self.csv_file, mode='a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(row)
            print(f"💾 Data percobaan #{trial_num} tersimpan ke: {self.csv_file}\n")
        else:
            print("⏭️  Trip tanpa rintangan dilewati (tidak ditulis ke CSV Replanning).\n")

        return {
            "saved": save_to_csv,
            "is_success": is_success,
            "has_replanned": run['has_replanned'],
            "astar_ms": astar_ms,
            "latency_ms": latency_ms,
        }

    def send_goal(self, name: str, gx: float, gy: float, gyaw_deg: float) -> bool:
        """Kirim perintah NavigateToPose sesuai titik yang dipilih/diinput user."""
        if not self.nav_client.wait_for_server(timeout_sec=4.0):
            print("⚠️ Action server /navigate_to_pose tidak aktif! Pastikan navigasi robot sudah dijalankan.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = euler_to_quaternion(float(gyaw_deg))

        with self.lock:
            self.active_goal = {
                'name': name,
                'x': float(gx),
                'y': float(gy),
                'yaw': math.radians(float(gyaw_deg)),
                'source': 'terminal'
            }

        print(f"\n🚀 Mengirim Goal ke '{name}': X={gx:.2f} m, Y={gy:.2f} m, Yaw={gyaw_deg:.1f}°...")
        fut = self.nav_client.send_goal_async(goal_msg)
        ev = threading.Event()
        holder = {}

        def _cb(f):
            try:
                holder['gh'] = f.result()
            except Exception:
                pass
            ev.set()

        fut.add_done_callback(_cb)
        if not ev.wait(timeout=4.0) or not holder.get('gh') or not holder['gh'].accepted:
            print("❌ Goal navigasi ditolak oleh Nav2 (pastikan titik berada di area free peta).")
            with self.lock:
                self.active_goal = None
            return False
        return True

    def print_recap_table(self):
        """Tampilkan rekapitulasi tabel CSV Skenario 4 untuk Bab 4 Skripsi."""
        if not os.path.exists(self.csv_file):
            print("\n⚠️ Belum ada file CSV pengujian Skenario 4.")
            return

        with open(self.csv_file, 'r', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))

        if not rows:
            print("\n⚠️ Belum ada rekaman data percobaan di CSV.")
            return

        print("\n" + "=" * 96)
        print("📋 REKAPITULASI EVALUASI SKENARIO 4: REPLANNING EVENT A* (BAB 4 SKRIPSI)")
        print("=" * 96)
        print(f"{'No':<3} | {'Target':<20} | {'Goal (X,Y)':<14} | {'Jarak Boks':<10} | {'T_A* (ms)':<9} | {'T_sys (ms)':<10} | {'Status':<9} | {'Hasil'}")
        print("-" * 96)

        astar_vals = []
        lat_vals = []
        success_replan_trials = 0
        replan_rows = 0

        for r in rows:
            ev = r.get('Event_Replan', '-')
            st = r.get('Status_Navigasi', '-')
            if ev == 'TERDETEKSI':
                replan_rows += 1
                if st == 'SUCCEEDED':
                    success_replan_trials += 1
            try:
                if r.get('Astar_Compute_Time_ms', '-') != '-':
                    astar_vals.append(float(r['Astar_Compute_Time_ms']))
            except ValueError:
                pass
            try:
                if r.get('Replanning_Latency_ms', '-') != '-':
                    lat_vals.append(float(r['Replanning_Latency_ms']))
            except ValueError:
                pass

            g_xy = f"({r.get('Goal_X','0')},{r.get('Goal_Y','0')})"
            print(
                f"{r.get('Percobaan','-'):<3} | {r.get('Nama_Target','-')[:20]:<20} | {g_xy:<14} | "
                f"{r.get('Jarak_Obstacle_Deteksi_m','-'):<10} | {r.get('Astar_Compute_Time_ms','-'):<9} | "
                f"{r.get('Replanning_Latency_ms','-'):<10} | {st:<9} | {r.get('Keterangan_Kelayakan','-')}"
            )

        print("-" * 96)
        if replan_rows > 0:
            sr = (success_replan_trials / replan_rows) * 100.0
            print(f"🏆 Ringkasan Percobaan Replanning ({replan_rows} kali): Success Rate = {sr:.1f}% ({success_replan_trials}/{replan_rows})")
        if astar_vals:
            print(f"   • Rata-rata Waktu Komputasi A*   : {sum(astar_vals)/len(astar_vals):.2f} ms "
                  f"(Min: {min(astar_vals):.2f} ms, Max: {max(astar_vals):.2f} ms)")
        if lat_vals:
            print(f"   • Rata-rata Latensi Sistem Total : {sum(lat_vals)/len(lat_vals):.1f} ms")
        print(f"📁 Lokasi File CSV: {self.csv_file}")
        print("=" * 96)

    def delete_last_csv_row(self):
        if not os.path.exists(self.csv_file):
            print("⚠️ File CSV belum ada.")
            return
        with open(self.csv_file, 'r', encoding='utf-8') as f:
            rows = list(csv.reader(f))
        if len(rows) <= 1:
            print("⚠️ Belum ada data untuk dihapus.")
            return
        last_row = rows[-1]
        c = input(f"⚠️ Yakin hapus baris #{last_row[0]} ('{last_row[2]}')? (y/n): ").strip().lower()
        if c == 'y':
            rows.pop()
            with open(self.csv_file, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerows(rows)
            print("✅ Baris terakhir berhasil dihapus dari CSV!")


def watch_navigation_step(node: EvaluasiReplanningNode):
    """Helper loop untuk memantau 1 siklus perjalanan aktif sampai selesai."""
    if node.nav_started_event.is_set():
        node.nav_started_event.clear()
        with node.lock:
            goal = dict(node.active_goal) if node.active_goal else {'name': 'Target', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            sp = dict(node.start_pose) if node.start_pose else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        print("\n" + "=" * 75)
        print(f"🚀 NAVIGASI AKTIF TERDETEKSI: Menuju '{goal['name']}'")
        print(f"📍 Target Goal : X = {goal['x']:.2f} m, Y = {goal['y']:.2f} m, Yaw = {math.degrees(goal['yaw']):+.1f}°")
        print(f"🛫 Pose Awal   : X = {sp['x']:.2f} m, Y = {sp['y']:.2f} m")
        print("👉 Silakan letakkan boks rintangan di jalur depan robot saat robot melaju...")
        print("=" * 75)

    if node.is_navigating:
        now = time.time()
        if now - node.last_log_time >= 0.5:
            node.last_log_time = now
            el = now - (node.nav_start_time or now)
            live = node.get_live_pose()
            tag_rep = "REPLAN_OK" if node.has_replanned else ("OBS_SEEN" if node.obstacle_detected_time else "NORMAL")
            sys.stdout.write(
                f"\r  ⏱️ [{el:4.1f}s] Speed: {node.current_speed:4.2f}m/s | "
                f"Pose: ({live['x']:5.2f}, {live['y']:5.2f}) | State: {tag_rep:<9} "
            )
            sys.stdout.flush()


def run_monitor_loop(node: EvaluasiReplanningNode):
    """Mode Standby / Monitor Pasif: user bebas kirim Goal lewat Web Dashboard atau RViz."""
    print("\n" + "-" * 75)
    print("👀 MONITOR SKENARIO 4 AKTIF: Silakan kirim Goal dari Web Dashboard atau RViz...")
    print("👉 (Tekan Ctrl+C kapan saja untuk kembali ke menu utama)")
    print("-" * 75)
    try:
        while rclpy.ok():
            time.sleep(0.05)
            watch_navigation_step(node)
            if node.goal_finished_event.is_set():
                node.goal_finished_event.clear()
                if node.finished_run_data:
                    node.finalize_and_save_run(node.finished_run_data, interactive_confirm=True)
                    node.finished_run_data = None
                if not node.is_navigating:
                    print("👀 Kembali standby memantau navigasi berikutnya (kirim Goal lagi atau taruh boks)...")
    except KeyboardInterrupt:
        print("\n🔙 Monitor dihentikan, kembali ke menu utama.")


def select_target_interactively(node: EvaluasiReplanningNode) -> Optional[Tuple[str, float, float, float]]:
    """Meminta user memilih waypoint dari waypoints.yaml atau memasukkan koordinat X, Y, Yaw sendiri."""
    print("\n" + "-" * 75)
    print("📍 PILIH TITIK TUJUAN PENGUJIAN REPLANNING:")
    if node.waypoints:
        mode_lbl = node.waypoints[0].get('mode', '?')
        print(f"   (Sumber waypoints.yaml profil '{mode_lbl}')")
        for i, wp in enumerate(node.waypoints):
            print(f"   [{i}] {wp['name']:<26} (X={wp['x']:.2f}, Y={wp['y']:.2f}, Yaw={math.degrees(wp['yaw']):.0f}°)")
    else:
        print("   ⚠️ waypoints.yaml tidak ditemukan / kosong. Silakan input koordinat manual.")
    print("   [C] Input Koordinat Manual (X, Y, Yaw)")
    print("   [B] Batal kembali ke menu")
    print("-" * 75)

    choice = input("Pilih nomor titik / C / B: ").strip()
    if choice.lower() == 'b' or not choice:
        return None
    if choice.lower() == 'c':
        try:
            name = input("  Nama titik uji (misal Koridor A): ").strip() or "Custom Uji Replan"
            gx = float(input("  Koordinat X (meter): ").strip())
            gy = float(input("  Koordinat Y (meter): ").strip())
            gyaw_str = input("  Sudut Yaw derajat [Default 0]: ").strip()
            gyaw = float(gyaw_str) if gyaw_str else 0.0
            return name, gx, gy, gyaw
        except ValueError:
            print("⚠️ Input koordinat harus berupa angka.")
            return None
    if choice.isdigit() and 0 <= int(choice) < len(node.waypoints):
        wp = node.waypoints[int(choice)]
        return wp['name'], float(wp['x']), float(wp['y']), math.degrees(float(wp['yaw']))

    print("⚠️ Pilihan tidak valid.")
    return None


def run_auto_target_trials(node: EvaluasiReplanningNode, name: str, gx: float, gy: float, gyaw_deg: float, trials: int):
    """Kirim Goal yang ditentukan user sebanyak N percobaan dan kawal tiap percobaan."""
    for t in range(1, trials + 1):
        print(f"\n🔄 PERCOBAAN {t} / {trials} menuju '{name}' ({gx:.2f}, {gy:.2f})")
        node.goal_finished_event.clear()
        node.finished_run_data = None
        if not node.send_goal(name, gx, gy, gyaw_deg):
            break

        try:
            while rclpy.ok() and not node.goal_finished_event.is_set():
                time.sleep(0.05)
                watch_navigation_step(node)

            if node.goal_finished_event.is_set():
                node.goal_finished_event.clear()
                if node.finished_run_data:
                    node.finalize_and_save_run(node.finished_run_data, interactive_confirm=False)
                    node.finished_run_data = None
        except KeyboardInterrupt:
            print("\n🔙 Pengawalan percobaan dihentikan.")
            break

        if t < trials:
            ans = input(
                f"👉 Percobaan {t} selesai. Angkat boks & posisikan ulang robot di titik Start.\n"
                f"   Tekan [ENTER] untuk Percobaan {t + 1} (atau ketik 'q' untuk berhenti): "
            ).strip().lower()
            if ans == 'q':
                break


def main():
    parser = argparse.ArgumentParser(
        description="Skrip Pengujian Skenario 4: Evaluasi Replanning A* (Support Monitor Dashboard & Pilihan Titik)"
    )
    parser.add_argument('--mode', type=str, default=None, choices=['auto', 'monitor'],
                        help="Kosongkan untuk Menu Interaktif. 'monitor' untuk langsung standby Web Dashboard/RViz, 'auto' untuk kirim goal langsung.")
    parser.add_argument('--trials', type=int, default=10,
                        help="Jumlah repetisi jika memakai mode auto (default: 10)")
    parser.add_argument('--goal-x', type=float, default=None,
                        help="Koordinat X Goal (wajib diisi user atau pilih dari waypoint, tidak ada default di luar peta)")
    parser.add_argument('--goal-y', type=float, default=None,
                        help="Koordinat Y Goal (wajib diisi user atau pilih dari waypoint, tidak ada default di luar peta)")
    parser.add_argument('--goal-yaw', type=float, default=0.0,
                        help="Sudut Yaw Goal dalam derajat (default: 0.0)")
    parser.add_argument('--goal-name', type=str, default="Target Uji Replan",
                        help="Label nama target untuk mode CLI auto")
    parser.add_argument('--obstacle-dist-min', type=float, default=0.35,
                        help="Jarak minimum deteksi boks di jalur (meter, default: 0.35)")
    parser.add_argument('--obstacle-dist-max', type=float, default=2.50,
                        help="Jarak maksimum deteksi boks di jalur (meter, default: 2.50)")
    parser.add_argument('--detour-threshold', type=float, default=0.25,
                        help="Geseran lateral minimum jalur /plan agar diakui sebagai event detour (meter, default: 0.25)")
    parser.add_argument('--pass-max-replan-time', type=float, default=200.0,
                        help="Ambang batas maksimum waktu replanning A* (ms, default: 200.0)")
    parser.add_argument('--out-dir', type=str, default=None,
                        help="Direktori penyimpanan CSV (default otomatis: <workspace>/hasil_data_pengujian_skripsi/pengujian_4_replanning)")

    ros_args = None
    if '--ros-args' in sys.argv:
        idx = sys.argv.index('--ros-args')
        app_args = sys.argv[1:idx]
        ros_args = sys.argv[idx:]
    else:
        app_args = sys.argv[1:]

    parsed_args = parser.parse_args(app_args)

    rclpy.init(args=ros_args)
    node = EvaluasiReplanningNode(parsed_args)

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    try:
        if parsed_args.mode == 'monitor':
            run_monitor_loop(node)

        elif parsed_args.mode == 'auto':
            if parsed_args.goal_x is not None and parsed_args.goal_y is not None:
                run_auto_target_trials(
                    node,
                    parsed_args.goal_name,
                    parsed_args.goal_x,
                    parsed_args.goal_y,
                    parsed_args.goal_yaw,
                    parsed_args.trials
                )
            else:
                target = select_target_interactively(node)
                if target:
                    run_auto_target_trials(node, *target, trials=parsed_args.trials)

        else:
            # Menu Interaktif Utama (seperti uji_amcl_dinamis_skripsi.py)
            while rclpy.ok():
                print("\n" + "=" * 75)
                print("🤖 BENCHMARK SKRIPSI AMR-01: SKENARIO 4 (REPLANNING A* & OBSTACLE BARU)")
                print("=" * 75)
                print(f"  Batas Kelayakan A* : <= {parsed_args.pass_max_replan_time:.0f} ms | Detour Threshold: >= {parsed_args.detour_threshold:.2f} m")
                print(f"  File Log CSV       : {node.csv_file}")
                print(f"  Total Data Rekaman : {node.get_total_csv_rows()} percobaan")
                print("-" * 75)
                print("  [1] MODE STANDBY / MONITOR PASIF (Kirim Goal bebas dari Web Dashboard / RViz)")
                print("  [2] Kirim Goal Sendiri (Pilih dari waypoints.yaml atau Input X, Y, Yaw)")
                print("  [3] Tampilkan Rekapitulasi Tabel & Statistik Bab 4")
                print("  [4] Hapus 1 Baris Data Terakhir CSV")
                print("  [5] Reset / Hapus Seluruh File CSV")
                print("  [0] Keluar")
                print("=" * 75)

                choice = input("Pilih menu (1/2/3/4/5/0) [Default: 1]: ").strip() or '1'

                if choice == '1':
                    run_monitor_loop(node)

                elif choice == '2':
                    target = select_target_interactively(node)
                    if target:
                        t_in = input("Jumlah pengulangan percobaan [Default: 1]: ").strip()
                        n_trials = int(t_in) if t_in.isdigit() and int(t_in) > 0 else 1
                        run_auto_target_trials(node, *target, trials=n_trials)

                elif choice == '3':
                    node.print_recap_table()

                elif choice == '4':
                    node.delete_last_csv_row()

                elif choice == '5':
                    c = input("⚠️ Yakin ingin menghapus seluruh file CSV Skenario 4? (ketik 'ya'): ").strip().lower()
                    if c == 'ya':
                        if os.path.exists(node.csv_file):
                            os.remove(node.csv_file)
                        node._init_csv()
                        print("✅ File CSV berhasil di-reset!")

                elif choice == '0':
                    print("👋 Keluar dari evaluasi Skenario 4.")
                    break

    except KeyboardInterrupt:
        print("\n🛑 Program dihentikan pengguna.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
