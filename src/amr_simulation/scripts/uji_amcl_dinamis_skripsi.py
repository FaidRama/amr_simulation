#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=============================================================================
SKRIPSI BAB 4 - SKENARIO 2: EVALUASI KEANDALAN LOKALISASI AMCL & NAVIGASI DINAMIS
(Dynamic Pose Tracking, Waypoint Navigation & Goal Arrival Accuracy)
=============================================================================
Penulis : Divala Zahra Oktavia (Teknik Komputer UNNES 2026)
Deskripsi:
  Mengevaluasi keandalan filter partikel AMCL dan kontrol DWA (DWB Local Planner)
  saat robot bergerak secara dinamis melintasi peta menuju titik tujuan (Goal Waypoint):
  1. Mode Monitor Pasif (Default):
     - Skrip standby memantau sistem. Operator bebas memerintahkan navigasi lewat
       Web Dashboard (1-Click Dispatch / Antar-Jemput) atau RViz2.
     - Begitu tujuan navigasi diterima, skrip otomatis mengunci waktu awal,
       merekam koordinat awal dan koordinat target.
     - Selama perjalanan, skrip memantau kestabilan partikel AMCL (sigma X, Y, Yaw).
     - Saat robot tiba di tujuan (Goal Reached), skrip otomatis menghentikan timer,
       menghitung galat estimasi AMCL, dan memberikan opsi verifikasi fisik manual di lantai.
  2. Mode Kirim Langsung dari Terminal:
     - Membaca daftar waypoint dari waypoints.yaml dan mengirimkan goal secara langsung.
  3. Metrik yang Dievaluasi:
     - Waktu Tempuh Perjalanan (T_travel [detik])
     - Panjang Lintasan Tempuh (L_path [meter]) & Kecepatan Rata-rata (V_avg [m/s])
     - Kepadatan Partikel AMCL saat Tiba (Sigma X, Sigma Y, Sigma Yaw)
     - Galat Posisi Akhir (e_pos [cm] -> Toleransi kelulusan: <= 5.0 cm)
     - Galat Orientasi Akhir (e_theta [deg] -> Toleransi kelulusan: <= 10.0° atau <= 3.0°)
     - Tingkat Keberhasilan Tiba di Titik Tujuan (Success Rate, SR [%])
  4. Penyimpanan Hasil:
     - Tersimpan otomatis ke ~/hasil_pengujian_skripsi/skenario2_evaluasi_amcl_dinamis.csv
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
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def euler_from_quaternion(q):
    """Konversi quaternion ke yaw (radian)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_euler(roll, pitch, yaw):
    """Konversi rpy (radian) ke Quaternion geometry_msgs."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def normalize_angle(angle):
    """Normalisasi sudut ke rentang [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_nav2_tolerances():
    """Membaca parameter toleransi kedatangan langsung dari nav2_params.yaml."""
    candidates = [
        os.path.join(os.path.expanduser('~'), 'Documents', 'amr_ws', 'src', 'amr_simulation', 'config', 'nav2_params.yaml'),
        os.path.join(os.path.expanduser('~'), 'deliver_amr', 'src', 'amr_simulation', 'config', 'nav2_params.yaml'),
    ]
    xy_tol = 0.05
    yaw_tol = 0.20
    for p in candidates:
        if os.path.exists(p):
            try:
                with open(p, 'r') as f:
                    cfg = yaml.safe_load(f) or {}
                ctrl = cfg.get('controller_server', {}).get('ros__parameters', {})
                checker = ctrl.get('general_goal_checker', {})
                if 'xy_goal_tolerance' in checker:
                    xy_tol = float(checker['xy_goal_tolerance'])
                if 'yaw_goal_tolerance' in checker:
                    yaw_tol = float(checker['yaw_goal_tolerance'])
                break
            except Exception:
                pass
    return xy_tol, yaw_tol


def timed_input(prompt: str, default_val: str = "", timeout_sec: float = 6.0, cancel_check_fn=None) -> str:
    """Membaca input terminal dengan batas waktu otomatis (non-blocking) agar tidak membekukan pengujian otomatis."""
    sys.stdout.write(prompt)
    sys.stdout.flush()

    if not sys.stdin.isatty():
        sys.stdout.write(f"{default_val}\n")
        sys.stdout.flush()
        return default_val

    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        # Cek apakah navigasi berikutnya sudah terlanjur jalan (misal auto return-home)
        if cancel_check_fn and cancel_check_fn():
            sys.stdout.write(f"\n⏱️ [AUTO-RESUME] Navigasi berikutnya terdeteksi jalan! Menggunakan estimasi sensor ({default_val}).\n")
            sys.stdout.flush()
            return default_val

        # Polling stdin 0.1 detik
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                line = sys.stdin.readline()
                val = line.strip()
                return val if val else default_val
        except Exception:
            return default_val

    sys.stdout.write(f"\n⏱️ [TIMEOUT {timeout_sec:.0f}s] Waktu input selesai. Menggunakan estimasi sensor ({default_val}).\n")
    sys.stdout.flush()
    return default_val


class EvaluasiAMCLDinamisNode(Node):
    def __init__(self):
        super().__init__('evaluasi_amcl_dinamis_node')

        # Toleransi Kelulusan Otomatis dari nav2_params.yaml
        xy_tol, yaw_tol = load_nav2_tolerances()
        self.pass_max_epos_cm = xy_tol * 100.0
        self.pass_max_eyaw_deg = math.degrees(yaw_tol)

        # Direktori Output CSV
        self.out_dir = os.path.expanduser('~/hasil_pengujian_skripsi')
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, 'skenario2_evaluasi_amcl_dinamis.csv')

        # QoS
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Action Client Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF2 Listener (sinkron dengan pose real-time map->base_footprint yang dievaluasi Nav2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, tl_qos)
        self.amcl_sub_vol = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, reliable_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, reliable_qos)
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.on_plan, reliable_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal_pose, reliable_qos)
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_action_status, reliable_qos)
        self.delivery_status_sub = self.create_subscription(
            String, '/amr/delivery_status', self.on_delivery_status, 10)

        # State Telemetri
        self.current_pose = None      # dict: {'x', 'y', 'yaw'}
        self.current_speed = 0.0      # m/s dari /odom
        self.sigma_x = 0.0
        self.sigma_y = 0.0
        self.sigma_yaw = 0.0
        self.last_amcl_stamp = 0.0

        # State Navigasi Dinamis
        self.is_navigating = False
        self.active_goal = None       # dict: {'name', 'x', 'y', 'yaw', 'source'}
        self.start_pose = None        # dict: {'x', 'y', 'yaw'}
        self.start_time = None
        self.active_path = None
        self.planned_path_len = 0.0
        self.actual_travel_dist = 0.0
        self.prev_odom_pos = None

        self.last_goal_id = None
        self.last_terminal_log_time = 0.0

        # Lock untuk event handling
        self.lock = threading.Lock()
        self.goal_reached_event = threading.Event()
        self.goal_aborted_event = threading.Event()
        self.nav_started_event = threading.Event()
        self.arrival_data = None

        # Konfigurasi Waypoint Peta
        self.waypoints = self.load_waypoints_config()

        self.ensure_csv_header()
        self.get_logger().info("✅ Node Evaluasi AMCL & Navigasi Dinamis Siap!")

    def load_waypoints_config(self) -> list:
        """Memuat daftar waypoint tersimpan dari waypoints.yaml."""
        candidates = [
            os.path.join(os.path.expanduser('~'), 'Documents', 'amr_ws', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.expanduser('~'), 'deliver_amr', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
        ]
        for p in candidates:
            if os.path.exists(p):
                try:
                    with open(p, 'r') as f:
                        cfg = yaml.safe_load(f) or {}
                    mode = cfg.get('operation_mode', 'real')
                    pts = cfg.get('profiles', {}).get(mode, {}).get('predefined_points', [])
                    if not pts and 'predefined_points' in cfg:
                        pts = cfg.get('predefined_points', [])
                    home = cfg.get('profiles', {}).get(mode, {}).get('home_position', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
                    res = [{'name': 'Home / Charging Station', 'x': home.get('x', 0.0), 'y': home.get('y', 0.0), 'yaw': home.get('yaw', 0.0)}]
                    for pt in pts:
                        res.append({
                            'name': pt.get('name', f"Point {len(res)}"),
                            'x': float(pt.get('x', 0.0)),
                            'y': float(pt.get('y', 0.0)),
                            'yaw': float(pt.get('yaw', 0.0))
                        })
                    return res
                except Exception as e:
                    self.get_logger().warn(f"Gagal memuat waypoints.yaml: {e}")
        return [
            {'name': 'Titik Awal (0,0)', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'name': 'Uji 1', 'x': 8.4, 'y': -2.55, 'yaw': 0.0}
        ]

    def get_live_pose(self) -> dict:
        """Membaca pose real-time dari TF map->base_footprint agar 100% sinkron dengan Nav2.
        Fallback ke /amcl_pose jika TF belum tersedia.
        """
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
            if self.current_pose:
                return dict(self.current_pose)
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q)

        cov = msg.pose.covariance
        self.sigma_x = math.sqrt(max(0.0, float(cov[0])))
        self.sigma_y = math.sqrt(max(0.0, float(cov[7])))
        self.sigma_yaw = math.sqrt(max(0.0, float(cov[35])))

        self.current_pose = {'x': float(p.x), 'y': float(p.y), 'yaw': yaw}
        self.last_amcl_stamp = time.time()

    def on_odom(self, msg: Odometry):
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        pos = msg.pose.pose.position
        if self.is_navigating and self.prev_odom_pos:
            dx = pos.x - self.prev_odom_pos[0]
            dy = pos.y - self.prev_odom_pos[1]
            d = math.hypot(dx, dy)
            if d < 1.0:  # filter jump
                self.actual_travel_dist += d
        self.prev_odom_pos = (pos.x, pos.y)

    def on_plan(self, msg: Path):
        self.active_path = msg
        if len(msg.poses) >= 2:
            tot = 0.0
            for i in range(1, len(msg.poses)):
                p0 = msg.poses[i - 1].pose.position
                p1 = msg.poses[i].pose.position
                tot += math.hypot(p1.x - p0.x, p1.y - p0.y)
            self.planned_path_len = tot

            # Ekstrak titik tujuan akhir dari lintasan Nav2
            lp = msg.poses[-1].pose
            dest_x = float(lp.position.x)
            dest_y = float(lp.position.y)
            dest_yaw = euler_from_quaternion(lp.orientation)

            # Cari waypoint terdekat
            dest_name = None
            for wp in self.waypoints:
                if math.hypot(wp['x'] - dest_x, wp['y'] - dest_y) < 0.35:
                    dest_name = wp['name']
                    break

            with self.lock:
                # Update jika active_goal kosong atau jika target tujuan berbeda jauh (> 0.40m)
                if not self.active_goal or math.hypot(self.active_goal['x'] - dest_x, self.active_goal['y'] - dest_y) > 0.40:
                    prefix = ""
                    if self.waypoints and math.hypot(self.waypoints[0]['x'] - dest_x, self.waypoints[0]['y'] - dest_y) < 0.35:
                        prefix = "[HOME] "
                    target_name = prefix + (dest_name if dest_name else f"Goal ({dest_x:.2f}, {dest_y:.2f})")
                    self.active_goal = {
                        'name': target_name,
                        'x': dest_x,
                        'y': dest_y,
                        'yaw': dest_yaw,
                        'source': 'nav_plan'
                    }

    def on_goal_pose(self, msg: PoseStamped):
        """Mendeteksi saat pengguna mengirim goal dari RViz atau Dashboard."""
        with self.lock:
            gx = float(msg.pose.position.x)
            gy = float(msg.pose.position.y)
            gyaw = euler_from_quaternion(msg.pose.orientation)

            # Cari apakah cocok dengan salah satu nama waypoint
            matched_name = "Custom Goal Target"
            for wp in self.waypoints:
                if math.hypot(wp['x'] - gx, wp['y'] - gy) < 0.35:
                    matched_name = wp['name']
                    break

            prefix = ""
            if self.waypoints and math.hypot(self.waypoints[0]['x'] - gx, self.waypoints[0]['y'] - gy) < 0.35:
                prefix = "[HOME] "

            self.active_goal = {
                'name': prefix + matched_name,
                'x': gx,
                'y': gy,
                'yaw': gyaw,
                'source': 'topic'
            }

    def on_delivery_status(self, msg: String):
        """Membaca status pengiriman misi dari waypoint_manager dashboard (mendukung 1x Antar, Antar-Jemput, & Return Home)."""
        try:
            data = json.loads(msg.data)
            mission = data.get('mission')
            if not mission or mission.get('status') != 'running':
                return

            phase = mission.get('current_phase', 'idle')
            tasks = mission.get('tasks', [])
            idx = mission.get('current_task_index', 0)

            target_info = None
            # Prioritas 1: Fase Pulang ke Home (Auto Return Home)
            if phase == 'returning_home':
                home_wp = self.waypoints[0] if self.waypoints else {'name': 'Home', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                target_info = {
                    'name': "[HOME] Stasiun Pangkalan",
                    'x': float(home_wp['x']),
                    'y': float(home_wp['y']),
                    'yaw': float(home_wp['yaw']),
                    'source': 'dashboard_home'
                }
            # Prioritas 2: Task Pickup atau Delivery
            elif 0 <= idx < len(tasks):
                task = tasks[idx]
                if phase == 'navigating_pickup':
                    pk = task.get('pickup') or {}
                    target_info = {
                        'name': f"[PICKUP] {pk.get('name', f'Titik Pickup {idx+1}')}",
                        'x': float(pk.get('x', 0.0)),
                        'y': float(pk.get('y', 0.0)),
                        'yaw': float(pk.get('yaw', 0.0)),
                        'source': 'dashboard_pickup'
                    }
                elif phase == 'navigating_delivery':
                    dl = task.get('delivery') or task.get('destination') or {}
                    prefix = "[DELIVERY]" if task.get('pickup') else "[SEKALI ANTAR]"
                    target_info = {
                        'name': f"{prefix} {dl.get('name', f'Titik Tujuan {idx+1}')}",
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

    def on_action_status(self, msg: GoalStatusArray):
        """Mendeteksi siklus hidup action NavigateToPose (mulai jalan, tiba, gagal)."""
        if not msg.status_list:
            return

        with self.lock:
            # 1. Cari status untuk goal spesifik yang sedang kita pantau
            target_status = None
            if self.last_goal_id is not None:
                for gs in msg.status_list:
                    if bytes(gs.goal_info.goal_id.uuid) == self.last_goal_id:
                        target_status = gs
                        break

            # 2. Jika belum mengunci goal_id atau baru mulai, cari goal baru yang sedang aktif
            if target_status is None:
                for gs in reversed(msg.status_list):
                    if gs.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                        target_status = gs
                        self.last_goal_id = bytes(gs.goal_info.goal_id.uuid)
                        break

            # 3. Fallback jika tidak ada status aktif
            if target_status is None:
                target_status = msg.status_list[-1]

            st = target_status.status

            # Navigasi Diterima & Mulai Berjalan
            if st in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if not self.is_navigating:
                    self.is_navigating = True
                    self.start_time = time.time()
                    self.actual_travel_dist = 0.0
                    self.start_pose = self.get_live_pose()
                    if not self.active_goal:
                        if self.active_path and len(self.active_path.poses) > 0:
                            lp = self.active_path.poses[-1].pose
                            dest_x = float(lp.position.x)
                            dest_y = float(lp.position.y)
                            dest_yaw = euler_from_quaternion(lp.orientation)
                            matched_name = "Target Navigasi"
                            for wp in self.waypoints:
                                if math.hypot(wp['x'] - dest_x, wp['y'] - dest_y) < 0.35:
                                    matched_name = wp['name']
                                    break
                            prefix = "[HOME] " if (self.waypoints and math.hypot(self.waypoints[0]['x'] - dest_x, self.waypoints[0]['y'] - dest_y) < 0.35) else ""
                            self.active_goal = {
                                'name': prefix + matched_name,
                                'x': dest_x,
                                'y': dest_y,
                                'yaw': dest_yaw,
                                'source': 'nav_plan'
                            }
                        else:
                            self.active_goal = {
                                'name': 'Goal Navigasi Aktif',
                                'x': self.start_pose['x'],
                                'y': self.start_pose['y'],
                                'yaw': self.start_pose['yaw'],
                                'source': 'auto'
                            }
                    self.goal_reached_event.clear()
                    self.goal_aborted_event.clear()
                    self.nav_started_event.set()

            # Navigasi Sukses Tiba di Tujuan (SUCCEEDED)
            elif st == GoalStatus.STATUS_SUCCEEDED:
                if self.is_navigating:
                    self.is_navigating = False
                    self.last_goal_id = None
                    end_time = time.time()
                    travel_time = end_time - (self.start_time or end_time)
                    final_amcl = self.get_live_pose()
                    goal = dict(self.active_goal) if self.active_goal else {'name': 'Target', 'x': final_amcl['x'], 'y': final_amcl['y'], 'yaw': final_amcl['yaw']}

                    self.arrival_data = {
                        'status': 'SUCCEEDED',
                        'travel_time': travel_time,
                        'goal': goal,
                        'start_pose': self.start_pose,
                        'final_amcl': final_amcl,
                        'planned_path_len': self.planned_path_len,
                        'actual_travel_dist': self.actual_travel_dist,
                        'sigma_x': self.sigma_x,
                        'sigma_y': self.sigma_y,
                        'sigma_yaw': self.sigma_yaw,
                    }
                    self.active_goal = None  # Reset active goal agar rute berikutnya (misal Home) tidak memakai nama lama
                    self.goal_reached_event.set()

            # Navigasi Dibatalkan atau Gagal (ABORTED / CANCELED)
            elif st in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                if self.is_navigating:
                    self.is_navigating = False
                    self.last_goal_id = None
                    self.active_goal = None
                    self.goal_aborted_event.set()

    def ensure_csv_header(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "No",
                    "Nama_Titik_Target",
                    "Goal_X_m",
                    "Goal_Y_m",
                    "Goal_Yaw_deg",
                    "AMCL_X_m",
                    "AMCL_Y_m",
                    "AMCL_Yaw_deg",
                    "Waktu_Tempuh_s",
                    "Panjang_Lintasan_m",
                    "Kecepatan_Rata2_mps",
                    "AMCL_Sigma_X_cm",
                    "AMCL_Sigma_Y_cm",
                    "AMCL_Sigma_Yaw_deg",
                    "Galat_Posisi_AMCL_cm",
                    "Galat_Orientasi_AMCL_deg",
                    "Galat_Posisi_Fisik_cm",
                    "Galat_Orientasi_Fisik_deg",
                    "Sumber_Ground_Truth",
                    "Status_Kedatangan",
                    "Status_Kelulusan",
                    "Waktu_Pengujian"
                ])

    def get_total_csv_rows(self) -> int:
        if not os.path.exists(self.csv_path):
            return 0
        try:
            with open(self.csv_path, 'r') as f:
                return sum(1 for _ in csv.DictReader(f))
        except Exception:
            return 0

    def process_arrival_result(self, arrival: dict):
        """Menghitung galat dan meminta verifikasi pengukuran fisik di lantai."""
        goal = arrival['goal']
        final_amcl = arrival['final_amcl']
        t_travel = arrival['travel_time']
        l_path = arrival['actual_travel_dist'] if arrival['actual_travel_dist'] > 0.1 else arrival['planned_path_len']
        v_avg = (l_path / t_travel) if t_travel > 0 else 0.0

        # Galat posisi menurut pembacaan AMCL vs Goal
        dx_amcl = final_amcl['x'] - goal['x']
        dy_amcl = final_amcl['y'] - goal['y']
        e_pos_amcl_m = math.hypot(dx_amcl, dy_amcl)
        e_pos_amcl_cm = e_pos_amcl_m * 100.0

        d_yaw_amcl_rad = abs(normalize_angle(final_amcl['yaw'] - goal['yaw']))
        e_yaw_amcl_deg = math.degrees(d_yaw_amcl_rad)

        sig_x_cm = arrival['sigma_x'] * 100.0
        sig_y_cm = arrival['sigma_y'] * 100.0
        sig_yaw_deg = math.degrees(arrival['sigma_yaw'])

        print("\n" + "=" * 78)
        print(f"🏁 ROBOT TIBA DI TITIK TUJUAN: '{goal['name']}' (GOAL REACHED)")
        print("=" * 78)
        print(f"⏱️ Waktu Tempuh Navigasi     : {t_travel:.2f} detik")
        print(f"🛣️ Jarak Tempuh / Lintasan  : {l_path:.2f} m  (Kecepatan Rata-rata: {v_avg:.2f} m/s)")
        print(f"📍 Koordinat Target Goal     : X = {goal['x']:.3f} m, Y = {goal['y']:.3f} m, Heading = {math.degrees(goal['yaw']):+.1f}°")
        print(f"🎯 Estimasi Posisi AMCL Akhir : X = {final_amcl['x']:.3f} m, Y = {final_amcl['y']:.3f} m, Heading = {math.degrees(final_amcl['yaw']):+.1f}°")
        print(f"📊 Kepadatan Partikel AMCL    : Sigma X: {sig_x_cm:.1f} cm | Sigma Y: {sig_y_cm:.1f} cm | Sigma Yaw: {sig_yaw_deg:.1f}°")
        print(f"📏 Selisih AMCL vs Goal       : Jarak = {e_pos_amcl_cm:.1f} cm | Sudut = {e_yaw_amcl_deg:.1f}°")

        # Deteksi lonjakan galat AMCL (Corridor Symmetry Slippage)
        is_amcl_slip = e_pos_amcl_cm > 50.0
        if is_amcl_slip:
            print("\n" + "⚠️ " * 26)
            print("⚠️ PERINGATAN ANOMALI AMCL (SLIP PARTIKEL DI LORONG SIMETRIS TERDETEKSI):")
            print(f"   • Selisih pembacaan sensor AMCL terhadap Goal mencapai {e_pos_amcl_cm:.1f} cm.")
            print("   • Hal ini wajar terjadi pada koridor lurus panjang karena LiDAR 2D kehilangan")
            print("     referensi longitudinal X (dinding kiri-kanan simetris dan seragam).")
            print("   • Nav2 telah berhasil membawa robot secara fisik ke titik target.")
            print("   👉 Segera ukur dengan meteran dan KETIK jarak galat fisik di bawah ini!")
            print("⚠️ " * 26)

        print("-" * 78)
        print("📐 VERIFIKASI PENGUKURAN FISIK GROUND TRUTH DI LANTAI:")
        print("   💡 Masukkan angka jika mengukur manual di lantai (ENTER = pakai estimasi AMCL).")
        timeout_pos = 20.0 if is_amcl_slip else 10.0
        print(f"   ⏳ (Waktu input: {timeout_pos:.0f} detik, otomatis lanjut jika robot jalan pulang)")
        user_pos = timed_input(
            f"   👉 Galat Jarak Fisik Robot di Lantai (cm) [Estimasi AMCL: {e_pos_amcl_cm:.1f} cm]: ",
            default_val=f"{e_pos_amcl_cm:.1f}",
            timeout_sec=timeout_pos,
            cancel_check_fn=lambda: self.is_navigating
        )

        try:
            final_pos_cm = float(user_pos)
            source_gt = "PENGUKURAN FISIK MANUAL (METERAN)" if abs(final_pos_cm - e_pos_amcl_cm) > 0.05 else "ESTIMASI AMCL SENSOR"
        except ValueError:
            final_pos_cm = e_pos_amcl_cm
            source_gt = "ESTIMASI AMCL SENSOR"

        if self.is_navigating:
            final_yaw_deg = e_yaw_amcl_deg
        else:
            timeout_yaw = 10.0 if is_amcl_slip else 5.0
            user_yaw = timed_input(
                f"   👉 Galat Sudut Hadap Fisik Robot (derajat) [Estimasi AMCL: {e_yaw_amcl_deg:.1f}°]: ",
                default_val=f"{e_yaw_amcl_deg:.1f}",
                timeout_sec=timeout_yaw,
                cancel_check_fn=lambda: self.is_navigating
            )
            try:
                final_yaw_deg = float(user_yaw)
            except ValueError:
                final_yaw_deg = e_yaw_amcl_deg

        # Evaluasi Kelulusan Berdasarkan nav2_params.yaml
        status_epos = "LULUS" if final_pos_cm <= self.pass_max_epos_cm else "GAGAL"
        status_eyaw = "LULUS" if final_yaw_deg <= self.pass_max_eyaw_deg else "GAGAL"
        status_total = "LULUS" if (status_epos == "LULUS" and status_eyaw == "LULUS") else "TIDAK LULUS"

        print("=" * 78)
        print(f"📊 HASIL EVALUASI RUN INI (Acuan nav2_params.yaml):")
        print(f"  • Galat Posisi Akhir   : {final_pos_cm:.1f} cm  (Toleransi: <= {self.pass_max_epos_cm:.1f} cm) -> [{status_epos}]")
        print(f"  • Galat Sudut Hadap    : {final_yaw_deg:.1f}°  (Toleransi: <= {self.pass_max_eyaw_deg:.1f}°) -> [{status_eyaw}]")
        print(f"  • Sumber Ground Truth  : {source_gt}")
        print(f"  • KESIMPULAN RUN INI   : [{status_total}]")
        print("=" * 78)

        # Simpan ke CSV
        row = [
            self.get_total_csv_rows() + 1,
            goal['name'],
            f"{goal['x']:.3f}",
            f"{goal['y']:.3f}",
            f"{math.degrees(goal['yaw']):.1f}",
            f"{final_amcl['x']:.3f}",
            f"{final_amcl['y']:.3f}",
            f"{math.degrees(final_amcl['yaw']):.1f}",
            f"{t_travel:.2f}",
            f"{l_path:.2f}",
            f"{v_avg:.2f}",
            f"{sig_x_cm:.1f}",
            f"{sig_y_cm:.1f}",
            f"{sig_yaw_deg:.1f}",
            f"{e_pos_amcl_cm:.1f}",
            f"{e_yaw_amcl_deg:.1f}",
            f"{final_pos_cm:.1f}",
            f"{final_yaw_deg:.1f}",
            source_gt,
            "SUCCEEDED",
            status_total,
            datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        ]
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        print(f"💾 Data berhasil disimpan ke: {self.csv_path}\n")

    def print_recap_table(self):
        """Menampilkan rekapitulasi pengujian dinamis untuk Bab 4 Skripsi."""
        if not os.path.exists(self.csv_path):
            print("\n⚠️ Belum ada data pengujian yang tersimpan.")
            return

        print("\n" + "=" * 90)
        print("📋 REKAPITULASI EVALUASI AMCL DINAMIS & ARRIVAL ACCURACY (BAB 4 SKRIPSI)")
        print("=" * 90)
        print(f"{'No':<3} | {'Titik Target':<20} | {'Waktu (s)':<10} | {'Jarak (m)':<10} | {'Galat Pos (cm)':<15} | {'Galat Yaw':<10} | {'Status':<10}")
        print("-" * 90)

        all_t = []
        all_epos = []
        all_eyaw = []
        success_count = 0
        total_runs = 0

        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                total_runs += 1
                t = float(row.get('Waktu_Tempuh_s', 0))
                ep = float(row.get('Galat_Posisi_Fisik_cm', 0))
                ey = float(row.get('Galat_Orientasi_Fisik_deg', 0))
                st = row.get('Status_Kelulusan', 'TIDAK LULUS')
                if st == 'LULUS':
                    success_count += 1
                all_t.append(t)
                all_epos.append(ep)
                all_eyaw.append(ey)
                print(f"{row['No']:<3} | {row['Nama_Titik_Target'][:20]:<20} | {t:<10.2f} | {float(row.get('Panjang_Lintasan_m', 0)):<10.2f} | {ep:<15.1f} | {ey:<10.1f}° | {st:<10}")

        print("-" * 90)
        if total_runs > 0:
            avg_t = sum(all_t) / total_runs
            avg_ep = sum(all_epos) / total_runs
            avg_ey = sum(all_eyaw) / total_runs
            sr = (success_count / total_runs) * 100.0
            print(f"🏆 RATA-RATA TOTAL ({total_runs} Pengujian):")
            print(f"  • Tingkat Keberhasilan (Success Rate) : {sr:.1f}% ({success_count}/{total_runs} Sukses)")
            print(f"  • Rata-rata Waktu Tempuh Navigasi     : {avg_t:.2f} detik")
            print(f"  • Rata-rata Galat Posisi Kedatangan   : {avg_ep:.2f} cm   (Batas Toleransi: <= {self.pass_max_epos_cm:.1f} cm)")
            print(f"  • Rata-rata Galat Sudut Hadap         : {avg_ey:.2f}°     (Batas Toleransi: <= {self.pass_max_eyaw_deg:.1f}°)")
            status_akhir = "MEMENUHI SYARAT KELULUSAN SKRIPSI" if (avg_ep <= self.pass_max_epos_cm and sr >= 80.0) else "PERLU PENGUJIAN TAMBAHAN"
            print(f"  • KESIMPULAN AKHIR                     : [{status_akhir}]")
        print("=" * 90)

    def send_direct_nav_goal(self, wp_idx: int):
        """Kirim goal navigasi langsung dari terminal."""
        if not (0 <= wp_idx < len(self.waypoints)):
            print("⚠️ Nomor titik tidak valid.")
            return

        wp = self.waypoints[wp_idx]
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            print("⚠️ Server Nav2 'navigate_to_pose' tidak aktif! Pastikan navigasi sudah dinyalakan di dashboard.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, wp['yaw'])

        with self.lock:
            self.active_goal = {
                'name': wp['name'],
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp['yaw']),
                'source': 'terminal'
            }

        print(f"\n🚀 Mengirim robot ke '{wp['name']}' (X={wp['x']}, Y={wp['y']})...")
        self.nav_client.send_goal_async(goal_msg)

    def edit_last_csv_row(self):
        """Mengoreksi nilai baris data terakhir di file CSV (misal setelah terlewat timeout ground truth)."""
        if not os.path.exists(self.csv_path):
            print("⚠️ File CSV belum ada.")
            return

        with open(self.csv_path, 'r') as f:
            rows = list(csv.reader(f))

        if len(rows) <= 1:
            print("⚠️ Belum ada data untuk dikoreksi.")
            return

        header = rows[0]
        last_row = rows[-1]
        print("\n" + "=" * 75)
        print("📝 KOREKSI DATA PENGUJIAN TERAKHIR:")
        print(f"  • Nomor Run       : {last_row[0]}")
        print(f"  • Titik Target    : {last_row[1]}")
        print(f"  • Estimasi AMCL   : Posisi = {last_row[14]} cm, Yaw = {last_row[15]}°")
        print(f"  • Rekaman Saat Ini: Posisi = {last_row[16]} cm, Yaw = {last_row[17]}° ({last_row[18]})")
        print(f"  • Status Kelulusan: [{last_row[20]}]")
        print("=" * 75)

        new_pos = input(f"👉 Masukkan Galat Posisi Fisik Baru (cm) [Tekan ENTER jika tetap {last_row[16]} cm]: ").strip()
        new_yaw = input(f"👉 Masukkan Galat Sudut Hadap Baru (derajat) [Tekan ENTER jika tetap {last_row[17]}°]: ").strip()

        if new_pos:
            try:
                val_p = float(new_pos)
                last_row[16] = f"{val_p:.1f}"
                last_row[18] = "PENGUKURAN FISIK MANUAL (METERAN)"
                st_p = "LULUS" if val_p <= self.pass_max_epos_cm else "GAGAL"
                val_y = float(last_row[17])
                st_y = "LULUS" if val_y <= self.pass_max_eyaw_deg else "GAGAL"
                last_row[20] = "LULUS" if (st_p == "LULUS" and st_y == "LULUS") else "TIDAK LULUS"
            except ValueError:
                print("⚠️ Angka posisi tidak valid, perubahan dibatalkan.")
                return

        if new_yaw:
            try:
                val_y = float(new_yaw)
                last_row[17] = f"{val_y:.1f}"
                val_p = float(last_row[16])
                st_p = "LULUS" if val_p <= self.pass_max_epos_cm else "GAGAL"
                st_y = "LULUS" if val_y <= self.pass_max_eyaw_deg else "GAGAL"
                last_row[20] = "LULUS" if (st_p == "LULUS" and st_y == "LULUS") else "TIDAK LULUS"
            except ValueError:
                print("⚠️ Angka sudut tidak valid, perubahan dibatalkan.")
                return

        rows[-1] = last_row
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(rows)

        print("\n✅ Data pengujian terakhir berhasil diperbarui!")
        print(f"   • Posisi Fisik Baru : {last_row[16]} cm (Toleransi <= {self.pass_max_epos_cm:.1f} cm)")
        print(f"   • Sudut Hadap Baru  : {last_row[17]}° (Toleransi <= {self.pass_max_eyaw_deg:.1f}°)")
        print(f"   • Status Kelulusan  : [{last_row[20]}]")
        print(f"   • Sumber Data       : {last_row[18]}")

    def delete_last_csv_row(self):
        """Menghapus 1 baris pengujian terakhir di CSV jika salah uji."""
        if not os.path.exists(self.csv_path):
            print("⚠️ File CSV belum ada.")
            return

        with open(self.csv_path, 'r') as f:
            rows = list(csv.reader(f))

        if len(rows) <= 1:
            print("⚠️ Belum ada data untuk dihapus.")
            return

        last_row = rows[-1]
        c = input(f"⚠️ Yakin hapus baris terakhir #{last_row[0]} ('{last_row[1]}')? (y/n): ").strip().lower()
        if c == 'y':
            rows.pop()
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(rows)
            print("✅ Baris terakhir berhasil dihapus dari CSV!")


def main(args=None):
    rclpy.init(args=args)
    node = EvaluasiAMCLDinamisNode()

    # Background spin thread untuk callback DDS ROS 2
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        while True:
            print("\n" + "=" * 75)
            print("🤖 BENCHMARK SKRIPSI AMR-01: SKENARIO 2 (AMCL DINAMIS & ARRIVAL ACCURACY)")
            print("=" * 75)
            print(f"  Status Monitor      : 🟢 AKTIF (Otomatis mendeteksi saat navigasi jalan)")
            print(f"  Toleransi nav2_params: Posisi <= {node.pass_max_epos_cm:.1f} cm | Sudut <= {node.pass_max_eyaw_deg:.1f}°")
            print(f"  File Log CSV        : {node.csv_path}")
            print(f"  Total Data Rekaman  : {node.get_total_csv_rows()} pengujian")
            print("-" * 75)
            print("  [1] MODE STANDBY / MONITOR PASIF (Jalankan navigasi dari Web Dashboard)")
            print("  [2] Kirim Navigasi Langsung dari Daftar Titik Terminal")
            print("  [3] Tampilkan Rekapitulasi Tabel & Statistik Bab 4")
            print("  [4] Koreksi / Edit Baris Data Terakhir CSV (Ground Truth)")
            print("  [5] Hapus 1 Baris Data Terakhir CSV")
            print("  [6] Reset / Hapus Seluruh File CSV")
            print("  [0] Keluar")
            print("=" * 75)

            choice = input("Pilih menu (1/2/3/4/5/6/0) [Default: 1]: ").strip() or '1'

            if choice == '1':
                print("\n" + "-" * 75)
                print("👀 MONITOR AKTIF: Silakan buka Web Dashboard dan klik titik tujuan...")
                print("👉 (Tekan Ctrl+C di terminal ini kapan saja jika ingin kembali ke menu)")
                print("-" * 75)
                try:
                    while True:
                        time.sleep(0.05)

                        if node.nav_started_event.is_set():
                            node.nav_started_event.clear()
                            with node.lock:
                                goal = dict(node.active_goal) if node.active_goal else {'name': 'Goal Navigasi Aktif', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                                sp = dict(node.start_pose) if node.start_pose else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                            print("\n" + "=" * 78)
                            print(f"🚀 NAVIGASI AKTIF TERDETEKSI: Menuju '{goal.get('name', 'Target')}'")
                            print(f"📍 Target Koordinat : X = {goal.get('x', 0.0):.3f} m, Y = {goal.get('y', 0.0):.3f} m, Yaw = {math.degrees(goal.get('yaw', 0.0)):+.1f}°")
                            print(f"🛫 Pose Awal Robot  : X = {sp.get('x', 0.0):.3f} m, Y = {sp.get('y', 0.0):.3f} m, Yaw = {math.degrees(sp.get('yaw', 0.0)):+.1f}°")
                            print("=" * 78)

                        if node.is_navigating:
                            now = time.time()
                            if now - node.last_terminal_log_time >= 0.5:
                                node.last_terminal_log_time = now
                                el = now - (node.start_time or now)
                                live = node.get_live_pose()
                                cur_x = live['x']
                                cur_y = live['y']
                                sig_xy = max(node.sigma_x, node.sigma_y) * 100.0
                                sys.stdout.write(
                                    f"\r  ⏱️ [{el:4.1f}s] Speed: {node.current_speed:4.2f}m/s | "
                                    f"Pose: ({cur_x:5.2f}, {cur_y:5.2f}) | "
                                    f"Sigma: {sig_xy:4.1f}cm "
                                )
                                sys.stdout.flush()

                        if node.goal_reached_event.is_set():
                            node.goal_reached_event.clear()
                            if node.arrival_data:
                                node.process_arrival_result(node.arrival_data)
                                node.arrival_data = None
                            if not node.is_navigating:
                                print("\n👀 Kembali standby memantau navigasi berikutnya...")

                        if node.goal_aborted_event.is_set():
                            node.goal_aborted_event.clear()
                            print("\n⚠️ Navigasi dibatalkan atau terhalang rintangan.")
                            if not node.is_navigating:
                                print("👀 Kembali standby memantau navigasi berikutnya...")

                except KeyboardInterrupt:
                    print("\n🔙 Kembali ke menu utama.")

            elif choice == '2':
                print("\nDaftar Titik Waypoint:")
                for i, wp in enumerate(node.waypoints):
                    print(f"  [{i}] {wp['name']:<24} (X={wp['x']:.2f}, Y={wp['y']:.2f}, Yaw={math.degrees(wp['yaw']):.0f}°)")
                idx_in = input("Pilih nomor waypoint yang ingin dituju (atau 'b' untuk batal): ").strip()
                if idx_in.isdigit():
                    node.send_direct_nav_goal(int(idx_in))
                    # Otomatis langsung masuk mode monitor
                    print("\n👀 Mengawal perjalanan robot...")
                    try:
                        while not node.goal_reached_event.is_set() and not node.goal_aborted_event.is_set():
                            time.sleep(0.05)

                            if node.nav_started_event.is_set():
                                node.nav_started_event.clear()
                                with node.lock:
                                    goal = dict(node.active_goal) if node.active_goal else {'name': 'Goal Navigasi Aktif', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                                    sp = dict(node.start_pose) if node.start_pose else {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                                print("\n" + "=" * 78)
                                print(f"🚀 NAVIGASI AKTIF TERDETEKSI: Menuju '{goal.get('name', 'Target')}'")
                                print(f"📍 Target Koordinat : X = {goal.get('x', 0.0):.3f} m, Y = {goal.get('y', 0.0):.3f} m, Yaw = {math.degrees(goal.get('yaw', 0.0)):+.1f}°")
                                print(f"🛫 Pose Awal Robot  : X = {sp.get('x', 0.0):.3f} m, Y = {sp.get('y', 0.0):.3f} m, Yaw = {math.degrees(sp.get('yaw', 0.0)):+.1f}°")
                                print("=" * 78)

                            if node.is_navigating:
                                now = time.time()
                                if now - node.last_terminal_log_time >= 0.5:
                                    node.last_terminal_log_time = now
                                    el = now - (node.start_time or now)
                                    live = node.get_live_pose()
                                    cur_x = live['x']
                                    cur_y = live['y']
                                    sig_xy = max(node.sigma_x, node.sigma_y) * 100.0
                                    sys.stdout.write(
                                        f"\r  ⏱️ [{el:4.1f}s] Speed: {node.current_speed:4.2f}m/s | "
                                        f"Pose: ({cur_x:5.2f}, {cur_y:5.2f}) | "
                                        f"Sigma: {sig_xy:4.1f}cm "
                                    )
                                    sys.stdout.flush()

                        if node.goal_reached_event.is_set():
                            node.goal_reached_event.clear()
                            if node.arrival_data:
                                node.process_arrival_result(node.arrival_data)
                                node.arrival_data = None

                        if node.goal_aborted_event.is_set():
                            node.goal_aborted_event.clear()
                            print("\n⚠️ Navigasi dibatalkan atau terhalang.")
                    except KeyboardInterrupt:
                        print("\n🔙 Pengawalan dihentikan.")

            elif choice == '3':
                node.print_recap_table()

            elif choice == '4':
                node.edit_last_csv_row()

            elif choice == '5':
                node.delete_last_csv_row()

            elif choice == '6':
                c = input("⚠️ Yakin ingin menghapus seluruh file CSV skenario 2 dinamis? (ketik 'ya'): ").strip()
                if c.lower() == 'ya':
                    if os.path.exists(node.csv_path):
                        os.remove(node.csv_path)
                    node.ensure_csv_header()
                    print("✅ File CSV berhasil di-reset!")

            elif choice == '0':
                print("👋 Keluar dari sistem evaluasi.")
                break

    except KeyboardInterrupt:
        print("\n🛑 Program dihentikan pengguna.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
