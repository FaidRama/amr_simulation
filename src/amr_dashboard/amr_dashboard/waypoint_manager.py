#!/usr/bin/env python3
"""
=================================================================
WAYPOINT MANAGER NODE
Node ROS2 untuk mengelola misi multi-waypoint delivery.
Menerima daftar task (pickup → delivery) dan mengeksekusi
secara sequential menggunakan Nav2 NavigateToPose action.

PENTING: Tidak menggunakan rclpy.spin_until_future_complete()
di thread terpisah — karena di ROS 2 Foxy, spin_until_future_complete
akan deadlock/crash jika main thread juga menjalankan rclpy.spin().
Sebagai gantinya, menggunakan callback + threading.Event.

Fitur:
1. Thread-safe action client dengan Event synchronization
2. Support return to home (titik awal) saat cancel maupun auto return
3. Konfirmasi operator di titik pickup ("Barang Sudah Dimuat")
   dan delivery ("Barang Sudah Diambil")
4. Status publisher berkala untuk monitoring di dashboard
5. Mitigasi Pintu Tertutup (Solusi A - Zona Koordinat Pintu):
   Jika robot terhalang di area pintu terdaftar, robot TIDAK abort,
   melainkan menunggu dengan status 'waiting_door' dan auto-retry saat pintu dibuka.
6. Penanganan Rintangan Koridor ('path_blocked') dengan retry berkala.
=================================================================
"""
import os
import json
import uuid
import time
import math
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

try:
    from tf2_ros import Buffer, TransformListener
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # === ACTION CLIENT untuk Nav2 ===
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # === PUBLISHERS ===
        # Status misi untuk dashboard
        self.status_pub = self.create_publisher(String, '/amr/delivery_status', 10)
        # Stop velocity (emergency stop)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # === SUBSCRIBERS ===
        # Menerima task baru dari dashboard
        self.task_sub = self.create_subscription(
            String, '/amr/delivery_tasks', self.on_task_received, 10)
        # Menerima perintah kontrol (cancel, pause, resume)
        self.control_sub = self.create_subscription(
            String, '/amr/delivery_control', self.on_control_received, 10)
            
        # Posisi robot (AMCL & Odom fallback)
        self.current_robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom_pose, 10)
            
        # TF2 Buffer & Listener untuk real-time map -> base_footprint / base_link
        self.tf_buffer = None
        self.tf_listener = None
        if HAS_TF2:
            try:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            except Exception as e:
                self.get_logger().warn(f'TF2 listener init error: {e}')
        
        # === CONFIGURATION (waypoints.yaml) ===
        self.home_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.auto_return_home = True
        self.doors = []
        self.door_wait_interval = 8.0
        self.door_max_retries = 15
        self.obstacle_max_retries = 2
        self.load_config()
        
        # === STATE MANAGEMENT ===
        self.mission = None          # Misi aktif saat ini
        self.is_paused = False       # Flag pause
        self.is_cancelled = False    # Flag cancel misi
        self.is_returning_home = False # Flag saat kembali ke titik awal
        self.stop_navigation = False # Flag untuk stop goal navigasi yang sedang berjalan
        self.confirm_received = False  # Flag konfirmasi pickup/delivery
        self.current_goal_handle = None
        
        # === NAVIGATION CALLBACK STATE ===
        # Event untuk sinkronisasi antara callback dan mission thread
        self.nav_event = threading.Event()
        self.nav_result_status = None  # Hasil navigasi terakhir
        self.goal_accepted_event = threading.Event()
        self.goal_accepted = False
        
        # Timer untuk publish status berkala
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('='*55)
        self.get_logger().info(' WAYPOINT MANAGER NODE - AKTIF')
        self.get_logger().info(f' Home Position: ({self.home_position.get("x", 0.0):.2f}, {self.home_position.get("y", 0.0):.2f})')
        self.get_logger().info(f' Auto Return Home: {self.auto_return_home}')
        self.get_logger().info(f' Pintu Terdaftar: {len(self.doors)} zona pintu')
        self.get_logger().info(' Menunggu perintah dari dashboard...')
        self.get_logger().info('='*55)
    
    def _on_amcl_pose(self, msg):
        """Simpan estimasi posisi robot dari AMCL."""
        self.current_robot_pose['x'] = float(msg.pose.pose.position.x)
        self.current_robot_pose['y'] = float(msg.pose.pose.position.y)

    def _on_odom_pose(self, msg):
        """Simpan estimasi posisi robot dari Odom sebagai fallback."""
        if not HAS_TF2 and self.current_robot_pose['x'] == 0.0 and self.current_robot_pose['y'] == 0.0:
            self.current_robot_pose['x'] = float(msg.pose.pose.position.x)
            self.current_robot_pose['y'] = float(msg.pose.pose.position.y)

    def get_current_pose(self):
        """Mendapatkan koordinat (x, y) robot saat ini di frame 'map'."""
        if HAS_TF2 and self.tf_buffer:
            for frame in ('base_footprint', 'base_link'):
                try:
                    t = self.tf_buffer.lookup_transform(
                        'map', frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05)
                    )
                    return float(t.transform.translation.x), float(t.transform.translation.y)
                except Exception:
                    pass
        return float(self.current_robot_pose['x']), float(self.current_robot_pose['y'])

    def check_near_door(self):
        """
        Memeriksa apakah posisi robot saat ini berada dalam radius area pintu.
        Mengembalikan (is_near, door_dict, distance).
        """
        if not self.doors:
            return False, None, 0.0
        
        rx, ry = self.get_current_pose()
        for door in self.doors:
            dx = float(door.get('x', 0.0))
            dy = float(door.get('y', 0.0))
            radius = float(door.get('radius', 1.5))
            dist = math.hypot(rx - dx, ry - dy)
            if dist <= radius:
                return True, door, dist
        return False, None, 0.0

    def load_config(self):
        """Load home_position, doors, dan settings dari waypoints.yaml sesuai mode aktif."""
        config_paths = [
            os.path.join(os.path.expanduser('~'), 'deliver_amr', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.expanduser('~'), 'Documents', 'amr_ws', 'src', 'amr_dashboard', 'config', 'waypoints.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            share_cfg = os.path.join(get_package_share_directory('amr_dashboard'), 'config', 'waypoints.yaml')
            config_paths.append(share_cfg)
        except Exception:
            pass

        for path in config_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        cfg = yaml.safe_load(f) or {}
                    mode = cfg.get('operation_mode', 'sim')
                    profile = cfg.get('profiles', {}).get(mode, {})
                    
                    if 'home_position' in profile:
                        self.home_position = profile['home_position']
                    elif 'home_position' in cfg:
                        self.home_position = cfg['home_position']
                    
                    # Muat daftar pintu (doors)
                    if 'doors' in profile:
                        self.doors = profile['doors']
                    elif 'doors' in cfg:
                        self.doors = cfg['doors']
                    else:
                        self.doors = []
                    
                    # Muat settings
                    settings = profile.get('settings', cfg.get('settings', {}))
                    if 'auto_return_home' in settings:
                        self.auto_return_home = settings['auto_return_home']
                    if 'door_wait_interval' in settings:
                        self.door_wait_interval = float(settings['door_wait_interval'])
                    if 'door_max_retries' in settings:
                        self.door_max_retries = int(settings['door_max_retries'])
                    if 'obstacle_max_retries' in settings:
                        self.obstacle_max_retries = int(settings['obstacle_max_retries'])
                        
                    self.get_logger().info(
                        f'Loaded config [{mode.upper()}] dari {path} | '
                        f'Home: ({self.home_position.get("x", 0.0):.2f}, {self.home_position.get("y", 0.0):.2f}) | '
                        f'Doors: {len(self.doors)}')
                    return
                except Exception as e:
                    self.get_logger().warn(f'Gagal membaca config {path}: {e}')
    
    def on_task_received(self, msg):
        """Menerima daftar task delivery dari dashboard (JSON string)."""
        try:
            data = json.loads(msg.data)
            tasks = data.get('tasks', [])
            
            if not tasks:
                self.get_logger().warn('Daftar task kosong, diabaikan.')
                return
            
            if self.mission and self.mission['status'] == 'running':
                self.get_logger().warn('Misi sedang berjalan! Cancel dulu sebelum kirim misi baru.')
                return
            
            # Buat misi baru
            self.mission = {
                'mission_id': str(uuid.uuid4())[:8],
                'tasks': tasks,
                'status': 'running',
                'current_task_index': 0,
                'current_phase': 'idle',
                'start_time': time.time(),
                'completed_tasks': 0,
                'total_tasks': len(tasks),
                'waiting_info': None
            }
            self.is_paused = False
            self.is_cancelled = False
            self.is_returning_home = False
            self.stop_navigation = False
            self.confirm_received = False
            
            self.get_logger().info(f'MISI BARU DITERIMA: {len(tasks)} task')
            for i, t in enumerate(tasks):
                self.get_logger().info(
                    f'  Task {i+1}: Pickup [{t["pickup"]["name"]}] → '
                    f'Delivery [{t["delivery"]["name"]}]')
            
            # Mulai eksekusi di thread terpisah agar tidak blocking
            mission_thread = threading.Thread(target=self.execute_mission, daemon=True)
            mission_thread.start()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
    
    def on_control_received(self, msg):
        """Menerima perintah kontrol: cancel, pause, resume, skip, confirm_pickup, confirm_delivery."""
        command = msg.data.strip().lower()
        
        if command == 'cancel':
            self.get_logger().info('PERINTAH CANCEL DITERIMA')
            # Jika user menekan cancel saat sedang returning_home atau misi sudah dibatalkan:
            # Lakukan hard emergency stop di tempat
            if self.is_returning_home or (self.is_cancelled and not self.mission):
                self.get_logger().info('Emergency stop: Pembatalan kedua, berhenti total di tempat!')
                self.stop_navigation = True
                self.nav_event.set()
                self.goal_accepted_event.set()
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                self.emergency_stop()
                if self.mission:
                    self.mission['status'] = 'cancelled'
                    self.mission['current_phase'] = 'idle'
                    self.mission['waiting_info'] = None
                return
            
            # Pembatalan pertama: batalkan task antrian & goal aktif saat ini,
            # lalu arahkan kembali ke titik awal (home)
            self.is_cancelled = True
            self.stop_navigation = True
            self.is_paused = False
            self.confirm_received = True  # Lepaskan wait loop jika sedang menunggu
            self.nav_event.set()
            self.goal_accepted_event.set()
            if self.current_goal_handle:
                self.get_logger().info('Membatalkan goal navigasi aktif...')
                self.current_goal_handle.cancel_goal_async()
            self.emergency_stop()
            if self.mission:
                self.mission['current_phase'] = 'returning_home'
                self.mission['waiting_info'] = None
            
        elif command == 'pause':
            self.get_logger().info('PERINTAH PAUSE DITERIMA')
            self.is_paused = True
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.emergency_stop()
            
        elif command == 'resume':
            self.get_logger().info('PERINTAH RESUME DITERIMA')
            self.is_paused = False
            
        elif command == 'skip':
            self.get_logger().info('PERINTAH SKIP TASK DITERIMA')
            self.confirm_received = True  # Lepaskan wait loop
            self.stop_navigation = True
            self.nav_event.set()
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
        
        elif command == 'confirm_pickup':
            self.get_logger().info('✓ KONFIRMASI PICKUP: Barang sudah dimuat!')
            self.confirm_received = True
        
        elif command == 'confirm_delivery':
            self.get_logger().info('✓ KONFIRMASI DELIVERY: Barang sudah diambil!')
            self.confirm_received = True
        
        elif command == 'reload_config':
            self.get_logger().info('↻ RELOAD CONFIG: Memperbarui konfigurasi mode, pintu & home position...')
            self.load_config()
    
    def emergency_stop(self):
        """Kirim cmd_vel zero untuk stop AMR."""
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
    
    def navigate_with_door_handling(self, target_x, target_y, target_yaw=0.0, target_phase='navigating', target_name='Target'):
        """
        Navigasi ke target dengan penanganan cerdas untuk pintu tertutup dan rintangan jalur.
        - Jika Nav2 gagal dan posisi robot berada dalam radius pintu terdaftar:
            Status beralih ke 'waiting_door', robot berhenti dan menunggu pintu dibuka (auto-retry).
        - Jika Nav2 gagal di tempat lain:
            Status beralih ke 'path_blocked', robot mencoba ulang beberapa kali (menghindari rintangan dinamis).
        """
        self.mission['current_phase'] = target_phase
        self.mission['waiting_info'] = None
        self.publish_status()
        
        # Percobaan awal navigasi normal
        success = self.navigate_to_point(target_x, target_y, target_yaw)
        if success or self.is_cancelled or self.stop_navigation:
            return success
        
        # Jika navigasi gagal, periksa apakah posisi robot berada di area pintu
        is_door, door, dist = self.check_near_door()
        if is_door:
            door_name = door.get('name', 'Pintu')
            self.get_logger().warn(
                f'🚪 [PINTU TERTUTUP] AMR terhalang di dekat [{door_name}] (jarak: {dist:.2f} m). '
                f'Robot TIDAK abort! Menunggu pintu dibuka...')
            
            for attempt in range(1, self.door_max_retries + 1):
                if self.is_cancelled or self.stop_navigation:
                    return False
                
                # Pasang status waiting_door untuk dashboard
                self.mission['current_phase'] = 'waiting_door'
                self.mission['waiting_info'] = {
                    'type': 'door',
                    'door_name': door_name,
                    'attempt': attempt,
                    'max_retries': self.door_max_retries,
                    'dist': round(dist, 2),
                    'message': f'Menunggu {door_name} dibuka ({attempt}/{self.door_max_retries})'
                }
                self.publish_status()
                
                # Tunggu jeda (default 8 detik) sambil memantau perintah cancel / skip
                wait_ticks = int(self.door_wait_interval * 2)
                for _ in range(wait_ticks):
                    if self.is_cancelled or self.stop_navigation:
                        return False
                    time.sleep(0.5)
                
                if self.is_cancelled or self.stop_navigation:
                    return False
                
                self.get_logger().info(
                    f'🔄 [RETRY {attempt}/{self.door_max_retries}] Mencoba navigasi ulang melalui [{door_name}]...')
                self.mission['current_phase'] = target_phase
                self.publish_status()
                
                # Coba kirim ulang goal navigasi
                retry_success = self.navigate_to_point(target_x, target_y, target_yaw)
                if retry_success:
                    self.get_logger().info(
                        f'✅ Pintu [{door_name}] telah dibuka! Robot melanjutkan perjalanan ke [{target_name}].')
                    self.mission['waiting_info'] = None
                    return True
                
                # Cek apakah robot masih berada di dekat pintu
                is_still_door, _, dist = self.check_near_door()
                if not is_still_door:
                    self.get_logger().warn('Posisi robot sudah bergeser dari area pintu.')
                    break
            
            self.get_logger().error(
                f'❌ Timeout: Pintu [{door_name}] tidak dibuka setelah {self.door_max_retries} percobaan.')
            self.mission['waiting_info'] = None
            return False
        
        else:
            # Gagal di koridor / open area (rintangan umum)
            self.get_logger().warn(
                f'⚠️ [JALUR TERHALANG] Terhalang obstacle di koridor menuju [{target_name}].')
            
            for attempt in range(1, self.obstacle_max_retries + 1):
                if self.is_cancelled or self.stop_navigation:
                    return False
                
                self.mission['current_phase'] = 'path_blocked'
                self.mission['waiting_info'] = {
                    'type': 'obstacle',
                    'target_name': target_name,
                    'attempt': attempt,
                    'max_retries': self.obstacle_max_retries,
                    'message': f'Jalur terhalang rintangan ({attempt}/{self.obstacle_max_retries})'
                }
                self.publish_status()
                
                # Tunggu jeda singkat (4 detik) agar orang / obstacle dinamis berlalu
                for _ in range(8):
                    if self.is_cancelled or self.stop_navigation:
                        return False
                    time.sleep(0.5)
                
                self.mission['current_phase'] = target_phase
                self.publish_status()
                self.get_logger().info(
                    f'🔄 [OBSTACLE RETRY {attempt}/{self.obstacle_max_retries}] Mencoba replan jalur...')
                
                retry_success = self.navigate_to_point(target_x, target_y, target_yaw)
                if retry_success:
                    self.get_logger().info(f'✅ Jalur menuju [{target_name}] kembali bersih!')
                    self.mission['waiting_info'] = None
                    return True
            
            self.get_logger().error(f'❌ Gagal mencapai [{target_name}]: Jalur tetap terhalang rintangan.')
            self.mission['waiting_info'] = None
            return False

    def execute_mission(self):
        """Eksekusi semua task dalam misi secara sequential."""
        if not self.mission:
            return
        
        tasks = self.mission['tasks']
        
        for i, task in enumerate(tasks):
            if self.is_cancelled:
                self.get_logger().info('Misi DIBATALKAN oleh user. Menghentikan antrian task.')
                break
            
            # Tunggu jika di-pause
            while self.is_paused and not self.is_cancelled:
                self.mission['status'] = 'paused'
                time.sleep(0.5)
            
            if self.is_cancelled:
                break
            
            self.mission['status'] = 'running'
            self.mission['current_task_index'] = i
            
            pickup = task['pickup']
            delivery = task['delivery']
            
            # === FASE 1: Navigasi ke titik PICKUP ===
            self.get_logger().info(
                f'--- Task {i+1}/{len(tasks)}: Menuju pickup [{pickup["name"]}] ---')
            
            success = self.navigate_with_door_handling(
                pickup['x'], pickup['y'], pickup.get('yaw', 0.0),
                target_phase='navigating_pickup', target_name=pickup['name'])
            
            if self.is_cancelled:
                break
            
            if not success:
                self.get_logger().warn(
                    f'Gagal mencapai pickup [{pickup["name"]}], skip ke task berikutnya.')
                task['status'] = 'failed'
                self.stop_navigation = False
                continue
            
            # Sampai di pickup — tunggu konfirmasi dari operator
            self.mission['current_phase'] = 'waiting_pickup'
            self.confirm_received = False
            self.get_logger().info(
                f'SAMPAI di pickup [{pickup["name"]}]. '
                f'Menunggu konfirmasi dari operator (tombol di dashboard)...')
            
            # Tunggu operator tekan tombol "Barang Sudah Dimuat"
            while not self.confirm_received and not self.is_cancelled:
                time.sleep(0.3)
            
            if self.is_cancelled:
                break
            
            self.get_logger().info(f'Konfirmasi diterima! Lanjut mengantar ke [{delivery["name"]}]')
            
            # Tunggu jika di-pause
            while self.is_paused and not self.is_cancelled:
                time.sleep(0.5)
            
            if self.is_cancelled:
                break
            
            # === FASE 2: Navigasi ke titik DELIVERY ===
            self.get_logger().info(
                f'--- Task {i+1}/{len(tasks)}: Menuju delivery [{delivery["name"]}] ---')
            
            success = self.navigate_with_door_handling(
                delivery['x'], delivery['y'], delivery.get('yaw', 0.0),
                target_phase='navigating_delivery', target_name=delivery['name'])
            
            if self.is_cancelled:
                break
            
            if not success:
                self.get_logger().warn(
                    f'Gagal mencapai delivery [{delivery["name"]}].')
                task['status'] = 'failed'
                self.stop_navigation = False
                continue
            
            # Sampai di delivery — tunggu konfirmasi dari operator
            self.mission['current_phase'] = 'waiting_delivery'
            self.confirm_received = False
            self.get_logger().info(
                f'SAMPAI di delivery [{delivery["name"]}]. '
                f'Menunggu konfirmasi dari operator (tombol di dashboard)...')
            
            # Tunggu operator tekan tombol "Barang Sudah Diambil"
            while not self.confirm_received and not self.is_cancelled:
                time.sleep(0.3)
            
            if self.is_cancelled:
                break
            
            self.get_logger().info('Konfirmasi delivery diterima!')
            task['status'] = 'completed'
            self.mission['completed_tasks'] = i + 1
            self.get_logger().info(
                f'✓ Task {i+1}/{len(tasks)} SELESAI: '
                f'{pickup["name"]} → {delivery["name"]}')
        
        # === PROSES SETELAH TASK SELESAI ATAU DIBATALKAN ===
        if self.is_cancelled:
            self.get_logger().info('='*50)
            self.get_logger().info(' MISI DIBATALKAN: Mengarahkan AMR kembali ke titik awal (home)...')
            self.get_logger().info('='*50)
            if self.mission:
                self.mission['status'] = 'running'
                self.mission['current_phase'] = 'returning_home'
                self.mission['waiting_info'] = None
            
            self.is_returning_home = True
            self.stop_navigation = False
            time.sleep(0.5)  # Beri waktu pembersihan goal sebelumnya
            
            home_x = self.home_position.get('x', 0.0)
            home_y = self.home_position.get('y', 0.0)
            home_yaw = self.home_position.get('yaw', 0.0)
            
            self.navigate_with_door_handling(
                home_x, home_y, home_yaw,
                target_phase='returning_home', target_name='Home')
            self.is_returning_home = False
            
            if self.mission:
                self.mission['status'] = 'cancelled'
                self.mission['current_phase'] = 'idle'
                self.mission['waiting_info'] = None
            self.get_logger().info('AMR telah kembali ke titik awal setelah pembatalan misi.')
            
        elif self.auto_return_home:
            self.get_logger().info('='*50)
            self.get_logger().info(' SEMUA TASK SELESAI: AMR kembali ke titik awal (home)...')
            self.get_logger().info('='*50)
            if self.mission:
                self.mission['status'] = 'running'
                self.mission['current_phase'] = 'returning_home'
                self.mission['waiting_info'] = None
            
            self.is_returning_home = True
            self.stop_navigation = False
            time.sleep(0.5)
            
            home_x = self.home_position.get('x', 0.0)
            home_y = self.home_position.get('y', 0.0)
            home_yaw = self.home_position.get('yaw', 0.0)
            
            self.navigate_with_door_handling(
                home_x, home_y, home_yaw,
                target_phase='returning_home', target_name='Home')
            self.is_returning_home = False
            
            if self.mission:
                self.mission['status'] = 'completed'
                self.mission['current_phase'] = 'idle'
                self.mission['waiting_info'] = None
            self.get_logger().info('='*50)
            self.get_logger().info(' MISI SELESAI — AMR telah kembali ke Home!')
            self.get_logger().info('='*50)
            
        else:
            if self.mission:
                self.mission['status'] = 'completed'
                self.mission['current_phase'] = 'idle'
                self.mission['waiting_info'] = None
            self.get_logger().info('='*50)
            self.get_logger().info(' MISI SELESAI — Semua task telah diproses!')
            self.get_logger().info('='*50)
    
    # === CALLBACK-BASED NAVIGATION (Thread-safe untuk ROS 2 Foxy) ===
    
    def _goal_response_callback(self, future):
        """Callback saat goal response diterima dari Nav2."""
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn('Goal DITOLAK oleh Nav2!')
                self.goal_accepted = False
                self.goal_accepted_event.set()
                return
            
            self.get_logger().info('Goal DITERIMA, AMR sedang bergerak...')
            self.current_goal_handle = goal_handle
            self.goal_accepted = True
            self.goal_accepted_event.set()
            
            # Request result — ini juga callback-based
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error di _goal_response_callback: {e}')
            self.goal_accepted = False
            self.goal_accepted_event.set()
    
    def _goal_result_callback(self, future):
        """Callback saat navigasi selesai (berhasil/gagal/dibatalkan)."""
        try:
            result = future.result()
            self.current_goal_handle = None
            
            if result and result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigasi BERHASIL!')
                self.nav_result_status = 'succeeded'
            elif result and result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigasi DIBATALKAN.')
                self.nav_result_status = 'canceled'
            else:
                self.get_logger().warn(f'Navigasi GAGAL (status: {result.status if result else "None"}).')
                self.nav_result_status = 'failed'
        except Exception as e:
            self.get_logger().error(f'Error di _goal_result_callback: {e}')
            self.nav_result_status = 'failed'
        
        # Signal bahwa navigasi selesai
        self.nav_event.set()
    
    def navigate_to_point(self, x, y, yaw=0.0):
        """
        Navigasi ke satu titik menggunakan Nav2 NavigateToPose action.
        Menggunakan callback + threading.Event (AMAN untuk ROS 2 Foxy).
        Return True jika berhasil, False jika gagal.
        """
        self.stop_navigation = False
        
        # Tunggu action server tersedia
        server_ready = False
        for _ in range(20):  # 20 x 0.5s = 10s timeout
            if self.stop_navigation:
                return False
            if self.nav_client.wait_for_server(timeout_sec=0.5):
                server_ready = True
                break
        
        if not server_ready:
            self.get_logger().error('Nav2 action server TIDAK TERSEDIA!')
            return False
        
        # Buat goal (stamp=0 agar TF2 menggunakan latest transform baik di mode Sim maupun Real)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        
        # Konversi yaw ke quaternion
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        self.get_logger().info(f'Navigasi ke ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        
        # Reset events
        self.nav_event.clear()
        self.goal_accepted_event.clear()
        self.goal_accepted = False
        self.nav_result_status = None
        
        # Kirim goal dengan callback (TIDAK menggunakan spin_until_future_complete!)
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        # Tunggu goal diterima/ditolak (max 15 detik)
        start_wait = time.time()
        while not self.goal_accepted_event.is_set():
            if self.stop_navigation:
                return False
            if time.time() - start_wait > 15.0:
                self.get_logger().warn('Timeout menunggu goal response!')
                return False
            self.goal_accepted_event.wait(timeout=0.5)
        
        if not self.goal_accepted:
            return False
        
        # Tunggu navigasi selesai (max 5 menit)
        while not self.nav_event.is_set():
            if self.stop_navigation:
                return False
            self.nav_event.wait(timeout=0.5)
        
        if self.stop_navigation:
            return False
        
        # Cek hasil
        return self.nav_result_status == 'succeeded'
    
    def publish_status(self):
        """Publish status misi ke topic untuk dashboard."""
        status = {
            'mission': None,
            'timestamp': time.time()
        }
        
        if self.mission:
            status['mission'] = {
                'mission_id': self.mission['mission_id'],
                'status': self.mission['status'],
                'current_task_index': self.mission['current_task_index'],
                'current_phase': self.mission['current_phase'],
                'completed_tasks': self.mission['completed_tasks'],
                'total_tasks': self.mission['total_tasks'],
                'elapsed_time': time.time() - self.mission['start_time'],
                'tasks': self.mission['tasks'],
                'waiting_info': self.mission.get('waiting_info', None)
            }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def get_status_dict(self):
        """Return status sebagai dictionary (untuk REST API)."""
        if self.mission:
            return {
                'mission_id': self.mission['mission_id'],
                'status': self.mission['status'],
                'current_task_index': self.mission['current_task_index'],
                'current_phase': self.mission['current_phase'],
                'completed_tasks': self.mission['completed_tasks'],
                'total_tasks': self.mission['total_tasks'],
                'elapsed_time': time.time() - self.mission['start_time'],
                'tasks': self.mission['tasks'],
                'waiting_info': self.mission.get('waiting_info', None)
            }
        return {'status': 'idle', 'current_phase': 'idle', 'waiting_info': None}


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint Manager dimatikan.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
