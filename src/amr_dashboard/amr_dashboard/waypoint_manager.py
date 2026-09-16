#!/usr/bin/env python3
"""
=================================================================
WAYPOINT MANAGER NODE
Node ROS2 untuk mengelola misi multi-waypoint delivery.
Menerima daftar task (pickup → delivery) dan mengeksekusi
secara sequential menggunakan Nav2 NavigateToPose action.
=================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import json
import uuid
import time
import math
import threading


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
        
        # === STATE MANAGEMENT ===
        self.mission = None          # Misi aktif saat ini
        self.is_paused = False       # Flag pause
        self.is_cancelled = False    # Flag cancel
        self.current_goal_handle = None
        self.mission_lock = threading.Lock()
        
        # Timer untuk publish status berkala
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('='*55)
        self.get_logger().info(' WAYPOINT MANAGER NODE - AKTIF')
        self.get_logger().info(' Menunggu perintah dari dashboard...')
        self.get_logger().info('='*55)
    
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
                'current_phase': 'idle',  # idle, navigating_pickup, at_pickup, navigating_delivery, at_delivery
                'start_time': time.time(),
                'completed_tasks': 0,
                'total_tasks': len(tasks)
            }
            self.is_paused = False
            self.is_cancelled = False
            
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
        """Menerima perintah kontrol: cancel, pause, resume."""
        command = msg.data.strip().lower()
        
        if command == 'cancel':
            self.get_logger().info('PERINTAH CANCEL DITERIMA')
            self.is_cancelled = True
            self.is_paused = False
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.emergency_stop()
            
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
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
    
    def emergency_stop(self):
        """Kirim cmd_vel zero untuk stop AMR."""
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
    
    def execute_mission(self):
        """Eksekusi semua task dalam misi secara sequential."""
        if not self.mission:
            return
        
        tasks = self.mission['tasks']
        
        for i, task in enumerate(tasks):
            if self.is_cancelled:
                self.mission['status'] = 'cancelled'
                self.get_logger().info('Misi DIBATALKAN oleh user.')
                break
            
            # Tunggu jika di-pause
            while self.is_paused and not self.is_cancelled:
                self.mission['status'] = 'paused'
                time.sleep(0.5)
            
            if self.is_cancelled:
                self.mission['status'] = 'cancelled'
                break
            
            self.mission['status'] = 'running'
            self.mission['current_task_index'] = i
            
            pickup = task['pickup']
            delivery = task['delivery']
            
            # === FASE 1: Navigasi ke titik PICKUP ===
            self.get_logger().info(
                f'--- Task {i+1}/{len(tasks)}: Menuju pickup [{pickup["name"]}] ---')
            self.mission['current_phase'] = 'navigating_pickup'
            
            success = self.navigate_to_point(
                pickup['x'], pickup['y'], pickup.get('yaw', 0.0))
            
            if self.is_cancelled:
                break
            
            if not success:
                self.get_logger().warn(
                    f'Gagal mencapai pickup [{pickup["name"]}], skip ke task berikutnya.')
                task['status'] = 'failed'
                continue
            
            # Sampai di pickup — tunggu sebentar (simulasi pengambilan barang)
            self.mission['current_phase'] = 'at_pickup'
            self.get_logger().info(
                f'SAMPAI di pickup [{pickup["name"]}]. Menunggu pengambilan barang...')
            time.sleep(3.0)  # Pause 3 detik di titik pickup
            
            # Tunggu jika di-pause
            while self.is_paused and not self.is_cancelled:
                time.sleep(0.5)
            
            if self.is_cancelled:
                break
            
            # === FASE 2: Navigasi ke titik DELIVERY ===
            self.get_logger().info(
                f'--- Task {i+1}/{len(tasks)}: Menuju delivery [{delivery["name"]}] ---')
            self.mission['current_phase'] = 'navigating_delivery'
            
            success = self.navigate_to_point(
                delivery['x'], delivery['y'], delivery.get('yaw', 0.0))
            
            if self.is_cancelled:
                break
            
            if not success:
                self.get_logger().warn(
                    f'Gagal mencapai delivery [{delivery["name"]}].')
                task['status'] = 'failed'
                continue
            
            # Sampai di delivery — tunggu sebentar (simulasi penyerahan barang)
            self.mission['current_phase'] = 'at_delivery'
            self.get_logger().info(
                f'SAMPAI di delivery [{delivery["name"]}]. Menyerahkan barang...')
            time.sleep(3.0)
            
            task['status'] = 'completed'
            self.mission['completed_tasks'] = i + 1
            self.get_logger().info(
                f'✓ Task {i+1}/{len(tasks)} SELESAI: '
                f'{pickup["name"]} → {delivery["name"]}')
        
        # Misi selesai
        if not self.is_cancelled:
            self.mission['status'] = 'completed'
            self.mission['current_phase'] = 'idle'
            self.get_logger().info('='*50)
            self.get_logger().info(' MISI SELESAI — Semua task telah diproses!')
            self.get_logger().info('='*50)
    
    def navigate_to_point(self, x, y, yaw=0.0):
        """
        Navigasi ke satu titik menggunakan Nav2 NavigateToPose action.
        Return True jika berhasil, False jika gagal.
        """
        # Tunggu action server
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server TIDAK TERSEDIA!')
            return False
        
        # Buat goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        
        # Konversi yaw ke quaternion
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        
        self.get_logger().info(f'Navigasi ke ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        
        # Kirim goal
        send_goal_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Goal DITOLAK oleh Nav2!')
            return False
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal DITERIMA, AMR sedang bergerak...')
        
        # Tunggu hasil
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)
        
        self.current_goal_handle = None
        
        result = result_future.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigasi BERHASIL!')
            return True
        elif result and result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigasi DIBATALKAN.')
            return False
        else:
            self.get_logger().warn('Navigasi GAGAL.')
            return False
    
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
                'tasks': self.mission['tasks']
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
                'tasks': self.mission['tasks']
            }
        return {'status': 'idle', 'current_phase': 'idle'}


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
