#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from ultralytics import YOLO

class Human_Awareness_Node(Node):
    def __init__(self):
        """
        Arsitektur Wrapper ROS 2 untuk Sistem Vision dan Manajemen Memori Spasial.
        """
        super().__init__('human_awareness_system_node')
        
        # 1. PUSTAKA AI & MEMORI TEMPORAL (Kodemu)
        # Sesuai target model, pastikan file .pt ada di folder workspace atau berikan path absolut
        self.model = YOLO("yolov8n-pose.pt")
        self.posisi_history = {}
        self.missing_count = {}
        
        # Hyperparameters Sistem
        self.treshold_gerakan = 30       
        self.kp_conf_treshold = 0.6      
        self.max_missing_frames = 45     

        # 2. INFRASTRUKTUR JEMBATAN ROS 2
        self.bridge = CvBridge()
        
        # Subscriber: Mengambil gambar aktif dari GPU rendering Gazebo
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publisher: Melempar status perilaku hasil inferensi AI ke Node FSM
        self.status_pub = self.create_publisher(String, '/human_status', 10)
        
        self.get_logger().info("====================================================")
        self.get_logger().info("NODE EDGE AI: Human Awareness System AKTIF & MEMANTAU")
        self.get_logger().info("====================================================")

    def clean_stale_tracks(self, active_ids):
        """
        Garbage Collection Memory RAM untuk mencegah Memory Leak pada sistem AMR.
        """
        stale_ids = set(self.posisi_history.keys()) - active_ids
        for tid in stale_ids:
            self.missing_count[tid] = self.missing_count.get(tid, 0) + 1
            if self.missing_count[tid] > self.max_missing_frames:
                self.posisi_history.pop(tid)
                self.missing_count.pop(tid)
                self.get_logger().warn(f"[Garbage Collector] ID {tid} dihapus dari RAM karena keluar area.")
                
        for tid in active_ids:
            if tid in self.missing_count:
                self.missing_count[tid] = 0

    def analyze_temporal_behavior(self, track_id, center_x, center_y):
        """
        Mengolah data riwayat koordinat untuk menentukan status Diam vs Berjalan.
        """
        if track_id not in self.posisi_history:
            self.posisi_history[track_id] = deque(maxlen=30)
            
        self.posisi_history[track_id].append((center_x, center_y))
        user_history = self.posisi_history[track_id]
        
        perilaku = "CALIBRATION"
        warna_perilaku = (0, 165, 255) 
        
        if len(user_history) >= 30:
            posisi_awal = user_history[0]
            posisi_akhir = user_history[-1]
            
            jarak_pergeseran = np.sqrt((posisi_akhir[0] - posisi_awal[0]) ** 2 + 
                                       (posisi_akhir[1] - posisi_awal[1]) ** 2)
            
            if jarak_pergeseran > self.treshold_gerakan:
                perilaku = "WALKING"
                warna_perilaku = (255, 0, 0)  
            else:
                perilaku = "IDLE"
                warna_perilaku = (0, 255, 0)  
                
        return perilaku, warna_perilaku

    def image_callback(self, msg):
        """
        Pipeline Utama: Konversi Citra ROS -> Inferensi AI -> Publikasi Logika FSM.
        """
        try:
            # Konversi matriks gambar dari tipe data ROS Message ke OpenCV Array
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Gagal konversi gambar: {e}")
            return

        # Jalankan tracking model YOLOv8-pose
        results = self.model.track(frame, persist=True, conf=0.6, verbose=False)
        active_ids = set()
        
        # Variabel penampung keputusan akhir untuk di-publish ke FSM
        final_fsm_decision = "NO_HUMAN"

        for r in results:
            boxes = r.boxes
            keypoints = r.keypoints

            for i, box in enumerate(boxes):
                track_id = int(box.id[0]) if box.id is not None else i
                active_ids.add(track_id)

                warna_box = (0, 255, 0)          
                state_jarak = "SAFE"
                target_width = 0
                metode_deteksi = "None"

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # 1. Analisis Perilaku Temporal
                perilaku, warna_text = self.analyze_temporal_behavior(track_id, center_x, center_y)
                
                # Assign status dasar dari hasil analisis temporal
                if perilaku == "WALKING":
                    final_fsm_decision = "WALKING"
                elif perilaku == "IDLE":
                    final_fsm_decision = "IDLE"

                # 2. ESTIMASI JARAK HIERARKIS & POSE SPASIAL
                if keypoints is not None and len(keypoints.xy) > i:
                    kp = keypoints.xy[i].cpu().numpy()
                    kp_conf = keypoints.conf[i].cpu().numpy()
    
                    CONF_BAHU = 0.5    
                    CONF_PINGGUL = 0.4 
                    CONF_WRIST = 0.3   
    
                    shoulder_visible = kp_conf[5] > CONF_BAHU and kp_conf[6] > CONF_BAHU
                    hips_visible = kp_conf[11] > CONF_PINGGUL and kp_conf[12] > CONF_PINGGUL
                    wrist_visible = kp_conf[9] > CONF_WRIST and kp_conf[10] > CONF_WRIST
    
                    # ANATOMICAL SANITY CHECK
                    if shoulder_visible and hips_visible:
                        jarak_y_tubuh = kp[11][1] - kp[5][1]
                        if jarak_y_tubuh < 30:
                            shoulder_visible = False
    
                    # Penentuan Jarak Spasial
                    if shoulder_visible:
                        target_width = np.sqrt((kp[6][0] - kp[5][0]) ** 2 + (kp[6][1] - kp[5][1]) ** 2)
                        metode_deteksi = "Bahu (Stabil)"
                        warna_fitur = (255, 0, 255)  
                        cv2.circle(frame, tuple(map(int, kp[5])), 6, warna_fitur, -1)
                        cv2.circle(frame, tuple(map(int, kp[6])), 6, warna_fitur, -1)
    
                        if hips_visible:
                            y_bahu_avg = (kp[5][1] + kp[6][1]) / 2
                            y_pinggul_avg = (kp[11][1] + kp[12][1]) / 2
                            tinggi_torso = y_pinggul_avg - y_bahu_avg
                            
                            if target_width < (tinggi_torso * 0.4):
                                target_width = tinggi_torso * 0.6 
                                metode_deteksi = "Torso (Menyamping)"
                                warna_fitur = (0, 165, 255) 
                                x_bahu_avg = int((kp[5][0] + kp[6][0]) / 2)
                                x_pinggul_avg = int((kp[11][0] + kp[12][0]) / 2)
                                cv2.line(frame, (x_bahu_avg, int(y_bahu_avg)), (x_pinggul_avg, int(y_pinggul_avg)), warna_fitur, 4)
    
                    elif hips_visible:
                        target_width = np.sqrt((kp[12][0] - kp[11][0]) ** 2 + (kp[12][1] - kp[11][1]) ** 2)
                        metode_deteksi = "Pinggul (Fallback)"
                        warna_fitur = (255, 255, 0)  
                        cv2.circle(frame, tuple(map(int, kp[11])), 6, warna_fitur, -1)
                        cv2.circle(frame, tuple(map(int, kp[12])), 6, warna_fitur, -1)
                    else:
                        target_width = 999  
                        metode_deteksi = "Blind (Terlalu Dekat)"
                        warna_fitur = (128, 128, 128)  

                    # Klasifikasi State Jarak untuk Pengaman Navigasi
                    if target_width > 150:
                        state_jarak = "DANGER"
                        warna_box = (0, 0, 255)  
                        final_fsm_decision = "DISTURBING" # Override: Manusia memotong jalur robot!
                    elif target_width > 85:
                        state_jarak = "WARNING"
                        warna_box = (0, 165, 255)  
                    else:
                        state_jarak = "SAFE"

                    # Heuristik Deteksi Perilaku Jahil / Panggilan (Tangan di atas Bahu)
                    if len(kp) > 10 and kp[5][1] != 0 and kp[9][1] != 0:
                        if wrist_visible and (kp[9][1] < kp[5][1] or kp[10][1] < kp[6][1]):
                            perilaku = "CALLING / JAHIL"
                            warna_text = (0, 0, 255)  
                            final_fsm_decision = "CALLING" # Override: Manusia berinteraksi dengan robot

                # 3. RENDER HUD INTERFACE (Untuk Keperluan Monitoring RViz/Screen)
                cv2.rectangle(frame, (x1, y1), (x2, y2), warna_box, 2)
                label_jarak = f"Sistem: {state_jarak}"
                label_fitur = f"Fitur: {metode_deteksi} ({int(target_width)} px)"
                label_perilaku = f"Aktivitas: {perilaku} [ID: {track_id}]"
                
                cv2.putText(frame, label_jarak, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, warna_box, 2)
                cv2.putText(frame, label_fitur, (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, warna_fitur, 2)
                cv2.putText(frame, label_perilaku, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, warna_text, 2)

        # 4. MANAJEMEN MEMORI & PUBLIKASI DATA KE ROS 2
        self.clean_stale_tracks(active_ids)
        
        # Kirim string keputusan akhir ke topik FSM
        fsm_msg = String()
        fsm_msg.data = final_fsm_decision
        self.status_pub.publish(fsm_msg)
        
        # Opsional: Tampilkan window OpenCV di WSL jika mau dipamerkan saat demo
        cv2.imshow("Monitor Edge AI - AMR Core", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Human_Awareness_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mematikan Node Edge AI...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()