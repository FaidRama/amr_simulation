#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
SCRIPT PENGUJIAN TERPADU HUMAN BEHAVIOR & RESPONS KECEPATAN AMR (SKRIPSI)
=============================================================================
Fungsi:
1. Menguji Akurasi Klasifikasi 3 Aktivitas (STANDING, WALKING, SITTING) & Arah Gerak
   melawan Ground Truth yang diperagakan di depan kamera robot.
2. Menguji Respons Dinamis Adaptasi Kecepatan (/human_state -> /cmd_vel_safe -> /odom)
   untuk mendapatkan bukti grafik penurunan kecepatan & waktu reaksi pengereman (ms).
=============================================================================
"""

import os
import sys
import csv
import json
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class BehaviorTestNode(Node):
    def __init__(self):
        super().__init__('behavior_test_node')

        self.lock = threading.Lock()
        self.human_info = {}
        self.human_state = "UNKNOWN"
        self.cmd_vel_in = 0.0
        self.cmd_vel_safe = 0.0
        self.odom_v = 0.0
        self.last_info_time = 0.0

        self.create_subscription(String, '/human_info', self.on_human_info, 10)
        self.create_subscription(String, '/human_state', self.on_human_state, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_cmd_vel_safe, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_human_info(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.human_info = data
                self.last_info_time = time.time()
        except Exception:
            pass

    def on_human_state(self, msg: String):
        with self.lock:
            self.human_state = msg.data

    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.cmd_vel_in = float(msg.linear.x)

    def on_cmd_vel_safe(self, msg: Twist):
        with self.lock:
            self.cmd_vel_safe = float(msg.linear.x)

    def on_odom(self, msg: Odometry):
        with self.lock:
            self.odom_v = float(msg.twist.twist.linear.x)

    def get_snapshot(self):
        with self.lock:
            return {
                "t": time.time(),
                "info": dict(self.human_info),
                "state": self.human_state,
                "cmd_in": self.cmd_vel_in,
                "cmd_safe": self.cmd_vel_safe,
                "odom_v": self.odom_v,
                "info_age": time.time() - self.last_info_time if self.last_info_time > 0 else 999.0
            }


def run_activity_accuracy_test(node: BehaviorTestNode, out_dir: str):
    """Mode 1: Uji Akurasi Klasifikasi 3 Aktivitas (Standing, Walking, Sitting)."""
    scenarios = {
        "1": ("STANDING", "STATIONARY", "Berdiri diam di depan kamera"),
        "2": ("WALKING", "OPPOSITE_DIRECTION", "Berjalan mendekati / berlawanan arah robot"),
        "3": ("WALKING", "SAME_DIRECTION", "Berjalan menjauh / searah robot"),
        "4": ("SITTING", "STATIONARY", "Duduk di kursi di depan kamera"),
    }

    print("\n" + "=" * 65)
    print(" 📋 PILIH SKENARIO GROUND TRUTH YANG AKAN DIPERAGAKAN:")
    for k, (act, dr, desc) in scenarios.items():
        print(f"  [{k}] {act} ({dr}) -> {desc}")
    print("=" * 65)

    choice = input("Masukkan pilihan (1-4): ").strip()
    if choice not in scenarios:
        print("❌ Pilihan tidak valid.")
        return

    gt_activity, gt_dir, desc = scenarios[choice]
    dur_str = input("Durasi perekaman dalam detik (default 5 detik): ").strip()
    duration = float(dur_str) if dur_str else 5.0

    os.makedirs(out_dir, exist_ok=True)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"uji_aktivitas_{gt_activity.lower()}_{ts_str}.csv")

    print(f"\n⏳ Bersiap memperagakan: {desc}")
    for i in range(3, 0, -1):
        print(f"   Mulai dalam {i}...")
        time.sleep(1.0)

    print(f"🔴 MEREKAM SELAMA {duration} DETIK...\n")
    records = []
    t_start = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "gt_activity", "gt_direction",
            "pred_activity", "pred_direction", "speed_m_s",
            "distance_m", "in_corridor", "fsm_state", "infer_ms", "fps"
        ])

        while (time.time() - t_start) < duration:
            snap = node.get_snapshot()
            info = snap["info"]
            if snap["info_age"] < 1.0 and info:
                p_act = info.get("activity", "NONE")
                p_dir = info.get("direction", "UNKNOWN")
                p_spd = info.get("speed_m_s", 0.0)
                dist = info.get("distance_m", "")
                in_corr = info.get("target_in_corridor", False)
                state = snap["state"]
                infer = info.get("infer_ms", 0)
                fps = info.get("fps", 0)
                elapsed = time.time() - t_start

                writer.writerow([
                    round(snap["t"], 3), round(elapsed, 3), gt_activity, gt_dir,
                    p_act, p_dir, p_spd, dist, in_corr, state, infer, fps
                ])
                records.append((p_act, p_dir, state))
                print(f"[{elapsed:4.1f}s] Pred: {p_act:8s} ({p_dir:18s}) | Dist: {str(dist):>5s}m | FSM: {state}")

            time.sleep(0.1)  # 10 Hz sampling

    if not records:
        print("❌ Tidak ada data dari /human_info! Pastikan human_awareness_node.py aktif.")
        return

    n_total = len(records)
    act_correct = sum(1 for a, d, s in records if a == gt_activity)
    act_acc = (act_correct / n_total) * 100.0

    if gt_activity == "WALKING":
        dir_correct = sum(1 for a, d, s in records if a == gt_activity and d == gt_dir)
        dir_acc = (dir_correct / n_total) * 100.0
    else:
        dir_acc = act_acc

    print("\n" + "=" * 65)
    print(f" 📊 HASIL UJI AKURASI AKTIVITAS: {gt_activity} ({gt_dir})")
    print(f"    Total Sampel         : {n_total} data")
    print(f"    Akurasi Aktivitas    : {act_acc:.1f}% ({act_correct}/{n_total})")
    if gt_activity == "WALKING":
        print(f"    Akurasi Arah Gerak   : {dir_acc:.1f}% ({dir_correct}/{n_total})")
    print(f"    File CSV Tersimpan   : {csv_path}")
    print("=" * 65)


def run_dynamic_speed_test(node: BehaviorTestNode, out_dir: str):
    """Mode 2: Uji Respons Dinamis Kecepatan & Waktu Reaksi Pengereman Robot."""
    print("\n" + "=" * 65)
    print(" 🏎️ UJI RESPONS DINAMIS KECEPATAN & PENGEREMAN ROBOT")
    print("    Pastikan Arduino, Navigasi/Teleop, & Human Awareness aktif.")
    print("    Script merekam /human_state, /cmd_vel, /cmd_vel_safe, & /odom (20 Hz).")
    print("=" * 65)

    dur_str = input("Durasi perekaman dalam detik (default 15 detik): ").strip()
    duration = float(dur_str) if dur_str else 15.0

    os.makedirs(out_dir, exist_ok=True)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"uji_respons_dinamis_{ts_str}.csv")

    print(f"\n🔴 MEREKAM RESPONS KECEPATAN SELAMA {duration} DETIK (Tekan Ctrl+C untuk stop cepat)...\n")
    t_start = time.time()
    safe_stop_trigger_t = None
    braking_reaction_ms = None

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_s", "human_state", "activity", "distance_m",
            "cmd_vel_raw", "cmd_vel_safe", "odom_linear_v"
        ])

        try:
            while (time.time() - t_start) < duration:
                snap = node.get_snapshot()
                elapsed = time.time() - t_start
                state = snap["state"]
                info = snap["info"]
                act = info.get("activity", "NONE")
                dist = info.get("distance_m", "")
                c_in = snap["cmd_in"]
                c_safe = snap["cmd_safe"]
                odom_v = snap["odom_v"]

                writer.writerow([
                    round(elapsed, 3), state, act, dist,
                    round(c_in, 3), round(c_safe, 3), round(odom_v, 3)
                ])

                # Deteksi waktu reaksi dari SAFE_STOP muncul hingga cmd_safe=0
                if state == "SAFE_STOP" and safe_stop_trigger_t is None:
                    safe_stop_trigger_t = snap["t"]
                if safe_stop_trigger_t is not None and braking_reaction_ms is None and abs(c_safe) < 0.01:
                    braking_reaction_ms = (snap["t"] - safe_stop_trigger_t) * 1000.0

                print(f"[{elapsed:5.2f}s] State: {state:13s} | Act: {act:8s} | cmd_in: {c_in:5.2f} | cmd_safe: {c_safe:5.2f} | odom_v: {odom_v:5.2f} m/s")
                time.sleep(0.05)  # 20 Hz
        except KeyboardInterrupt:
            pass

    print("\n" + "=" * 65)
    print(" 📊 RINGKASAN UJI RESPONS DINAMIS")
    if braking_reaction_ms is not None:
        print(f"    Waktu Reaksi Rem (SAFE_STOP -> cmd_safe=0) : {braking_reaction_ms:.1f} ms")
    else:
        print("    Waktu Reaksi Rem : Tidak ada transisi SAFE_STOP selama perekaman.")
    print(f"    File CSV Tersimpan : {csv_path}")
    print("=" * 65)


def main():
    rclpy.init()
    node = BehaviorTestNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    out_dir = os.path.expanduser("~/hasil_pengujian_behavior")

    try:
        while rclpy.ok():
            print("\n" + "=" * 65)
            print(" 🧪 MENU PENGUJIAN SKRIPSI HUMAN-AWARE BEHAVIOR AMR")
            print("=" * 65)
            print("  [1] Uji Akurasi Klasifikasi 3 Aktivitas (Standing / Walking / Sitting)")
            print("  [2] Uji Respons Dinamis Kecepatan & Waktu Reaksi Pengereman Robot")
            print("  [0] Keluar")
            print("=" * 65)

            menu = input("Pilih mode pengujian (0-2): ").strip()
            if menu == "1":
                run_activity_accuracy_test(node, out_dir)
            elif menu == "2":
                run_dynamic_speed_test(node, out_dir)
            elif menu == "0":
                break
            else:
                print("❌ Pilihan tidak dikenal.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
