#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
=SUITE PENGUJIAN TERPADU HUMAN-AWARE BEHAVIOR AMR (SKRIPSI — UJI 1 s/d UJI 8)
=============================================================================
Selaras dengan TEMPLATE_RANGKUMAN_PENGUJIAN_HUMAN_AWARE.md:
  [1] Uji 1 : Kalibrasi & Akurasi Estimasi Jarak Monokuler (+ Variasi Tinggi)
  [2] Uji 2 : Selektivitas Filter Koridor Tengah (In vs Outside Corridor)
  [3] Uji 3 : Akurasi Klasifikasi Aktivitas, Arah, Ego-Motion & Multi-Person
  [4] Uji 4 : Stabilitas FSM (Hysteresis & Debounce / Anti-Chattering)
  [5] Uji 5 : Respons Dinamis Kecepatan & Pengereman Adaptif (20 Hz)
  [6] Uji 6 : Integrasi Interaksi Sosial 4-Fase, Kontekstual & Fail-Safe
  [7] Uji 7 : Komparasi End-to-End (Nav2 Baseline vs Human-Aware Adaptif)
  [8] Uji 8 : Performa Komputasi Edge AI (FPS/Inferensi) & Pencahayaan
  [9] Diagnostik Live Status Topik ROS 2 Pra-Uji
=============================================================================
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time
from collections import Counter
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


ROBOT_RADIUS_M = 0.29  # Radius footprint sasis robot (sesuai nav2_params & evaluasi_navigasi)


def resolve_output_dir(custom_dir: str = None) -> str:
    """Simpan hasil ke <workspace_root>/hasil_pengujian_behavior atau fallback ~/hasil_pengujian_behavior."""
    if custom_dir:
        return os.path.abspath(os.path.expanduser(custom_dir))
    for base in (os.path.dirname(os.path.abspath(__file__)), os.getcwd()):
        cur = os.path.abspath(base)
        while True:
            if os.path.isdir(os.path.join(cur, ".git")) or (
                os.path.basename(cur) != "src"
                and os.path.isdir(os.path.join(cur, "src", "amr_simulation"))
                and os.path.isdir(os.path.join(cur, "src", "amr_dashboard"))
            ):
                return os.path.join(cur, "hasil_pengujian_behavior")
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return os.path.expanduser("~/hasil_pengujian_behavior")


def sanitize_slug(text: str, default: str = "uji") -> str:
    cleaned = "".join(ch if (ch.isalnum() or ch in ("-", "_", ".")) else "_" for ch in text.strip().lower())
    cleaned = cleaned.strip("_.")
    return cleaned if cleaned else default


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def mean_sd(values):
    if not values:
        return 0.0, 0.0
    avg = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return avg, sd


def count_state_transitions(seq) -> int:
    if len(seq) < 2:
        return 0
    return sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])


def raw_state_without_hysteresis(ratio, t_med: float = 0.60, t_near: float = 0.85, in_corridor: bool = True) -> str:
    """State pembanding murni tanpa hysteresis maupun debounce pada rasio bbox."""
    if not in_corridor or ratio is None:
        return "NORMAL"
    if ratio >= t_near:
        return "SAFE_STOP"
    if ratio >= t_med:
        return "REDUCED_SPEED"
    return "NORMAL"


def expected_zone_from_distance(d_m: float) -> str:
    if d_m <= 1.80:
        return "NEAR"
    if d_m <= 2.70:
        return "MEDIUM"
    return "FAR"


def countdown(sec: int = 3, label: str = ""):
    if label:
        print(f"\n⏳ Bersiap: {label}")
    for i in range(sec, 0, -1):
        print(f"   Mulai dalam {i}...")
        time.sleep(1.0)


def input_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} (default {default}): ").strip().replace(",", ".")
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"   ⚠️ Input tidak valid, memakai default {default}")
        return default


if HAS_ROS2:
    class BehaviorTestNode(Node):
        def __init__(self):
            super().__init__("behavior_test_node")

            self.lock = threading.Lock()
            self.human_info = {}
            self.human_state = "UNKNOWN"
            self.human_status = "UNKNOWN"
            self.human_nav_mode = "UNKNOWN"
            self.cmd_vel_in = 0.0
            self.cmd_vel_w_in = 0.0
            self.cmd_vel_safe = 0.0
            self.cmd_vel_w_safe = 0.0
            self.odom_v = 0.0
            self.odom_w = 0.0
            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_yaw = 0.0
            self.min_scan_range = None
            self.voice_status = {}
            self.last_delivery_log = ""

            self.ts = {
                "info": 0.0,
                "state": 0.0,
                "status": 0.0,
                "nav_mode": 0.0,
                "cmd_in": 0.0,
                "cmd_safe": 0.0,
                "odom": 0.0,
                "scan": 0.0,
                "voice": 0.0,
            }

            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.create_subscription(String, "/human_info", self.on_human_info, 10)
            self.create_subscription(String, "/human_state", self.on_human_state, 10)
            self.create_subscription(String, "/human_status", self.on_human_status, 10)
            self.create_subscription(String, "/human_nav_mode", self.on_human_nav_mode, 10)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
            self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel_safe, 10)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, sensor_qos)
            self.create_subscription(String, "/robot_voice/status", self.on_voice_status, 10)
            self.create_subscription(String, "/amr/delivery_log", self.on_delivery_log, 10)

        def on_human_info(self, msg: String):
            try:
                data = json.loads(msg.data)
                with self.lock:
                    self.human_info = data
                    self.ts["info"] = time.time()
            except Exception:
                pass

        def on_human_state(self, msg: String):
            with self.lock:
                self.human_state = msg.data.strip().upper()
                self.ts["state"] = time.time()

        def on_human_status(self, msg: String):
            with self.lock:
                self.human_status = msg.data.strip().upper()
                self.ts["status"] = time.time()

        def on_human_nav_mode(self, msg: String):
            with self.lock:
                self.human_nav_mode = msg.data.strip().upper()
                self.ts["nav_mode"] = time.time()

        def on_cmd_vel(self, msg: Twist):
            with self.lock:
                self.cmd_vel_in = float(msg.linear.x)
                self.cmd_vel_w_in = float(msg.angular.z)
                self.ts["cmd_in"] = time.time()

        def on_cmd_vel_safe(self, msg: Twist):
            with self.lock:
                self.cmd_vel_safe = float(msg.linear.x)
                self.cmd_vel_w_safe = float(msg.angular.z)
                self.ts["cmd_safe"] = time.time()

        def on_odom(self, msg: Odometry):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            with self.lock:
                self.odom_v = float(msg.twist.twist.linear.x)
                self.odom_w = float(msg.twist.twist.angular.z)
                self.odom_x = float(pos.x)
                self.odom_y = float(pos.y)
                self.odom_yaw = yaw
                self.ts["odom"] = time.time()

        def on_scan(self, msg: LaserScan):
            valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.05]
            min_r = min(valid) if valid else None
            with self.lock:
                self.min_scan_range = min_r
                self.ts["scan"] = time.time()

        def on_voice_status(self, msg: String):
            try:
                data = json.loads(msg.data)
                with self.lock:
                    self.voice_status = data
                    self.ts["voice"] = time.time()
            except Exception:
                pass

        def on_delivery_log(self, msg: String):
            with self.lock:
                self.last_delivery_log = msg.data

        def get_snapshot(self):
            now = time.time()
            with self.lock:
                ages = {k: (now - t if t > 0.0 else 999.0) for k, t in self.ts.items()}
                min_r = self.min_scan_range
                clearance = max(0.0, min_r - ROBOT_RADIUS_M) if min_r is not None else None
                return {
                    "t": now,
                    "info": dict(self.human_info),
                    "state": self.human_state,
                    "status": self.human_status,
                    "nav_mode": self.human_nav_mode,
                    "cmd_in": self.cmd_vel_in,
                    "cmd_w_in": self.cmd_vel_w_in,
                    "cmd_safe": self.cmd_vel_safe,
                    "cmd_w_safe": self.cmd_vel_w_safe,
                    "odom_v": self.odom_v,
                    "odom_w": self.odom_w,
                    "odom_x": self.odom_x,
                    "odom_y": self.odom_y,
                    "odom_yaw": self.odom_yaw,
                    "min_scan_m": min_r,
                    "clearance_m": clearance,
                    "voice": dict(self.voice_status),
                    "last_log": self.last_delivery_log,
                    "ages": ages,
                    "info_age": ages["info"],
                }


# =============================================================================
# UJI 1: AKURASI ESTIMASI JARAK MONOKULER & VARIASI TINGGI BADAN
# =============================================================================
def run_test_1_distance_accuracy(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji1_jarak")
    os.makedirs(sub_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print(" 📏 UJI 1: AKURASI ESTIMASI JARAK MONOKULER & VARIASI TINGGI BADAN")
    print("=" * 70)
    subject = input("Nama/label subjek (mis. Subjek_A_170cm): ").strip() or "subjek_ref"
    height_cm = input_float("Tinggi badan subjek (cm)", 170.0)
    duration = input_float("Durasi perekaman per titik jarak (detik)", 5.0)

    session_summary = []

    while True:
        d_true = input_float("\nMasukkan jarak ground truth meteran (m, mis. 1.0 / 1.5 / 2.0 / 3.0 / 4.0)", 2.0)
        def_zone = expected_zone_from_distance(d_true)
        exp_zone = input(f"Zona seharusnya [NEAR/MEDIUM/FAR] (default {def_zone}): ").strip().upper() or def_zone

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        slug = sanitize_slug(subject)
        csv_path = os.path.join(sub_dir, f"uji_jarak_{slug}_{d_true:.2f}m_{ts_str}.csv")

        countdown(3, f"Subjek {subject} ({height_cm:.0f} cm) berdiri di jarak {d_true:.2f} m")
        print(f"🔴 MEREKAM SELAMA {duration:.1f} DETIK (10 Hz)...\n")

        dists, ratios_h, abs_errs, rel_errs = [], [], [], []
        zone_preds = []
        t_start = time.time()

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "elapsed_s", "subject", "height_cm",
                "true_distance_m", "ratio_h", "ratio_active", "est_distance_m",
                "abs_error_m", "rel_error_pct", "expected_zone", "pred_zone",
                "in_corridor", "confidence"
            ])
            while (time.time() - t_start) < duration:
                snap = node.get_snapshot()
                info = snap["info"]
                elapsed = time.time() - t_start
                if snap["info_age"] < 1.0 and info and info.get("distance_m") is not None:
                    d_est = float(info["distance_m"])
                    r_h = info.get("ratio_h")
                    r_act = info.get("ratio")
                    z_pred = info.get("zone", "NONE")
                    in_corr = bool(info.get("target_in_corridor", False))
                    conf = info.get("confidence", 0.0)

                    err_abs = abs(d_est - d_true)
                    err_rel = (err_abs / max(d_true, 1e-6)) * 100.0

                    writer.writerow([
                        round(snap["t"], 3), round(elapsed, 3), subject, height_cm,
                        round(d_true, 3), r_h, r_act, round(d_est, 3),
                        round(err_abs, 4), round(err_rel, 2), exp_zone, z_pred,
                        in_corr, conf
                    ])

                    dists.append(d_est)
                    if r_h is not None:
                        ratios_h.append(float(r_h))
                    abs_errs.append(err_abs)
                    rel_errs.append(err_rel)
                    zone_preds.append(z_pred)

                    print(
                        f"[{elapsed:4.1f}s] d_true={d_true:.2f}m | d_est={d_est:.2f}m "
                        f"| err={err_abs*100:4.1f}cm ({err_rel:4.1f}%) | h/H={r_h} | Zona={z_pred}"
                    )
                time.sleep(0.1)

        if not dists:
            print("❌ Tidak ada data estimasi jarak valid! Pastikan manusia di koridor & human_awareness_node aktif.")
        else:
            d_mean, d_sd = mean_sd(dists)
            rh_mean, _ = mean_sd(ratios_h)
            mae = statistics.mean(abs_errs)
            rmse = math.sqrt(sum(e * e for e in abs_errs) / len(abs_errs))
            mean_rel = statistics.mean(rel_errs)
            zone_acc = (sum(1 for z in zone_preds if z == exp_zone) / len(zone_preds)) * 100.0
            dom_zone = Counter(zone_preds).most_common(1)[0][0]

            session_summary.append((d_true, rh_mean, d_mean, d_sd, mae, rmse, mean_rel, exp_zone, dom_zone, zone_acc))
            print("\n" + "-" * 70)
            print(f" 📊 HASIL TITIK d_true = {d_true:.2f} m ({len(dists)} sampel)")
            print(f"    h/H Rata-rata       : {rh_mean:.3f}")
            print(f"    Estimasi Mean ± SD  : {d_mean:.3f} ± {d_sd:.3f} m")
            print(f"    MAE / RMSE          : {mae:.3f} m ({mae*100:.1f} cm) / {rmse:.3f} m")
            print(f"    Error Relatif       : {mean_rel:.2f}%")
            print(f"    Ketepatan Zona      : {dom_zone} ({zone_acc:.1f}% cocok dgn {exp_zone})")
            print(f"    CSV Tersimpan       : {csv_path}")
            print("-" * 70)

        again = input("Lanjut titik jarak berikutnya untuk subjek ini? (y/n): ").strip().lower()
        if again != "y":
            break

    if session_summary:
        print("\n" + "=" * 70)
        print(f" 📋 REKAP UJI 1 — SUBJEK: {subject} ({height_cm:.0f} cm)")
        print(" d_true |  h/H  |  d_est ± SD   |  MAE(m) | RMSE(m) | Err(%) | Zona (Acc%)")
        for row in session_summary:
            print(
                f"  {row[0]:4.2f}m | {row[1]:5.3f} | {row[2]:.2f} ± {row[3]:.2f}m "
                f"|  {row[4]:5.3f}  |  {row[5]:5.3f}  | {row[6]:5.1f}% | {row[8]} ({row[9]:.0f}%)"
            )
        overall_mae = statistics.mean(r[4] for r in session_summary)
        overall_rmse = math.sqrt(statistics.mean(r[5] ** 2 for r in session_summary))
        overall_rel = statistics.mean(r[6] for r in session_summary)
        print(f" ➡️ Gabungan: MAE = {overall_mae:.3f} m | RMSE = {overall_rmse:.3f} m | Err = {overall_rel:.2f}%")
        print("=" * 70)


# =============================================================================
# UJI 2: SELEKTIVITAS FILTER KORIDOR TENGAH (IN VS OUTSIDE CORRIDOR)
# =============================================================================
def run_test_2_corridor_selectivity(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji2_koridor")
    os.makedirs(sub_dir, exist_ok=True)

    presets = {
        "1": ("tengah_medium", True, "REDUCED_SPEED", "Tengah Koridor (jarak MEDIUM ~2.2m)"),
        "2": ("tengah_near", True, "SAFE_STOP", "Tengah Koridor (jarak NEAR ~1.3m)"),
        "3": ("luar_kiri_medium", False, "NORMAL", "Di Luar Koridor Kiri (jarak ~2.2m)"),
        "4": ("luar_kanan_medium", False, "NORMAL", "Di Luar Koridor Kanan (jarak ~2.2m)"),
        "5": ("luar_samping_near", False, "NORMAL", "Di Luar Koridor Samping Dekat (~1.3m)"),
    }

    print("\n" + "=" * 70)
    print(" 🎯 UJI 2: SELEKTIVITAS FILTER KORIDOR TENGAH (SPATIAL FILTERING)")
    for k, (_, exp_in, exp_st, desc) in presets.items():
        print(f"  [{k}] {desc} -> target_in_corridor={exp_in}, FSM={exp_st}")
    print("=" * 70)

    ch = input("Pilih skenario (1-5): ").strip()
    if ch not in presets:
        print("❌ Pilihan tidak valid.")
        return

    label, exp_in_corr, exp_state, desc = presets[ch]
    duration = input_float("Durasi perekaman (detik)", 5.0)

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_koridor_{label}_{ts_str}.csv")

    countdown(3, desc)
    print(f"🔴 MEREKAM SELAMA {duration:.1f} DETIK (10 Hz)...\n")

    records = []
    t_start = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "scenario", "expected_in_corridor",
            "expected_state", "persons_total", "corridor_persons",
            "target_in_corridor", "distance_m", "fsm_state", "human_status"
        ])
        while (time.time() - t_start) < duration:
            snap = node.get_snapshot()
            info = snap["info"]
            elapsed = time.time() - t_start
            if snap["info_age"] < 1.0 and info:
                n_p = int(info.get("persons", 0))
                n_c = int(info.get("corridor_persons", 0))
                in_c = bool(info.get("target_in_corridor", False))
                dist = info.get("distance_m", "")
                st = snap["state"]
                h_status = snap["status"]

                writer.writerow([
                    round(snap["t"], 3), round(elapsed, 3), label, exp_in_corr,
                    exp_state, n_p, n_c, in_c, dist, st, h_status
                ])
                records.append((in_c, st, n_p))
                print(
                    f"[{elapsed:4.1f}s] Persons={n_p} | InCorr={str(in_c):5s} ({n_c}) "
                    f"| Dist={str(dist):>5s}m | FSM={st}"
                )
            time.sleep(0.1)

    if not records:
        print("❌ Tidak ada data dari /human_info!")
        return

    n_tot = len(records)
    in_true_pct = (sum(1 for r in records if r[0]) / n_tot) * 100.0
    state_ok_pct = (sum(1 for r in records if r[1] == exp_state) / n_tot) * 100.0
    st_dist = dict(Counter(r[1] for r in records))

    print("\n" + "=" * 70)
    print(f" 📊 HASIL UJI 2 KORIDOR: {desc}")
    print(f"    Total Sampel              : {n_tot}")
    print(f"    in_corridor = True        : {in_true_pct:.1f}%")
    if exp_in_corr:
        print(f"    True Positive Rate (TPR)  : {in_true_pct:.1f}%")
    else:
        false_alarm_pct = (sum(1 for r in records if r[1] != "NORMAL") / n_tot) * 100.0
        print(f"    False Alarm Rate (!NORMAL): {false_alarm_pct:.1f}%")
    print(f"    Kecocokan State ({exp_state:13s}): {state_ok_pct:.1f}%")
    print(f"    Distribusi State FSM      : {st_dist}")
    print(f"    CSV Tersimpan             : {csv_path}")
    print("=" * 70)


# =============================================================================
# UJI 3: AKURASI KLASIFIKASI AKTIVITAS, ARAH, EGO-MOTION & MULTI-PERSON
# =============================================================================
def run_test_3_activity_accuracy(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji3_aktivitas")
    os.makedirs(sub_dir, exist_ok=True)

    scenarios = {
        "1": ("STANDING", "STATIONARY", "static", "Berdiri diam di depan kamera (robot diam)"),
        "2": ("WALKING", "OPPOSITE_DIRECTION", "static", "Berjalan mendekati / berlawanan arah robot"),
        "3": ("WALKING", "SAME_DIRECTION", "static", "Berjalan menjauh / searah robot"),
        "4": ("SITTING", "STATIONARY", "static", "Duduk di kursi di depan kamera (robot diam)"),
        "5": ("STANDING", "STATIONARY", "ego_motion", "EGO-MOTION: Berdiri diam saat robot melaju maju"),
        "6": ("SITTING", "STATIONARY", "ego_motion", "EGO-MOTION: Duduk di kursi saat robot melaju maju"),
        "7": ("MULTI_PERSON", "ANY", "multi_person", "MULTI-PERSON: Mis. 1 duduk + 1 berdiri/berjalan di koridor"),
    }

    print("\n" + "=" * 70)
    print(" 📋 UJI 3: AKURASI KLASIFIKASI AKTIVITAS, ARAH, EGO-MOTION & MULTI-PERSON")
    for k, (act, dr, tag, desc) in scenarios.items():
        print(f"  [{k}] {act:12s} ({dr:18s}) -> {desc}")
    print("=" * 70)

    choice = input("Masukkan pilihan (1-7): ").strip()
    if choice not in scenarios:
        print("❌ Pilihan tidak valid.")
        return

    gt_activity, gt_dir, tag, desc = scenarios[choice]
    exp_fsm_multi = None
    if gt_activity == "MULTI_PERSON":
        gt_activity = input("Aktivitas target kritis yang diharapkan [STANDING/WALKING/SITTING] (default STANDING): ").strip().upper() or "STANDING"
        exp_fsm_multi = input("State FSM yang diharapkan [NORMAL/REDUCED_SPEED/SAFE_STOP] (default SAFE_STOP): ").strip().upper() or "SAFE_STOP"

    duration = input_float("Durasi perekaman (detik)", 5.0)
    steady_start_s = input_float("Mulai hitung fase steady-state dari detik ke-", 1.0)

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_aktivitas_{tag}_{gt_activity.lower()}_{ts_str}.csv")

    countdown(3, desc)
    print(f"🔴 MEREKAM SELAMA {duration:.1f} DETIK (10 Hz)...\n")

    records = []
    t_start = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "mode_tag", "gt_activity", "gt_direction",
            "pred_activity", "pred_direction", "speed_m_s", "robot_v_odom",
            "distance_m", "in_corridor", "n_persons", "corridor_persons",
            "fsm_state", "infer_ms", "fps"
        ])

        while (time.time() - t_start) < duration:
            snap = node.get_snapshot()
            info = snap["info"]
            if snap["info_age"] < 1.0 and info:
                p_act = info.get("activity", "NONE")
                p_dir = info.get("direction", "UNKNOWN")
                p_spd = float(info.get("speed_m_s", 0.0) or 0.0)
                rob_v = float(info.get("robot_v_odom", snap["odom_v"]) or 0.0)
                dist = info.get("distance_m", "")
                in_corr = bool(info.get("target_in_corridor", False))
                n_p = int(info.get("persons", 0))
                n_c = int(info.get("corridor_persons", 0))
                state = snap["state"]
                infer = float(info.get("infer_ms", 0) or 0)
                fps = float(info.get("fps", 0) or 0)
                elapsed = time.time() - t_start

                writer.writerow([
                    round(snap["t"], 3), round(elapsed, 3), tag, gt_activity, gt_dir,
                    p_act, p_dir, p_spd, round(rob_v, 3), dist, in_corr, n_p, n_c,
                    state, infer, fps
                ])
                records.append({
                    "elapsed": elapsed, "act": p_act, "dir": p_dir,
                    "spd": p_spd, "rob_v": rob_v, "state": state,
                    "infer": infer, "fps": fps, "n_p": n_p
                })
                print(
                    f"[{elapsed:4.1f}s] Pred: {p_act:8s} ({p_dir:18s}) | v_h={p_spd:.2f} "
                    f"v_rob={rob_v:.2f} | Dist={str(dist):>5s}m | N={n_p} | FSM={state}"
                )
            time.sleep(0.1)

    if not records:
        print("❌ Tidak ada data dari /human_info! Pastikan human_awareness_node.py aktif.")
        return

    n_total = len(records)
    act_ok = sum(1 for r in records if r["act"] == gt_activity)
    act_acc = (act_ok / n_total) * 100.0

    if gt_activity == "WALKING" and gt_dir != "ANY":
        dir_ok = sum(1 for r in records if r["act"] == gt_activity and r["dir"] == gt_dir)
        dir_acc = (dir_ok / n_total) * 100.0
    else:
        dir_ok, dir_acc = act_ok, act_acc

    steady_recs = [r for r in records if r["elapsed"] >= steady_start_s]
    if steady_recs:
        if gt_activity == "WALKING" and gt_dir != "ANY":
            st_ok = sum(1 for r in steady_recs if r["act"] == gt_activity and r["dir"] == gt_dir)
        else:
            st_ok = sum(1 for r in steady_recs if r["act"] == gt_activity)
        steady_acc = (st_ok / len(steady_recs)) * 100.0
    else:
        st_ok, steady_acc = 0, 0.0

    pred_counts = dict(Counter(f"{r['act']}({r['dir']})" for r in records))
    fsm_counts = dict(Counter(r["state"] for r in records))
    spd_avg = statistics.mean(r["spd"] for r in records)
    spd_max = max(r["spd"] for r in records)
    inf_avg, inf_sd = mean_sd([r["infer"] for r in records])
    fps_avg, _ = mean_sd([r["fps"] for r in records])

    print("\n" + "=" * 70)
    print(f" 📊 HASIL UJI 3 AKTIVITAS [{tag}]: {gt_activity} ({gt_dir})")
    print(f"    Total Sampel                 : {n_total}")
    print(f"    Akurasi Aktivitas            : {act_acc:.1f}% ({act_ok}/{n_total})")
    if gt_activity == "WALKING" and gt_dir != "ANY":
        print(f"    Akurasi Aktivitas + Arah     : {dir_acc:.1f}% ({dir_ok}/{n_total})")
    print(f"    Akurasi Steady-State (>={steady_start_s:.1f}s): {steady_acc:.1f}% ({st_ok}/{len(steady_recs)})")
    if exp_fsm_multi:
        fsm_match = (sum(1 for r in records if r["state"] == exp_fsm_multi) / n_total) * 100.0
        print(f"    Ketepatan FSM ({exp_fsm_multi:13s}): {fsm_match:.1f}%")
    print(f"    Kecepatan Manusia Avg / Maks : {spd_avg:.2f} / {spd_max:.2f} m/s")
    print(f"    Inferensi / FPS              : {inf_avg:.2f} ± {inf_sd:.2f} ms | {fps_avg:.2f} FPS")
    print(f"    Distribusi Prediksi (CM)     : {pred_counts}")
    print(f"    Distribusi State FSM         : {fsm_counts}")
    print(f"    CSV Tersimpan                : {csv_path}")
    print("=" * 70)


# =============================================================================
# UJI 4: STABILITAS FSM (HYSTERESIS, DEBOUNCE & ANTI-CHATTERING)
# =============================================================================
def run_test_4_fsm_stability(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji4_fsm")
    os.makedirs(sub_dir, exist_ok=True)

    modes = {
        "1": ("batas_medium", "Subjek di perbatasan zona MEDIUM (ratio ~0.58-0.62) selama 15s"),
        "2": ("batas_near", "Subjek di perbatasan zona NEAR (ratio ~0.83-0.87) selama 15s"),
        "3": ("flicker_oklusi", "Gangguan flicker / oklusi kamera sesaat (1-3 frame hilang)"),
    }

    print("\n" + "=" * 70)
    print(" ⚖️ UJI 4: STABILITAS FSM (HYSTERESIS & DEBOUNCE / ANTI-CHATTERING)")
    print("    Otomatis membandingkan state FSM AKTUAL vs state MENTAH (tanpa hysteresis/debounce)!")
    for k, (_, desc) in modes.items():
        print(f"  [{k}] {desc}")
    print("=" * 70)

    ch = input("Pilih skenario (1-3): ").strip()
    if ch not in modes:
        print("❌ Pilihan tidak valid.")
        return

    label, desc = modes[ch]
    duration = input_float("Durasi perekaman (detik)", 15.0)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_fsm_chattering_{label}_{ts_str}.csv")

    countdown(3, desc)
    print(f"🔴 MEREKAM SELAMA {duration:.1f} DETIK (20 Hz)...\n")

    fsm_states, raw_states = [], []
    t_start = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "scenario", "ratio_active",
            "t_med", "t_near", "in_corridor", "zone",
            "raw_state_no_hyst", "fsm_state_actual", "human_nav_mode"
        ])
        while (time.time() - t_start) < duration:
            snap = node.get_snapshot()
            info = snap["info"]
            elapsed = time.time() - t_start
            if snap["info_age"] < 1.0 and info:
                ratio = info.get("ratio")
                t_med = float(info.get("t_med", 0.60) or 0.60)
                t_near = float(info.get("t_near", 0.85) or 0.85)
                in_c = bool(info.get("target_in_corridor", False))
                zone = info.get("zone", "NONE")
                st_actual = snap["state"]
                st_raw = raw_state_without_hysteresis(ratio, t_med=t_med, t_near=t_near, in_corridor=in_c)

                writer.writerow([
                    round(snap["t"], 3), round(elapsed, 3), label,
                    ratio if ratio is not None else "",
                    t_med, t_near, in_c, zone, st_raw, st_actual, snap["nav_mode"]
                ])
                fsm_states.append(st_actual)
                raw_states.append(st_raw)
                r_str = f"{ratio:.3f}" if ratio is not None else " None"
                print(f"[{elapsed:5.2f}s] r={r_str} | Zona={zone:6s} | Raw(NoHyst)={st_raw:13s} | FSM_Aktual={st_actual}")
            time.sleep(0.05)

    if not fsm_states:
        print("❌ Tidak ada data terekam!")
        return

    raw_trans = count_state_transitions(raw_states)
    fsm_trans = count_state_transitions(fsm_states)
    reduction_pct = ((raw_trans - fsm_trans) / raw_trans * 100.0) if raw_trans > 0 else 0.0

    print("\n" + "=" * 70)
    print(f" 📊 HASIL UJI 4 STABILITAS FSM: {desc}")
    print(f"    Total Sampel (20 Hz)                 : {len(fsm_states)}")
    print(f"    Transisi Tanpa Hysteresis/Debounce   : {raw_trans} kali")
    print(f"    Transisi Dengan FSM Aktual           : {fsm_trans} kali")
    print(f"    Reduksi Chattering                   : {reduction_pct:.1f}%")
    print(f"    Distribusi State FSM Aktual          : {dict(Counter(fsm_states))}")
    print(f"    CSV Tersimpan                        : {csv_path}")
    print("=" * 70)


# =============================================================================
# UJI 5: RESPONS DINAMIS KECEPATAN & PENGEREMAN ADAPTIF (20 HZ)
# =============================================================================
def run_test_5_dynamic_speed(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji5_respons_dinamis")
    os.makedirs(sub_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print(" 🏎️ UJI 5: RESPONS DINAMIS KECEPATAN & PENGEREMAN ROBOT (20 Hz)")
    print("    Pastikan Arduino, Navigasi/Teleop, & Human Awareness aktif.")
    print("=" * 70)

    rep_label = sanitize_slug(input("Label repetisi (mis. uji_1, uji_2, dst): ").strip() or "uji_1")
    duration = input_float("Durasi perekaman (detik)", 20.0)

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_respons_dinamis_{rep_label}_{ts_str}.csv")

    print(f"\n🔴 MEREKAM SELAMA {duration:.1f} DETIK (Tekan Ctrl+C untuk stop cepat)...\n")
    t_start = time.time()

    reduced_enter_t = None
    reduced_settled_ms = None
    safe_stop_t = None
    brake_cmd_ms = None
    brake_phys_ms = None
    suspend_goal_s = None
    stop_dist_est = None
    stop_clearance_m = None

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_s", "human_state", "human_nav_mode", "activity", "distance_m",
            "cmd_vel_raw", "cmd_vel_safe", "odom_linear_v", "clearance_m"
        ])

        try:
            while (time.time() - t_start) < duration:
                snap = node.get_snapshot()
                elapsed = time.time() - t_start
                state = snap["state"]
                nav_mode = snap["nav_mode"]
                info = snap["info"]
                act = info.get("activity", "NONE")
                dist = info.get("distance_m", "")
                c_in = snap["cmd_in"]
                c_safe = snap["cmd_safe"]
                odom_v = snap["odom_v"]
                clr = snap["clearance_m"]

                writer.writerow([
                    round(elapsed, 3), state, nav_mode, act, dist,
                    round(c_in, 3), round(c_safe, 3), round(odom_v, 3),
                    round(clr, 3) if clr is not None else ""
                ])

                # 1. Ramping NORMAL -> REDUCED_SPEED (mencapai <= 0.205 m/s)
                if state == "REDUCED_SPEED" and reduced_enter_t is None and c_safe > 0.25:
                    reduced_enter_t = snap["t"]
                if reduced_enter_t is not None and reduced_settled_ms is None and abs(c_safe) <= 0.205:
                    reduced_settled_ms = (snap["t"] - reduced_enter_t) * 1000.0

                # 2. Reaksi SAFE_STOP -> cmd_safe=0, odom=0, dan suspend goal (cmd_in=0)
                if state == "SAFE_STOP" and safe_stop_t is None:
                    safe_stop_t = snap["t"]
                if safe_stop_t is not None:
                    if brake_cmd_ms is None and abs(c_safe) < 0.01:
                        brake_cmd_ms = (snap["t"] - safe_stop_t) * 1000.0
                    if brake_phys_ms is None and abs(odom_v) < 0.01:
                        brake_phys_ms = (snap["t"] - safe_stop_t) * 1000.0
                        stop_dist_est = dist
                        stop_clearance_m = clr
                    if suspend_goal_s is None and (snap["t"] - safe_stop_t) > 0.3 and abs(c_in) < 0.01:
                        suspend_goal_s = snap["t"] - safe_stop_t

                print(
                    f"[{elapsed:5.2f}s] {state:13s} ({nav_mode:12s}) | cmd_in:{c_in:5.2f} "
                    f"| cmd_safe:{c_safe:5.2f} | odom:{odom_v:5.2f} m/s | d:{str(dist):>5s}m"
                )
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    actual_meas = input("\nOpsional — masukkan jarak fisik aktual ke manusia saat berhenti (meter, kosongkan jika skip): ").strip()

    print("\n" + "=" * 70)
    print(f" 📊 RINGKASAN UJI 5 RESPONS DINAMIS ({rep_label})")
    print(f"    Durasi Ramping 0.45 -> 0.20 m/s        : {f'{reduced_settled_ms:.1f} ms' if reduced_settled_ms is not None else '-'}")
    print(f"    Waktu Reaksi Rem (SAFE_STOP->cmd=0)    : {f'{brake_cmd_ms:.1f} ms' if brake_cmd_ms is not None else '-'}")
    print(f"    Waktu Henti Fisik (SAFE_STOP->odom=0)  : {f'{brake_phys_ms:.1f} ms' if brake_phys_ms is not None else '-'}")
    print(f"    Delay Suspend Goal Nav2 (cmd_raw=0)    : {f'{suspend_goal_s:.2f} s' if suspend_goal_s is not None else '-'}")
    print(f"    Jarak Estimasi / Clearance saat Stop   : {stop_dist_est} m / {f'{stop_clearance_m:.2f} m' if stop_clearance_m is not None else '-'}")
    if actual_meas:
        print(f"    Jarak Fisik Aktual Meteran             : {actual_meas} m")
    print(f"    CSV Tersimpan                          : {csv_path}")
    print("=" * 70)


# =============================================================================
# UJI 6: INTEGRASI INTERAKSI SOSIAL 4-FASE, KONTEKSTUAL & FAIL-SAFE
# =============================================================================
def run_test_6_social_interaction(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji6_interaksi")
    os.makedirs(sub_dir, exist_ok=True)

    cases = {
        "6a": "STANDING singkat (<5s) lalu minggir -> SAFE_STOP & auto-resume",
        "6b": "STANDING lama (>15s) -> 4-Fase (0s Stop -> 2s Suspend -> 5s Suara -> 15s Bypass -> +8s Normal)",
        "6c": "SITTING (duduk di kursi) di jalur -> tetap NORMAL & langsung bypass DWB",
        "6d": "WALKING SAME_DIRECTION di depan robot -> melambat adaptif tanpa hard-stop",
        "6e": "WALKING OPPOSITE_DIRECTION berpapasan -> REDUCED -> SAFE_STOP -> lanjut",
        "6f": "FAIL-SAFE: Matikan kamera / human_awareness_node saat SAFE_STOP (cek timeout 3.0s)",
        "6g": "FAIL-SAFE: Putus speaker Bluetooth saat navigasi (cek non-blocking)",
    }

    print("\n" + "=" * 70)
    print(" 🤝 UJI 6: INTEGRASI INTERAKSI SOSIAL 4-FASE, KONTEKSTUAL & FAIL-SAFE")
    for k, desc in cases.items():
        print(f"  [{k}] {desc}")
    print("=" * 70)

    code = input("Pilih kode skenario (6a-6g): ").strip().lower()
    if code not in cases:
        print("❌ Kode tidak valid.")
        return

    max_dur = input_float("Durasi maksimum perekaman (detik, bisa stop cepat dgn Ctrl+C)", 45.0)
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_interaksi_{code}_{ts_str}.csv")

    print(f"\n🔴 MEREKAM SKENARIO [{code}]: {cases[code]}")
    print("   Tekan Ctrl+C setelah skenario selesai...\n")

    t_start = time.time()
    events = {
        "t_wait_human": None,
        "t_goal_suspend": None,
        "t_voice_5s_milestone": None,
        "t_bypass_start": None,
        "t_clear_normal": None,
        "t_failsafe_relax": None,
    }
    min_clr = None
    prev_mode = None
    state_counts = Counter()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_s", "human_state", "human_nav_mode", "activity", "direction",
            "distance_m", "cmd_vel_raw", "cmd_vel_safe", "odom_v",
            "clearance_m", "info_age_s", "voice_connected", "voice_queue"
        ])

        try:
            while (time.time() - t_start) < max_dur:
                snap = node.get_snapshot()
                elapsed = time.time() - t_start
                st = snap["state"]
                mode = snap["nav_mode"]
                info = snap["info"]
                act = info.get("activity", "NONE")
                dr = info.get("direction", "UNKNOWN")
                dist = info.get("distance_m", "")
                c_in = snap["cmd_in"]
                c_safe = snap["cmd_safe"]
                odom_v = snap["odom_v"]
                clr = snap["clearance_m"]
                info_age = snap["info_age"]
                v_conn = snap["voice"].get("connected", "")
                v_q = snap["voice"].get("queue_size", 0)

                if clr is not None:
                    min_clr = clr if min_clr is None else min(min_clr, clr)
                state_counts[st] += 1

                writer.writerow([
                    round(elapsed, 3), st, mode, act, dr, dist,
                    round(c_in, 3), round(c_safe, 3), round(odom_v, 3),
                    round(clr, 3) if clr is not None else "",
                    round(info_age, 2), v_conn, v_q
                ])

                # Deteksi otomatis timestamp fase
                if (mode == "WAIT_HUMAN" or st == "SAFE_STOP") and events["t_wait_human"] is None:
                    events["t_wait_human"] = elapsed
                if events["t_wait_human"] is not None:
                    dt_wait = elapsed - events["t_wait_human"]
                    if events["t_goal_suspend"] is None and dt_wait >= 0.5 and abs(c_in) < 0.01:
                        events["t_goal_suspend"] = elapsed
                    if events["t_voice_5s_milestone"] is None and (v_q and int(v_q) > 0):
                        events["t_voice_5s_milestone"] = elapsed
                    if events["t_bypass_start"] is None and mode == "BYPASS_HUMAN":
                        events["t_bypass_start"] = elapsed
                    if events["t_bypass_start"] is not None and events["t_clear_normal"] is None and mode == "NORMAL":
                        events["t_clear_normal"] = elapsed
                    if code == "6f" and events["t_failsafe_relax"] is None and info_age >= 2.8 and mode == "NORMAL":
                        events["t_failsafe_relax"] = elapsed

                if mode != prev_mode:
                    print(f"  >>> [{elapsed:5.2f}s] PERUBAHAN MODE: {prev_mode} -> {mode} (state={st}, act={act})")
                    prev_mode = mode
                elif int(elapsed * 10) % 5 == 0:
                    print(
                        f"[{elapsed:5.2f}s] {st:13s} | {mode:12s} | {act:8s} | d={str(dist):>5s}m "
                        f"| v_safe={c_safe:4.2f} | odom={odom_v:4.2f} | clr={f'{clr:.2f}m' if clr is not None else '-'}"
                    )
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    t0_w = events["t_wait_human"]
    print("\n" + "=" * 70)
    print(f" 📊 TIMELINE VERIFIKASI UJI 6 [{code}]: {cases[code]}")
    if t0_w is not None:
        print(f"    Fase 1 — Masuk WAIT_HUMAN / SAFE_STOP : t = {t0_w:.2f} s (Δ = 0.00 s)")
        if events["t_goal_suspend"] is not None:
            print(f"    Fase 2 — Goal Nav2 Suspended (cmd=0)  : t = {events['t_goal_suspend']:.2f} s (Δ = {events['t_goal_suspend'] - t0_w:.2f} s)")
        if events["t_voice_5s_milestone"] is not None:
            print(f"    Fase 3 — Antrean Suara Terdeteksi     : t = {events['t_voice_5s_milestone']:.2f} s (Δ = {events['t_voice_5s_milestone'] - t0_w:.2f} s)")
        if events["t_bypass_start"] is not None:
            print(f"    Fase 4 — Masuk BYPASS_HUMAN           : t = {events['t_bypass_start']:.2f} s (Δ = {events['t_bypass_start'] - t0_w:.2f} s)")
        if events["t_clear_normal"] is not None and events["t_bypass_start"] is not None:
            print(f"    Fase 5 — Kembali NORMAL pasca-Bypass  : t = {events['t_clear_normal']:.2f} s (durasi bypass = {events['t_clear_normal'] - events['t_bypass_start']:.2f} s)")
        if events["t_failsafe_relax"] is not None:
            print(f"    Fail-Safe Timeout -> NORMAL           : t = {events['t_failsafe_relax']:.2f} s")
    else:
        print("    Tidak ada fase WAIT_HUMAN/SAFE_STOP (sesuai harapan untuk skenario SITTING/NORMAL).")
    print(f"    Minimum Clearance (D_min)             : {f'{min_clr:.3f} m' if min_clr is not None else '-'}")
    print(f"    Distribusi State                      : {dict(state_counts)}")
    print(f"    CSV Tersimpan                         : {csv_path}")
    print("=" * 70)


# =============================================================================
# UJI 7: KOMPARASI END-TO-END (NAV2 BASELINE VS HUMAN-AWARE ADAPTIF)
# =============================================================================
def run_test_7_baseline_vs_adaptive(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji7_komparasi")
    os.makedirs(sub_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print(" 🏆 UJI 7: KOMPARASI NAV2 BASELINE VS HUMAN-AWARE ADAPTIF")
    print("    Merekam trajektori /odom, clearance /scan_filtered, kecepatan, & state FSM (20 Hz).")
    print("=" * 70)

    mode = input("Pilih mode uji [BASELINE / ADAPTIF] (default ADAPTIF): ").strip().upper() or "ADAPTIF"
    if mode not in ("BASELINE", "ADAPTIF"):
        print("❌ Mode harus BASELINE atau ADAPTIF.")
        return

    rep_label = sanitize_slug(input("Label repetisi (mis. uji_1..uji_5): ").strip() or "uji_1")
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"eval_{mode.lower()}_manusia_{rep_label}_{ts_str}.csv")
    rekap_path = os.path.join(sub_dir, "rekap_komparasi_baseline_vs_adaptif.csv")

    input("\nSiap menjalankan misi navigasi? Tekan ENTER tepat saat robot mulai bergerak (Ctrl+C saat tiba)...")
    print(f"🔴 MEREKAM [{mode} - {rep_label}] pada 20 Hz (Tekan Ctrl+C begitu robot sampai tujuan)...\n")

    t_start = time.time()
    path_length = 0.0
    smoothness_rad = 0.0
    min_clr = None
    max_decel = 0.0
    prev_x, prev_y, prev_yaw = None, None, None
    prev_v, prev_t = None, None
    state_counter = Counter()
    n_samples = 0

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_s", "mode", "x_m", "y_m", "yaw_rad",
            "odom_v", "odom_w", "cmd_vel_raw", "cmd_vel_safe",
            "human_state", "human_nav_mode", "min_scan_m", "clearance_m"
        ])
        try:
            while True:
                snap = node.get_snapshot()
                elapsed = time.time() - t_start
                x, y, yaw = snap["odom_x"], snap["odom_y"], snap["odom_yaw"]
                v, w = snap["odom_v"], snap["odom_w"]
                clr = snap["clearance_m"]
                min_s = snap["min_scan_m"]

                if prev_x is not None:
                    step_d = math.hypot(x - prev_x, y - prev_y)
                    if step_d < 0.5:  # lindungi dari jump tak wajar
                        path_length += step_d
                    if step_d > 0.005:
                        smoothness_rad += abs(normalize_angle(yaw - prev_yaw))
                if prev_v is not None and prev_t is not None:
                    dt = snap["t"] - prev_t
                    if dt > 0.01:
                        decel = (prev_v - v) / dt
                        if 0.0 < decel < 5.0:
                            max_decel = max(max_decel, decel)

                prev_x, prev_y, prev_yaw = x, y, yaw
                prev_v, prev_t = v, snap["t"]
                if clr is not None:
                    min_clr = clr if min_clr is None else min(min_clr, clr)
                state_counter[snap["state"]] += 1
                n_samples += 1

                writer.writerow([
                    round(elapsed, 3), mode, round(x, 4), round(y, 4), round(yaw, 4),
                    round(v, 3), round(w, 3), round(snap["cmd_in"], 3), round(snap["cmd_safe"], 3),
                    snap["state"], snap["nav_mode"],
                    round(min_s, 3) if min_s is not None else "",
                    round(clr, 3) if clr is not None else ""
                ])

                if n_samples % 10 == 0:
                    print(
                        f"[{elapsed:5.1f}s] L={path_length:5.2f}m | v={v:4.2f}m/s "
                        f"| D_min={f'{min_clr:.2f}m' if min_clr is not None else '-'} | FSM={snap['state']}"
                    )
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    travel_time = time.time() - t_start
    ok_str = input("\nApakah robot BERHASIL mencapai tujuan? (y/n, default y): ").strip().lower()
    status_misi = "GAGAL" if ok_str == "n" else "SUKSES"
    near_miss_raw = input("Jumlah tabrakan / near-miss selama uji (default 0): ").strip()
    near_miss = int(near_miss_raw) if near_miss_raw.isdigit() else 0

    write_header = not os.path.exists(rekap_path)
    with open(rekap_path, "a", newline="", encoding="utf-8") as rf:
        rw = csv.writer(rf)
        if write_header:
            rw.writerow([
                "timestamp", "mode", "label", "status_misi", "near_miss",
                "d_min_m", "travel_time_s", "path_length_m", "path_smoothness_rad",
                "max_decel_m_s2", "csv_file"
            ])
        rw.writerow([
            ts_str, mode, rep_label, status_misi, near_miss,
            round(min_clr, 3) if min_clr is not None else "",
            round(travel_time, 2), round(path_length, 3),
            round(smoothness_rad, 3), round(max_decel, 3), os.path.basename(csv_path)
        ])

    print("\n" + "=" * 70)
    print(f" 📊 HASIL UJI 7 [{mode} — {rep_label}]")
    print(f"    Status Misi               : {status_misi} (Tabrakan/Near-Miss: {near_miss})")
    print(f"    Travel Time (T)           : {travel_time:.2f} s")
    print(f"    Path Length (L)           : {path_length:.3f} m")
    print(f"    Minimum Clearance (D_min) : {f'{min_clr:.3f} m' if min_clr is not None else '-'}")
    print(f"    Path Smoothness           : {smoothness_rad:.3f} rad")
    print(f"    Distribusi State FSM      : {dict(state_counter)}")
    print(f"    CSV Detail                : {csv_path}")
    print(f"    Ditambahkan ke Rekap      : {rekap_path}")
    print("=" * 70)


# =============================================================================
# UJI 8: PERFORMA KOMPUTASI EDGE AI & ROBUSTNESS PENCAHAYAAN
# =============================================================================
def run_test_8_edge_and_lighting(node, out_dir: str):
    sub_dir = os.path.join(out_dir, "uji8_edge_ai")
    os.makedirs(sub_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print(" ⚡ UJI 8: PERFORMA KOMPUTASI EDGE AI & ROBUSTNESS PENCAHAYAAN")
    print("=" * 70)
    cond = sanitize_slug(input("Kondisi pencahayaan (mis. terang / redup / backlight): ").strip() or "terang")
    lux_str = input("Intensitas cahaya dalam Lux (opsional, tekan ENTER untuk skip): ").strip()
    gt_act = input("Aktivitas subjek di depan kamera [STANDING/SITTING/WALKING] (default STANDING): ").strip().upper() or "STANDING"
    duration = input_float("Durasi perekaman (detik)", 10.0)

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(sub_dir, f"uji_edge_pencahayaan_{cond}_{ts_str}.csv")

    countdown(3, f"Kondisi pencahayaan '{cond}' — subjek memperagakan {gt_act}")
    print(f"🔴 MEREKAM SELAMA {duration:.1f} DETIK (10 Hz)...\n")

    infers, fps_list, confs = [], [], []
    detected_cnt, act_ok_cnt, total_cnt = 0, 0, 0
    t_start = time.time()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "elapsed_s", "lighting_cond", "lux", "gt_activity",
            "human_detected", "confidence", "pred_activity", "distance_m",
            "infer_ms", "fps"
        ])
        while (time.time() - t_start) < duration:
            snap = node.get_snapshot()
            info = snap["info"]
            elapsed = time.time() - t_start
            if snap["info_age"] < 1.0 and info:
                total_cnt += 1
                det = int(info.get("persons", 0)) > 0
                conf = info.get("confidence")
                act = info.get("activity", "NONE")
                dist = info.get("distance_m", "")
                inf_ms = float(info.get("infer_ms", 0.0) or 0.0)
                fps_v = float(info.get("fps", 0.0) or 0.0)

                if det:
                    detected_cnt += 1
                if conf is not None:
                    confs.append(float(conf))
                if act == gt_act:
                    act_ok_cnt += 1
                infers.append(inf_ms)
                fps_list.append(fps_v)

                writer.writerow([
                    round(snap["t"], 3), round(elapsed, 3), cond, lux_str, gt_act,
                    det, conf if conf is not None else "", act, dist, inf_ms, fps_v
                ])
                print(
                    f"[{elapsed:4.1f}s] Det={str(det):5s} | Conf={str(conf):>5s} | Act={act:8s} "
                    f"| Infer={inf_ms:4.0f}ms | FPS={fps_v:4.1f}"
                )
            time.sleep(0.1)

    if total_cnt == 0:
        print("❌ Tidak ada data dari /human_info!")
        return

    inf_avg, inf_sd = mean_sd(infers)
    fps_avg, fps_sd = mean_sd(fps_list)
    conf_avg, _ = mean_sd(confs)
    det_rate = (detected_cnt / total_cnt) * 100.0
    act_acc = (act_ok_cnt / total_cnt) * 100.0

    print("\n" + "=" * 70)
    print(f" 📊 HASIL UJI 8 EDGE AI & PENCAHAYAAN ({cond})")
    print(f"    Total Sampel                : {total_cnt}")
    print(f"    Detection Rate              : {det_rate:.1f}% ({detected_cnt}/{total_cnt})")
    print(f"    Confidence Rata-rata        : {conf_avg:.3f}")
    print(f"    Akurasi Aktivitas ({gt_act:8s}): {act_acc:.1f}%")
    print(f"    Inferensi Mean ± SD (Min-Mx): {inf_avg:.2f} ± {inf_sd:.2f} ms ({min(infers):.0f}–{max(infers):.0f} ms)")
    print(f"    FPS Mean ± SD (Min-Mx)      : {fps_avg:.2f} ± {fps_sd:.2f} FPS ({min(fps_list):.1f}–{max(fps_list):.1f} FPS)")
    print(f"    CSV Tersimpan               : {csv_path}")
    print("=" * 70)


# =============================================================================
# MENU 9: DIAGNOSTIK STATUS TOPIK ROS 2 PRA-UJI
# =============================================================================
def run_diagnostics(node):
    snap = node.get_snapshot()
    ages = snap["ages"]

    def fmt(age):
        return f"🟢 AKTIF ({age:.2f}s lalu)" if age < 3.0 else "🔴 TIDAK ADA DATA"

    print("\n" + "=" * 70)
    print(" 🩺 DIAGNOSTIK STATUS TOPIK ROS 2 PRA-UJI")
    print("=" * 70)
    print(f"  /human_info        : {fmt(ages['info'])} | act={snap['info'].get('activity', '-')}, d={snap['info'].get('distance_m', '-')}, FPS={snap['info'].get('fps', '-')}")
    print(f"  /human_state       : {fmt(ages['state'])} | val={snap['state']}")
    print(f"  /human_status      : {fmt(ages['status'])} | val={snap['status']}")
    print(f"  /human_nav_mode    : {fmt(ages['nav_mode'])} | val={snap['nav_mode']}")
    print(f"  /cmd_vel           : {fmt(ages['cmd_in'])} | v={snap['cmd_in']:.2f}, w={snap['cmd_w_in']:.2f}")
    print(f"  /cmd_vel_safe      : {fmt(ages['cmd_safe'])} | v={snap['cmd_safe']:.2f}, w={snap['cmd_w_safe']:.2f}")
    print(f"  /odom              : {fmt(ages['odom'])} | v={snap['odom_v']:.2f} m/s, pos=({snap['odom_x']:.2f}, {snap['odom_y']:.2f})")
    clr_str = f"{snap['clearance_m']:.2f} m" if snap['clearance_m'] is not None else "-"
    print(f"  /scan_filtered     : {fmt(ages['scan'])} | D_min_clearance={clr_str}")
    print(f"  /robot_voice/status: {fmt(ages['voice'])} | connected={snap['voice'].get('connected', '-')}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Suite Pengujian Skripsi Human-Aware Behavior AMR (Uji 1-8)")
    parser.add_argument("--out-dir", default=None, help="Direktori output CSV (default: <ws>/hasil_pengujian_behavior)")
    args, _ = parser.parse_known_args()

    if not HAS_ROS2:
        print("❌ ROS 2 (rclpy) tidak tersedia di environment ini. Jalankan di lingkungan ROS 2 aktif.")
        sys.exit(1)

    rclpy.init()
    node = BehaviorTestNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    out_dir = resolve_output_dir(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n📁 Direktori penyimpanan hasil uji: {out_dir}")

    try:
        while rclpy.ok():
            print("\n" + "=" * 70)
            print(" 🧪 SUITE PENGUJIAN SKRIPSI HUMAN-AWARE BEHAVIOR AMR (UJI 1 – 8)")
            print("=" * 70)
            print("  [1] Uji 1 : Akurasi Estimasi Jarak Monokuler & Variasi Tinggi Badan")
            print("  [2] Uji 2 : Selektivitas Filter Koridor Tengah (In vs Outside)")
            print("  [3] Uji 3 : Akurasi Klasifikasi Aktivitas, Arah, Ego-Motion & Multi-Person")
            print("  [4] Uji 4 : Stabilitas FSM (Hysteresis & Debounce / Anti-Chattering)")
            print("  [5] Uji 5 : Respons Dinamis Kecepatan & Pengereman Adaptif (20 Hz)")
            print("  [6] Uji 6 : Integrasi Interaksi Sosial 4-Fase, Kontekstual & Fail-Safe")
            print("  [7] Uji 7 : Komparasi End-to-End (Nav2 Baseline vs Human-Aware Adaptif)")
            print("  [8] Uji 8 : Performa Komputasi Edge AI (FPS/Inferensi) & Pencahayaan")
            print("  [9] Diagnostik Live Status Topik ROS 2 Pra-Uji")
            print("  [0] Keluar")
            print("=" * 70)

            menu = input("Pilih mode pengujian (0-9): ").strip()
            if menu == "1":
                run_test_1_distance_accuracy(node, out_dir)
            elif menu == "2":
                run_test_2_corridor_selectivity(node, out_dir)
            elif menu == "3":
                run_test_3_activity_accuracy(node, out_dir)
            elif menu == "4":
                run_test_4_fsm_stability(node, out_dir)
            elif menu == "5":
                run_test_5_dynamic_speed(node, out_dir)
            elif menu == "6":
                run_test_6_social_interaction(node, out_dir)
            elif menu == "7":
                run_test_7_baseline_vs_adaptive(node, out_dir)
            elif menu == "8":
                run_test_8_edge_and_lighting(node, out_dir)
            elif menu == "9":
                run_diagnostics(node)
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
