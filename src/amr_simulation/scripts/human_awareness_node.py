#!/usr/bin/env python3
"""
human_awareness_node.py

Human-Aware Behavior AMR:
Camera ROS 2 / local camera / video
    -> YOLO11n person detection
    -> Bounding box feature
    -> Distance estimation
    -> Proximity classifier
    -> FSM
    -> ROS 2 /human_state  (+ /human_info JSON untuk keterangan dashboard)

Mode kamera ROS:
  --source ros
  --image-topic /camera/image_raw/compressed
"""

import argparse
import csv
import json
import sys
import time
import threading
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

DEFAULT_MODEL = str(SCRIPT_DIR / "yolo11n.pt") if (SCRIPT_DIR / "yolo11n.pt").exists() else "yolo11n.pt"

from fsm import HumanAwareFSM, ProximityClassifier, Zone, State


# ============================================================
# VISUAL SETTINGS
# ============================================================

GREEN = (80, 200, 80)
YELLOW = (0, 200, 255)
RED = (0, 0, 255)
GRAY = (170, 170, 170)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

FONT = cv2.FONT_HERSHEY_SIMPLEX

STATE_COLORS = {
    State.NORMAL: GREEN,
    State.REDUCED_SPEED: YELLOW,
    State.SAFE_STOP: RED,
}

ZONE_COLORS = {
    Zone.FAR: GREEN,
    Zone.MEDIUM: YELLOW,
    Zone.NEAR: RED,
    Zone.NONE: GRAY,
}

STATE_RESPONSE = {
    State.NORMAL: "Operasi normal",
    State.REDUCED_SPEED: "Kecepatan dikurangi",
    State.SAFE_STOP: "Robot berhenti (Safe Stop)",
}

FEAT_NAME = {
    "w": "LEBAR (w/W)",
    "h": "TINGGI (h/H)",
}

DEFAULT_CALIB_H = "2.0:0.80,3.0:0.55,4.0:0.42"
DEFAULT_CALIB_W = "1.0:0.55,2.0:0.28,3.0:0.19"


# ============================================================
# DISTANCE ESTIMATOR
# ============================================================

class DistanceEstimator:
    def __init__(self, calib_str: str, alpha: float = 0.3):
        pairs = []

        for item in calib_str.split(","):
            d_str, r_str = item.strip().split(":")
            distance = float(d_str)
            ratio = float(r_str)

            if distance <= 0 or ratio <= 0:
                raise ValueError("Jarak dan rasio harus > 0")

            pairs.append((distance, ratio))

        if not pairs:
            raise ValueError("Data kalibrasi kosong")

        distances = np.array([p[0] for p in pairs])
        inverse_ratios = np.array([1.0 / p[1] for p in pairs])

        if len(pairs) == 1:
            self.a = float(distances[0] / inverse_ratios[0])
            self.b = 0.0
        else:
            self.a, self.b = (
                float(v)
                for v in np.polyfit(inverse_ratios, distances, 1)
            )

        ratios = [p[1] for p in pairs]
        self.r_min = min(ratios)
        self.r_max = max(ratios)

        predicted = self.a * inverse_ratios + self.b
        self.rmse = float(
            np.sqrt(np.mean((predicted - distances) ** 2))
        )

        self.n_points = len(pairs)
        self.alpha = alpha
        self._ema = None

    def estimate(self, ratio: float):
        distance = max(
            0.1,
            self.a * (1.0 / max(ratio, 1e-6)) + self.b
        )

        in_range = (
            self.r_min * 0.85
            <= ratio
            <= self.r_max * 1.15
        )

        return distance, in_range

    def smooth(self, distance: float) -> float:
        if self._ema is None:
            self._ema = distance
        else:
            self._ema = (
                self.alpha * distance
                + (1 - self.alpha) * self._ema
            )

        return self._ema

    def reset(self):
        self._ema = None


# ============================================================
# PERSON DETECTION HELPERS
# ============================================================

def extract_persons(result, frame_w, frame_h, edge_margin=6):
    persons = []

    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)

        cut_w = (
            x1 <= edge_margin
            or x2 >= frame_w - edge_margin
        )

        cut_h = (
            y1 <= edge_margin
            or y2 >= frame_h - edge_margin
        )

        persons.append({
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "w": width,
            "h": height,
            "ratio_w": width / frame_w,
            "ratio_h": height / frame_h,
            "conf": float(box.conf[0]),
            "cut_w": cut_w,
            "cut_h": cut_h,
            "at_edge": cut_w or cut_h,
        })

    return persons


def pick_feature(person, mode):
    if mode == "width":
        return "w"

    if mode == "height":
        return "h"

    # Mode auto: gunakan tinggi selama bbox tidak terpotong
    if not person["cut_h"]:
        return "h"

    return "w"


# ============================================================
# DRAWING HELPERS
# ============================================================

def put_text(img, text, org, scale=0.6, color=WHITE, thick=1):
    cv2.putText(
        img, text, org, FONT, scale,
        BLACK, thick + 2, cv2.LINE_AA
    )
    cv2.putText(
        img, text, org, FONT, scale,
        color, thick, cv2.LINE_AA
    )


def text_size(text, scale, thick=1):
    (width, height), base = cv2.getTextSize(
        text, FONT, scale, thick
    )
    return width, height, base


def draw_panel(img, x, y, width, height, alpha=0.55):
    x0, y0 = max(0, x), max(0, y)
    x1 = min(img.shape[1], x + width)
    y1 = min(img.shape[0], y + height)

    if x1 <= x0 or y1 <= y0:
        return

    sub = img[y0:y1, x0:x1]
    cv2.addWeighted(
        np.zeros_like(sub), alpha,
        sub, 1 - alpha, 0, sub
    )


def draw_person(
    frame, person, is_target, zone,
    distance, in_range, calibrated, scale
):
    x1 = int(person["x1"])
    y1 = int(person["y1"])
    x2 = int(person["x2"])
    y2 = int(person["y2"])

    mark = "" if calibrated else "*"
    approx = "~" if in_range else "~?"

    distance_text = (
        f"{approx}{distance:.1f} m{mark}"
        if distance is not None
        else "-"
    )

    feature_cut = (
        person["cut_h"]
        if person["feat"] == "h"
        else person["cut_w"]
    )

    if is_target:
        color = ZONE_COLORS.get(zone, WHITE)

        cv2.rectangle(
            frame, (x1, y1), (x2, y2),
            color, max(2, int(3 * scale))
        )

        label = (
            f"TARGET [{person['feat'].upper()}] "
            f"{distance_text}"
        )

        label_scale = 0.65 * scale
        tw, th, base = text_size(label, label_scale, 2)

        top = max(0, y1 - th - base - 8)

        cv2.rectangle(
            frame,
            (x1, top),
            (x1 + tw + 10, top + th + base + 8),
            color,
            -1
        )

        cv2.putText(
            frame,
            label,
            (x1 + 5, top + th + 3),
            FONT,
            label_scale,
            BLACK,
            2,
            cv2.LINE_AA
        )

        if feature_cut:
            put_text(
                frame,
                "Tepi frame: jarak kurang andal",
                (x1, min(frame.shape[0] - 5, y2 + int(18 * scale))),
                0.5 * scale,
                YELLOW
            )

    else:
        cv2.rectangle(
            frame, (x1, y1), (x2, y2), GRAY, 1
        )

        put_text(
            frame,
            distance_text,
            (x1 + 3, max(14, y1 - 6)),
            0.5 * scale,
            GRAY
        )


def draw_top_bar(frame, state, zone, scale):
    width = frame.shape[1]
    height = int(46 * scale)

    color = STATE_COLORS.get(state, GRAY)

    cv2.rectangle(
        frame, (0, 0), (width, height), color, -1
    )

    cv2.putText(
        frame,
        f"STATE: {state.value}",
        (int(12 * scale), int(30 * scale)),
        FONT,
        0.85 * scale,
        BLACK,
        2,
        cv2.LINE_AA
    )

    zone_text = f"ZONA: {zone.value}"
    tw, _, _ = text_size(zone_text, 0.7 * scale, 2)

    cv2.putText(
        frame,
        zone_text,
        (width - tw - int(12 * scale), int(30 * scale)),
        FONT,
        0.7 * scale,
        BLACK,
        2,
        cv2.LINE_AA
    )

    response = STATE_RESPONSE.get(state, "")
    if response:
        rw, _, _ = text_size(response, 0.55 * scale, 1)

        cv2.putText(
            frame,
            response,
            ((width - rw) // 2, int(30 * scale)),
            FONT,
            0.55 * scale,
            BLACK,
            1,
            cv2.LINE_AA
        )

    return height


def draw_info_panel(frame, top, rows, scale):
    line_h = int(24 * scale)
    pad = int(8 * scale)
    width = int(390 * scale)
    height = pad * 2 + line_h * len(rows)

    x = int(8 * scale)
    y = top + int(8 * scale)

    draw_panel(frame, x, y, width, height)

    for i, (label, value, value_color) in enumerate(rows):
        ty = y + pad + line_h * (i + 1) - int(6 * scale)

        put_text(
            frame, label,
            (x + pad, ty),
            0.5 * scale, GRAY
        )

        put_text(
            frame, value,
            (x + int(180 * scale), ty),
            0.55 * scale, value_color, 1
        )


def draw_proximity_bar(frame, ratio, t_medium, t_near, scale):
    height, width = frame.shape[:2]

    bar_h = int(20 * scale)
    margin = int(12 * scale)

    y = height - margin - bar_h
    x0, x1 = margin, width - margin
    bar_width = x1 - x0

    bar_max = min(1.0, t_near * 1.5)

    def px(value):
        return x0 + int(
            bar_width
            * min(value, bar_max)
            / max(bar_max, 1e-4)
        )

    xm = px(t_medium)
    xn = px(t_near)

    cv2.rectangle(frame, (x0, y), (xm, y + bar_h), GREEN, -1)
    cv2.rectangle(frame, (xm, y), (xn, y + bar_h), YELLOW, -1)
    cv2.rectangle(frame, (xn, y), (x1, y + bar_h), RED, -1)
    cv2.rectangle(frame, (x0, y), (x1, y + bar_h), BLACK, 1)

    for name, left, right in (
        ("FAR", x0, xm),
        ("MEDIUM", xm, xn),
        ("NEAR", xn, x1),
    ):
        tw, th, _ = text_size(name, 0.45 * scale, 1)

        cv2.putText(
            frame,
            name,
            (left + (right - left - tw) // 2,
             y + (bar_h + th) // 2),
            FONT,
            0.45 * scale,
            BLACK,
            1,
            cv2.LINE_AA
        )

    put_text(
        frame, f"T_med={t_medium:.2f}",
        (xm - int(35 * scale), y - int(6 * scale)),
        0.4 * scale, WHITE
    )

    put_text(
        frame, f"T_near={t_near:.2f}",
        (xn - int(35 * scale), y - int(6 * scale)),
        0.4 * scale, WHITE
    )

    if ratio is not None:
        marker_x = px(ratio)

        triangle = np.array([
            [marker_x, y - 2],
            [marker_x - int(7 * scale), y - int(14 * scale)],
            [marker_x + int(7 * scale), y - int(14 * scale)],
        ])

        cv2.fillPoly(frame, [triangle], WHITE)
        cv2.polylines(
            frame, [triangle], True, BLACK, 1, cv2.LINE_AA
        )


# ============================================================
# MAIN NODE
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Human-Aware AMR ROS 2"
    )

    parser.add_argument(
        "--source", default="0",
        help="'ros' untuk kamera ROS, '0' untuk kamera lokal, atau path video"
    )
    parser.add_argument(
        "--image-topic",
        default="/camera/image_raw/compressed",
        help="Topik kamera bertipe CompressedImage"
    )
    parser.add_argument(
        "--model", default=DEFAULT_MODEL,
        help="Model YOLO .pt atau TensorRT .engine"
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument(
        "--corridor-width", type=float, default=0.30,
        help="Lebar koridor tengah sebagai proporsi lebar frame (0.1-1.0); default 0.30"
    )

    parser.add_argument(
        "--feature",
        choices=["height", "width", "auto"],
        default="height"
    )

    parser.add_argument("--t-near-enter", type=float, default=0.95)
    parser.add_argument("--t-near-exit", type=float, default=0.90)
    parser.add_argument("--t-med-enter", type=float, default=0.85)
    parser.add_argument("--t-med-exit", type=float, default=0.70)
    parser.add_argument("--debounce", type=int, default=8)

    parser.add_argument("--calib-h", default=DEFAULT_CALIB_H)
    parser.add_argument("--calib-w", default=DEFAULT_CALIB_W)
    parser.add_argument("--smooth", type=float, default=0.3)

    parser.add_argument("--true-distance", type=float, default=None)
    parser.add_argument("--log", default="experiment_log.csv")
    parser.add_argument("--save-video", default=None)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--publish-annotated", action="store_true", default=False,
        help="(Debug) Publish frame ber-bbox/overlay ke ROS 2. "
             "Default MATI: dashboard cukup pakai frame kamera asli "
             "+ teks di --info-topic"
    )
    parser.add_argument(
        "--info-topic", default="/human_info",
        help="Topik std_msgs/String berisi JSON hasil human-aware "
             "(untuk keterangan di bawah frame dashboard)"
    )
    parser.add_argument(
        "--annotated-topic",
        default="/camera/image_annotated/compressed",
        help="Topik output frame ter-anotasi (CompressedImage)"
    )

    args, _ = parser.parse_known_args()

    if not 0.1 <= args.corridor_width <= 1.0:
        parser.error("--corridor-width harus berada pada rentang 0.1 sampai 1.0")

    # --------------------------------------------------------
    # ROS INITIALIZATION
    # --------------------------------------------------------

    rclpy.init()
    ros_node = Node("human_awareness_node")

    state_pub = ros_node.create_publisher(
        String, "/human_state", 10
    )
    status_pub = ros_node.create_publisher(
        String, "/human_status", 10
    )

    info_pub = ros_node.create_publisher(
        String, args.info_topic, 10
    )

    annotated_pub = None
    annotated_raw_pub = None
    if args.publish_annotated:
        annotated_pub = ros_node.create_publisher(
            CompressedImage, args.annotated_topic, 1
        )
        annotated_raw_pub = ros_node.create_publisher(
            Image, "/camera/image_annotated", 1
        )

    use_ros_camera = args.source.lower() in ("ros", "topic")
    # Callback kamera hanya MENYIMPAN frame JPEG terbaru (murah).
    # Decode + YOLO dilakukan di loop utama hanya untuk frame terbaru,
    # jadi frame lama otomatis dilewati dan tidak menumpuk (anti-lag).
    latest = {"data": None, "seq": 0}
    frame_lock = threading.Lock()
    executor = None
    spin_thread = None
    camera_sub = None
    cap = None

    # Latest-frame QoS: hindari antrean gambar lama
    camera_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT
    )

    if use_ros_camera:
        def camera_callback(msg):
            if not msg.format.lower().startswith("jpeg"):
                ros_node.get_logger().warning(
                    f"Format compressed tidak dikenal: {msg.format}"
                )
                return

            with frame_lock:
                latest["data"] = msg.data
                latest["seq"] += 1

        camera_sub = ros_node.create_subscription(
            CompressedImage,
            args.image_topic,
            camera_callback,
            camera_qos
        )

        ros_node.get_logger().info(
            f"Menunggu kamera pada topik: {args.image_topic}"
        )

        # Spin ROS di thread sendiri supaya callback kamera tidak
        # tertahan oleh inferensi YOLO di loop utama.
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        spin_thread = threading.Thread(
            target=executor.spin, daemon=True
        )
        spin_thread.start()

    else:
        source = (
            int(args.source)
            if args.source.isdigit()
            else args.source
        )

        if isinstance(source, int):
            if sys.platform.startswith("linux"):
                cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            elif sys.platform.startswith("win"):
                cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
            else:
                cap = cv2.VideoCapture(source)
        else:
            cap = cv2.VideoCapture(source)

        if not cap.isOpened():
            ros_node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError(
                f"Gagal membuka sumber gambar: {source}"
            )

    # --------------------------------------------------------
    # MODEL AND ALGORITHM INITIALIZATION
    # --------------------------------------------------------

    try:
        est_h = DistanceEstimator(
            args.calib_h, alpha=args.smooth
        )
        est_w = DistanceEstimator(
            args.calib_w, alpha=args.smooth
        )

        estimators = {"h": est_h, "w": est_w}

        calibrated_by = {
            "h": args.calib_h != DEFAULT_CALIB_H,
            "w": args.calib_w != DEFAULT_CALIB_W,
        }

        classifier = ProximityClassifier(
            t_near_enter=args.t_near_enter,
            t_near_exit=args.t_near_exit,
            t_med_enter=args.t_med_enter,
            t_med_exit=args.t_med_exit,
        )

        fsm = HumanAwareFSM(
            debounce_frames=args.debounce
        )

        model = YOLO(args.model)

        person_ids = [
            class_id
            for class_id, name in model.names.items()
            if name == "person"
        ]

        if not person_ids:
            raise RuntimeError(
                "Class 'person' tidak ditemukan pada model."
            )

        person_id = person_ids[0]

        # ----------------------------------------------------
        # CSV LOGGING
        # ----------------------------------------------------

        log_path = Path(args.log)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        log_file = open(
            log_path, "w", newline="", encoding="utf-8"
        )

        writer = csv.writer(log_file)
        writer.writerow([
            "frame", "timestamp", "n_persons", "human_detected",
            "bbox_w_px", "bbox_h_px", "ratio_w", "ratio_h",
            "active_feature", "distance_est_m", "at_edge",
            "confidence", "zone", "state", "infer_ms", "fps",
            "true_distance_m"
        ])

    except Exception:
        if cap is not None:
            cap.release()
        ros_node.destroy_node()
        rclpy.shutdown()
        raise

    # --------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------

    video_writer = None
    frame_count = 0
    miss_count = 0
    fps_ema = None
    last_feat = None
    last_seq = -1
    t_start = time.time()
    last_no_frame_log = 0.0

    ros_node.get_logger().info(
        "Human-Aware berjalan. Output: /human_state"
    )

    try:
        while rclpy.ok():
            t_frame = time.perf_counter()

            if use_ros_camera:
                with frame_lock:
                    seq = latest["seq"]
                    data = latest["data"]

                # Tidak ada frame baru -> tunggu, jangan proses ulang
                # frame yang sama.
                if data is None or seq == last_seq:
                    now = time.time()

                    if (
                        data is None
                        and now - last_no_frame_log >= 3.0
                    ):
                        ros_node.get_logger().info(
                            f"Menunggu frame dari {args.image_topic}..."
                        )
                        last_no_frame_log = now

                    time.sleep(0.003)
                    continue

                last_seq = seq

                frame = cv2.imdecode(
                    np.frombuffer(data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )

                if frame is None:
                    continue

            else:
                ret, frame = cap.read()

                if not ret:
                    ros_node.get_logger().warning(
                        "Sumber gambar berhenti atau frame gagal dibaca."
                    )
                    break

            frame_count += 1
            height, width = frame.shape[:2]
            scale = float(
                np.clip(width / 960.0, 0.6, 1.4)
            )

            # 1. YOLO PERSON DETECTION
            t0 = time.perf_counter()

            result = model(
                frame,
                verbose=False,
                conf=args.conf,
                imgsz=args.imgsz,
                classes=[person_id],
            )[0]

            infer_ms = (
                time.perf_counter() - t0
            ) * 1000.0

            # 2. BOUNDING BOX FEATURES
            persons = extract_persons(
                result, width, height
            )

            for person in persons:
                person["feat"] = pick_feature(
                    person, args.feature
                )

                person["r"] = (
                    person["ratio_h"]
                    if person["feat"] == "h"
                    else person["ratio_w"]
                )

                person["dist"], person["in_range"] = (
                    estimators[person["feat"]].estimate(
                        person["r"]
                    )
                )

            # 3. CENTRAL CORRIDOR + TARGET SELECTION
            # Koridor tengah menjadi proxy area lintasan AMR.
            # Manusia dianggap berada di koridor jika minimal 60%
            # lebar bounding box-nya berada di dalam koridor.

            corridor_half = (width * args.corridor_width) / 2.0
            corridor_x1 = int((width / 2.0) - corridor_half)
            corridor_x2 = int((width / 2.0) + corridor_half)

            corridor_persons = []

            for person in persons:
                # Hitung lebar bounding box yang beririsan
                # dengan koridor tengah.
                overlap_left = max(
                    person["x1"], corridor_x1
                )
                overlap_right = min(
                    person["x2"], corridor_x2
                )

                overlap_width = max(
                    0.0, overlap_right - overlap_left
                )

                # Proporsi bbox yang berada di dalam koridor.
                person["corridor_overlap"] = (
                    overlap_width / max(person["w"], 1.0)
                )

                # Minimal 60% bbox harus berada di koridor.
                person["in_corridor"] = (
                    person["corridor_overlap"] >= 0.60
                )

                if person["in_corridor"]:
                    corridor_persons.append(person)

            # Pilih target terdekat hanya dari manusia yang
            # memenuhi syarat koridor.
            target = None

            if corridor_persons:
                if args.feature == "height":
                    target = max(
                        corridor_persons,
                        key=lambda p: p["h"]
                    )

                elif args.feature == "width":
                    target = max(
                        corridor_persons,
                        key=lambda p: p["w"]
                    )

                else:
                    target = max(
                        corridor_persons,
                        key=lambda p: p["w"] * p["h"]
                    )

            # 4. PROXIMITY AND DISTANCE SMOOTHING
            force_near = False

            if target is None:
                miss_count += 1

                if miss_count >= 15:
                    est_h.reset()
                    est_w.reset()
                    last_feat = None

                active_ratio = None
                dist_smooth = None

            else:
                miss_count = 0
                feature = target["feat"]
                active_ratio = target["r"]

                if (
                    args.feature == "auto"
                    and target["cut_w"]
                    and target["cut_h"]
                ):
                    force_near = True

                if (
                    last_feat is not None
                    and feature != last_feat
                    and estimators[last_feat]._ema is not None
                ):
                    estimators[feature]._ema = (
                        estimators[last_feat]._ema
                    )

                last_feat = feature

                dist_smooth = estimators[feature].smooth(
                    target["dist"]
                )

                target["dist"] = dist_smooth

            # 5. PROXIMITY CLASSIFICATION AND FSM
            # SAFE_STOP berlaku di mana pun: cek semua manusia untuk zona NEAR.
            # REDUCED_SPEED hanya boleh dipicu target yang masuk koridor.
            near_person_outside_or_inside = any(
                person["r"] >= args.t_near_enter
                for person in persons
            )

            if near_person_outside_or_inside or force_near:
                zone = classifier.update(
                    active_ratio,
                    force_near=True
                )
            elif target is None:
                # Tidak ada manusia di koridor dan tidak ada manusia NEAR.
                # Jangan biarkan MEDIUM lama bertahan hanya karena hysteresis.
                zone = Zone.FAR
            else:
                # Target ada di koridor: zona FAR/MEDIUM/NEAR normal dari classifier.
                zone = classifier.update(
                    active_ratio,
                    force_near=False
                )

            state = fsm.update(zone)

            # 6. PUBLISH STATE ONCE PER FRAME
            state_msg = String()
            state_msg.data = state.value
            state_pub.publish(state_msg)

            # Publish juga ke /human_status untuk dashboard AMR
            status_msg = String()
            if target is not None:
                status_msg.data = state.value
            elif persons:
                status_msg.data = "HUMAN_OUTSIDE_CORRIDOR"
            else:
                status_msg.data = "NO_HUMAN"
            status_pub.publish(status_msg)

            # 7. FPS
            inst_fps = 1.0 / max(
                time.perf_counter() - t_frame,
                1e-6
            )

            fps_ema = (
                inst_fps
                if fps_ema is None
                else 0.9 * fps_ema + 0.1 * inst_fps
            )

            # 7b. PUBLISH HASIL SEBAGAI TEKS (untuk keterangan dashboard & canvas overlay)
            boxes_data = []
            for p in persons:
                is_tgt = bool(target is not None and p is target)
                boxes_data.append({
                    "x1": round(float(p["x1"]), 1),
                    "y1": round(float(p["y1"]), 1),
                    "x2": round(float(p["x2"]), 1),
                    "y2": round(float(p["y2"]), 1),
                    "conf": round(float(p["conf"]), 2),
                    "is_target": is_tgt,
                    "dist": round(float(dist_smooth), 2) if (is_tgt and dist_smooth is not None) else None,
                    "zone": zone.value if is_tgt else "NONE"
                })

            info = {
                "stamp": round(time.time(), 3),
                "frame": frame_count,
                "frame_width": width,
                "frame_height": height,
                "boxes": boxes_data,
                "state": state.value,
                "zone": zone.value,
                "response": STATE_RESPONSE.get(state, ""),
                "persons": len(persons),
                "corridor_persons": len(corridor_persons),
                "corridor_width_ratio": args.corridor_width,
                "target_in_corridor": bool(target is not None),
                "distance_m": None,
                "in_range": None,
                "calibrated": None,
                "feature": None,
                "feature_name": None,
                "ratio": None,
                "ratio_h": None,
                "ratio_w": None,
                "bbox_h_px": None,
                "bbox_w_px": None,
                "confidence": None,
                "at_edge": None,
                "fps": round(fps_ema, 2),
                "infer_ms": round(infer_ms),
                "t_med": args.t_med_enter,
                "t_near": args.t_near_enter,
            }

            if target is not None:
                feat = target["feat"]
                info.update({
                    "distance_m": round(dist_smooth, 2),
                    "in_range": bool(target["in_range"]),
                    "calibrated": bool(calibrated_by[feat]),
                    "feature": feat,
                    "feature_name": FEAT_NAME[feat],
                    "ratio": round(target["r"], 3),
                    "ratio_h": round(target["ratio_h"], 3),
                    "ratio_w": round(target["ratio_w"], 3),
                    "bbox_h_px": int(target["h"]),
                    "bbox_w_px": int(target["w"]),
                    "confidence": round(target["conf"], 3),
                    "at_edge": bool(target["at_edge"]),
                })

            info_msg = String()
            info_msg.data = json.dumps(info)
            info_pub.publish(info_msg)

            # 8. CSV LOGGING
            writer.writerow([
                frame_count,
                f"{time.time():.3f}",
                len(persons),
                target is not None,
                int(target["w"]) if target else 0,
                int(target["h"]) if target else 0,
                round(target["ratio_w"], 4) if target else 0,
                round(target["ratio_h"], 4) if target else 0,
                target["feat"] if target else "-",
                round(dist_smooth, 3)
                if dist_smooth is not None else "",
                bool(target and target["at_edge"]),
                round(target["conf"], 3) if target else 0.0,
                zone.value,
                state.value,
                round(infer_ms, 1),
                round(fps_ema, 1),
                args.true_distance
                if args.true_distance is not None else "",
            ])

            if frame_count % 30 == 0:
                log_file.flush()

            # 9. VISUALIZATION / VIDEO / DASHBOARD STREAM
            need_render = (not args.no_display) or args.save_video or (annotated_pub is not None)
            if need_render:
                # Visualisasi koridor tengah yang digunakan untuk keputusan.
                overlay = frame.copy()
                cv2.rectangle(
                    overlay, (corridor_x1, 0),
                    (corridor_x2, height - 1), (255, 180, 0), -1
                )
                cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)
                cv2.line(
                    frame, (corridor_x1, 0), (corridor_x1, height - 1),
                    (255, 180, 0), max(1, int(2 * scale))
                )
                cv2.line(
                    frame, (corridor_x2, 0), (corridor_x2, height - 1),
                    (255, 180, 0), max(1, int(2 * scale))
                )
                put_text(
                    frame, f"KORIDOR TENGAH {args.corridor_width:.0%}",
                    (corridor_x1 + 5, min(height - 10, int(70 * scale))),
                    0.5 * scale, (255, 180, 0)
                )

                for person in persons:
                    if person is not target:
                        draw_person(
                            frame, person, False, zone,
                            person["dist"], person["in_range"],
                            calibrated_by[person["feat"]], scale
                        )

                if target is not None:
                    draw_person(
                        frame, target, True, zone,
                        dist_smooth, target["in_range"],
                        calibrated_by[target["feat"]], scale
                    )

                top = draw_top_bar(
                    frame, state, zone, scale
                )

                active_feature = (
                    target["feat"]
                    if target is not None
                    else (
                        "h" if args.feature == "height"
                        else "w"
                    )
                )

                if target is not None:
                    distance_text = (
                        f"~{dist_smooth:.2f} m"
                        + (
                            ""
                            if calibrated_by[active_feature]
                            else " *"
                        )
                    )

                    if not target["in_range"]:
                        distance_text += " (luar rentang)"

                    rows = [
                        ("Orang terdeteksi", str(len(persons)), WHITE),
                        ("Irisan koridor", f"{len(corridor_persons)} orang", (255, 180, 0)),
                        (
                            "Jarak estimasi",
                            distance_text,
                            ZONE_COLORS.get(zone, WHITE)
                        ),
                        (
                            "Fitur aktif",
                            f"{FEAT_NAME[target['feat']]} {target['r']:.2f}",
                            WHITE
                        ),
                        (
                            "Tinggi bbox (h/H)",
                            f"{target['ratio_h']:.2f} ({int(target['h'])} px)",
                            WHITE
                        ),
                        (
                            "Lebar bbox (w/W)",
                            f"{target['ratio_w']:.2f} ({int(target['w'])} px)",
                            WHITE
                        ),
                        (
                            "Confidence",
                            f"{target['conf']:.2f}",
                            WHITE
                        ),
                    ]

                else:
                    rows = [
                        ("Orang terdeteksi", str(len(persons)), WHITE),
                        (
                            "Irisan koridor",
                            "0 orang" if persons else "-",
                            (255, 180, 0) if persons else GRAY
                        ),
                        ("Jarak estimasi", "-", GRAY),
                        (
                            "Fitur aktif",
                            FEAT_NAME[active_feature],
                            GRAY
                        ),
                        ("Tinggi bbox (h/H)", "-", GRAY),
                        ("Lebar bbox (w/W)", "-", GRAY),
                        ("Confidence", "-", GRAY),
                    ]

                rows.append((
                    "FPS / Inferensi",
                    f"{fps_ema:.1f} / {infer_ms:.0f} ms",
                    WHITE
                ))

                draw_info_panel(
                    frame, top, rows, scale
                )

                draw_proximity_bar(
                    frame,
                    active_ratio,
                    args.t_med_enter,
                    args.t_near_enter,
                    scale
                )

                # Publish annotated frame ke dashboard via ROS 2 CompressedImage
                if annotated_pub is not None:
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                    ret_enc, enc_bytes = cv2.imencode('.jpg', frame, encode_param)
                    if ret_enc:
                        img_msg = CompressedImage()
                        img_msg.header.stamp = ros_node.get_clock().now().to_msg()
                        img_msg.header.frame_id = "camera_link"
                        img_msg.format = "jpeg"
                        img_msg.data = enc_bytes.tobytes()
                        annotated_pub.publish(img_msg)

                # Publish juga ke topik raw Image (/camera/image_annotated)
                if annotated_raw_pub is not None:
                    try:
                        raw_msg = Image()
                        raw_msg.header.stamp = ros_node.get_clock().now().to_msg()
                        raw_msg.header.frame_id = "camera_link"
                        raw_msg.height = height
                        raw_msg.width = width
                        raw_msg.encoding = "bgr8"
                        raw_msg.is_bigendian = 0
                        raw_msg.step = width * 3
                        raw_msg.data = frame.tobytes()
                        annotated_raw_pub.publish(raw_msg)
                    except Exception:
                        pass

                if args.save_video:
                    if video_writer is None:
                        video_writer = cv2.VideoWriter(
                            args.save_video,
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            20.0,
                            (width, height)
                        )

                        if not video_writer.isOpened():
                            raise RuntimeError(
                                "Gagal membuat file video output."
                            )

                    video_writer.write(frame)

                if not args.no_display:
                    cv2.imshow(
                        "Human-Aware AMR",
                        frame
                    )

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

    except KeyboardInterrupt:
        print("\nPengujian dihentikan oleh pengguna.")

    finally:
        if cap is not None:
            cap.release()

        if video_writer is not None:
            video_writer.release()

        if not args.no_display:
            cv2.destroyAllWindows()

        log_file.flush()
        log_file.close()

        if executor is not None:
            executor.shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=1.0)

        ros_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        elapsed = time.time() - t_start
        average_fps = (
            frame_count / elapsed
            if elapsed > 0 else 0
        )

        print(
            f"Selesai: {frame_count} frame, "
            f"{elapsed:.1f} detik, "
            f"rata-rata {average_fps:.1f} FPS."
        )
        print(f"Log CSV: {args.log}")


if __name__ == "__main__":
    main()