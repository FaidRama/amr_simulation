
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
    -> ROS 2 /human_state

Mode kamera ROS:
  --source ros
  --image-topic /camera/image_raw/compressed
"""

import argparse
import csv
import sys
import time
import threading
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
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
        "--feature",
        choices=["height", "width", "auto"],
        default="height"
    )

    parser.add_argument("--t-near-enter", type=float, default=0.75)
    parser.add_argument("--t-near-exit", type=float, default=0.65)
    parser.add_argument("--t-med-enter", type=float, default=0.45)
    parser.add_argument("--t-med-exit", type=float, default=0.38)
    parser.add_argument("--debounce", type=int, default=8)

    parser.add_argument("--calib-h", default=DEFAULT_CALIB_H)
    parser.add_argument("--calib-w", default=DEFAULT_CALIB_W)
    parser.add_argument("--smooth", type=float, default=0.3)

    parser.add_argument("--true-distance", type=float, default=None)
    parser.add_argument("--log", default="experiment_log.csv")
    parser.add_argument("--save-video", default=None)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument(
        "--publish-annotated", action="store_true", default=True,
        help="Publish frame ter-anotasi ke topik ROS 2 untuk dashboard"
    )
    parser.add_argument(
        "--annotated-topic",
        default="/camera/image_annotated/compressed",
        help="Topik output frame ter-anotasi (CompressedImage)"
    )

    args, _ = parser.parse_known_args()

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
    latest_frame = {"frame": None}
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

            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

            if frame is not None:
                latest_frame["frame"] = frame

        camera_sub = ros_node.create_subscription(
            CompressedImage,
            args.image_topic,
            camera_callback,
            camera_qos
        )

        ros_node.get_logger().info(
            f"Menunggu kamera pada topik: {args.image_topic}"
        )

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
    t_start = time.time()
    last_no_frame_log = 0.0

    ros_node.get_logger().info(
        "Human-Aware berjalan. Output: /human_state"
    )

    try:
        while rclpy.ok():
            t_frame = time.perf_counter()

            # Penting: proses callback kamera sebelum membaca frame
            if use_ros_camera:
                rclpy.spin_once(
                    ros_node, timeout_sec=0.001
                )

                frame = latest_frame["frame"]

                if frame is None:
                    now = time.time()

                    if now - last_no_frame_log >= 3.0:
                        ros_node.get_logger().info(
                            f"Menunggu frame dari {args.image_topic}..."
                        )
                        last_no_frame_log = now

                    time.sleep(0.005)
                    continue

                # Salin frame supaya tidak diproses saat referensi berubah
                frame = frame.copy()

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

            # 3. TARGET SELECTION
            target = None

            if persons:
                if args.feature == "height":
                    target = max(
                        persons, key=lambda p: p["h"]
                    )
                elif args.feature == "width":
                    target = max(
                        persons, key=lambda p: p["w"]
                    )
                else:
                    target = max(
                        persons,
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
            zone = classifier.update(
                active_ratio,
                force_near=force_near
            )

            state = fsm.update(zone)

            # 6. PUBLISH STATE ONCE PER FRAME
            state_msg = String()
            state_msg.data = state.value
            state_pub.publish(state_msg)

            # Publish juga ke /human_status untuk dashboard AMR
            status_msg = String()
            status_msg.data = state.value if target is not None else "NO_HUMAN"
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
                        ("Orang terdeteksi", "0", WHITE),
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
