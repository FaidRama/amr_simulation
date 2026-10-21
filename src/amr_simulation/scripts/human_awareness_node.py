"""
main.py (Versi Lengkap / Full Feature)
Mencakup:
  - Deteksi YOLO11n (person)
  - Fitur Utama TINGGI bbox (h/H), LEBAR (w/W), dan AUTO
  - Estimasi jarak empiris d = a*(1/r) + b
  - ProximityClassifier dengan Hysteresis (T_ENTER > T_EXIT)
  - FSM dengan Debounce temporal
  - Visualisasi HUD lengkap: Top Bar, Info Panel Transparan, Proximity Gauge Bar
  - Perekaman Video Output (--save-video) & Logging CSV komprehensif
"""

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from fsm import HumanAwareFSM, ProximityClassifier, Zone, State


# ---------------------------------------------------------------- Visual & Palet
GREEN = (80, 200, 80)
YELLOW = (0, 200, 255)
RED = (0, 0, 255)
GRAY = (170, 170, 170)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX

STATE_COLORS = {State.NORMAL: GREEN, State.REDUCED_SPEED: YELLOW, State.SAFE_STOP: RED}
ZONE_COLORS = {Zone.FAR: GREEN, Zone.MEDIUM: YELLOW, Zone.NEAR: RED, Zone.NONE: GRAY}
STATE_RESPONSE = {
    State.NORMAL: "Operasi normal",
    State.REDUCED_SPEED: "Kecepatan dikurangi",
    State.SAFE_STOP: "Robot berhenti (Safe Stop)",
}

FEAT_NAME = {"w": "LEBAR (w/W)", "h": "TINGGI (h/H)"}

# Data empiris awal sesuai dokumen penelitian (jarak m : rasio)
DEFAULT_CALIB_H = "2.0:0.80,3.0:0.55,4.0:0.42"
DEFAULT_CALIB_W = "1.0:0.55,2.0:0.28,3.0:0.19"


# ---------------------------------------------------------------- Estimasi Jarak
class DistanceEstimator:
    def __init__(self, calib_str: str, alpha: float = 0.3):
        pairs = []
        for item in calib_str.split(","):
            d_str, r_str = item.strip().split(":")
            d, r = float(d_str), float(r_str)
            if d <= 0 or r <= 0:
                raise ValueError("Jarak dan rasio harus > 0")
            pairs.append((d, r))
        if not pairs:
            raise ValueError("Data kalibrasi kosong")

        dist = np.array([p[0] for p in pairs])
        inv = np.array([1.0 / p[1] for p in pairs])
        if len(pairs) == 1:
            self.a, self.b = float(dist[0] / inv[0]), 0.0
        else:
            self.a, self.b = (float(v) for v in np.polyfit(inv, dist, 1))

        ratios = [p[1] for p in pairs]
        self.r_min, self.r_max = min(ratios), max(ratios)
        pred = self.a * inv + self.b
        self.rmse = float(np.sqrt(np.mean((pred - dist) ** 2)))
        self.n_points = len(pairs)
        self.alpha = alpha
        self._ema = None

    def estimate(self, ratio: float):
        d = max(0.1, self.a * (1.0 / max(ratio, 1e-6)) + self.b)
        in_range = self.r_min * 0.85 <= ratio <= self.r_max * 1.15
        return d, in_range

    def smooth(self, d: float) -> float:
        self._ema = d if self._ema is None else self.alpha * d + (1 - self.alpha) * self._ema
        return self._ema

    def reset(self):
        self._ema = None


# ---------------------------------------------------------------- Ekstraksi Objek
def extract_persons(result, frame_w: int, frame_h: int, edge_margin: int = 6):
    persons = []
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        w = max(1.0, x2 - x1)
        h = max(1.0, y2 - y1)
        cut_w = x1 <= edge_margin or x2 >= frame_w - edge_margin
        cut_h = y1 <= edge_margin or y2 >= frame_h - edge_margin
        persons.append({
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "w": w, "h": h,
            "ratio_w": w / frame_w,
            "ratio_h": h / frame_h,
            "conf": float(box.conf[0]),
            "cut_w": cut_w,
            "cut_h": cut_h,
            "at_edge": cut_w or cut_h,
        })
    return persons


def pick_feature(p, mode: str) -> str:
    if mode == "width":
        return "w"
    if mode == "height":
        return "h"
    # Mode auto: gunakan tinggi jika vertikal utuh; jika terpotong, beralih ke lebar
    if not p["cut_h"]:
        return "h"
    return "w"


# ---------------------------------------------------------------- Helper Gambar & HUD
def put_text(img, text, org, scale=0.6, color=WHITE, thick=1):
    cv2.putText(img, text, org, FONT, scale, BLACK, thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


def text_size(text, scale, thick=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, thick)
    return w, h, base


def draw_panel(img, x, y, w, h, alpha=0.55):
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(img.shape[1], x + w), min(img.shape[0], y + h)
    if x1 <= x0 or y1 <= y0:
        return
    sub = img[y0:y1, x0:x1]
    cv2.addWeighted(np.zeros_like(sub), alpha, sub, 1 - alpha, 0, sub)


def draw_person(frame, p, is_target, zone, dist_m, in_range, calibrated, s):
    x1, y1, x2, y2 = int(p["x1"]), int(p["y1"]), int(p["x2"]), int(p["y2"])
    mark = "" if calibrated else "*"
    approx = "~" if in_range else "~?"
    dist_txt = f"{approx}{dist_m:.1f} m{mark}" if dist_m is not None else "-"
    unreliable = p["cut_h"] if p["feat"] == "h" else p["cut_w"]

    if is_target:
        color = ZONE_COLORS.get(zone, WHITE)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, max(2, int(3 * s)))
        label = f"TARGET [{p['feat'].upper()}] {dist_txt}"
        scale = 0.65 * s
        tw, th, base = text_size(label, scale, 2)
        top = max(0, y1 - th - base - 8)
        cv2.rectangle(frame, (x1, top), (x1 + tw + 10, top + th + base + 8), color, -1)
        cv2.putText(frame, label, (x1 + 5, top + th + 3), FONT, scale, BLACK, 2, cv2.LINE_AA)
        if unreliable:
            put_text(frame, "tepi frame: jarak kurang andal",
                     (x1, min(frame.shape[0] - 5, y2 + int(18 * s))), 0.5 * s, YELLOW)
    else:
        cv2.rectangle(frame, (x1, y1), (x2, y2), GRAY, 1)
        put_text(frame, dist_txt, (x1 + 3, max(14, y1 - 6)), 0.5 * s, GRAY)


def draw_top_bar(frame, state, zone, s):
    W = frame.shape[1]
    h = int(46 * s)
    color = STATE_COLORS.get(state, GRAY)
    cv2.rectangle(frame, (0, 0), (W, h), color, -1)
    cv2.putText(frame, f"STATE: {state.value}", (int(12 * s), int(30 * s)),
                FONT, 0.85 * s, BLACK, 2, cv2.LINE_AA)
    zone_txt = f"ZONA: {zone.value}"
    tw, _, _ = text_size(zone_txt, 0.7 * s, 2)
    cv2.putText(frame, zone_txt, (W - tw - int(12 * s), int(30 * s)),
                FONT, 0.7 * s, BLACK, 2, cv2.LINE_AA)
    resp = STATE_RESPONSE.get(state, "")
    if resp:
        rw, _, _ = text_size(resp, 0.55 * s, 1)
        cv2.putText(frame, resp, ((W - rw) // 2, int(30 * s)), FONT, 0.55 * s, BLACK, 1, cv2.LINE_AA)
    return h


def draw_info_panel(frame, top, rows, s):
    line_h = int(24 * s)
    pad = int(8 * s)
    w = int(390 * s)
    h = pad * 2 + line_h * len(rows)
    x, y = int(8 * s), top + int(8 * s)
    draw_panel(frame, x, y, w, h)
    for i, (label, value, vcolor) in enumerate(rows):
        ty = y + pad + line_h * (i + 1) - int(6 * s)
        put_text(frame, label, (x + pad, ty), 0.5 * s, GRAY)
        put_text(frame, value, (x + int(180 * s), ty), 0.55 * s, vcolor, 1)


def draw_proximity_bar(frame, ratio, t_medium, t_near, s):
    H, W = frame.shape[:2]
    bar_h = int(20 * s)
    margin = int(12 * s)
    y = H - margin - bar_h
    x0, x1 = margin, W - margin
    width = x1 - x0
    bar_max = min(1.0, t_near * 1.5)

    def px(v):
        return x0 + int(width * min(v, bar_max) / max(bar_max, 1e-4))

    xm, xn = px(t_medium), px(t_near)
    cv2.rectangle(frame, (x0, y), (xm, y + bar_h), GREEN, -1)
    cv2.rectangle(frame, (xm, y), (xn, y + bar_h), YELLOW, -1)
    cv2.rectangle(frame, (xn, y), (x1, y + bar_h), RED, -1)
    cv2.rectangle(frame, (x0, y), (x1, y + bar_h), BLACK, 1)

    for name, a, b in (("FAR", x0, xm), ("MEDIUM", xm, xn), ("NEAR", xn, x1)):
        tw, th, _ = text_size(name, 0.45 * s, 1)
        cv2.putText(frame, name, (a + (b - a - tw) // 2, y + (bar_h + th) // 2),
                    FONT, 0.45 * s, BLACK, 1, cv2.LINE_AA)

    put_text(frame, f"T_med={t_medium:.2f}", (xm - int(35 * s), y - int(6 * s)), 0.4 * s, WHITE)
    put_text(frame, f"T_near={t_near:.2f}", (xn - int(35 * s), y - int(6 * s)), 0.4 * s, WHITE)

    if ratio is not None:
        mx = px(ratio)
        tri = np.array([[mx, y - 2], [mx - int(7 * s), y - int(14 * s)], [mx + int(7 * s), y - int(14 * s)]])
        cv2.fillPoly(frame, [tri], WHITE)
        cv2.polylines(frame, [tri], True, BLACK, 1, cv2.LINE_AA)


# ---------------------------------------------------------------- Main Program
def main():
    parser = argparse.ArgumentParser(description="Human-Aware AMR Modul Deployment")
    parser.add_argument("--source", default="0", help="Kamera index atau path file video")
    parser.add_argument("--model", default="yolo11n.pt", help="File model (yolo11n.pt atau yolo11n.engine)")
    parser.add_argument("--imgsz", type=int, default=640, help="Resolusi input inferensi YOLO")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold deteksi person")
    parser.add_argument("--feature", choices=["height", "width", "auto"], default="height",
                        help="Fitur indikator kedekatan: height (default), width, atau auto")

    # Ambang Hysteresis (T_ENTER > T_EXIT)
    parser.add_argument("--t-near-enter", type=float, default=0.75, help="Ambang MASUK zona NEAR")
    parser.add_argument("--t-near-exit", type=float, default=0.65, help="Ambang KELUAR zona NEAR")
    parser.add_argument("--t-med-enter", type=float, default=0.45, help="Ambang MASUK zona MEDIUM")
    parser.add_argument("--t-med-exit", type=float, default=0.38, help="Ambang KELUAR zona MEDIUM")

    parser.add_argument("--debounce", type=int, default=8, help="Frame delay saat transisi melonggar")
    parser.add_argument("--calib-h", default=DEFAULT_CALIB_H, help="Kalibrasi TINGGI (meter:rasio)")
    parser.add_argument("--calib-w", default=DEFAULT_CALIB_W, help="Kalibrasi LEBAR (meter:rasio)")
    parser.add_argument("--smooth", type=float, default=0.3, help="Faktor EMA penghalus jarak")
    parser.add_argument("--true-distance", type=float, default=None, help="Jarak ground truth fisik (m)")
    parser.add_argument("--log", default="experiment_log.csv", help="Path CSV pencatatan")
    parser.add_argument("--save-video", default=None, help="Path output file video (mis. out.mp4)")
    parser.add_argument("--no-display", action="store_true", help="Nonaktifkan GUI (wajib untuk run mandiri di Jetson)")
    args = parser.parse_args()

    if not (args.t_near_enter > args.t_near_exit > args.t_med_enter > args.t_med_exit > 0):
        parser.error("Aturan Hysteresis harus dipenuhi: T_NEAR_ENTER > T_NEAR_EXIT > T_MED_ENTER > T_MED_EXIT > 0")

    # Estimator Jarak
    est_h = DistanceEstimator(args.calib_h, alpha=args.smooth)
    est_w = DistanceEstimator(args.calib_w, alpha=args.smooth)
    estimators = {"h": est_h, "w": est_w}
    calibrated_by = {"h": args.calib_h != DEFAULT_CALIB_H, "w": args.calib_w != DEFAULT_CALIB_W}

    # Classifier & FSM
    classifier = ProximityClassifier(
        t_near_enter=args.t_near_enter, t_near_exit=args.t_near_exit,
        t_med_enter=args.t_med_enter, t_med_exit=args.t_med_exit
    )
    fsm = HumanAwareFSM(debounce_frames=args.debounce)

    # YOLO Detector
    model = YOLO(args.model)
    person_id = [k for k, v in model.names.items() if v == "person"][0]

    # Video Source (Cross-Platform: Windows pakai DSHOW, Linux/Jetson pakai V4L2)
    import sys
    source = int(args.source) if args.source.isdigit() else args.source
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

    # CSV Logging Setup
    log_path = Path(args.log)
    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow([
        "frame", "timestamp", "n_persons", "human_detected",
        "bbox_w_px", "bbox_h_px", "ratio_w", "ratio_h", "active_feature",
        "distance_est_m", "at_edge", "confidence", "zone", "state",
        "infer_ms", "fps", "true_distance_m"
    ])

    video_writer = None
    frame_count = 0
    miss_count = 0
    fps_ema = None
    last_feat = None
    t_start = time.time()

    print(f"Modul Human-Aware berjalan [Fitur Utama: {args.feature.upper()}].")

    try:
        while True:
            t_frame = time.perf_counter()
            ret, frame = cap.read()
            if not ret:
                print("Stream selesai atau tidak terbaca.")
                break

            frame_count += 1
            H, W = frame.shape[:2]
            s = float(np.clip(W / 960.0, 0.6, 1.4))

            # 1. Inferensi YOLO
            t0 = time.perf_counter()
            result = model(frame, verbose=False, conf=args.conf, imgsz=args.imgsz, classes=[person_id])[0]
            infer_ms = (time.perf_counter() - t0) * 1000.0

            # 2. Ekstraksi Fitur
            persons = extract_persons(result, W, H)
            for p in persons:
                p["feat"] = pick_feature(p, args.feature)
                p["r"] = p["ratio_h"] if p["feat"] == "h" else p["ratio_w"]
                p["dist"], p["in_range"] = estimators[p["feat"]].estimate(p["r"])

            # 3. Pemilihan Target (Terdekat)
            target = None
            if persons:
                if args.feature == "height":
                    target = max(persons, key=lambda p: p["h"])
                elif args.feature == "width":
                    target = max(persons, key=lambda p: p["w"])
                else:  # auto
                    target = max(persons, key=lambda p: p["w"] * p["h"])

            # 4. Proximity & Jarak Smoothing
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
                feat = target["feat"]
                active_ratio = target["r"]

                if args.feature == "auto" and target["cut_w"] and target["cut_h"]:
                    force_near = True

                if last_feat is not None and feat != last_feat and estimators[last_feat]._ema is not None:
                    estimators[feat]._ema = estimators[last_feat]._ema
                last_feat = feat

                dist_smooth = estimators[feat].smooth(target["dist"])
                target["dist"] = dist_smooth

            # 5. Klasifikasi Zona & Evaluasi FSM
            zone = classifier.update(active_ratio, force_near=force_near)
            state = fsm.update(zone)

            # 6. Perhitungan FPS
            inst_fps = 1.0 / max(time.perf_counter() - t_frame, 1e-6)
            fps_ema = inst_fps if fps_ema is None else 0.9 * fps_ema + 0.1 * inst_fps

            # 7. Logging CSV
            writer.writerow([
                frame_count, f"{time.time():.3f}", len(persons), target is not None,
                int(target["w"]) if target else 0,
                int(target["h"]) if target else 0,
                round(target["ratio_w"], 4) if target else 0,
                round(target["ratio_h"], 4) if target else 0,
                target["feat"] if target else "-",
                round(dist_smooth, 3) if dist_smooth else "",
                bool(target and target["at_edge"]),
                round(target["conf"], 3) if target else 0.0,
                zone.value, state.value,
                round(infer_ms, 1), round(fps_ema, 1),
                args.true_distance if args.true_distance is not None else ""
            ])

            # 8. Render Visualisasi & Simpan Video
            if not args.no_display or args.save_video:
                for p in persons:
                    if p is not target:
                        draw_person(frame, p, False, zone, p["dist"], p["in_range"], calibrated_by[p["feat"]], s)
                if target is not None:
                    draw_person(frame, target, True, zone, dist_smooth, target["in_range"], calibrated_by[target["feat"]], s)

                top = draw_top_bar(frame, state, zone, s)
                active_feat = target["feat"] if target is not None else ("h" if args.feature == "height" else "w")

                if target is not None:
                    dist_txt = f"~{dist_smooth:.2f} m" + ("" if calibrated_by[active_feat] else " *")
                    if not target["in_range"]:
                        dist_txt += " (luar rentang)"
                    rows = [
                        ("Orang terdeteksi", str(len(persons)), WHITE),
                        ("Jarak estimasi", dist_txt, ZONE_COLORS.get(zone, WHITE)),
                        ("Fitur aktif", f"{FEAT_NAME[target['feat']]}  {target['r']:.2f}", WHITE),
                        ("Tinggi bbox (h/H)", f"{target['ratio_h']:.2f}  ({int(target['h'])} px)", WHITE),
                        ("Lebar bbox (w/W)", f"{target['ratio_w']:.2f}  ({int(target['w'])} px)", WHITE),
                        ("Confidence", f"{target['conf']:.2f}", WHITE),
                    ]
                else:
                    rows = [
                        ("Orang terdeteksi", "0", WHITE),
                        ("Jarak estimasi", "-", GRAY),
                        ("Fitur aktif", FEAT_NAME[active_feat], GRAY),
                        ("Tinggi bbox (h/H)", "-", GRAY),
                        ("Lebar bbox (w/W)", "-", GRAY),
                        ("Confidence", "-", GRAY),
                    ]
                rows.append(("FPS / Inferensi", f"{fps_ema:.1f} / {infer_ms:.0f} ms", WHITE))
                draw_info_panel(frame, top, rows, s)

                draw_proximity_bar(frame, active_ratio, args.t_med_enter, args.t_near_enter, s)

                if args.save_video:
                    if video_writer is None:
                        video_writer = cv2.VideoWriter(args.save_video, cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (W, H))
                    video_writer.write(frame)

                if not args.no_display:
                    cv2.imshow("Human-Aware AMR Simulation", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

    finally:
        cap.release()
        if video_writer is not None:
            video_writer.release()
        cv2.destroyAllWindows()
        log_file.close()
        elapsed = time.time() - t_start
        fps_avg = frame_count / elapsed if elapsed > 0 else 0
        print(f"Selesai. {frame_count} frame diproses dalam {elapsed:.1f} detik (~{fps_avg:.1f} FPS).")
        print(f"Log tersimpan di: {log_path.resolve()}")


if __name__ == "__main__":
    main()