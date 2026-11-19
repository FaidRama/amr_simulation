#!/usr/bin/env python3
"""
fsm.py
Finite State Machine untuk modul Human-Aware Behavior AMR.
- Zona: NONE, FAR, MEDIUM, NEAR.
- State: NORMAL, REDUCED_SPEED, SAFE_STOP.
- Hysteresis pada klasifikasi kedekatan.
- Miss tolerance untuk detector flicker.
- Transisi ke state yang lebih aman langsung; transisi ke state yang
  kurang ketat memakai debounce.
"""

from enum import Enum
import math
from typing import Optional


class Zone(Enum):
    NONE = "NONE"
    FAR = "FAR"
    MEDIUM = "MEDIUM"
    NEAR = "NEAR"


class State(Enum):
    NORMAL = "NORMAL"
    REDUCED_SPEED = "REDUCED_SPEED"
    SAFE_STOP = "SAFE_STOP"


class Activity(Enum):
    STANDING = "STANDING"
    WALKING = "WALKING"
    SITTING = "SITTING"
    NONE = "NONE"


class Direction(Enum):
    SAME_DIRECTION = "SAME_DIRECTION"
    OPPOSITE_DIRECTION = "OPPOSITE_DIRECTION"
    STATIONARY = "STATIONARY"
    UNKNOWN = "UNKNOWN"


_SEVERITY = {
    State.NORMAL: 0,
    State.REDUCED_SPEED: 1,
    State.SAFE_STOP: 2,
}


class ProximityClassifier:
    def __init__(self, t_near_enter=0.85, t_near_exit=0.80,
                 t_med_enter=0.60, t_med_exit=0.50,
                 miss_tolerance: int = 3):
        if not (t_near_enter > t_near_exit):
            raise ValueError("t_near_enter harus lebih besar dari t_near_exit")
        if not (t_med_enter > t_med_exit):
            raise ValueError("t_med_enter harus lebih besar dari t_med_exit")
        if miss_tolerance < 1:
            raise ValueError("miss_tolerance minimal 1")

        self.t_near_enter = t_near_enter
        self.t_near_exit = t_near_exit
        self.t_med_enter = t_med_enter
        self.t_med_exit = t_med_exit
        self.miss_tolerance = miss_tolerance
        self.current_zone = Zone.NONE
        self._miss_count = 0

    def reset(self):
        self.current_zone = Zone.NONE
        self._miss_count = 0

    def update(self, ratio: Optional[float], force_near: bool = False) -> Zone:
        # Penting: force_near harus dicek sebelum ratio=None.
        # Dengan begitu manusia NEAR di luar koridor tetap bisa memicu SAFE_STOP
        # walaupun tidak ada target di koridor tengah.
        if force_near:
            self._miss_count = 0
            self.current_zone = Zone.NEAR
            return self.current_zone

        if ratio is None:
            self._miss_count += 1
            if self._miss_count >= self.miss_tolerance:
                self.current_zone = Zone.NONE
            return self.current_zone

        self._miss_count = 0

        if self.current_zone == Zone.NEAR:
            if ratio < self.t_near_exit:
                self.current_zone = (
                    Zone.MEDIUM if ratio >= self.t_med_exit else Zone.FAR
                )
        elif self.current_zone == Zone.MEDIUM:
            if ratio >= self.t_near_enter:
                self.current_zone = Zone.NEAR
            elif ratio < self.t_med_exit:
                self.current_zone = Zone.FAR
        else:  # FAR atau NONE
            if ratio >= self.t_near_enter:
                self.current_zone = Zone.NEAR
            elif ratio >= self.t_med_enter:
                self.current_zone = Zone.MEDIUM
            else:
                self.current_zone = Zone.FAR

        return self.current_zone


class HumanActivityClassifier:
    """
    Klasifikasi 3 Aktivitas Manusia (STANDING, WALKING, SITTING) dan Arah Gerak
    menggunakan 17 Keypoints YOLO11n-Pose + pelacakan kecepatan temporal.
    """
    def __init__(self, history_size: int = 10, standing_speed_thresh: float = 0.22,
                 sitting_knee_thresh: float = 135.0):
        self.history_size = history_size
        self.standing_speed_thresh = standing_speed_thresh
        self.sitting_knee_thresh = sitting_knee_thresh
        self.history = []

    def reset(self):
        self.history.clear()

    @staticmethod
    def calculate_angle(p1, p2, p3) -> float:
        """Hitung sudut 2D (derajat) pada titik p2 antara p1->p2 dan p3->p2."""
        v1 = (float(p1[0] - p2[0]), float(p1[1] - p2[1]))
        v2 = (float(p3[0] - p2[0]), float(p3[1] - p2[1]))
        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])
        if n1 < 1e-4 or n2 < 1e-4:
            return 180.0
        dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))

    def detect_sitting(self, keypoints, bbox_w: float, bbox_h: float) -> bool:
        """
        Deteksi postur duduk dari sudut sendi lutut (hip-knee-ankle),
        orientasi paha (hip_y vs knee_y), atau fallback rasio aspek H/W.
        """
        if keypoints is not None and len(keypoints) >= 17:
            l_hip, l_knee, l_ankle = keypoints[11], keypoints[13], keypoints[15]
            r_hip, r_knee, r_ankle = keypoints[12], keypoints[14], keypoints[16]

            angles = []
            if l_hip[2] >= 0.20 and l_knee[2] >= 0.20 and l_ankle[2] >= 0.20:
                angles.append(self.calculate_angle(l_hip[:2], l_knee[:2], l_ankle[:2]))
            if r_hip[2] >= 0.20 and r_knee[2] >= 0.20 and r_ankle[2] >= 0.20:
                angles.append(self.calculate_angle(r_hip[:2], r_knee[:2], r_ankle[:2]))

            if angles and min(angles) < self.sitting_knee_thresh:
                return True

            if (l_hip[2] >= 0.20 and l_knee[2] >= 0.20) or (r_hip[2] >= 0.20 and r_knee[2] >= 0.20):
                dy = abs(l_hip[1] - l_knee[1]) if l_hip[2] >= 0.20 and l_knee[2] >= 0.20 else abs(r_hip[1] - r_knee[1])
                dx = abs(l_hip[0] - l_knee[0]) if l_hip[2] >= 0.20 and l_knee[2] >= 0.20 else abs(r_hip[0] - r_knee[0])
                if dx > 1.1 * dy:
                    return True

            # Deteksi dari belakang (punggung orang): saat lutut tertutup punggung/sandaran kursi,
            # bandingkan tinggi torso (bahu ke pinggul) vs tinggi kaki vertikal (pinggul ke mata kaki).
            # Pada posisi duduk di kursi, jarak vertikal pinggul ke mata kaki pendek (leg_h/torso_h < 1.35).
            l_sh, r_sh = keypoints[5], keypoints[6]
            has_sh = (l_sh[2] >= 0.20 or r_sh[2] >= 0.20)
            has_hip = (l_hip[2] >= 0.20 or r_hip[2] >= 0.20)
            has_ank = (l_ankle[2] >= 0.20 or r_ankle[2] >= 0.20)
            if has_sh and has_hip and has_ank:
                sh_y = l_sh[1] if l_sh[2] >= 0.20 else r_sh[1]
                hip_y = l_hip[1] if l_hip[2] >= 0.20 else r_hip[1]
                ank_y = l_ankle[1] if l_ankle[2] >= 0.20 else r_ankle[1]
                torso_h = max(1.0, abs(hip_y - sh_y))
                leg_h = max(1.0, abs(ank_y - hip_y))
                if leg_h / torso_h < 1.35:
                    return True

        # Fallback rasio aspek: orang duduk di kursi memiliki rasio H/W < 1.50
        # (orang dewasa berdiri tegak di koridor memiliki rasio H/W > 2.0)
        aspect_ratio = bbox_h / max(bbox_w, 1.0)
        return aspect_ratio < 1.50

    def classify(
        self,
        keypoints,
        bbox_w: float,
        bbox_h: float,
        center_x: float,
        distance: Optional[float],
        timestamp: float,
        frame_w: float = 640.0
    ):
        """Mengembalikan (Activity, Direction, speed_m_s)."""
        if distance is None:
            self.reset()
            return Activity.NONE, Direction.UNKNOWN, 0.0

        self.history.append({'t': timestamp, 'dist': distance, 'cx': center_x})
        if len(self.history) > self.history_size:
            self.history.pop(0)

        if self.detect_sitting(keypoints, bbox_w, bbox_h):
            return Activity.SITTING, Direction.STATIONARY, 0.0

        if len(self.history) < 3:
            return Activity.STANDING, Direction.STATIONARY, 0.0

        dt = self.history[-1]['t'] - self.history[0]['t']
        if dt < 0.10:
            return Activity.STANDING, Direction.STATIONARY, 0.0

        dD = self.history[-1]['dist'] - self.history[0]['dist']
        radial_speed = dD / dt
        dcx_ratio = (self.history[-1]['cx'] - self.history[0]['cx']) / max(frame_w, 1.0)
        lateral_speed = (dcx_ratio * distance) / dt
        total_speed = math.hypot(radial_speed, lateral_speed)

        if total_speed < self.standing_speed_thresh and abs(radial_speed) < 0.18:
            return Activity.STANDING, Direction.STATIONARY, round(total_speed, 2)

        if radial_speed <= -0.18:
            direction = Direction.OPPOSITE_DIRECTION
        elif radial_speed >= 0.15:
            direction = Direction.SAME_DIRECTION
        else:
            direction = Direction.OPPOSITE_DIRECTION

        return Activity.WALKING, direction, round(total_speed, 2)


class HumanAwareFSM:
    def __init__(self, debounce_frames: int = 8):
        if debounce_frames < 1:
            raise ValueError("debounce_frames minimal 1")
        self.state = State.NORMAL
        self.debounce_frames = debounce_frames
        self._pending_state = None
        self._pending_count = 0

    def reset(self):
        self.state = State.NORMAL
        self._pending_state = None
        self._pending_count = 0

    def _target_state(self, zone: Zone) -> State:
        if zone in (Zone.NONE, Zone.FAR):
            return State.NORMAL
        if zone == Zone.MEDIUM:
            return State.REDUCED_SPEED
        if zone == Zone.NEAR:
            return State.SAFE_STOP
        return State.NORMAL

    def evaluate_activity_state(
        self,
        zone: Zone,
        activity: Activity,
        in_corridor: bool,
        distance: Optional[float] = None,
        direction: Direction = Direction.STATIONARY,
    ) -> State:
        """
        Pemetaan 3 Aktivitas Manusia terhadap Output FSM dengan batas jarak Zona (FAR/MEDIUM/NEAR):
        - Jika jarak jauh (Zone.FAR / Zone.NONE) -> NORMAL.
        1. STANDING: luar lintasan / FAR -> NORMAL; MEDIUM -> REDUCED_SPEED; NEAR -> SAFE_STOP.
        2. WALKING: luar lintasan / FAR -> NORMAL;
                    searah di MEDIUM -> NORMAL (kecuali dekat di NEAR -> REDUCED_SPEED);
                    berlawanan arah di MEDIUM -> REDUCED_SPEED; berlawanan arah di NEAR -> SAFE_STOP.
        3. SITTING: luar lintasan / FAR -> NORMAL; MEDIUM -> REDUCED_SPEED; NEAR -> SAFE_STOP.
        """
        # 1. Permintaan Khusus Dosen:
        # Jika orang SEDANG DUDUK di kursi (di tengah koridor sekalipun):
        # Robot menganggapnya sebagai rintangan pasif, status tetap NORMAL,
        # sehingga robot otomatis langsung bermanuver membelok menghindar via Costmap & DWB
        # tanpa menjalankan perintah reduce maupun safe stop.
        if activity == Activity.SITTING:
            return State.NORMAL

        # 2. Jika orang di luar koridor lintasan robot atau zona FAR -> NORMAL
        if not in_corridor or activity == Activity.NONE or zone in (Zone.NONE, Zone.FAR):
            return State.NORMAL

        # 3. Target BERDIRI / BERJALAN di dalam koridor dan di jarak dekat (ZONA NEAR <= 1.8m):
        if zone == Zone.NEAR:
            # Jika orang berjalan searah menjauhi robot, cukup REDUCED_SPEED kecuali mepet <= 1.0m
            if activity == Activity.WALKING and direction == Direction.SAME_DIRECTION:
                if distance is not None and distance <= 1.0:
                    return State.SAFE_STOP
                return State.REDUCED_SPEED
            return State.SAFE_STOP

        # 4. Target BERDIRI / BERJALAN di dalam koridor dan di jarak sedang (ZONA MEDIUM 1.8m - 2.7m):
        if activity == Activity.STANDING:
            return State.REDUCED_SPEED

        if activity == Activity.WALKING:
            if direction == Direction.SAME_DIRECTION:
                if distance is not None and distance <= 2.0:
                    return State.REDUCED_SPEED
                return State.NORMAL
            return State.REDUCED_SPEED

        return State.NORMAL

    def update(
        self,
        zone: Zone,
        activity: Optional[Activity] = None,
        in_corridor: bool = True,
        distance: Optional[float] = None,
        direction: Direction = Direction.STATIONARY,
    ) -> State:
        if activity is not None:
            target = self.evaluate_activity_state(
                zone=zone,
                activity=activity,
                in_corridor=in_corridor,
                distance=distance,
                direction=direction,
            )
        else:
            target = self._target_state(zone)

        # State yang sama atau lebih ketat langsung diterapkan.
        if _SEVERITY[target] >= _SEVERITY[self.state]:
            self.state = target
            self._pending_state = None
            self._pending_count = 0
            return self.state

        # Penurunan tingkat kewaspadaan harus stabil selama N frame.
        if target == self._pending_state:
            self._pending_count += 1
        else:
            self._pending_state = target
            self._pending_count = 1

        if self._pending_count >= self.debounce_frames:
            self.state = target
            self._pending_state = None
            self._pending_count = 0

        return self.state
