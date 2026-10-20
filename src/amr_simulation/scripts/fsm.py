"""
fsm.py
Finite State Machine untuk modul Human-Aware Behavior AMR.
Sesuai Dokumen Metode Penelitian:
  - 3 State: NORMAL, REDUCED_SPEED, SAFE_STOP
  - 3 Zona Kedekatan: FAR, MEDIUM, NEAR (ditambah NONE jika tidak ada orang)
  - Menggabungkan Hysteresis (T_ENTER > T_EXIT) dan Debounce temporal
    agar robot tidak berosilasi saat manusia berada di batas zona.
"""

from enum import Enum


class Zone(Enum):
    NONE = "NONE"       # Tidak ada manusia terdeteksi
    FAR = "FAR"         # Zona aman, operasi normal
    MEDIUM = "MEDIUM"   # Zona waspada, perlambatan
    NEAR = "NEAR"       # Zona bahaya, berhenti


class State(Enum):
    NORMAL = "NORMAL"
    REDUCED_SPEED = "REDUCED_SPEED"
    SAFE_STOP = "SAFE_STOP"


_SEVERITY = {
    State.NORMAL: 0,
    State.REDUCED_SPEED: 1,
    State.SAFE_STOP: 2,
}


class ProximityClassifier:
    """
    Klasifikasi zona kedekatan dengan Hysteresis (T_ENTER > T_EXIT).
    Mencegah zona melompat-lompat akibat fluktuasi bounding box di perbatasan ambang.
    """
    def __init__(self, t_near_enter=0.80, t_near_exit=0.70,
                 t_med_enter=0.45, t_med_exit=0.38):
        self.t_near_enter = t_near_enter
        self.t_near_exit = t_near_exit
        self.t_med_enter = t_med_enter
        self.t_med_exit = t_med_exit
        self.current_zone = Zone.NONE

    def update(self, ratio: float | None, force_near: bool = False) -> Zone:
        if ratio is None:
            self.current_zone = Zone.NONE
            return self.current_zone

        if force_near:
            self.current_zone = Zone.NEAR
            return self.current_zone

        # Evaluasi dengan aturan Hysteresis
        if self.current_zone == Zone.NEAR:
            # Tetap NEAR kecuali rasio turun di bawah threshold keluar NEAR
            if ratio < self.t_near_exit:
                self.current_zone = Zone.MEDIUM if ratio >= self.t_med_exit else Zone.FAR
        elif self.current_zone == Zone.MEDIUM:
            # Naik ke NEAR jika tembus ambang masuk NEAR
            if ratio >= self.t_near_enter:
                self.current_zone = Zone.NEAR
            # Turun ke FAR jika jatuh di bawah ambang keluar MEDIUM
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


class HumanAwareFSM:
    def __init__(self, debounce_frames: int = 8):
        """
        FSM dengan aturan transisi:
        - Memperketat (Severity naik: NORMAL -> REDUCED_SPEED / SAFE_STOP): LANGSUNG (Instan).
        - Melonggarkan (Severity turun: SAFE_STOP -> REDUCED_SPEED -> NORMAL): 
          Memerlukan kondisi stabil selama `debounce_frames`.
        """
        self.state = State.NORMAL
        self.debounce_frames = debounce_frames
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

    def update(self, zone: Zone) -> State:
        target = self._target_state(zone)

        # Transisi memperketat (keamanan lebih tinggi) -> segera dieksekusi tanpa delay
        if _SEVERITY[target] >= _SEVERITY[self.state]:
            self.state = target
            self._pending_state = None
            self._pending_count = 0
            return self.state

        # Transisi melonggarkan -> butuh stabilitas frame (debounce)
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