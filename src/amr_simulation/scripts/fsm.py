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


_SEVERITY = {
    State.NORMAL: 0,
    State.REDUCED_SPEED: 1,
    State.SAFE_STOP: 2,
}


class ProximityClassifier:
    def __init__(self, t_near_enter=0.95, t_near_exit=0.90,
                 t_med_enter=0.85, t_med_exit=0.70,
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

    def update(self, zone: Zone) -> State:
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
