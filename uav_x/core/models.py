"""Domain models for the civilian disaster-response swarm."""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np

@dataclass
class UAV:
    uid: str
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    battery_pct: float = 100.0
    role: str = "STANDBY"
    task_id: str | None = None
    mode: str = "CONNECTED"
    speed_mps: float = 12.0
    home: np.ndarray | None = None
    failed: bool = False

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        self.home = self.position.copy() if self.home is None else np.asarray(self.home, dtype=float)

    def distance_to(self, point: np.ndarray) -> float:
        return float(np.linalg.norm(self.position - point))

@dataclass
class PoI:
    pid: str
    position: np.ndarray
    priority: int = 3
    status: str = "PENDING"
    assigned_uav_id: str | None = None
    survey_progress: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)

@dataclass
class GCS:
    position: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)

@dataclass
class ScenarioConfig:
    seed: int = 7
    duration_s: float = 120.0
    dt: float = 1.0
    arena_m: float = 500.0
    radio_range_m: float = 180.0
    min_separation_m: float = 18.0
    reserve_pct: float = 22.0
    packet_loss: float = 0.04
    failure_time_s: float | None = None
    outage_start_s: float | None = None
    outage_duration_s: float = 0.0
    emergency_time_s: float | None = 65.0

    @property
    def ticks(self) -> int:
        return int(self.duration_s / self.dt)

def clamp_move(uav: UAV, destination: np.ndarray, dt: float) -> None:
    delta = np.asarray(destination) - uav.position
    distance = float(np.linalg.norm(delta))
    if distance < 1e-9:
        uav.velocity[:] = 0
        return
    step = min(distance, uav.speed_mps * dt)
    uav.velocity = delta / distance * (step / dt)
    uav.position += delta / distance * step

def battery_step(uav: UAV, dt: float) -> None:
    motion = float(np.linalg.norm(uav.velocity))
    uav.battery_pct = max(0.0, uav.battery_pct - dt * (0.035 + 0.004 * motion))

def enforce_separation(uavs: list[UAV], minimum_m: float, arena_m: float) -> bool:
    """Deterministic safety shield; separates overlapping active vehicles."""
    changed = False
    for i, a in enumerate(uavs):
        if a.failed: continue
        for b in uavs[i + 1:]:
            if b.failed: continue
            delta = a.position - b.position
            distance = float(np.linalg.norm(delta))
            if distance >= minimum_m: continue
            direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
            correction = direction * ((minimum_m - distance) / 2.0)
            a.position += correction; b.position -= correction
            a.position[:2] = np.clip(a.position[:2], 0.0, arena_m)
            b.position[:2] = np.clip(b.position[:2], 0.0, arena_m)
            changed = True
    return changed
