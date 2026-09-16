"""Deterministic terrain, line-of-sight, and weather models."""
from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np

class TerrainModel:
    def __init__(self, arena_m: float, seed: int = 7, grid: int = 40, heightmap_path: str | None = None):
        self.arena_m, self.grid = float(arena_m), grid
        if heightmap_path and Path(heightmap_path).exists():
            payload = json.loads(Path(heightmap_path).read_text(encoding="utf-8"))
            self.grid = int(payload["grid"]); self.x = np.linspace(0, self.arena_m, self.grid); self.y = np.linspace(0, self.arena_m, self.grid)
            self.elevation = np.asarray(payload["elevation_m"], dtype=float)
            return
        rng = np.random.default_rng(seed); self.x = np.linspace(0, arena_m, grid); self.y = np.linspace(0, arena_m, grid)
        X, Y = np.meshgrid(self.x, self.y); noise = rng.normal(0, 1.0, (grid, grid))
        self.elevation = 5 + 8*np.sin(X/75)*np.cos(Y/95) + 2*noise
        self.elevation = np.maximum(self.elevation, 0.0)
    def height_at(self, x: float, y: float) -> float:
        fx = np.clip(float(x) / self.arena_m * (self.grid-1), 0, self.grid-1); fy = np.clip(float(y) / self.arena_m * (self.grid-1), 0, self.grid-1)
        x0, y0 = int(fx), int(fy); x1, y1 = min(x0+1, self.grid-1), min(y0+1, self.grid-1); tx, ty = fx-x0, fy-y0
        return float((1-ty)*((1-tx)*self.elevation[y0,x0]+tx*self.elevation[y0,x1])+ty*((1-tx)*self.elevation[y1,x0]+tx*self.elevation[y1,x1]))

    def terrain_clear(self, start: np.ndarray, end: np.ndarray, clearance_m: float = 12.0, samples: int = 12) -> tuple[np.ndarray, float] | None:
        """Raise a waypoint above the highest terrain crossed by its segment."""
        highest = max(self.height_at(*(start + a*(end-start))[:2]) for a in np.linspace(0.0, 1.0, samples))
        if end[2] < highest + clearance_m:
            raised = np.asarray(end, dtype=float).copy(); raised[2] = highest + clearance_m; return raised, highest
        return None
    def los_clear(self, p1: np.ndarray, p2: np.ndarray, samples: int = 20) -> bool:
        p1, p2 = np.asarray(p1), np.asarray(p2)
        for a in np.linspace(0.05, .95, samples):
            p = p1 + a*(p2-p1)
            if p[2] <= self.height_at(p[0], p[1]) + 2.0: return False
        return True

@dataclass
class WeatherState:
    wind_mps: np.ndarray
    visibility: float = 1.0

class WeatherModel:
    def __init__(self, seed: int = 7): self.rng = np.random.default_rng(seed); self.state = WeatherState(np.zeros(3))
    def step(self, time_s: float) -> WeatherState:
        self.state.wind_mps = np.array([2.5*np.sin(time_s/17), 2.0*np.cos(time_s/23), 0.0])
        self.state.visibility = float(np.clip(.85 + .1*np.sin(time_s/31), .5, 1.0)); return self.state
