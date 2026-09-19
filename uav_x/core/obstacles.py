"""Lightweight AABB obstacle model shared by simulation and visualizer.

Coordinates are ENU metres: ``x`` east, ``y`` north, ``z`` altitude.  The
model is deliberately dependency-free so obstacle checks remain deterministic
and usable on the judge machine without a 3-D engine.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    kind: str
    center: np.ndarray
    size: np.ndarray
    radio_occluder: bool = False

    @property
    def lo(self) -> np.ndarray:
        return self.center - self.size / 2.0

    @property
    def hi(self) -> np.ndarray:
        return self.center + self.size / 2.0


class ObstacleField:
    def __init__(self, obstacles: list[Obstacle] | None = None, clearance_m: float = 4.0):
        self.obstacles = obstacles or []
        self.clearance_m = float(clearance_m)

    @classmethod
    def from_json(cls, path: str | Path, clearance_m: float = 4.0) -> "ObstacleField":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        rows = data.get("obstacles", data if isinstance(data, list) else [])
        return cls([Obstacle(str(row["id"]), str(row.get("kind", "obstacle")),
                             np.asarray(row["center_m"], dtype=float),
                             np.asarray(row["size_m"], dtype=float),
                             bool(row.get("radio_occluder", False))) for row in rows], clearance_m)

    def point_clear(self, point: np.ndarray) -> bool:
        p = np.asarray(point, dtype=float)
        return not any(np.all(p >= o.lo - self.clearance_m) and np.all(p <= o.hi + self.clearance_m)
                       for o in self.obstacles)

    def containing(self, point: np.ndarray) -> Obstacle | None:
        """Return an obstacle whose safety volume contains ``point``."""
        p = np.asarray(point, dtype=float)
        for obstacle in self.obstacles:
            if np.all(p >= obstacle.lo - self.clearance_m) and np.all(p <= obstacle.hi + self.clearance_m):
                return obstacle
        return None

    def blocking(self, start: np.ndarray, end: np.ndarray) -> Obstacle | None:
        """Return the first AABB intersected by a segment, if any."""
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        direction = b - a
        best: tuple[float, Obstacle] | None = None
        for obstacle in self.obstacles:
            lo, hi = obstacle.lo - self.clearance_m, obstacle.hi + self.clearance_m
            t0, t1 = 0.0, 1.0
            for axis in range(3):
                if abs(direction[axis]) < 1e-12:
                    if a[axis] < lo[axis] or a[axis] > hi[axis]:
                        break
                    continue
                ta = (lo[axis] - a[axis]) / direction[axis]
                tb = (hi[axis] - a[axis]) / direction[axis]
                if ta > tb: ta, tb = tb, ta
                t0, t1 = max(t0, ta), min(t1, tb)
                if t0 > t1: break
            else:
                if best is None or t0 < best[0]: best = (t0, obstacle)
        return None if best is None else best[1]

    def detour(self, start: np.ndarray, destination: np.ndarray) -> tuple[np.ndarray, Obstacle] | None:
        obstacle = self.blocking(start, destination)
        if obstacle is None: return None
        waypoint = np.asarray(destination, dtype=float).copy()
        safe_altitude = obstacle.hi[2] + self.clearance_m + 2.0
        # Climb vertically before crossing the obstacle footprint.  Raising
        # only the final destination still creates a diagonal segment through
        # a tower while the vehicle is climbing.
        expanded_xy = np.r_[obstacle.lo[:2] - self.clearance_m, obstacle.hi[:2] + self.clearance_m]
        over_footprint = (expanded_xy[0] <= start[0] <= expanded_xy[2] and
                          expanded_xy[1] <= start[1] <= expanded_xy[3])
        if start[2] < safe_altitude and (over_footprint or self.blocking(start, destination) is obstacle):
            waypoint = np.asarray(start, dtype=float).copy()
            waypoint[2] = safe_altitude
        else:
            waypoint[2] = max(waypoint[2], safe_altitude)
        return waypoint, obstacle
