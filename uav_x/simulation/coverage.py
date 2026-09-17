"""Lightweight ground-coverage model for UAV survey missions."""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

@dataclass
class CoverageGrid:
    arena_m: float
    cell_size_m: float = 20.0
    covered: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.arena_m = float(self.arena_m)
        self.cell_size_m = max(2.0, float(self.cell_size_m))
        self.size = int(np.ceil(self.arena_m / self.cell_size_m))
        self.covered = np.zeros((self.size, self.size), dtype=bool)

    def mark_footprint(self, x: float, y: float, radius_m: float) -> int:
        """Mark cells touched by a circular downward camera footprint."""
        radius = max(0.0, float(radius_m))
        cx = int(np.clip(float(x) / self.cell_size_m, 0, self.size - 1))
        cy = int(np.clip(float(y) / self.cell_size_m, 0, self.size - 1))
        span = int(np.ceil(radius / self.cell_size_m)) + 1
        changed = 0
        for gy in range(max(0, cy - span), min(self.size, cy + span + 1)):
            for gx in range(max(0, cx - span), min(self.size, cx + span + 1)):
                px = (gx + .5) * self.cell_size_m
                py = (gy + .5) * self.cell_size_m
                if (px - x) ** 2 + (py - y) ** 2 <= radius ** 2:
                    if not self.covered[gy, gx]:
                        self.covered[gy, gx] = True
                        changed += 1
        return changed

    @property
    def fraction(self) -> float:
        return float(np.mean(self.covered))

    def coverage_at(self, x: float, y: float) -> bool:
        gx = int(np.clip(float(x) / self.cell_size_m, 0, self.size - 1))
        gy = int(np.clip(float(y) / self.cell_size_m, 0, self.size - 1))
        return bool(self.covered[gy, gx])
