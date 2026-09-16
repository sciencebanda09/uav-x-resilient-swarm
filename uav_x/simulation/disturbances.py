"""Scenario disturbance scheduler shared by simulation and tests."""
from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class Disturbance:
    time_s: float; kind: str; details: dict

class DisturbanceScheduler:
    def __init__(self, disturbances: list[Disturbance]): self.events = sorted(disturbances, key=lambda x: x.time_s); self._fired = set()
    def due(self, time_s: float) -> list[Disturbance]:
        out = []
        for i, event in enumerate(self.events):
            if i not in self._fired and event.time_s <= time_s:
                self._fired.add(i); out.append(event)
        return out
