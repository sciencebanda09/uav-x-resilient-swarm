"""Generic delayed/lossy packet queue for UAV-to-UAV/GCS messages."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class Packet:
    source_id: str; payload: dict; sent_time_s: float; delivery_time_s: float

class DelayedPacketQueue:
    def __init__(self, seed: int = 7): self.rng = np.random.default_rng(seed); self.pending = []; self.sent = 0; self.delivered = 0
    def send(self, source_id: str, payload: dict, now_s: float, latency_ms: float, loss: float) -> bool:
        self.sent += 1
        if self.rng.random() < loss: return False
        self.pending.append(Packet(source_id, payload, now_s, now_s + latency_ms/1000.0)); return True
    def drain(self, now_s: float) -> list[Packet]:
        ready = [p for p in self.pending if p.delivery_time_s <= now_s]; self.pending = [p for p in self.pending if p.delivery_time_s > now_s]; self.delivered += len(ready); return ready
