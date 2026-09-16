"""Reproducible configuration utilities ported into the public UAV-X layer."""
from __future__ import annotations
import hashlib, json
from dataclasses import asdict, is_dataclass
import numpy as np

class SeedManager:
    def __init__(self, seed: int = 7): self.seed = int(seed); self._streams = {}
    def get(self, name: str) -> np.random.Generator:
        if name not in self._streams:
            value = (self.seed + sum((i + 1) * ord(c) for i, c in enumerate(name))) & 0xFFFFFFFF
            self._streams[name] = np.random.default_rng(value)
        return self._streams[name]

def config_hash(config) -> str:
    data = asdict(config) if is_dataclass(config) else dict(config)
    return hashlib.sha256(json.dumps(data, sort_keys=True, default=str).encode()).hexdigest()[:16]
