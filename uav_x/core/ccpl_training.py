"""Small delayed-consequence environment used to train UAV-X CCPL policies."""
from __future__ import annotations
from collections import deque
import numpy as np

class SwarmCCPLEnv:
    consequence_delay = 3

    def __init__(self, seed: int = 7, max_steps: int = 80):
        self.rng = np.random.default_rng(seed); self.max_steps = max_steps
        self.action_dim = 6; self.reset()

    def reset(self):
        self.step_n = 0; self.done = False; self.battery = 1.0
        self.connected = 1.0; self.pending = deque(maxlen=self.consequence_delay + 1)
        self.delayed_hits = 0; self.original_consequence = 0.0
        self._state = np.array([0.2, 0.2, 0.1, 1.0, 1.0, 0.3, 0.0, 0.0], dtype=np.float32)
        return self._state.copy()

    def step(self, action: int):
        action = int(action); self.step_n += 1
        movement_cost = 0.01 + 0.008 * (action in {0, 1, 2})
        self.battery = max(0.0, self.battery - movement_cost)
        link_risk = 0.35 if action == 0 else 0.08 if action == 1 else 0.18
        consequence = float(np.clip(0.55 * (1.0 - self.battery) + link_risk + self.rng.normal(0, .015), 0, 1))
        self.original_consequence += consequence; self.pending.append(consequence)
        delayed = self.pending.popleft() if len(self.pending) > self.consequence_delay else 0.0
        if delayed > 0.65: self.delayed_hits += 1
        reward = (1.0 if action == 0 else 0.7 if action == 1 else 0.4) - consequence
        self.done = self.step_n >= self.max_steps or self.battery <= 0.05
        self.connected = max(0.0, min(1.0, self.connected + (0.04 if action == 1 else -0.015 if action == 0 else 0.01)))
        self._state = np.array([consequence, self.step_n / self.max_steps, action / 5.0,
                                self.battery, self.connected, link_risk,
                                float(delayed > 0), float(self.delayed_hits > 0)], dtype=np.float32)
        return self._state.copy(), float(reward), float(delayed), self.done, {"delayed_hits": self.delayed_hits}

    def episode_stats(self):
        return {"delayed_hits": self.delayed_hits, "original_consequence": self.original_consequence}
