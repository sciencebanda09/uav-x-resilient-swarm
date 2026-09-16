"""Standalone CCPL policy adapter for UAV-X.

The adapter never imports the private ``aegis-x`` tree. Install the public
CCPL package from ``requirements-ccpl.txt`` to enable the learned policy.
"""
from __future__ import annotations

from typing import Any
import numpy as np


class CCPLUnavailable(RuntimeError):
    """Raised when explicit CCPL policy was requested but is unavailable."""


class CCPLPolicy:
    ACTIONS = ("SURVEY", "RELAY", "RECOVER", "RETURN", "CHARGE", "STANDBY")

    def __init__(self, seed: int = 7, checkpoint: str | None = None,
                 required: bool = False):
        self.seed = seed
        self.agent: Any = None
        self.backend = "heuristic-fallback"
        self.checkpoint = checkpoint
        self._last_observation: np.ndarray | None = None
        self._last_action: int | None = None
        try:
            from ccpl import make_ccpl  # type: ignore
        except ImportError as exc:
            if required:
                raise CCPLUnavailable(
                    "CCPL is not installed; run: python -m pip install -r requirements-ccpl.txt"
                ) from exc
            return
        try:
            self.agent = make_ccpl(state_dim=8, action_dim=len(self.ACTIONS),
                                   constraint_d=1.0, seed=seed)
            if checkpoint:
                if not hasattr(self.agent, "load"):
                    raise CCPLUnavailable("installed CCPL agent does not expose load()")
                self.agent.load(checkpoint)
            self.backend = "ccpl-standalone"
        except Exception as exc:
            if required:
                raise CCPLUnavailable(f"CCPL initialization failed: {exc}") from exc

    def observation(self, *, risk: float, distance: float, speed: float,
                    battery: float, connected: bool, priority: int,
                    relay: bool = False) -> np.ndarray:
        return np.array([
            np.clip(risk / 30.0, 0.0, 1.0),
            np.clip(distance / 500.0, 0.0, 1.0),
            np.clip(speed / 20.0, 0.0, 1.0),
            np.clip(battery, 0.0, 1.0),
            float(connected),
            np.clip(priority / 5.0, 0.0, 1.0),
            float(relay),
            float(not connected),
        ], dtype=np.float32)

    def select_action(self, observation: np.ndarray) -> int:
        if self.agent is None:
            return 0
        obs = np.asarray(observation, dtype=np.float32)
        if hasattr(self.agent, "select_action"):
            return int(self.agent.select_action(obs, eval_mode=True))
        return int(self.agent.predict(obs))

    def penalty(self, *, risk: float, distance: float, speed: float,
                battery: float, connected: bool, priority: int,
                relay: bool = False) -> float:
        """Return a cost used only to re-rank already-safe actions."""
        obs = self.observation(risk=risk, distance=distance, speed=speed,
                                battery=battery, connected=connected,
                                priority=priority, relay=relay)
        if self.agent is not None:
            action = self.select_action(obs)
            self._last_observation, self._last_action = obs, action
            return float(0.15 * action + 0.85 * (1.0 - battery) +
                         (0.35 if not connected else 0.0))
        consequence = (0.55 * (1.0 - float(np.clip(battery, 0, 1))) +
                       0.30 * (0.0 if connected else 1.0) +
                       0.15 * min(1.0, distance / 500.0))
        state_lambda = 0.5 + 0.5 * (1.0 - float(np.clip(battery, 0, 1)))
        return float(8.0 * state_lambda * consequence)

    def record_transition(self, observation: np.ndarray, action: int,
                          reward: float, next_observation: np.ndarray,
                          consequence: float, done: bool) -> None:
        if self.agent is not None and hasattr(self.agent, "observe_transition"):
            self.agent.observe_transition(observation, action, consequence)

    def record_tick_consequence(self, consequence: float) -> None:
        """Feed the latest delayed mission consequence into CCPL."""
        if self.agent is not None and self._last_observation is not None and self._last_action is not None:
            self.agent.observe_transition(self._last_observation, self._last_action, float(consequence))

    def save_checkpoint(self, path: str) -> None:
        if self.agent is not None and hasattr(self.agent, "save"):
            self.agent.save(path)

    def train(self, env: Any, episodes: int = 10, update_freq: int = 4) -> list:
        if self.agent is None:
            raise CCPLUnavailable("CCPL training requires the standalone package")
        return self.agent.fit(env, episodes=episodes, update_freq=update_freq, verbose=False)

    def backend_info(self) -> dict[str, Any]:
        return {"backend": self.backend, "checkpoint": self.checkpoint,
                "actions": list(self.ACTIONS)}

    def close(self) -> None:
        return None
