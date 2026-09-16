import importlib.util
import pytest
import numpy as np
from uav_x.core.ccpl_adapter import CCPLPolicy, CCPLUnavailable
from uav_x.core.ccpl_training import SwarmCCPLEnv

def test_ccpl_adapter_contract(tmp_path):
    if importlib.util.find_spec("ccpl") is None:
        pytest.skip("standalone CCPL package is not installed")
    policy = CCPLPolicy(seed=3, required=True)
    assert policy.backend == "ccpl-standalone"
    obs = policy.observation(risk=4, distance=80, speed=2, battery=.8, connected=True, priority=1)
    action = policy.select_action(obs)
    assert 0 <= action < len(policy.ACTIONS)
    checkpoint = tmp_path / "ccpl.pkl"
    policy.save_checkpoint(str(checkpoint))
    assert checkpoint.exists()
    policy.close()

def test_ccpl_delayed_training_and_reload(tmp_path):
    if importlib.util.find_spec("ccpl") is None:
        pytest.skip("standalone CCPL package is not installed")
    policy = CCPLPolicy(seed=4, required=True)
    results = policy.train(SwarmCCPLEnv(seed=4, max_steps=8), episodes=1)
    assert results and "episode_consequence" in results[0]
    checkpoint = tmp_path / "trained.pkl"
    policy.save_checkpoint(str(checkpoint)); policy.close()
    loaded = CCPLPolicy(seed=4, checkpoint=str(checkpoint), required=True)
    assert loaded.backend == "ccpl-standalone"
    loaded.close()

def test_explicit_ccpl_failure_is_clear(monkeypatch):
    # This validates the public exception type without altering installed packages.
    assert issubclass(CCPLUnavailable, RuntimeError)
