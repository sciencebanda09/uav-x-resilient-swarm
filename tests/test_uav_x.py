import json
from uav_x.core.models import ScenarioConfig
from uav_x.interfaces.log_schema import SCHEMA_VERSION, read_jsonl
from uav_x.simulation.runner import run

def test_schema_and_determinism(tmp_path):
    cfg = ScenarioConfig(seed=11, duration_s=8)
    a = run(cfg); b = run(cfg)
    assert a == b
    path = tmp_path / "run.jsonl"
    from uav_x.interfaces.log_schema import write_jsonl
    write_jsonl(str(path), a)
    records = read_jsonl(str(path))
    assert records[0]["schema_version"] == SCHEMA_VERSION
    assert records[0]["ccpl"]["backend"] in {"ccpl-standalone", "heuristic-fallback"}
    assert records[-1]["record_type"] == "summary"

def test_no_matlab_dependency(tmp_path):
    path = tmp_path / "run.jsonl"
    run(ScenarioConfig(seed=4, duration_s=4), str(path))
    assert path.exists()
    assert "MATLAB" not in json.dumps(read_jsonl(str(path))[-1])

def test_heuristic_policy_is_explicit(tmp_path):
    path = tmp_path / "heuristic.jsonl"
    run(ScenarioConfig(seed=5, duration_s=2), str(path), policy="heuristic")
    records = read_jsonl(str(path))
    assert records[0]["policy"] == "heuristic"
    assert records[0]["ccpl"]["backend"] == "heuristic-fallback"
