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

def test_streaming_run_writes_complete_log(tmp_path):
    path = tmp_path / "stream.jsonl"
    records = run(ScenarioConfig(seed=5, duration_s=3, sensor_mode="camera_metadata"),
                  str(path), policy="heuristic", streaming=True)
    persisted = read_jsonl(str(path))
    assert len(persisted) == len(records)
    assert persisted[-1]["record_type"] == "summary"
    tick = next(record for record in persisted if record["record_type"] == "tick")
    assert tick["uavs"][0]["sensors"]["camera"]["width"] == 320

def test_configurable_fleet_size_and_sensor_noise():
    records = run(ScenarioConfig(seed=6, duration_s=2, fleet_size=10,
                                 gps_noise_m=0.5, imu_noise_mps2=0.1), policy="heuristic")
    tick = next(record for record in records if record["record_type"] == "tick")
    assert len(tick["uavs"]) == 10
    assert tick["uavs"][0]["sensors"]["gps_position_m"] is not None
