from uav_x.core.models import GCS, ScenarioConfig, UAV, safe_destination
from uav_x.simulation.runner import run

def event_types(records):
    return {r.get("event_type") for r in records if r["record_type"] == "event"}

def test_predictive_safety_constraints():
    target, causes = safe_destination(
        current=[10, 10, 50], destination=[-30, 10, 50],
        others=[[0, 10, 50]], minimum_m=18, arena_m=100, margin_m=2)
    assert 2 <= target[0] <= 98
    assert "geofence" in causes or "separation" in causes

def test_outage_recovery_scenario():
    records = run(ScenarioConfig(seed=2, duration_s=8, outage_start_s=2, outage_duration_s=2), policy="heuristic")
    kinds = event_types(records)
    assert "LINK_OUTAGE" in kinds
    assert records[-1]["packet_delivery_ratio"] <= 1.0

def test_failure_and_emergency_scenarios():
    records = run(ScenarioConfig(seed=3, duration_s=70, failure_time_s=3, emergency_time_s=4), policy="heuristic")
    kinds = event_types(records)
    assert "UAV_FAILURE" in kinds
    assert "EMERGENCY_POI" in kinds
    assert records[-1]["recovery_events"] >= 0

def test_metrics_have_communication_and_safety_fields():
    summary = run(ScenarioConfig(seed=9, duration_s=5), policy="heuristic")[-1]
    for field in ("packet_delivery_ratio", "mean_latency_ms", "p95_latency_ms",
                  "mission_completion_time_s", "battery_violations",
                  "geofence_violations", "collision_count",
                  "minimum_inter_uav_separation_m"):
        assert field in summary
    assert summary["minimum_inter_uav_separation_m"] >= 18.0

def test_outage_reduces_packet_delivery():
    summary = run(ScenarioConfig(seed=12, duration_s=6, outage_start_s=1, outage_duration_s=3), policy="heuristic")[-1]
    assert summary["packet_delivery_ratio"] < 1.0

def test_return_home_and_combined_disturbance():
    returned = run(ScenarioConfig(seed=14, duration_s=16, initial_battery_pct=35, return_home_time_s=2), policy="heuristic")
    assert "RETURN_HOME" in event_types(returned)
    combined = run(ScenarioConfig(seed=15, duration_s=8, failure_time_s=3, outage_start_s=2, outage_duration_s=3), policy="heuristic")
    kinds = event_types(combined)
    assert {"UAV_FAILURE", "LINK_OUTAGE"}.issubset(kinds)
