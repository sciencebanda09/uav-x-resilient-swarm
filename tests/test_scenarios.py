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
                  "geofence_violations", "safety_intervention_count",
                  "minimum_inter_uav_separation_m"):
        assert field in summary
    assert summary["minimum_inter_uav_separation_m"] >= 18.0
    assert "actual_collision_count" in summary
    assert "obstacle_violations" in summary

def test_outage_reduces_packet_delivery():
    summary = run(ScenarioConfig(seed=12, duration_s=6, outage_start_s=1, outage_duration_s=3), policy="heuristic")[-1]
    assert summary["packet_delivery_ratio"] < 1.0

def test_packets_include_route_and_hop_diagnostics():
    records = run(ScenarioConfig(seed=12, duration_s=3), policy="heuristic")
    packets = [p for r in records if r["record_type"] == "tick" for p in r["packets"]]
    assert packets
    assert {"packet_type", "route", "hop_count", "failed_hop"}.issubset(packets[0])

def test_return_home_and_combined_disturbance():
    returned = run(ScenarioConfig(seed=14, duration_s=16, initial_battery_pct=35, return_home_time_s=2), policy="heuristic")
    assert "RETURN_HOME" in event_types(returned)
    combined = run(ScenarioConfig(seed=15, duration_s=8, failure_time_s=3, outage_start_s=2, outage_duration_s=3), policy="heuristic")
    kinds = event_types(combined)
    assert {"UAV_FAILURE", "LINK_OUTAGE"}.issubset(kinds)

def test_relay_reallocations_counts_recover_transitions():
    """Regression: relay_reallocations must count RECOVER transitions too — RECOVER
    is the isolated-UAV relay-seeking role, not a separate mechanism from RELAY."""
    summary = run(ScenarioConfig(seed=17, duration_s=120, outage_start_s=35, outage_duration_s=15), policy="heuristic")[-1]
    assert summary["relay_reallocations"] > 0

def test_charging_uav_never_flagged_as_battery_violation():
    """Regression: a UAV mid-CHARGE must not be bounced to STANDBY by assign() and
    then flagged as a battery_reserve SAFETY_VIOLATION on the next tick."""
    summary = run(ScenarioConfig(seed=29, duration_s=120, initial_battery_pct=35, return_home_time_s=10), policy="heuristic")[-1]
    assert summary["battery_violations"] == 0

def test_isolated_uav_does_not_deadlock_indefinitely():
    """Regression: an isolated UAV stuck oscillating around an unreachable relay
    midpoint must escape to a direct GCS return within a bounded number of ticks,
    rather than holding an assigned PoI forever with zero survey progress."""
    records = run(ScenarioConfig(seed=7, duration_s=240), policy="heuristic")
    ticks = [r for r in records if r["record_type"] == "tick"]
    final_pois = ticks[-1]["pois"]
    assert all(p["status"] == "SURVEYED" for p in final_pois), \
        f"PoIs not completed within 240s: {[p['id'] for p in final_pois if p['status'] != 'SURVEYED']}"

def test_connectivity_availability_is_time_averaged_not_last_tick():
    """Regression: connectivity_availability must reflect the fraction of ticks
    the swarm was reachable, not just whichever instant happened to be the final
    tick — a scenario that recovers repeatedly should not report 0.0 availability
    just because the run happened to end mid-disconnection."""
    summary = run(ScenarioConfig(seed=31, duration_s=120, failure_time_s=48,
                                  outage_start_s=45, outage_duration_s=15,
                                  packet_loss=0.08), policy="heuristic")[-1]
    assert summary["connectivity_availability"] > 0.5

def test_simultaneous_failure_outage_completes_most_pois():
    """Regression: the hardest combined-disturbance scenario (UAV loss + comms
    outage together) must not leave the swarm permanently stuck in group-wide
    isolation — isolation-recovery state should not fully reset on a single
    transient reconnect."""
    summary = run(ScenarioConfig(seed=31, duration_s=120, failure_time_s=48,
                                  outage_start_s=45, outage_duration_s=15,
                                  packet_loss=0.08, radio_range_m=220), policy="heuristic")[-1]
    assert summary["mission_completion_rate"] >= 0.75
    assert summary["battery_violations"] == 0
    assert summary["geofence_violations"] == 0

def test_separation_shield_maintains_margin_not_just_bare_minimum():
    """Regression: the predictive separation shield must resolve conflicts to a
    genuine safety margin above min_separation_m, not merely clear the bare
    threshold — a shield that only pushes UAVs to exactly the minimum will have
    them drift back into conflict on the very next tick, reporting a trajectory
    minimum that hovers at the boundary instead of showing real headroom."""
    summary = run(ScenarioConfig(seed=7, duration_s=120), policy="heuristic")[-1]
    cfg_min = ScenarioConfig(seed=7, duration_s=120).min_separation_m
    assert summary["minimum_inter_uav_separation_m"] >= cfg_min * 1.1
