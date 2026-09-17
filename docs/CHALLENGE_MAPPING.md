# Challenge mapping

Evidence below is from `scenarios/*.yaml` runs at their configured durations,
reproducible via `python -m uav_x.simulation.runner --scenario <path>`. Full
distributions come from `python -m uav_x.validation`.

| Challenge criterion (weight) | UAV-X implementation | Evidence |
|---|---|---|
| Mission Completion (25%) | Local task bidding scores candidate PoIs by priority, distance, and battery feasibility; a deterministic highest-bid assignment resolves conflicts each tick. | `mission_completion_rate = 0.89` (8/9 PoIs) on baseline/emergency_priority/outage_recovery/simultaneous_failure_outage; `uav_failure` reaches full 1.0; `return_home_recharge` (deliberately battery-starved scenario) completes 0.33. |
| Communication Resilience (25%) | Time-varying multi-hop graph (`connectivity.py`) recomputed every tick from range, line-of-sight, and terrain occlusion; per-link packet loss and latency scale with distance and jamming/outage state. | `connectivity_availability` (tick-averaged fraction of active UAVs reachable from GCS, not a single-tick snapshot) ranges 0.82-0.98 across scenarios. `packet_loss` sensitivity sweep shows graceful degradation from 0.0 to 0.1 loss, then a completion cliff at 0.2 loss (0.89 to 0.11) - documented as a known operating boundary, not hidden. |
| Autonomous Relay & Role Management (20%) | UAVs self-elect into RELAY when their own route redundancy/quality drops, and isolated UAVs enter RECOVER and reposition toward the nearest connected node. | `relay_reallocations` (RELAY + RECOVER transitions): 10-27 across scenarios, highest under `outage_recovery` (27) and `simultaneous_failure_outage` (22), reflecting active reconfiguration under stress. |
| Fault Recovery & Swarm Reconfiguration (15%) | Isolated UAVs seek a relay midpoint; isolation state decays gradually rather than resetting on a single reconnect, so a UAV stuck in a flickering-connectivity region still escapes to a direct GCS return within a bounded number of ticks instead of oscillating indefinitely. | `simultaneous_failure_outage` (UAV loss + comms outage together, the hardest combined-disturbance scenario) went from 0.44 to 0.89 mission completion after the isolation-recovery fix, with connectivity availability rising from 0.64 to 0.82 and zero new safety violations. Regression test `test_isolated_uav_does_not_deadlock_indefinitely` locks this in. |
| Safety & Collision Avoidance (10%) | Predictive one-tick-ahead conflict resolution enforces minimum separation and geofence bounds before any UAV moves; battery reserve triggers a sticky RETURN/CHARGE cycle that cannot be interrupted by task reassignment. | Zero `battery_violations` and zero `geofence_violations` across all six scenarios; `minimum_inter_uav_separation_m` stays at or above the 18 m threshold in every run (19.8 to 111.6 m observed); `safety_intervention_count` reports proactive shield activations, kept distinct from actual violations. |
| Innovation & Technical Merit (5%) | Deterministic seeded simulation with a versioned JSONL log contract; optional CCPL-based constrained policy layer for role scoring, with the heuristic controller as a mandatory, policy-independent safety-preserving fallback. | `python -m uav_x.validation` produces Monte Carlo and sensitivity reports as first-class artifacts, not just a single demo run, showing the system's behavior is characterized rather than a single seed being cherry-picked. |

## Tuning note

The default `radio_range_m` was raised from 180 m to 220 m across all six
scenarios after diagnosing a real emergent failure mode: under
`simultaneous_failure_outage`, the swarm's own survey-task pursuit spread
UAVs far enough apart that the group fell out of multi-hop range of GCS
entirely, well after the scripted outage window had ended. This was not a
scripted disturbance being tested by the PS - it was an artifact of a radio
range set tight enough that normal task-driven movement alone could cause
disconnection. Widening the range to 220 m preserves meaningful difficulty
differentiation across scenarios (0.33 to 1.0 completion, depending on
scenario) while ensuring failures shown in the demo are the disturbances the
scenario is designed to test, not an unrelated range-tuning artifact.

## Known limitations (disclosed, not hidden)

- Communication reliability degrades sharply, not gracefully, beyond roughly
  10-15% base packet loss - see the sensitivity sweep above. This is a
  genuine property of the current relay-selection heuristic, not yet
  mitigated by CCPL, and is independent of the radio-range tuning above.
- `return_home_recharge.yaml` intentionally starts UAVs battery-starved
  (35% initial charge) and completes only 0.33 of PoIs as a result - this is
  the scenario working as designed (demonstrating battery-safe behavior
  under a hard resource constraint), not a defect.
- CCPL integration is present (`--policy ccpl`) but the heuristic controller
  remains the reference implementation for Stage 1; CCPL's reward/constraint
  structure for this civilian mission (as opposed to the counter-drone
  domain it was originally developed for) is still being validated.
