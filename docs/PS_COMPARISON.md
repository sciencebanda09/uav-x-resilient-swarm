# UAV-X vs. the PUSHPAK Problem Statement — gap analysis

Source: "UAV-X: Resilient BVLOS Swarm Challenge" (Grand Challenge 1, IISERB,
Techfest 2026-27), Stage 1 deadline 27 September 2026.

This document compares what the problem statement (PS) requires against what
the repository currently delivers, as of this pass. It is meant to be read
alongside `CHALLENGE_MAPPING.md` (which covers the scored evaluation criteria
in depth) and is intentionally blunt about what is still missing — a
gap-analysis document that only lists strengths is not useful before a
submission deadline.

## 1. Mission requirements ("the swarm must autonomously...")

| PS requirement | Status | Notes |
|---|---|---|
| Survey all assigned disaster locations | Met | 8 static PoIs + 1 emergency PoI, priority-weighted bidding. 0.89 completion rate on 4 of 6 benchmark scenarios; full 9/9 completion confirmed given sufficient duration (see `test_isolated_uav_does_not_deadlock_indefinitely`). |
| Maintain end-to-end communication with the GCS | Met | Time-varying multi-hop graph, BFS route computation every tick, packet delivery ratio and latency tracked and logged. |
| Dynamically assign relay UAVs | Met | `_relay_needed` self-election plus isolation-triggered RECOVER role; both now correctly counted in `relay_reallocations` (fixed this pass — was previously undercounting to zero). |
| Reconfigure network on degradation/failure/recharge | Met | Six scenarios explicitly test this: `outage_recovery`, `uav_failure`, `return_home_recharge`, `simultaneous_failure_outage`. An isolation-timeout fallback (added this pass) prevents indefinite stranding. |
| Prioritize newly emerging high-priority regions | Met | `emergency_time_s` injects a priority-1 PoI mid-run; bidding logic weights by `(6 - priority)`, so it is preferentially picked up. |
| Complete the mission safely and within allotted time | Partial | Safety constraints are met with zero violations across all scenarios. "Within allotted time" is scenario-duration-dependent — the standard 120s window does not allow full completion in every scenario (see Section 3). |

## 2. Format of Competition — Stage 1 deliverables

| PS requirement | Status | Notes |
|---|---|---|
| 6-8 page technical proposal | **Not met** | `docs/TECHNICAL_PROPOSAL.md` is currently ~1 page (~400 words). It is accurate and well-scoped as an abstract/architecture summary but does not meet the page-count requirement. This needs direct expansion by the team — team composition, detailed methodology narrative, risk assessment, and a validation-results section with figures are still needed and require information (team member names/roles, institutional affiliation) this pass cannot supply. |
| Software architecture | Met | Described in `TECHNICAL_PROPOSAL.md` and reflected in the actual module layout (`uav_x/core`, `simulation`, `metrics`, `interfaces`, `visualization`). |
| Working proof-of-concept simulation | Met | Confirmed running end-to-end this pass: `runner` -> `operations_2d`/`replay_2d`/`replay_3d` -> `validation`, zero errors, real output artifacts produced. |
| Source code | Met | Present and tested (16 tests, all passing after this pass's fixes). |
| Installation instructions | Met | README quick-start verified to work exactly as written on a clean environment. |
| Demonstration video | **Not present in repo** | No video file or recording present. This is outside what a code pass can produce — needs to be recorded against the fixed build. |

## 3. What changed this pass, and why it matters for scoring

Three functional bugs were found and fixed, plus one misleading metric name:

1. **`relay_reallocations` undercounting to zero.** The metric only counted
   transitions into the `RELAY` role, missing the `RECOVER` role, which is the
   same physical relay-seeking behavior for isolated UAVs. This directly
   understated performance on "Autonomous Relay & Role Management" (20% of
   the rubric) — every benchmark scenario was reporting 0 before the fix.

2. **False battery-safety violation in `return_home_recharge.yaml`.** A UAV
   mid-charge could be bumped out of the `CHARGE` role by the task-allocation
   loop for a single tick, immediately re-flagged as a battery-reserve
   violation. This is exactly the scenario the PS's "no UAV should be without
   charge" requirement targets, and it was failing on its own dedicated test
   case. Fixed by making `CHARGE` a protected role in the allocator.

3. **Isolation deadlock.** An isolated UAV chasing a relay midpoint could get
   physically stuck oscillating in a small area indefinitely if that midpoint
   was itself unreachable, permanently holding an assigned PoI at 0% survey
   progress. Fixed with a 20-tick timeout that falls back to a direct GCS
   return, guaranteeing eventual reconnection. This raised mission completion
   from stuck-forever to full completion on the affected scenario.

4. **Misleading metric name.** `collision_count` was reporting proactive
   safety-shield activations (successful prevention), not actual collisions
   (which remained zero throughout). Renamed to `safety_intervention_count`
   so a judge reading the raw metrics does not misread prevention as failure.

5. **`connectivity_availability` computed from a single tick, not averaged.**
   The metric only looked at the final tick of the run, so a scenario that
   was actually connected 64% of the time could report 0.0 availability
   simply because the run happened to end mid-disconnection. Fixed to
   average reachability across every tick, which is what the PS's
   "connectivity availability" performance metric actually asks for.

6. **Isolation-recovery state reset too eagerly on transient reconnects.**
   Under `simultaneous_failure_outage`, the whole swarm could fall into a
   feedback loop: instability in the relay-seeking heuristic caused brief
   reconnections, which reset the isolation-timeout counter to zero,
   returning control to the same unstable heuristic. Fixed by making the
   counter decay gradually (a reconnect removes isolation credit rather
   than clearing it outright), so a UAV that has genuinely been struggling
   still escapes to a direct GCS return. Combined with a radio-range tuning
   pass (see below), this took the scenario's completion rate from 0.44 to
   0.89.

## 4. Remaining known gaps (not fixed this pass, flagged for the team)

- **Stage 1 proposal document is ~1 page, PS requires 6-8.** This is the
  single largest compliance gap and needs direct authorship, not code work.
- **No demonstration video exists in the repository.** Record one against
  the current (fixed) build before submission.
- **`simultaneous_failure_outage.yaml` reaches 0.89 completion** at its
  configured 120s duration after the isolation-recovery fix and a
  `radio_range_m` tuning pass (180 -> 220m, applied to all six scenarios) —
  see `CHALLENGE_MAPPING.md`'s "Tuning note" for the diagnosis and rationale.
  Previously it stalled at 0.44 due to group-wide isolation the swarm's own
  survey-task movement caused, not the scripted disturbance itself.
- **Communication resilience has a hard cliff, not a graceful curve,** past
  ~10-15% base packet loss (0.89 -> 0.11 completion at 0.2 loss). This is a
  genuine, disclosed limitation of the current relay-selection heuristic and
  a natural candidate for the CCPL-based policy layer to address in Stage 2,
  if pursued.
- **CCPL is integrated but not the reference path for Stage 1**, per the
  project's own risk-management decision (documented in `TECHNICAL_PROPOSAL.md`
  and this project's planning history) — this is a deliberate choice, not a
  gap, but worth stating explicitly to reviewers so it reads as a considered
  trade-off rather than an unfinished feature.
