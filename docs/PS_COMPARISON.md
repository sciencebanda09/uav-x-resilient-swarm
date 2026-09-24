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
| Survey all assigned disaster locations | Met (16/18 runs at 1.0) | 10 random PoIs over 2700 s: 1.0 in baseline, outage, emergency, recharge; 0.9 in 2/18 failure runs (dead-carrier imagery requeued, clock runs out). |
| Maintain end-to-end communication with the GCS | Met as ferrying + mesh capability | Multi-hop graph + reliability routing + retransmission; connected PDR 0.991–1.0; raw availability/PDR 0.28–0.45 is the disclosed ferrying signature at 6×100 m over 1 km². |
| Dynamically assign relay UAVs | Met | RELAY self-election with bid cooldown + taskless RECOVER with GCS-direct escape; counted in `relay_reallocations`. |
| Reconfigure network on degradation/failure/recharge | Met | Six 2700 s scenarios; ferrying holds tasks through isolation; reserve/CHARGE/recall cycle reuses vehicles. |
| Prioritize newly emerging high-priority regions | Met | Priority-1 injection at 1200 s preferentially picked up; `emergency_response_s_max` 163–641 s reported. |
| Complete the mission safely and within allotted time | Met | 1.0 completion at 1032–1841 s (inside 2700 s), 6/6 landed (or 5/5 + failed), zero safety violations in all 18 runs. |

## 2. Format of Competition — Stage 1 deliverables

| PS requirement | Status | Notes |
|---|---|---|
| 6-8 page technical proposal | **Met** | `docs/TECHNICAL_PROPOSAL.md` is ~3100 words (~6.9 pages with tables/figures); only team names/affiliation need filling before email. |
| Software architecture | Met | Described in `TECHNICAL_PROPOSAL.md` and reflected in the actual module layout (`uav_x/core`, `simulation`, `metrics`, `interfaces`, `visualization`). |
| Working proof-of-concept simulation | Met | Confirmed running end-to-end: `runner` -> `operations_2d`/video -> `validation`, 30/30 tests, 18/18 benchmark runs with zero safety violations. |
| Source code | Met | Present and tested (30 tests passing; optional CCPL coverage remains separate from the heuristic path). |
| Installation instructions | Met | README quick-start verified to work exactly as written on a clean environment. |
| Demonstration video | **Met (un-narrated cut)** | `artifacts/stage1_full_demo.mp4` (~16 s, 1280×720, canonical-log replay); narrated voiceover still to record against the frozen build. |

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

## 4. Remaining known gaps (figure retune complete; flagged for the team)

- **Figure retune landed.** World is now 1000 m / 100 m / 5 m/s / 20 m / 2700 s
  with GCS outside, ground takeoff, random 10-PoI spawn, ferrying reports,
  recall landing, and report/landing metrics. Benchmark: 16/18 at 1.0,
  zero collisions/battery/geofence violations, full landing roll-call.
- **10 s report bound is measured, not met** (max 518–806 s on far PoIs):
  ferry physics at this scale. Disclosed in proposal §9 and mapping doc.
- **Narrated video + team names** are the only manual items left before email.
- **Communication resilience still degrades at high base packet loss.** The
  default bounded retransmission policy substantially improves delivery, but
  the sensitivity report remains the authoritative evidence for the operating
  boundary rather than claiming perfect service under severe loss.
- **CCPL is integrated but not the reference path for Stage 1**, per the
  project's own risk-management decision (documented in `TECHNICAL_PROPOSAL.md`
  and this project's planning history) — this is a deliberate choice, not a
  gap, but worth stating explicitly to reviewers so it reads as a considered
  trade-off rather than an unfinished feature.
