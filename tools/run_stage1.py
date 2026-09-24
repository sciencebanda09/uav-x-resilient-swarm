"""One-command Stage 1 evidence pipeline.

Runs tests, deterministic benchmarks, 2700-second (45-minute) validation,
randomized hidden disturbances, ablations, a canonical failure/recovery log,
viewer export, and a portable judge-facing animated video.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run_step(name: str, args: list[str]) -> None:
    log_path = ROOT / "logs" / f"stage1_{name}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"[stage1] {name} ...", flush=True)
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT) + os.pathsep + environment.get("PYTHONPATH", "")
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run(args, cwd=ROOT, env=environment,
                                stdout=log, stderr=subprocess.STDOUT, check=False)
    if result.returncode:
        raise SystemExit(f"Stage 1 step failed: {name}. See {log_path}")
    print(f"[stage1] {name} complete", flush=True)


def relative(path: str) -> str:
    return str(Path(path).as_posix())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2700.0)
    parser.add_argument("--hidden-cases", type=int, default=12)
    parser.add_argument("--skip-video", action="store_true")
    args = parser.parse_args()

    py = sys.executable
    run_step("tests", [py, "-m", "pytest", "-q"])
    run_step("benchmark", [py, "-m", "uav_x.benchmarks", "--all",
                            "--duration", str(args.duration),
                            "--out", "reports/stage1_benchmark.json"])
    run_step("validation", [py, "-m", "uav_x.validation",
                             "--duration", str(args.duration),
                             "--out", "reports/validation.json"])
    run_step("hidden_disturbances", [py, "tools/hidden_disturbance_eval.py",
                                      "--cases", str(args.hidden_cases),
                                      "--duration", str(args.duration),
                                      "--out", "reports/hidden_disturbances.json"])
    run_step("ablations", [py, "tools/ablation_eval.py",
                            "--out", "reports/ablations.json"])
    run_step("failure_recovery_log", [py, "-m", "uav_x.simulation.runner",
                                       "--scenario", "scenarios/simultaneous_failure_outage.yaml",
                                       "--policy", "heuristic",
                                       "--out", "runs/stage1_failure_recovery.jsonl"])
    run_step("viewer_export", [py, "-m", "uav_x.interfaces.viewer_export",
                                "runs/stage1_failure_recovery.jsonl",
                                "--out", "runs/stage1_failure_recovery_replay.json"])
    video_artifact = "artifacts/stage1_failure_recovery.gif"
    if not args.skip_video:
        try:
            run_step("matlab_failure_recovery_video", [py, "tools/run_matlab_video.py",
                                                         "runs/stage1_failure_recovery.jsonl",
                                                         "--out", "artifacts/stage1_failure_recovery_matlab.mp4"])
            video_artifact = "artifacts/stage1_failure_recovery_matlab.mp4"
        except SystemExit:
            print("[stage1] MATLAB video unavailable; using portable Python fallback", flush=True)
            run_step("failure_recovery_video", [py, "tools/build_stage1_video.py",
                                                  "runs/stage1_failure_recovery.jsonl",
                                                  "--out", "artifacts/stage1_failure_recovery.gif"])

    report = {
        "suite": "uav-x-stage1-evidence/v1",
        "duration_s": args.duration,
        "hidden_cases": args.hidden_cases,
        "commands": {
            "tests": "python -m pytest -q",
            "benchmark": "python -m uav_x.benchmarks --all --duration 2700",
            "validation": "python -m uav_x.validation --duration 2700",
            "hidden_disturbances": "python tools/hidden_disturbance_eval.py --cases 12 --duration 2700",
            "ablations": "python tools/ablation_eval.py",
        },
        "reports": {
            "benchmark": relative("reports/stage1_benchmark.json"),
            "validation": relative("reports/validation.json"),
            "hidden_disturbances": relative("reports/hidden_disturbances.json"),
            "ablations": relative("reports/ablations.json"),
        },
        "artifacts": {
            "canonical_log": relative("runs/stage1_failure_recovery.jsonl"),
            "viewer_replay": relative("runs/stage1_failure_recovery_replay.json"),
            "failure_recovery_video": relative(video_artifact),
        },
    }
    validation = json.loads((ROOT / "reports/validation.json").read_text(encoding="utf-8"))
    hidden = json.loads((ROOT / "reports/hidden_disturbances.json").read_text(encoding="utf-8"))
    report["headline_metrics"] = {
        "validation_mean_completion": validation["monte_carlo"]["mean_completion"],
        "validation_mean_delivery": validation["monte_carlo"]["mean_delivery"],
        "hidden_pass_rate": hidden["pass_rate"],
    }
    (ROOT / "reports/stage1_package_summary.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report["headline_metrics"], indent=2))
    print("[stage1] evidence package complete")


if __name__ == "__main__":
    main()
