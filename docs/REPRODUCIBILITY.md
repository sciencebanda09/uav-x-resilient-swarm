# Reproducibility

The authoritative path requires Python only (thread caps documented for
memory-constrained machines):

```powershell
$env:OPENBLAS_NUM_THREADS='1'; $env:OMP_NUM_THREADS='1'; $env:MPLBACKEND='Agg'
python -m pip install -r requirements.txt
python -m uav_x.simulation.runner --seed 7 --duration 2700
python -m uav_x.visualization.operations_2d runs/uav_x_run.jsonl
```

For long runs, flush the canonical log incrementally:

```powershell
py -3.11 -m uav_x.simulation.runner --duration 2700 --stream --out runs\streamed.jsonl
```

Generate the complete benchmark report:

```powershell
python -m uav_x.benchmarks --all --duration 2700 --out reports/stage1_benchmark.json
python -m uav_x.validation --duration 2700 --out reports/validation.json
```

The benchmark uses seeds 7, 17, and 27 across six challenge-facing scenarios
(45-minute mission horizon each) and writes per-seed plus aggregate statistics.

For the complete Stage 1 evidence package, use one command
(~100 machine-minutes for the full mission suite; the smoke path is
`pytest -q` plus a single short run above):

```powershell
python tools\run_stage1.py
```

This additionally runs randomized hidden-disturbance cases and controlled
ablations, then generates a canonical failure/recovery replay and video at
`artifacts/`. MATLAB-capable machines use the native `VideoWriter` path; the
portable GIF renderer is retained as a fallback for headless environments.

The canonical log format is `uav-x-log/v1` JSONL. Every run records its seed, simulator parameters, per-tick UAV/PoI/link state, events, survey coverage, and final metrics. Re-running with the same seed and configuration produces identical records.

MATLAB is optional and consumes the frozen log after the Python run.
`matlab/*.m` scripts read the same schema (additive report/landing fields
are ignored by older scripts):

```matlab
operations_2d('run.jsonl')
replay_3d('run.jsonl')
plot_metrics('run.jsonl')
```

For the Stage 1 failure/recovery movie, `tools/run_matlab_video.py` first tries
the official MATLAB Engine API for Python and then falls back to the MATLAB
command-line executable. Both paths call the MATLAB-native renderer and
`VideoWriter`; the portable GIF is used only when MATLAB cannot export in the
current environment.

The `auto` policy uses standalone CCPL when installed and records an explicit fallback otherwise. To require CCPL, install `requirements-ccpl.txt` and pass `--policy ccpl`. The heuristic path remains available for CI and offline reproduction.

To train and evaluate a CCPL checkpoint:

```powershell
python -m pip install -r requirements-ccpl.txt
python -m uav_x.train_ccpl --episodes 25
python -m uav_x.benchmarks --duration 2700
```
