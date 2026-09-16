# Reproducibility

The authoritative path requires Python only:

```powershell
python -m pip install -r requirements.txt
python -m uav_x.simulation.runner --seed 7 --duration 120
python -m uav_x.visualization.operations_2d runs/uav_x_run.jsonl
```

The canonical log format is `uav-x-log/v1` JSONL. Every run records its seed, simulator parameters, per-tick UAV/PoI/link state, events, and final metrics. Re-running with the same seed and configuration produces identical records.

MATLAB is optional and consumes the frozen log after the Python run:

```matlab
operations_2d('run.jsonl')
replay_3d('run.jsonl')
plot_metrics('run.jsonl')
```

The `auto` policy uses standalone CCPL when installed and records an explicit fallback otherwise. To require CCPL, install `requirements-ccpl.txt` and pass `--policy ccpl`. The heuristic path remains available for CI and offline reproduction.

To train and evaluate a CCPL checkpoint:

```powershell
python -m pip install -r requirements-ccpl.txt
python -m uav_x.train_ccpl --episodes 25
python -m uav_x.benchmarks --duration 120
```
