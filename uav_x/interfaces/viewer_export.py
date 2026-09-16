"""Convert canonical UAV-X JSONL logs into browser-friendly replay JSON."""
from __future__ import annotations
import argparse, json
from pathlib import Path
from .log_schema import read_jsonl

def export(log_path: str, output: str = "runs/replay.json") -> None:
    records = read_jsonl(log_path)
    payload = {"schema_version": "uav-x-viewer/v1", "manifest": next((r for r in records if r["record_type"] == "manifest"), {}), "ticks": [r for r in records if r["record_type"] == "tick"], "events": [r for r in records if r["record_type"] == "event"], "summary": next((r for r in records if r["record_type"] == "summary"), {})}
    Path(output).parent.mkdir(parents=True, exist_ok=True); Path(output).write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="runs/replay.json"); a = p.parse_args(); export(a.log, a.out)
