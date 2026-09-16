"""Versioned JSONL records shared by simulation, metrics, and visualizers."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Iterable

SCHEMA_VERSION = "uav-x-log/v1"
ROLES = {"SURVEY", "RELAY", "RECOVER", "RETURN", "CHARGE", "STANDBY"}
UAV_MODES = {"CONNECTED", "DEGRADED", "ISOLATED", "RETURNING", "CHARGING"}
POI_STATUSES = {"PENDING", "ASSIGNED", "SURVEYED", "EXPIRED"}

@dataclass
class LogEnvelope:
    run_id: str
    record_type: str
    tick: int
    time_s: float
    schema_version: str = SCHEMA_VERSION
    payload: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"schema_version": self.schema_version, "run_id": self.run_id,
                "record_type": self.record_type, "tick": self.tick,
                "time_s": self.time_s, **self.payload}

def validate_record(record: dict[str, Any]) -> None:
    required = {"schema_version", "run_id", "record_type", "tick", "time_s"}
    missing = required - record.keys()
    if missing:
        raise ValueError(f"missing log fields: {sorted(missing)}")
    if record["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema {record['schema_version']!r}")
    if record["record_type"] not in {"manifest", "tick", "event", "summary"}:
        raise ValueError(f"invalid record_type {record['record_type']!r}")

def write_jsonl(path: str, records: Iterable[LogEnvelope | dict[str, Any]]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        for item in records:
            record = item.to_dict() if isinstance(item, LogEnvelope) else item
            validate_record(record)
            handle.write(json.dumps(record, separators=(",", ":")) + "\n")

def read_jsonl(path: str) -> list[dict[str, Any]]:
    with open(path, encoding="utf-8") as handle:
        records = [json.loads(line) for line in handle if line.strip()]
    for record in records:
        validate_record(record)
    return records
