"""Incremental JSONL recording for long runs without retaining all frames."""
from __future__ import annotations
import json
from pathlib import Path
from typing import Any
from .log_schema import LogEnvelope, validate_record

class StreamingRecorder:
    def __init__(self, path: str, retain_records: bool = True) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("w", encoding="utf-8")
        self.retain_records = retain_records
        self.records: list[dict[str, Any]] = []

    def write(self, item: LogEnvelope | dict[str, Any]) -> dict[str, Any]:
        record = item.to_dict() if isinstance(item, LogEnvelope) else item
        validate_record(record)
        self.handle.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.handle.flush()
        if self.retain_records:
            self.records.append(record)
        return record

    def close(self) -> None:
        if not self.handle.closed:
            self.handle.close()

    def __enter__(self) -> "StreamingRecorder":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

class RecordingList(list):
    """List-compatible sink that mirrors appended records to disk."""
    def __init__(self, recorder: StreamingRecorder | None = None) -> None:
        super().__init__()
        self.recorder = recorder

    def append(self, item) -> None:
        record = item.to_dict() if isinstance(item, LogEnvelope) else item
        if self.recorder is not None:
            self.recorder.write(record)
        super().append(record)
