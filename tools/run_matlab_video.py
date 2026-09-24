"""Invoke the MATLAB-native Stage 1 video renderer from Python.

This is intentionally a thin bridge: MATLAB owns scene rendering and
VideoWriter; Python owns the canonical-log pipeline and artifact paths.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def find_matlab() -> Path | None:
    candidates = []
    if os.environ.get("MATLAB_ROOT"):
        candidates.append(Path(os.environ["MATLAB_ROOT"]) / "bin" / "matlab.exe")
    candidates.extend((Path(r"D:\matlab\bin\matlab.exe"),
                       Path(r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe")))
    return next((path for path in candidates if path.exists()), None)


def render_with_engine(matlab_dir: str, log_path: str, out_path: str) -> bool:
    """Use the official MATLAB Engine API when it is installed.

    The engine is optional because MathWorks ships it with MATLAB rather than
    PyPI.  Keeping this path first means Python still owns orchestration while
    MATLAB owns figure rendering and VideoWriter.
    """
    if importlib.util.find_spec("matlab.engine") is None:
        return False
    import matlab.engine  # type: ignore[import-not-found]

    engine = None
    try:
        engine = matlab.engine.start_matlab("-nodesktop -nosplash")
        engine.addpath(matlab_dir, nargout=0)
        engine.uav_x_failure_recovery_2d(log_path, out_path, nargout=0)
        return True
    finally:
        if engine is not None:
            engine.quit()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--out", default="artifacts/stage1_failure_recovery_matlab.mp4")
    parser.add_argument("--matlab")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--no-engine", action="store_true",
                        help="skip the MATLAB Engine API and use matlab.exe")
    args = parser.parse_args()
    matlab = Path(args.matlab) if args.matlab else find_matlab()
    engine_available = (not args.no_engine and
                        importlib.util.find_spec("matlab.engine") is not None)
    if (matlab is None or not matlab.exists()) and not engine_available:
        print("MATLAB executable not found")
        return 2
    output = (ROOT / args.out).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()
    matlab_dir = (ROOT / "matlab").resolve().as_posix().replace("'", "''")
    log_path = (ROOT / args.log).resolve().as_posix().replace("'", "''")
    out_path = output.as_posix().replace("'", "''")
    if not args.no_engine:
        try:
            if render_with_engine(matlab_dir, log_path, out_path):
                if output.exists() and output.stat().st_size >= 100_000:
                    print(output)
                    return 0
                if output.exists():
                    output.unlink()
        except Exception as exc:  # noqa: BLE001 - CLI fallback is intentional
            print(f"MATLAB Engine unavailable: {exc}")

    expression = (f"addpath('{matlab_dir}'); "
                  f"uav_x_failure_recovery_2d('{log_path}','{out_path}');")
    try:
        result = subprocess.run([str(matlab), "-nosplash", "-batch", expression],
                                cwd=ROOT, check=False, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        print("MATLAB video render timed out")
        return 4
    if result.returncode != 0 or not output.exists() or output.stat().st_size < 100_000:
        if output.exists():
            output.unlink()
        return result.returncode or 3
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
