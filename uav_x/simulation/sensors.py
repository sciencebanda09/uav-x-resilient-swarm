"""Deterministic lightweight sensor models for replay and autonomy experiments."""
from __future__ import annotations
import numpy as np
from ..core.models import UAV

class SensorModel:
    def __init__(self, seed: int, gps_noise_m: float = 0.0,
                 imu_noise_mps2: float = 0.0, mode: str = "metadata",
                 camera_fov_deg: float = 70.0) -> None:
        self.rng = np.random.default_rng(seed)
        self.gps_noise_m = max(0.0, float(gps_noise_m))
        self.imu_noise_mps2 = max(0.0, float(imu_noise_mps2))
        self.camera_fov_deg = float(np.clip(camera_fov_deg, 20.0, 120.0))
        if mode not in {"off", "metadata", "camera_metadata"}:
            raise ValueError("sensor_mode must be off, metadata, or camera_metadata")
        self.mode = mode

    def read(self, uav: UAV) -> dict:
        if self.mode == "off":
            return {"gps_position_m": None, "imu_acceleration_mps2": None,
                    "camera": None}
        gps = uav.position + self.rng.normal(0.0, self.gps_noise_m, 3)
        imu = uav.acceleration_mps2 + self.rng.normal(0.0, self.imu_noise_mps2, 3)
        camera = None
        if self.mode == "camera_metadata":
            footprint = 2.0 * max(0.0, float(uav.position[2])) * np.tan(np.radians(self.camera_fov_deg) / 2.0)
            camera = {"frame_available": True, "width": 320, "height": 180,
                      "format": "metadata-only", "fov_deg": round(self.camera_fov_deg, 2),
                      "ground_footprint_diameter_m": round(float(footprint), 3)}
        return {"gps_position_m": [round(float(x), 4) for x in gps],
                "imu_acceleration_mps2": [round(float(x), 4) for x in imu],
                "camera": camera}
