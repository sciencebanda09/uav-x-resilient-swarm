"""Domain models for the civilian disaster-response swarm."""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np

@dataclass
class UAV:
    uid: str
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    battery_pct: float = 100.0
    role: str = "STANDBY"
    task_id: str | None = None
    mode: str = "CONNECTED"
    speed_mps: float = 12.0
    home: np.ndarray | None = None
    failed: bool = False
    vehicle_type: str = "heavy_lift_hexacopter"
    attitude_rpy_rad: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity_rps: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    motor_thrust_n: np.ndarray = field(default_factory=lambda: np.zeros(6))
    wind_mps: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mass_kg: float = 8.5
    max_thrust_n: float = 180.0
    max_tilt_rad: float = math.radians(28.0)
    max_yaw_rate_rps: float = math.radians(70.0)
    climb_rate_mps: float = 8.0
    battery_capacity_wh: float = 720.0
    energy_used_wh: float = 0.0
    drag_area_m2: float = 0.55
    propulsion_efficiency: float = 0.72

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        self.attitude_rpy_rad = np.asarray(self.attitude_rpy_rad, dtype=float)
        self.angular_velocity_rps = np.asarray(self.angular_velocity_rps, dtype=float)
        self.acceleration_mps2 = np.asarray(self.acceleration_mps2, dtype=float)
        self.motor_thrust_n = np.asarray(self.motor_thrust_n, dtype=float)
        self.wind_mps = np.asarray(self.wind_mps, dtype=float)
        self.home = self.position.copy() if self.home is None else np.asarray(self.home, dtype=float)

    def distance_to(self, point: np.ndarray) -> float:
        return float(np.linalg.norm(self.position - point))

@dataclass
class PoI:
    pid: str
    position: np.ndarray
    priority: int = 3
    status: str = "PENDING"
    assigned_uav_id: str | None = None
    survey_progress: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)

@dataclass
class GCS:
    position: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)

@dataclass
class ScenarioConfig:
    seed: int = 7
    duration_s: float = 120.0
    dt: float = 1.0
    arena_m: float = 500.0
    radio_range_m: float = 180.0
    min_separation_m: float = 18.0
    reserve_pct: float = 22.0
    packet_loss: float = 0.04
    failure_time_s: float | None = None
    outage_start_s: float | None = None
    outage_duration_s: float = 0.0
    emergency_time_s: float | None = 65.0
    geofence_margin_m: float = 2.0
    initial_battery_pct: float = 100.0
    return_home_time_s: float | None = None
    fleet_size: int = 6
    payload_kg: float = 1.5
    wind_scale: float = 1.0
    gps_noise_m: float = 0.0
    imu_noise_mps2: float = 0.0
    telemetry_delay_ticks: int = 0
    radio_bandwidth_kbps: float = 1800.0
    packet_queue_kb: float = 256.0
    retransmit_limit: int = 0
    sensor_mode: str = "metadata"
    survey_altitude_agl_m: float = 50.0
    camera_fov_deg: float = 70.0
    survey_dwell_s: float = 8.0

    @property
    def ticks(self) -> int:
        return int(self.duration_s / self.dt)

def clamp_move(uav: UAV, destination: np.ndarray, dt: float) -> None:
    delta = np.asarray(destination) - uav.position
    distance = float(np.linalg.norm(delta))
    if distance < 1e-9:
        uav.velocity[:] = 0
        return
    step = min(distance, uav.speed_mps * dt)
    uav.velocity = delta / distance * (step / dt)
    uav.position += delta / distance * step

def advance_dynamics(uav: UAV, destination: np.ndarray, dt: float) -> None:
    """Deterministic, bounded heavy-lift hexacopter approximation.

    This is a kinematic rigid-body approximation: acceleration, tilt, yaw,
    thrust, drag, and wind are integrated without an external physics engine.
    Safety constraints are applied by the caller before this function.
    """
    if uav.failed:
        uav.velocity[:] = 0.0; uav.acceleration_mps2[:] = 0.0; uav.motor_thrust_n[:] = 0.0
        return
    dt = max(float(dt), 1e-6)
    error = np.asarray(destination, dtype=float) - uav.position
    desired = error * 1.8 - uav.velocity * 0.75
    desired[2] = float(np.clip(desired[2], -uav.climb_rate_mps, uav.climb_rate_mps))
    desired[:2] = np.clip(desired[:2], -uav.speed_mps, uav.speed_mps)
    battery_factor = float(np.clip((uav.battery_pct - 5.0) / 95.0, 0.35, 1.0))
    max_accel = 7.5 * battery_factor
    acceleration = desired - uav.velocity
    norm = float(np.linalg.norm(acceleration))
    if norm > max_accel: acceleration *= max_accel / norm
    # Quadratic aerodynamic drag acts on air-relative velocity.  This keeps
    # wind useful as a disturbance without treating it as an arbitrary force.
    air_relative = uav.velocity - uav.wind_mps
    drag_factor = 0.5 * 1.225 * uav.drag_area_m2 / max(uav.mass_kg, 0.1)
    acceleration += -drag_factor * air_relative * float(np.linalg.norm(air_relative))
    uav.acceleration_mps2 = acceleration
    uav.velocity += acceleration * dt
    speed_limit = uav.speed_mps * battery_factor
    speed = float(np.linalg.norm(uav.velocity))
    if speed > speed_limit: uav.velocity *= speed_limit / speed
    uav.position += uav.velocity * dt
    horizontal = float(np.linalg.norm(uav.acceleration_mps2[:2]))
    target_pitch = float(np.clip(-uav.acceleration_mps2[0] / 9.81, -1, 1) * uav.max_tilt_rad)
    target_roll = float(np.clip(uav.acceleration_mps2[1] / 9.81, -1, 1) * uav.max_tilt_rad)
    heading = math.atan2(float(error[1]), float(error[0])) if np.linalg.norm(error[:2]) > 1e-6 else uav.attitude_rpy_rad[2]
    yaw_error = (heading - uav.attitude_rpy_rad[2] + math.pi) % (2 * math.pi) - math.pi
    yaw_rate = float(np.clip(yaw_error / dt, -uav.max_yaw_rate_rps, uav.max_yaw_rate_rps))
    uav.angular_velocity_rps = np.array([(target_roll-uav.attitude_rpy_rad[0])/dt,
                                          (target_pitch-uav.attitude_rpy_rad[1])/dt, yaw_rate])
    uav.attitude_rpy_rad += uav.angular_velocity_rps * dt * 0.35
    uav.attitude_rpy_rad[:2] = np.clip(uav.attitude_rpy_rad[:2], -uav.max_tilt_rad, uav.max_tilt_rad)
    uav.attitude_rpy_rad[2] = (uav.attitude_rpy_rad[2] + math.pi) % (2*math.pi) - math.pi
    # Total thrust must balance gravity at hover; do not add gravity twice.
    thrust = (uav.acceleration_mps2[2] + 9.81) * uav.mass_kg / 6.0
    thrust = float(np.clip(thrust, 0.0, uav.max_thrust_n / 6.0 * battery_factor))
    uav.motor_thrust_n[:] = thrust

def battery_step(uav: UAV, dt: float) -> None:
    return battery_step_with_load(uav, dt)

def battery_step_with_load(uav: UAV, dt: float, payload_kg: float = 0.0,
                           wind_scale: float = 1.0) -> None:
    dt = max(float(dt), 0.0)
    speed = float(np.linalg.norm(uav.velocity))
    airspeed = float(np.linalg.norm(uav.velocity - uav.wind_mps * max(0.0, wind_scale)))
    climb = abs(float(uav.velocity[2]))
    mass = uav.mass_kg + max(0.0, float(payload_kg))
    gravity = mass * 9.81
    # Induced power for a small multirotor disk, plus profile/parasite power.
    rotor_disk_area = 0.72
    induced = gravity * math.sqrt(gravity / max(2.0 * 1.225 * rotor_disk_area, 1e-6))
    hover_power = induced / max(uav.propulsion_efficiency, 0.2)
    parasite_power = 0.5 * 1.225 * uav.drag_area_m2 * airspeed**3
    climb_power = gravity * climb / max(uav.propulsion_efficiency, 0.2)
    power_w = max(80.0, hover_power + parasite_power + climb_power)
    energy_wh = power_w * dt / 3600.0
    uav.energy_used_wh += energy_wh
    capacity = max(float(uav.battery_capacity_wh), 1.0)
    uav.battery_pct = max(0.0, uav.battery_pct - energy_wh / capacity * 100.0)

def safe_destination(current: np.ndarray, destination: np.ndarray, others: list[np.ndarray],
                     minimum_m: float, arena_m: float, margin_m: float = 2.0) -> tuple[np.ndarray, list[str]]:
    """Predictively constrain a waypoint before movement."""
    target = np.asarray(destination, dtype=float).copy()
    events: list[str] = []
    clipped = target.copy()
    clipped[:2] = np.clip(clipped[:2], margin_m, arena_m - margin_m)
    if not np.allclose(clipped, target):
        events.append("geofence")
        target = clipped
    for other in others:
        delta = target - other
        distance = float(np.linalg.norm(delta))
        if distance < minimum_m:
            direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
            target = other + direction * minimum_m
            target[:2] = np.clip(target[:2], margin_m, arena_m - margin_m)
            events.append("separation")
    return target, events
