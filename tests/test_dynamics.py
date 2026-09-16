import numpy as np
from uav_x.core.models import UAV, advance_dynamics


def test_hexacopter_dynamics_produces_attitude_and_six_motor_output():
    u = UAV("UAV-01", np.array([0.0, 0.0, 50.0]))
    advance_dynamics(u, np.array([40.0, 20.0, 55.0]), 1.0)
    assert u.vehicle_type == "heavy_lift_hexacopter"
    assert u.motor_thrust_n.shape == (6,)
    assert np.linalg.norm(u.velocity) > 0
    assert np.linalg.norm(u.attitude_rpy_rad[:2]) <= u.max_tilt_rad + 1e-9


def test_failed_hexacopter_is_stationary():
    u = UAV("UAV-01", np.array([0.0, 0.0, 50.0]), failed=True)
    advance_dynamics(u, np.array([40.0, 20.0, 55.0]), 1.0)
    assert np.allclose(u.velocity, 0)
    assert np.allclose(u.motor_thrust_n, 0)
