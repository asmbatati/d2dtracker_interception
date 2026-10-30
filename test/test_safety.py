"""Unit tests for the CBF velocity safety filter."""
from d2dtracker_interception.safety import CBFVelocityFilter
import numpy as np


def test_safe_command_passes_unchanged():
    f = CBFVelocityFilter(z_min=1.0, alpha=1.0)
    v, active = f.apply([0.0, 0.0, 5.0], [3.0, 0.0, -1.0])
    assert not active
    assert np.allclose(v, [3.0, 0.0, -1.0])


def test_descent_is_limited_near_the_floor():
    # 0.5 m above the floor with alpha 2: descent capped at 1 m/s, the
    # horizontal part of the command is kept.
    f = CBFVelocityFilter(z_min=1.0, alpha=2.0)
    v, active = f.apply([0.0, 0.0, 1.5], [4.0, -2.0, -6.0])
    assert active
    assert np.allclose(v, [4.0, -2.0, -1.0])


def test_below_floor_forces_climb():
    f = CBFVelocityFilter(z_min=1.0, alpha=1.0)
    v, _ = f.apply([0.0, 0.0, 0.4], [1.0, 0.0, -3.0])
    assert v[2] >= 0.6 - 1e-9


def test_geofence_removes_outward_radial_velocity_at_boundary():
    f = CBFVelocityFilter(z_min=-np.inf, fence_radius=10.0, alpha=1.0)
    v, active = f.apply([10.0, 0.0, 5.0], [5.0, 3.0, 0.0])
    assert active
    assert v[0] <= 1e-9          # no outward motion on the fence
    assert np.isclose(v[1], 3.0)  # tangential motion kept


def test_floor_and_fence_together_and_speed_cap():
    f = CBFVelocityFilter(z_min=1.0, fence_radius=10.0, alpha=1.0, vmax=4.0)
    p = [9.9, 0.0, 1.05]
    v, _ = f.apply(p, [8.0, 0.0, -8.0])
    A, b, _ = f.constraints(p)
    assert np.all(A @ v >= b - 1e-7)
    assert np.linalg.norm(v) <= 4.0 + 1e-9


def test_filtered_rollout_never_breaks_the_floor():
    # Integrate a command that dives at 6 m/s: the filtered state approaches
    # the floor but never crosses it (forward invariance, discretized).
    f = CBFVelocityFilter(z_min=1.0, alpha=2.0)
    p = np.array([0.0, 0.0, 4.0])
    for _ in range(400):
        v, _ = f.apply(p, [2.0, 0.0, -6.0])
        p = p + 0.02 * v
        assert p[2] >= 1.0 - 1e-6
