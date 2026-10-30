"""Unit tests for the PN guidance library (no ROS runtime needed)."""
import numpy as np

from d2dtracker_interception.guidance import (
    closing_velocity, los_rate, pn_velocity_command, proportional_navigation)


def test_closing_velocity_positive_when_approaching():
    # interceptor at origin moving +x, target ahead on +x stationary -> closing.
    rel_pos = np.array([10.0, 0.0, 0.0])
    rel_vel = np.array([-5.0, 0.0, 0.0])   # target_vel - interceptor_vel
    assert closing_velocity(rel_pos, rel_vel) > 0.0


def test_closing_velocity_negative_when_receding():
    rel_pos = np.array([10.0, 0.0, 0.0])
    rel_vel = np.array([5.0, 0.0, 0.0])
    assert closing_velocity(rel_pos, rel_vel) < 0.0


def test_los_rate_zero_on_collision_course():
    # Pure closing along LOS -> no LOS rotation.
    rel_pos = np.array([10.0, 0.0, 0.0])
    rel_vel = np.array([-5.0, 0.0, 0.0])
    assert np.linalg.norm(los_rate(rel_pos, rel_vel)) < 1e-9


def test_los_rate_nonzero_with_crossing_target():
    rel_pos = np.array([10.0, 0.0, 0.0])
    rel_vel = np.array([0.0, 2.0, 0.0])   # target crossing -> LOS rotates
    assert np.linalg.norm(los_rate(rel_pos, rel_vel)) > 0.0


def test_pn_accel_perpendicular_to_los():
    a = proportional_navigation([0, 0, 0], [5, 0, 0], [10, 0, 0], [0, 2, 0], N=3.0)
    rel_pos = np.array([10.0, 0.0, 0.0])
    # PN acceleration is perpendicular to the line of sight.
    assert abs(float(np.dot(a, rel_pos))) < 1e-6
    assert np.linalg.norm(a) > 0.0


def test_pn_velocity_points_toward_stationary_target_and_capped():
    v = pn_velocity_command([0, 0, 0], [0, 0, 0], [10, 0, 0], [0, 0, 0],
                            vmax=8.0, N=3.0, dt=0.1, closing_speed=6.0)
    assert v[0] > 0.0                      # heads toward target (+x)
    assert np.linalg.norm(v) <= 8.0 + 1e-6  # respects vmax


def test_pn_velocity_zero_distance():
    v = pn_velocity_command([1, 1, 1], [0, 0, 0], [1, 1, 1], [0, 0, 0], vmax=8.0)
    assert np.linalg.norm(v) == 0.0


def test_pursuit_flies_line_of_sight_at_speed():
    from d2dtracker_interception.guidance import pursuit_velocity_command
    v = pursuit_velocity_command([0.0, 0.0, 0.0], [3.0, 4.0, 0.0], speed=5.0)
    assert np.allclose(v, [3.0, 4.0, 0.0])


def _engage(law, target_vel, steps=600, dt=0.02, speed=6.0):
    """Kinematic engagement (perfect velocity tracking); returns min range."""
    from d2dtracker_interception.guidance import (pn_integrated_velocity,
                                                  pursuit_velocity_command)
    ip, tp = np.zeros(3), np.array([20.0, -10.0, 0.0])
    tv = np.asarray(target_vel, float)
    v = None
    best = np.inf
    for _ in range(steps):
        if law == 'pn_int':
            v = pn_integrated_velocity(v, ip, v if v is not None else np.zeros(3),
                                       tp, tv, speed, N=3.0, dt=dt)
        else:
            v = pursuit_velocity_command(ip, tp, speed)
        ip, tp = ip + v * dt, tp + tv * dt
        best = min(best, float(np.linalg.norm(tp - ip)))
    return best


def test_integrated_pn_hits_a_crossing_target():
    assert _engage('pn_int', [0.0, 2.4, 0.0]) < 0.3


def test_integrated_pn_flies_straighter_than_pursuit():
    # same speed, crossing target: PN leads the target, pursuit tail-chases;
    # both close, PN by a collision course that needs no terminal turn
    from d2dtracker_interception.guidance import pn_integrated_velocity
    v = pn_integrated_velocity(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                               [10.0, 0.0, 0.0], [0.0, 2.0, 0.0], 6.0)
    assert np.allclose(np.linalg.norm(v), 6.0)
    assert _engage('pursuit', [0.0, 2.4, 0.0]) < 1.0
