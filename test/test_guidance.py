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
