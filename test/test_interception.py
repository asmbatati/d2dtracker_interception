"""Unit tests for the interception strategy library (no ROS runtime needed)."""
from d2dtracker_interception.interception_modes import (
    closest_interception_sample, distance2D, interceptor_trajectory)
from d2dtracker_interception.interception_point import (
    find_optimal_intersection, reach_time)


def test_reach_time_monotonic_with_distance():
    near = reach_time([0, 0, 0, 0, 0, 0], [1, 0, 0], vmax=5.0, amax=2.0)
    far = reach_time([0, 0, 0, 0, 0, 0], [50, 0, 0], vmax=5.0, amax=2.0)
    assert 0.0 < near < far


def test_reach_time_zero_distance():
    assert reach_time([3, 3, 3, 0, 0, 0], [3, 3, 3], vmax=5.0, amax=2.0) == 0.0


def test_find_optimal_intersection_picks_feasible_sample():
    # Target marches away in +x at 0.5 m/s sampled every 1 s; fast interceptor.
    states = [[float(k), 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0] for k in range(20)]
    interceptor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    state, idx = find_optimal_intersection(states, interceptor, dt=1.0,
                                           vmax=10.0, amax=5.0)
    assert state is not None
    assert 0 <= idx < len(states)


def test_find_optimal_intersection_empty():
    state, idx = find_optimal_intersection([], [0] * 9, dt=0.1, vmax=8.0, amax=4.0)
    assert state is None and idx == -1


def test_interceptor_trajectory_moves_toward_target():
    traj = interceptor_trajectory(10.0, 0.0, 2.0, 0.0, 0.0, vint=0.0,
                                  vmax=5.0, amax=2.0, dt=0.1, mpc_horizon_length=20)
    assert len(traj) == 20
    # Each waypoint should get closer to the target in x.
    assert traj[-1][0] > traj[0][0]
    assert all(abs(z - 2.0) < 1e-9 for (_, _, z) in traj)


def test_closest_interception_sample_returns_index_or_minus_one():
    xs = [float(k) for k in range(10)]
    ys = [0.0] * 10
    zs = [0.0] * 10
    idx = closest_interception_sample(xs, ys, zs, 0.0, 0.0, 0.0,
                                      vmax=10.0, amax=5.0, dt=1.0)
    assert idx == -1 or 0 <= idx < 10


def test_distance2d():
    assert distance2D(0, 0, 3, 4) == 5.0
