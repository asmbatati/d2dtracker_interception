"""Optimal interception-point estimation.

Given a sequence of *predicted* target states (sampled at a fixed time step)
and the interceptor's current state, find the earliest predicted target sample
the interceptor can physically reach in time. This is the rendezvous point the
interceptor should fly toward.

A state is the 9-vector ``[x, y, z, vx, vy, vz, ax, ay, az]``. Only the position
components are required by these routines; velocity/acceleration are accepted so
the same structure can be reused by richer models.
"""
import math


def reach_time(interceptor_state, target_position, vmax, amax):
    """Estimate the minimum time for the interceptor to fly to a point.

    Kinematic bang-bang estimate: accelerate from the current speed at ``amax``
    up to ``vmax``, then cruise. Returns time in seconds.
    """
    px, py, pz = interceptor_state[0], interceptor_state[1], interceptor_state[2]
    vx, vy, vz = interceptor_state[3], interceptor_state[4], interceptor_state[5]
    distance = math.sqrt((target_position[0] - px) ** 2 +
                         (target_position[1] - py) ** 2 +
                         (target_position[2] - pz) ** 2)
    if distance <= 0.0:
        return 0.0
    v0 = math.sqrt(vx * vx + vy * vy + vz * vz)
    if amax <= 0.0:
        return distance / max(vmax, 1e-6)

    # Distance covered while accelerating v0 -> vmax.
    t_accel = max(vmax - v0, 0.0) / amax
    d_accel = v0 * t_accel + 0.5 * amax * t_accel * t_accel
    if d_accel >= distance:
        # Reaches the point before hitting vmax: solve 0.5*a*t^2 + v0*t - d = 0.
        disc = v0 * v0 + 2.0 * amax * distance
        return (-v0 + math.sqrt(disc)) / amax
    # Accelerate, then cruise the remainder at vmax.
    return t_accel + (distance - d_accel) / max(vmax, 1e-6)


def time_to_intercept(interceptor_state, target_state, vmax, amax):
    """Backwards-compatible alias: reach time to a target *state*'s position."""
    return reach_time(interceptor_state, target_state[:3], vmax, amax)


def find_optimal_intersection(predicted_target_states, interceptor_state,
                              dt, vmax, amax):
    """Return the earliest reachable predicted target state (the rendezvous).

    ``predicted_target_states`` is an ordered list of 9-vectors sampled every
    ``dt`` seconds (index ``k`` occurs at ``t = k * dt``). The chosen state is
    the first one the interceptor can reach no later than the target does
    (``reach_time <= k * dt``). If none is feasible within the horizon, the last
    predicted state is returned (chase its furthest known point).

    Returns ``(state, index)`` or ``(None, -1)`` if the list is empty.
    """
    if not predicted_target_states:
        return None, -1

    for k, state in enumerate(predicted_target_states):
        t_target = k * dt
        if reach_time(interceptor_state, state[:3], vmax, amax) <= t_target:
            return state, k

    return predicted_target_states[-1], len(predicted_target_states) - 1


if __name__ == '__main__':
    demo_states = [
        [1, 2, 3, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
        [2, 3, 4, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0],
        [5, 6, 4, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
    ]
    interceptor = [0, 0, 0, 1, 1, 1, 0, 0, 0]
    state, idx = find_optimal_intersection(demo_states, interceptor,
                                           dt=0.5, vmax=8.0, amax=4.0)
    print(f'optimal intercept state={state} at index={idx}')
