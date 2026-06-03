"""Guidance laws for interception (no ROS dependency, easy to unit-test).

Proportional Navigation (PN) is the classic missile/interceptor guidance law:
command an acceleration proportional to the line-of-sight (LOS) rotation rate,
which drives the LOS rate to zero -> collision course. Here we expose true PN
and a helper that turns the PN acceleration into a velocity setpoint suitable
for a velocity-controlled multirotor.
"""
import numpy as np


def closing_velocity(rel_pos, rel_vel):
    """Return the closing speed (>0 means range is decreasing)."""
    dist = float(np.linalg.norm(rel_pos))
    if dist < 1e-6:
        return 0.0
    return float(-np.dot(rel_pos, rel_vel) / dist)


def los_rate(rel_pos, rel_vel):
    """Angular velocity vector of the line of sight: omega = (r x v) / (r.r)."""
    rr = float(np.dot(rel_pos, rel_pos))
    if rr < 1e-9:
        return np.zeros(3)
    return np.cross(rel_pos, rel_vel) / rr


def proportional_navigation(interceptor_pos, interceptor_vel,
                            target_pos, target_vel, N=3.0):
    """Return the true-PN commanded acceleration (perpendicular to the LOS).

    a_cmd = N * Vc * (omega x los_hat)
    """
    rel_pos = np.asarray(target_pos, float) - np.asarray(interceptor_pos, float)
    rel_vel = np.asarray(target_vel, float) - np.asarray(interceptor_vel, float)
    dist = float(np.linalg.norm(rel_pos))
    if dist < 1e-6:
        return np.zeros(3)
    los_hat = rel_pos / dist
    vc = closing_velocity(rel_pos, rel_vel)
    omega = los_rate(rel_pos, rel_vel)
    return N * vc * np.cross(omega, los_hat)


def pn_velocity_command(interceptor_pos, interceptor_vel, target_pos, target_vel,
                        vmax, N=3.0, dt=0.1, closing_speed=None):
    """PN as a velocity setpoint for a velocity-controlled multirotor.

    Combines a closing component along the LOS with the PN lateral correction,
    then caps the result at ``vmax``.
    """
    rel_pos = np.asarray(target_pos, float) - np.asarray(interceptor_pos, float)
    dist = float(np.linalg.norm(rel_pos))
    if dist < 1e-6:
        return np.zeros(3)
    los_hat = rel_pos / dist

    a_cmd = proportional_navigation(interceptor_pos, interceptor_vel,
                                    target_pos, target_vel, N)
    v = np.asarray(interceptor_vel, float) + a_cmd * dt
    # lateral component (PN-driven) + a guaranteed closing component along LOS
    v_lateral = v - np.dot(v, los_hat) * los_hat
    speed = vmax if closing_speed is None else closing_speed
    v_cmd = speed * los_hat + v_lateral

    n = float(np.linalg.norm(v_cmd))
    if n > vmax and n > 0.0:
        v_cmd = v_cmd * (vmax / n)
    return v_cmd
