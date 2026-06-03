"""Geometric interception-trajectory strategies.

A small library of closed-form interception manoeuvres used by the interceptor
node. All routines operate on plain floats / lists, with no ROS dependency, so
they are easy to unit-test.
"""
import math


def distance2D(x1, y1, x2, y2):
    """Euclidean distance in the XY plane."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def dist3D(x1, y1, z1, x2, y2, z2):
    """Euclidean distance in 3D."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def yaw_to_tgt(x1, y1, x2, y2):
    """Heading (rad) from point 1 toward point 2."""
    return math.atan2((y2 - y1), (x2 - x1))


def interceptor_trajectory(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt,
                           mpc_horizon_length):
    """Algorithm 1: direct-attack trajectory toward the target.

    Builds an acceleration-limited straight-line path from the interceptor
    toward the target heading, holding altitude at ``ztgt``.
    """
    x, y, z = xint, yint, ztgt
    vel = vint
    phi_t = yaw_to_tgt(xint, yint, xtgt, ytgt)
    traj = []
    for _ in range(int(mpc_horizon_length)):
        vel = min(vel + amax * dt, vmax)
        x += vel * dt * math.cos(phi_t)
        y += vel * dt * math.sin(phi_t)
        traj.append((x, y, z))
    return traj


def closest_interception_sample(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt):
    """Algorithm 2: earliest reachable sample on a sampled target trajectory."""
    x, y = xint, yint
    vel = vint
    dflown = 0.0
    num_samples = len(xtgt)
    for i in range(num_samples):
        vel = min(vel + amax * dt, vmax)
        dflown += vel * dt
        if dflown > distance2D(x, y, xtgt[i], ytgt[i]):
            return i
    return -1


def find_reference_sample(xtgt, ytgt, ztgt, xp, yp, zp, dist):
    """Algorithm 3: first sample (from the path end) at arc-distance ``dist``."""
    i = -1
    tmpdist = dist3D(xtgt, ytgt, ztgt, xp[i], yp[i], zp[i])
    while tmpdist < dist and (-i) < len(xp):
        i -= 1
        tmpdist += dist3D(xp[i + 1], yp[i + 1], zp[i + 1], xp[i], yp[i], zp[i])
    return i


def reference_trajectory(xe, ye, ze, n, p, des_vel, dt, xint, yint, zint):
    """Algorithm 4: head-on collision trajectory toward the closest sample."""
    traj = []
    dist = dist3D(xe[p], ye[p], ze[p], xint, yint, zint)
    m = p
    for i in range(p - 1, -1, -1):
        tmpdist = dist3D(xe[i], ye[i], ze[i], xint, yint, zint)
        if tmpdist < dist:
            dist = tmpdist
            m = i
    traj.append((xe[m], ye[m], ze[m]))
    tmpdist = 0.0
    for i in range(m - 1, n, -1):
        tmpdist += dist3D(xe[i], ye[i], ze[i], xe[i - 1], ye[i - 1], ze[i - 1])
        if tmpdist > dt * des_vel:
            traj.append((xe[i], ye[i], ze[i]))
            tmpdist -= dt * des_vel
    return traj
