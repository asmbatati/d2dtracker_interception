"""Control-barrier-function (CBF) safety filter for velocity commands.

A velocity-controlled multirotor is modelled as a single integrator, p_dot = v.
Each safety constraint is a barrier h_i(p) >= 0; the filter returns the
velocity closest to the desired one that keeps every barrier forward-invariant:

    min_v ||v - v_des||^2   s.t.   grad h_i(p) . v >= -alpha * h_i(p)   for all i

Barriers (each enabled by a finite bound):

* altitude floor  h = z - z_min           (no ground strikes)
* ceiling         h = z_max - z
* geofence        h = R^2 - |p_xy - c|^2  (vertical cylinder of radius R)

With at most a handful of half-space constraints the QP is solved exactly by
enumerating active sets (KKT check), so the filter has no solver dependency.
v = 0 satisfies every constraint whenever p is inside the safe set (all
right-hand sides are <= 0 there), so the feasible set is a convex set that
contains the origin; scaling the result down to a speed cap keeps it feasible.
"""
from itertools import combinations

import numpy as np


class CBFVelocityFilter:
    """Minimal-change velocity filter for altitude / ceiling / geofence."""

    def __init__(self, z_min=1.0, z_max=float('inf'), fence_radius=float('inf'),
                 fence_center=(0.0, 0.0), alpha=1.0, vmax=None):
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.fence_radius = float(fence_radius)
        self.fence_center = np.asarray(fence_center, dtype=float)
        self.alpha = float(alpha)
        self.vmax = None if vmax is None else float(vmax)

    def constraints(self, p):
        """Return (A, b, h): rows a_i . v >= b_i, and the barrier values."""
        p = np.asarray(p, dtype=float)
        rows, rhs, hs = [], [], []
        if np.isfinite(self.z_min):
            h = p[2] - self.z_min
            rows.append([0.0, 0.0, 1.0])
            rhs.append(-self.alpha * h)
            hs.append(h)
        if np.isfinite(self.z_max):
            h = self.z_max - p[2]
            rows.append([0.0, 0.0, -1.0])
            rhs.append(-self.alpha * h)
            hs.append(h)
        if np.isfinite(self.fence_radius):
            d = p[:2] - self.fence_center
            h = self.fence_radius ** 2 - float(d @ d)
            rows.append([-2.0 * d[0], -2.0 * d[1], 0.0])
            rhs.append(-self.alpha * h)
            hs.append(h)
        return np.array(rows, dtype=float).reshape(-1, 3), np.array(rhs), np.array(hs)

    def apply(self, p, v_des):
        """Return (v_safe, active): the filtered command and whether it changed."""
        v_des = np.asarray(v_des, dtype=float)
        A, b, _ = self.constraints(p)
        v = self._solve(A, b, v_des)
        if self.vmax is not None:
            n = float(np.linalg.norm(v))
            if n > self.vmax > 0.0:
                v = v * (self.vmax / n)
        return v, bool(np.linalg.norm(v - v_des) > 1e-9)

    @staticmethod
    def _solve(A, b, v_des, tol=1e-9):
        if A.size == 0 or np.all(A @ v_des >= b - tol):
            return v_des
        best, best_cost = None, float('inf')
        m = A.shape[0]
        for k in range(1, m + 1):
            for S in combinations(range(m), k):
                As, bs = A[list(S)], b[list(S)]
                G = As @ As.T
                if abs(np.linalg.det(G)) < 1e-12:
                    continue
                lam = np.linalg.solve(G, bs - As @ v_des)
                if np.any(lam < -tol):
                    continue
                v = v_des + As.T @ lam
                if np.all(A @ v >= b - 1e-7):
                    cost = float(np.sum((v - v_des) ** 2))
                    if cost < best_cost:
                        best, best_cost = v, cost
        return best if best is not None else np.zeros(3)
