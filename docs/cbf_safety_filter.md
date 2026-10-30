# CBF safety filter for interception commands

First step toward "Geometric Tracking and Manifold-CBF for Certified Drone
Interception": a control-barrier-function filter that sits between any
interception law (PN, pursuit, a learned policy) and the autopilot, and
changes the command only as much as needed to keep the interceptor inside a
safe set. Code: `d2dtracker_interception/safety.py`; tests:
`test/test_safety.py`.

## Model and constraints

The interceptor is velocity-controlled (PX4 OFFBOARD velocity setpoints), so
the filter uses the single integrator `p_dot = v`. A constraint is a barrier
`h(p) >= 0`; the safe set is `C = {p : h_i(p) >= 0 for all i}`. With an
extended class-K function `alpha * h`, the CBF condition

    dh/dt = grad h(p) . v  >=  -alpha * h(p)

keeps `C` forward invariant: `h` may decay toward 0 no faster than
exponentially, so the vehicle approaches a boundary but does not cross it.

| barrier | h(p) | grad h |
|---|---|---|
| altitude floor | `z - z_min` | `(0, 0, 1)` |
| ceiling | `z_max - z` | `(0, 0, -1)` |
| cylindrical geofence | `R^2 - |p_xy - c|^2` | `(-2(x - cx), -2(y - cy), 0)` |

## The filter

    v* = argmin_v ||v - v_des||^2   s.t.   A v >= b,   A_i = grad h_i,  b_i = -alpha h_i

Each row is a half-space in velocity space, so this is the Euclidean
projection of `v_des` onto a convex polyhedron. With at most three or four rows,
`CBFVelocityFilter.apply` solves it exactly. It enumerates active sets,
solves `A_S A_S^T lambda = b_S - A_S v_des`, keeps solutions with
`lambda >= 0` that satisfy every row, and returns the closest one. There is no
solver dependency and the cost is a few microseconds.

Inside the safe set every `b_i <= 0`, so `v = 0` is feasible and the feasible
polyhedron contains the origin. Scaling the solution toward 0 for a speed cap
therefore stays feasible. Outside the safe set (e.g. below the floor) the
filter commands a return at rate `alpha * |h|`.

## Use

```bash
# guidance node (PN or pursuit), in the benchmark scene
ros2 launch d2dtracker_interception guidance.launch.py law:=pn cbf:=true cbf_z_min:=1.5 cbf_alpha:=1.0
# any policy in the RL evaluation env (learned or scripted)
ros2 run d2dtracker_rl evaluate_intercept --policy pn --cbf --target-policy evasive ...
ros2 run d2dtracker_rl evaluate_intercept --model .../model_evasive_40k.zip --cbf ...
```

What it is for here: the PN law flew into the ground after an overshoot in the
benchmark scene (z 3.8 → 0.1 m in 1.5 s at 6 m/s). With the floor barrier the
vertical part of such a command is clipped to `-alpha (z - z_min)` while the
horizontal pursuit is untouched.

## Toward the manifold version (open)

1. **Second-order / attitude-aware barriers.** The single integrator ignores
   the time PX4 needs to change velocity. A high-order CBF on the double
   integrator `p_ddot = a` (barrier on `h_dot + k h`), or a backup-set CBF
   over PX4's velocity loop, makes the certificate hold for the real response.
2. **Perception keep-in on SO(3).** Keep the target inside the camera's field
   of view: `h = cos(theta_fov/2) - b_cam(R) . (p_t - p)/|p_t - p|`, a barrier
   defined on the attitude manifold. That is where the geometric (SE(3))
   controller in `interception_control` comes in: it tracks attitude
   directly, so the barrier can act on `R` and `omega`, not only on `v`.
3. **Inter-vehicle barrier for capture.** Capture wants range 0, safety wants
   no collision. A barrier on relative speed at short range (e.g. `h = r - r_min`
   only for closing speed above a limit) would trade these off explicitly.
4. **Certificates.** Report the filter's activity (`active` flag) and the
   minimum barrier value per episode next to capture rate, so "certified"
   means measured invariance in every evaluated episode.
