# d2dtracker_interception

Interception strategy library for the drone-to-drone interception workspace.
It holds pure-Python geometry and guidance functions (rendezvous point,
Proportional Navigation) with no ROS dependency, plus two stand-alone nodes
built on them. In the pluggable pipeline it is not launched as a node:
`interception_planning` imports its functions for the `rendezvous`, `pn_lead`
and `head_on` planner backends. The two nodes are separate "strategy" methods
from RUNBOOK.md (Strategy A and Strategy B).

Build type: `ament_python`. License: MIT.

## Library

| Module | Functions | Used by |
|---|---|---|
| `interception_point.py` | `reach_time` (bang-bang accel-then-cruise time to a point), `time_to_intercept`, `find_optimal_intersection` (first predicted sample with `reach_time <= k * dt`; else the last sample) | `interceptor` node, planner `rendezvous` |
| `interception_modes.py` | `interceptor_trajectory` (Algorithm 1), `closest_interception_sample` (Algorithm 2), `find_reference_sample` (Algorithm 3), `reference_trajectory` (Algorithm 4, head-on), `distance2D`, `dist3D`, `yaw_to_tgt` | `interceptor` node, planner `head_on` |
| `guidance.py` | `closing_velocity`, `los_rate`, `proportional_navigation` (true PN, `a = N * Vc * (omega x los_hat)`), `pn_velocity_command` (LOS closing speed + PN lateral term, capped at `vmax`) | `guidance` node, planner `pn_lead` |
| `frames.py` | `body_to_world`, `odom_velocity_world` (rotate the MAVROS odom twist from body to world frame) | both nodes |

States are 9-vectors `[x, y, z, vx, vy, vz, ax, ay, az]`.

`msg/State.msg` and `interception_pseudo_code` are kept for reference; the
message is not generated (an `ament_python` package cannot build interfaces).

## Backends (strategies)

| Name | Executable | What it does |
|---|---|---|
| Rendezvous (Strategy A) | `interceptor` | Publishes the rendezvous point and an acceleration-limited path toward it. Does not command the FCU |
| Proportional Navigation (Strategy B) | `guidance` | Flies the interceptor with PN velocity setpoints. Arms and requests OFFBOARD itself |

## Node: `interceptor` (node name `interceptor_node`)

| Direction | Topic | Type | QoS |
|---|---|---|---|
| sub | `interceptor_odom` | `nav_msgs/Odometry` | sensor data |
| sub | `target_predicted_path` | `nav_msgs/Path` | depth 10 |
| sub | `target_odom` | `nav_msgs/Odometry` | sensor data |
| pub | `intercept_point` | `geometry_msgs/PoseStamped` | depth 10 |
| pub | `interception_path` | `nav_msgs/Path` | depth 10 |

Once a `target_predicted_path` message arrives its positions are used (velocity
set to zero); otherwise a constant-velocity prediction is built from
`target_odom`. Odom positions get the spawn offsets added; odom velocities are
rotated from body to world frame.

| Parameter | Default |
|---|---|
| `vmax`, `amax` | `8.0`, `4.0` |
| `prediction_dt` | `0.1` (s) |
| `prediction_horizon` | `30` (samples, fallback prediction) |
| `rate` | `10.0` (Hz) |
| `frame_id` | `map` |
| `interceptor_spawn`, `target_spawn` | `[0.0, 0.0, 0.0]`, `[10.0, 0.0, 0.0]` |
| `target_path_in_world` | `true` (do not offset the predicted path) |

`capture_radius` (`1.0`) is declared but not used by the loop.

## Node: `guidance` (node name `guidance_node`)

| Direction | Topic / service | Type | QoS |
|---|---|---|---|
| sub | `mavros/state` | `mavros_msgs/State` | reliable, transient local |
| sub | `mavros/local_position/odom` | `nav_msgs/Odometry` | sensor data |
| sub | `target_odom` | `nav_msgs/Odometry` | sensor data |
| pub | `mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | sensor data |
| client | `mavros/cmd/arming`, `mavros/set_mode` | `CommandBool`, `SetMode` | - |

Until it engages, it sends a position setpoint at its first x/y and
`takeoff_height`. With `climb_first` it engages once a target odom has arrived,
the vehicle is armed and altitude >= 0.9 * `takeoff_height`; with
`climb_first` false it engages as soon as a target odom arrives. Engaged, it
sends PN velocity setpoints with yaw along the velocity, using world-frame
velocities of both drones. With `auto_arm`, a 1 s timer requests OFFBOARD,
then arming, once the loop has run 50 times. Reaching `capture_radius` only
logs a message.

| Parameter | Default |
|---|---|
| `nav_gain` | `3.0` (PN constant N) |
| `vmax` | `8.0` |
| `closing_speed` | `6.0` |
| `capture_radius` | `1.0` |
| `takeoff_height` | `4.0` |
| `auto_arm` | `true` |
| `climb_first` | `true` |
| `rate` | `30.0` (Hz) |
| `interceptor_spawn`, `target_spawn` | `[0.0, 0.0, 0.0]`, `[10.0, 0.0, 0.0]` |

## Build

```bash
cd ~/drone_interception_ws/ros2_ws
colcon build --symlink-install --packages-select d2dtracker_interception
source install/setup.bash
```

## Run

Every terminal needs the sim's RMW and domain:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=77   # must match the sim's ros_domain_id:= (default 77)
```

```bash
# Strategy A: rendezvous point + path
ros2 launch d2dtracker_interception interceptor.launch.py
# Strategy B: Proportional Navigation (flies the interceptor)
ros2 launch d2dtracker_interception guidance.launch.py nav_gain:=3.0 vmax:=8.0
```

- `interceptor.launch.py`: arguments `interceptor_odom`
  (`/interceptor/mavros/local_position/odom`), `target_odom`
  (`/target/mavros/local_position/odom`), `target_predicted_path`
  (`/target/predicted_path`), `vmax`, `amax`. The node has no namespace, so it
  publishes `/intercept_point` and `/interception_path`. Sets
  `use_sim_time: True`.
- `guidance.launch.py`: arguments `namespace` (`interceptor`), `nav_gain`,
  `vmax`, `target_odom`. Sets `use_sim_time: True`.

Run only one interceptor controller at a time. The `guidance` node must not run
together with `pipeline.launch.py` heads `fsm`, `offboard` or `none` (their
`control_node` publishes to the same setpoint topic) or with `orchestrator:=bt`.

In the pipeline, the library is selected through the planner:

```bash
ros2 launch drone_interception_sim pipeline.launch.py planner:=rendezvous
ros2 launch drone_interception_sim pipeline.launch.py planner:=pn_lead
ros2 launch drone_interception_sim pipeline.launch.py planner:=head_on
```

## Tests

`test/test_interception.py` (7 tests) and `test/test_guidance.py` (7 tests)
cover the library without a ROS runtime. `test/` also has the
copyright/flake8/pep257 lint tests and `test/test_frames.py` for `frames.py`.

```bash
cd ~/drone_interception_ws/ros2_ws/src/d2dtracker_interception
python3 -m pytest test
```

Measured result for the interception, guidance and lint tests: 14 passed,
1 skipped, 2 failed. The skip is the copyright test (marked skip). The 2
failures are lint, not functional: flake8 (I100 import order in `setup.py`)
and pep257 (D213 docstring style). `test_frames.py` is a new, uncommitted
file and is not part of that count.
