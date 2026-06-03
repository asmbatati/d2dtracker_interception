#!/usr/bin/env python3
"""Interception strategy node.

Subscribes to the interceptor's odometry and the target's predicted path (or,
as a fallback, the target's odometry from which a constant-velocity prediction
is built), computes the optimal rendezvous point with
:func:`find_optimal_intersection`, and publishes:

  * ``~/intercept_point`` (geometry_msgs/PoseStamped) - the rendezvous point.
  * ``~/interception_path`` (nav_msgs/Path) - an acceleration-limited path from
    the interceptor toward the rendezvous, consumable by a tracking controller
    or the d2dtracker_states FSM/Behaviour-Tree.

Uses only standard messages so the package stays pure ``ament_python`` (no
custom rosidl generation, which ament_python cannot do).
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .interception_modes import interceptor_trajectory
from .interception_point import find_optimal_intersection


class InterceptorNode(Node):
    """Compute and publish the optimal interception point and path."""

    def __init__(self):
        super().__init__('interceptor_node')

        self.declare_parameter('vmax', 8.0)
        self.declare_parameter('amax', 4.0)
        self.declare_parameter('prediction_dt', 0.1)
        self.declare_parameter('prediction_horizon', 30)  # samples for the fallback
        self.declare_parameter('capture_radius', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rate', 10.0)
        # Spawn offsets: each odom is in its own EKF frame; results are in world.
        self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])
        # Is target_predicted_path already in the world frame? (don't offset it)
        self.declare_parameter('target_path_in_world', True)

        self.vmax = self.get_parameter('vmax').value
        self.amax = self.get_parameter('amax').value
        self.dt = self.get_parameter('prediction_dt').value
        self.horizon = int(self.get_parameter('prediction_horizon').value)
        self.capture_radius = self.get_parameter('capture_radius').value
        self.frame_id = self.get_parameter('frame_id').value
        self.ispawn = list(self.get_parameter('interceptor_spawn').value)
        self.tspawn = list(self.get_parameter('target_spawn').value)
        self.path_in_world = self.get_parameter('target_path_in_world').value

        self.interceptor_state = None      # [x,y,z,vx,vy,vz,ax,ay,az]
        self.predicted_states = []         # list of 9-vectors
        self.target_odom = None

        self.create_subscription(Odometry, 'interceptor_odom',
                                 self.interceptorOdomCb, qos_profile_sensor_data)
        self.create_subscription(Path, 'target_predicted_path',
                                 self.targetPathCb, 10)
        self.create_subscription(Odometry, 'target_odom',
                                 self.targetOdomCb, qos_profile_sensor_data)

        self.point_pub = self.create_publisher(PoseStamped, 'intercept_point', 10)
        self.path_pub = self.create_publisher(Path, 'interception_path', 10)

        period = 1.0 / max(self.get_parameter('rate').value, 1e-3)
        self.create_timer(period, self.update)

    # --- callbacks ---------------------------------------------------------
    def interceptorOdomCb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        # to world frame (odom + spawn offset)
        self.interceptor_state = [p.x + self.ispawn[0], p.y + self.ispawn[1],
                                  p.z + self.ispawn[2], v.x, v.y, v.z, 0.0, 0.0, 0.0]

    def targetPathCb(self, msg: Path):
        off = [0.0, 0.0, 0.0] if self.path_in_world else self.tspawn
        states = []
        for ps in msg.poses:
            p = ps.pose.position
            states.append([p.x + off[0], p.y + off[1], p.z + off[2],
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.predicted_states = states

    def targetOdomCb(self, msg: Odometry):
        self.target_odom = msg

    # --- helpers -----------------------------------------------------------
    def _constant_velocity_prediction(self):
        """Build a predicted target path from target odom (constant velocity)."""
        if self.target_odom is None:
            return []
        p = self.target_odom.pose.pose.position
        v = self.target_odom.twist.twist.linear
        # target odom -> world frame
        px, py, pz = p.x + self.tspawn[0], p.y + self.tspawn[1], p.z + self.tspawn[2]
        states = []
        for k in range(self.horizon):
            t = k * self.dt
            states.append([px + v.x * t, py + v.y * t, pz + v.z * t,
                          v.x, v.y, v.z, 0.0, 0.0, 0.0])
        return states

    # --- main loop ---------------------------------------------------------
    def update(self):
        if self.interceptor_state is None:
            return

        predicted = self.predicted_states or self._constant_velocity_prediction()
        if not predicted:
            return

        state, idx = find_optimal_intersection(
            predicted, self.interceptor_state, self.dt, self.vmax, self.amax)
        if state is None:
            return

        stamp = self.get_clock().now().to_msg()

        point = PoseStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.frame_id
        point.pose.position.x = float(state[0])
        point.pose.position.y = float(state[1])
        point.pose.position.z = float(state[2])
        point.pose.orientation.w = 1.0
        self.point_pub.publish(point)

        # Acceleration-limited straight-line trajectory toward the rendezvous.
        xi, yi, zi = self.interceptor_state[0:3]
        vxi, vyi, vzi = self.interceptor_state[3:6]
        speed = math.sqrt(vxi * vxi + vyi * vyi + vzi * vzi)
        traj = interceptor_trajectory(
            state[0], state[1], state[2], xi, yi, speed,
            self.vmax, self.amax, self.dt, max(idx, 1))

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        for (x, y, z) in traj:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = InterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
