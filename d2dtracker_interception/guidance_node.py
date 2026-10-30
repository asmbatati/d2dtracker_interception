#!/usr/bin/env python3
"""Proportional-Navigation interceptor controller (a 'strategy' method).

Flies the interceptor straight onto the target using PN velocity setpoints. This
is an alternative to the Behaviour-Tree / FSM / RL controllers - run ONE of them
at a time, since they all command the same FCU. The node self-arms, switches to
OFFBOARD, climbs to ``takeoff_height`` and only then engages with PN (like the
other mission heads, which take off before pursuing; with ``climb_first`` false
it engages from the ground as soon as a target is seen).

With ``cbf`` true the engaged velocity command passes through a CBF safety
filter (``safety.CBFVelocityFilter``): altitude floor ``cbf_z_min``, optional
ceiling ``cbf_z_max`` and geofence ``cbf_fence_radius`` around the world origin
(0 disables those two), barrier gain ``cbf_alpha``.

Topics (relative; launch under the interceptor namespace):
  sub  mavros/local_position/odom   (nav_msgs/Odometry)  - interceptor state
  sub  target_odom                  (nav_msgs/Odometry)  - target state
  pub  mavros/setpoint_raw/local    (mavros_msgs/PositionTarget) - velocity cmd
"""
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data,
                       QoSProfile, ReliabilityPolicy)

from .frames import odom_velocity_world
from .guidance import pn_velocity_command, pursuit_velocity_command
from .safety import CBFVelocityFilter


class GuidanceNode(Node):
    """PN guidance controller for the interceptor."""

    def __init__(self):
        super().__init__('guidance_node')
        self.declare_parameter('law', 'pn')            # pn | pursuit
        self.declare_parameter('nav_gain', 3.0)        # PN constant N
        self.declare_parameter('vmax', 8.0)
        self.declare_parameter('closing_speed', 6.0)
        self.declare_parameter('capture_radius', 1.0)
        self.declare_parameter('takeoff_height', 4.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('climb_first', True)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('cbf', False)
        self.declare_parameter('cbf_z_min', 1.5)
        self.declare_parameter('cbf_z_max', 0.0)          # 0 = no ceiling
        self.declare_parameter('cbf_fence_radius', 0.0)   # 0 = no geofence
        self.declare_parameter('cbf_alpha', 1.0)
        # Each drone's odom is in its own EKF/spawn frame; add spawn offsets to
        # compare them in a common (world) frame.
        self.declare_parameter('interceptor_spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])

        self.law = self.get_parameter('law').value
        if self.law not in ('pn', 'pursuit'):
            raise ValueError(f"law must be 'pn' or 'pursuit', got {self.law!r}")
        self.N = self.get_parameter('nav_gain').value
        self.vmax = self.get_parameter('vmax').value
        self.closing_speed = self.get_parameter('closing_speed').value
        self.capture_radius = self.get_parameter('capture_radius').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.engaged = not self.get_parameter('climb_first').value
        self.climb_xy = None
        self.dt = 1.0 / max(self.get_parameter('rate').value, 1e-3)
        self.ispawn = np.array(self.get_parameter('interceptor_spawn').value, dtype=float)
        self.tspawn = np.array(self.get_parameter('target_spawn').value, dtype=float)
        self.cbf = None
        if self.get_parameter('cbf').value:
            z_max = float(self.get_parameter('cbf_z_max').value)
            fence = float(self.get_parameter('cbf_fence_radius').value)
            self.cbf = CBFVelocityFilter(
                z_min=self.get_parameter('cbf_z_min').value,
                z_max=z_max if z_max > 0.0 else float('inf'),
                fence_radius=fence if fence > 0.0 else float('inf'),
                alpha=self.get_parameter('cbf_alpha').value, vmax=self.vmax)

        state_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.state = State()
        self.odom = None
        self.target = None
        self.counter = 0

        self.create_subscription(State, 'mavros/state', self._state_cb, state_qos)
        self.create_subscription(Odometry, 'mavros/local_position/odom',
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'target_odom',
                                 self._target_cb, qos_profile_sensor_data)

        self.sp_pub = self.create_publisher(PositionTarget,
                                            'mavros/setpoint_raw/local',
                                            qos_profile_sensor_data)
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.create_timer(self.dt, self._loop)
        self.create_timer(1.0, self._arm_offboard)

    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _target_cb(self, msg):
        self.target = msg

    def _arm_offboard(self):
        if not self.auto_arm or self.counter < 50:
            return
        if self.state.mode != 'OFFBOARD' and self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.mode_client.call_async(req)
            return
        if not self.state.armed and self.arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)

    def _publish_velocity(self, vx, vy, vz, frame_id):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = frame_id
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY +
                        PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX +
                        PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(vx), float(vy), float(vz)
        sp.yaw = float(np.arctan2(vy, vx))
        self.sp_pub.publish(sp)

    def _publish_position(self, x, y, z, frame_id):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = frame_id
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY +
                        PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX +
                        PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x, sp.position.y, sp.position.z = float(x), float(y), float(z)
        self.sp_pub.publish(sp)

    def _loop(self):
        self.counter += 1
        if self.odom is None:
            return
        frame_id = self.odom.header.frame_id or 'map'
        p = self.odom.pose.pose.position

        # Climb to altitude / hold until a target appears and the climb is done
        # (commands are in the interceptor's local frame, so use raw odom here).
        if self.target is None or not self.engaged:
            if self.climb_xy is None:
                self.climb_xy = (p.x, p.y)
            self._publish_position(self.climb_xy[0], self.climb_xy[1],
                                   self.takeoff_height, frame_id)
            if (self.target is not None and self.state.armed
                    and p.z >= 0.9 * self.takeoff_height):
                self.engaged = True
                self.get_logger().info('PN engaged at %.1f m' % p.z)
            return

        # World-frame geometry (odom + spawn offset).
        ipos = np.array([p.x, p.y, p.z]) + self.ispawn
        # odom twist is body-frame: rotate to world before PN uses it
        ivel = odom_velocity_world(self.odom)
        tp = self.target.pose.pose.position
        tpos = np.array([tp.x, tp.y, tp.z]) + self.tspawn
        tvel = odom_velocity_world(self.target)

        if float(np.linalg.norm(tpos - ipos)) <= self.capture_radius:
            self.get_logger().info('Target captured (PN).', throttle_duration_sec=2.0)

        # PN velocity is frame-invariant under translation (both ENU); publish
        # directly in the interceptor's local frame.
        if self.law == 'pursuit':
            v_cmd = pursuit_velocity_command(ipos, tpos, self.closing_speed)
        else:
            v_cmd = pn_velocity_command(ipos, ivel, tpos, tvel, self.vmax,
                                        N=self.N, dt=self.dt,
                                        closing_speed=self.closing_speed)
        if self.cbf is not None:
            v_cmd, _ = self.cbf.apply(ipos, v_cmd)
        self._publish_velocity(v_cmd[0], v_cmd[1], v_cmd[2], frame_id)


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
