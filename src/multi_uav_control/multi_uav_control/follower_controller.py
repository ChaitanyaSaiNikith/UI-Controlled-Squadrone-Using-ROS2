#!/usr/bin/env python3
"""
follower_controller.py — Velocity PD controller for one follower drone.

One instance per follower, parameterised by drone_id (1–N).

Control law (NED frame):
  velocity_cmd = Kp * (target - current_pos) - Kd * current_velocity

This mirrors how the leader is controlled — velocity offboard mode is proven
reliable in PX4 SITL multi-instance. Position mode offboard is not used because
follower instances do not engage it reliably.

Warmup sequence:
  1. Send 50 zero-velocity setpoints at 50 Hz (velocity=True heartbeat).
  2. Request offboard mode.
  3. Auto-arm once leader arms, has target, and has position.
"""

import math

import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleStatus, VehicleLocalPosition,
)

WARMUP_COUNT = 50
D_SAFE  = 6.0   # metres — repulsion + speed-cap activates within this radius
K_REP   = 8.0   # repulsion gain
D_MIN   = 0.3   # clamp to avoid singularity at zero distance


def _px4_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _clamp(val: float, limit: float) -> float:
    return max(-limit, min(limit, val))


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.declare_parameter('drone_id',  1)
        self.declare_parameter('kp',        0.5)
        self.declare_parameter('kd',        0.3)
        self.declare_parameter('max_vel_h', 4.0)
        self.declare_parameter('max_vel_v', 2.0)

        self.declare_parameter('spawn_north_m', 3.0)

        self._id     = self.get_parameter('drone_id').value
        self._kp     = self.get_parameter('kp').value
        self._kd     = self.get_parameter('kd').value
        self._mvh    = self.get_parameter('max_vel_h').value
        self._mvv    = self.get_parameter('max_vel_v').value
        self._spawn_m = self.get_parameter('spawn_north_m').value

        ns = f'drone{self._id}'
        self._sysid = self._id + 1

        qos = _px4_qos()

        # ── publishers ──────────────────────────────────────────────────────
        self._ocm_pub  = self.create_publisher(
            OffboardControlMode,
            f'/{ns}/fmu/in/offboard_control_mode', qos)
        self._tsp_pub  = self.create_publisher(
            TrajectorySetpoint,
            f'/{ns}/fmu/in/trajectory_setpoint', qos)
        self._vcmd_pub = self.create_publisher(
            VehicleCommand,
            f'/{ns}/fmu/in/vehicle_command', qos)

        # ── subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            f'/{ns}/target_position', self._target_cb, 10)
        self.create_subscription(
            VehicleLocalPosition,
            f'/{ns}/fmu/out/vehicle_local_position_v1', self._local_pos_cb, qos)
        self.create_subscription(
            VehicleStatus,
            f'/{ns}/fmu/out/vehicle_status_v2', self._status_cb, qos)
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2', self._leader_status_cb, qos)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1', self._leader_pos_cb, qos)
        self.create_subscription(
            Float32MultiArray, '/fleet_world_positions', self._fleet_cb, 10)

        # ── state ────────────────────────────────────────────────────────────
        self._target : tuple[float, float, float] | None = None
        self._pos    : tuple[float, float, float] | None = None
        self._vel    : tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._armed              = False
        self._leader_armed       = False
        self._leader_alt         = 0.0
        self._landing            = False
        self._warmup_count       = 0
        self._offboard_engaged   = False
        self._fleet_positions: list[float] = []

        self.create_timer(1.0 / 50.0, self._loop)
        self.get_logger().info(
            f'FollowerController started for {ns} '
            f'(Kp={self._kp} Kd={self._kd} max_h={self._mvh} max_v={self._mvv})')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _target_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._target = (p.x, p.y, p.z)

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (msg.x, msg.y, msg.z)
        self._vel = (msg.vx, msg.vy, msg.vz)

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._armed = (msg.arming_state == 2)
        if not self._armed and self._landing:
            self.get_logger().info(f'drone{self._id}: disarmed after landing')
            self._landing = False

    def _leader_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._leader_alt = msg.z

    def _leader_status_cb(self, msg: VehicleStatus) -> None:
        was_armed = self._leader_armed
        self._leader_armed = (msg.arming_state == 2)
        if was_armed and not self._leader_armed and self._armed:
            self.get_logger().info(f'drone{self._id}: leader disarmed — landing')
            self._landing = True

    def _fleet_cb(self, msg: Float32MultiArray) -> None:
        self._fleet_positions = list(msg.data)

    # ── main loop ────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        self._publish_heartbeat()

        # Phase 1 — warmup
        if self._warmup_count < WARMUP_COUNT:
            self._send_velocity(0.0, 0.0, 0.0)
            self._warmup_count += 1
            return

        # Phase 2 — engage offboard (once)
        if not self._offboard_engaged:
            self._engage_offboard()
            self._offboard_engaged = True
            self.get_logger().info(f'drone{self._id}: offboard mode requested')
            self._send_velocity(0.0, 0.0, 0.0)
            return

        # Phase 3 — auto-arm once leader is armed and we have target + position
        if not self._armed:
            if self._target is not None and self._pos is not None and self._leader_armed:
                self._arm()
                self._engage_offboard()
            self._send_velocity(0.0, 0.0, 0.0)
            return

        # Landing — descend at 1 m/s until near ground, then disarm
        if self._landing:
            if self._pos is not None and self._pos[2] > -0.5:
                self._vehicle_command(400, 0.0, 21196.0)
                self._landing = False
                self._armed = False
                self.get_logger().info(f'drone{self._id}: landed and disarmed')
            else:
                self._send_velocity(0.0, 0.0, 1.0)
            return

        # Phase 4 — hover until leader is above 3 m
        if self._target is None or self._pos is None or self._leader_alt > -3.0:
            self._send_velocity(0.0, 0.0, 0.0)
            return

        # Phase 5 — PD control + APF collision avoidance
        vx, vy, vz = self._pd_control()
        rx, ry, rz, min_dist = self._apf_repulsion()
        vx += rx
        vy += ry
        vz += rz
        # Cap speed based on proximity: full speed at D_SAFE, zero at contact.
        # This ensures repulsion can always overpower PD attraction when close.
        if min_dist < D_SAFE:
            speed_factor = max(min_dist / D_SAFE, 0.05)
            eff_mvh = self._mvh * speed_factor
            eff_mvv = self._mvv * speed_factor
        else:
            eff_mvh = self._mvh
            eff_mvv = self._mvv
        # Re-saturate with proximity-adjusted limits
        h = math.sqrt(vx**2 + vy**2)
        if h > eff_mvh:
            scale = eff_mvh / h
            vx *= scale
            vy *= scale
        vz = _clamp(vz, eff_mvv)
        self._send_velocity(vx, vy, vz)

    # ── APF repulsion ────────────────────────────────────────────────────────

    def _apf_repulsion(self) -> tuple[float, float, float, float]:
        """
        Artificial potential field repulsion from nearby drones.
        World-NED repulsion == local-NED repulsion (frames differ by translation only).
        Fleet array layout: [ldr_x,ldr_y,ldr_z, f1_x,f1_y,f1_z, ...]
        Self index = self._id (leader=0, follower i=i).
        Returns (vrx, vry, vrz, min_dist) — min_dist used for velocity capping.
        """
        if self._pos is None or len(self._fleet_positions) < 3:
            return (0.0, 0.0, 0.0, float('inf'))

        px, py, pz = self._pos
        my_wx = px + self._spawn_m * self._id
        my_wy = py
        my_wz = pz

        vrx = vry = vrz = 0.0
        min_dist = float('inf')
        n = len(self._fleet_positions) // 3
        for j in range(n):
            if j == self._id:
                continue  # skip self
            b = j * 3
            dx = my_wx - self._fleet_positions[b]
            dy = my_wy - self._fleet_positions[b + 1]
            dz = my_wz - self._fleet_positions[b + 2]
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            if d < min_dist:
                min_dist = d
            if d >= D_SAFE:
                continue
            d_eff = max(d, D_MIN)
            mag = K_REP * (1.0/d_eff - 1.0/D_SAFE) / (d_eff * d_eff)
            vrx += mag * dx / d_eff
            vry += mag * dy / d_eff
            vrz += mag * dz / d_eff

        return (vrx, vry, vrz, min_dist)

    # ── PD controller ────────────────────────────────────────────────────────

    def _pd_control(self) -> tuple[float, float, float]:
        tx, ty, tz = self._target  # type: ignore[misc]
        px, py, pz = self._pos    # type: ignore[misc]
        cvx, cvy, cvz = self._vel

        ex = tx - px
        ey = ty - py
        ez = tz - pz

        vx = self._kp * ex - self._kd * cvx
        vy = self._kp * ey - self._kd * cvy
        vz = self._kp * ez - self._kd * cvz

        # Saturate horizontal speed
        h = math.sqrt(vx**2 + vy**2)
        if h > self._mvh:
            scale = self._mvh / h
            vx *= scale
            vy *= scale

        vz = _clamp(vz, self._mvv)

        dist = math.sqrt(ex**2 + ey**2 + ez**2)
        self.get_logger().info(
            f'drone{self._id}: err=({ex:.2f},{ey:.2f},{ez:.2f}) '
            f'vel=({vx:.2f},{vy:.2f},{vz:.2f}) dist={dist:.1f}m',
            throttle_duration_sec=1.0)

        return vx, vy, vz

    # ── PX4 helpers ──────────────────────────────────────────────────────────

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _publish_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp    = self._ts()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._ocm_pub.publish(msg)

    def _send_velocity(self, vx: float, vy: float, vz: float) -> None:
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = self._ts()
        msg.position  = [nan, nan, nan]
        msg.velocity  = [float(vx), float(vy), float(vz)]
        msg.yawspeed  = 0.0
        msg.yaw       = nan
        self._tsp_pub.publish(msg)

    def _vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = self._sysid
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._vcmd_pub.publish(msg)

    def _arm(self) -> None:
        self.get_logger().info(f'drone{self._id}: arming')
        self._vehicle_command(400, 1.0, 21196.0)

    def _engage_offboard(self) -> None:
        self._vehicle_command(176, 1.0, 6.0)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
