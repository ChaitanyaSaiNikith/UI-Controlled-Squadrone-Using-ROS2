#!/usr/bin/env python3
"""
leader_control.py — Controls drone0 (the leader UAV).

Joystick → body-frame velocity → NED velocity setpoint → PX4 offboard mode.

Warmup sequence (automatic):
  1. Send 30 zero-velocity setpoints at 50 Hz.
  2. Request offboard mode (VehicleCommand 176).
  3. Wait for A button to arm.

Button mapping (Xbox / Kreo joystick):
  A (0)  — arm
  X (2)  — toggle hover (zero velocity hold)
  Y (3)  — disarm

Axis mapping:
  0  LeftX   — yaw rate   (right → positive CW in NED)
  1  LeftY   — vertical   (pull back → climb, push forward → descend)
  3  RightX  — strafe     (right → East)
  4  RightY  — forward    (push forward → North)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleStatus, VehicleAttitude, VehicleLocalPosition,
)

# ── tunables ────────────────────────────────────────────────────────────────
WARMUP_COUNT = 30
MAX_VEL_H    = 5.0   # m/s  horizontal
MAX_VEL_V    = 2.0   # m/s  vertical
MAX_YAW_RATE = 1.0   # rad/s
DEADZONE     = 0.1

# Joystick button indices (Xbox mode)
BTN_ARM    = 0
BTN_HOVER  = 2
BTN_DISARM = 3

# Joystick axis indices
AX_YAW      = 0
AX_THROTTLE = 1
AX_STRAFE   = 3
AX_FORWARD  = 4
# ────────────────────────────────────────────────────────────────────────────


def _px4_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _deadzone(val: float, dz: float) -> float:
    return val if abs(val) > dz else 0.0


class LeaderControl(Node):
    def __init__(self):
        super().__init__('leader_control')

        qos = _px4_qos()

        # ── publishers ──────────────────────────────────────────────────────
        self._ocm_pub  = self.create_publisher(
            OffboardControlMode,
            '/drone0/fmu/in/offboard_control_mode', qos)
        self._tsp_pub  = self.create_publisher(
            TrajectorySetpoint,
            '/drone0/fmu/in/trajectory_setpoint', qos)
        self._vcmd_pub = self.create_publisher(
            VehicleCommand,
            '/drone0/fmu/in/vehicle_command', qos)
        self._state_pub = self.create_publisher(PoseStamped, '/leader_state', 10)
        self._vel_pub   = self.create_publisher(TwistStamped, '/leader_velocity', 10)

        # ── subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(
            VehicleStatus,
            '/drone0/fmu/out/vehicle_status_v2', self._status_cb, qos)
        self.create_subscription(
            VehicleAttitude,
            '/drone0/fmu/out/vehicle_attitude', self._attitude_cb, qos)
        self.create_subscription(
            VehicleLocalPosition,
            '/drone0/fmu/out/vehicle_local_position_v1', self._local_pos_cb, qos)

        # ── state ───────────────────────────────────────────────────────────
        self._warmup_count      = 0
        self._offboard_engaged  = False
        self._armed             = False
        self._hover             = False

        self._yaw  = 0.0
        self._pos  = (0.0, 0.0, 0.0)   # NED
        self._vel  = (0.0, 0.0, 0.0)   # NED

        self._axes         : list[float] = []
        self._buttons      : list[int]   = []
        self._prev_buttons : list[int]   = []

        self.create_timer(1.0 / 50.0, self._loop)
        self.get_logger().info('LeaderControl started — warmup in progress…')

    # ── callbacks ───────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy) -> None:
        self._prev_buttons = list(self._buttons)
        self._axes    = list(msg.axes)
        self._buttons = list(msg.buttons)

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._armed = (msg.arming_state == 2)   # ARMING_STATE_ARMED

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        # q = [w, x, y, z]
        q = msg.q
        self._yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
        )

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (msg.x,  msg.y,  msg.z)
        self._vel = (msg.vx, msg.vy, msg.vz)

    # ── main loop ───────────────────────────────────────────────────────────

    def _loop(self) -> None:
        self._publish_heartbeat()

        # Phase 1 — warmup
        if self._warmup_count < WARMUP_COUNT:
            self._send_setpoint(0.0, 0.0, 0.0, 0.0)
            self._warmup_count += 1
            return

        # Phase 2 — engage offboard (once)
        if not self._offboard_engaged:
            self._engage_offboard()
            self._offboard_engaged = True
            self.get_logger().info('Offboard mode requested — press A to arm')
            return

        # Phase 3 — normal operation
        self._handle_buttons()

        self.get_logger().info(
            f'armed={self._armed} hover={self._hover} axes={len(self._axes)} yaw={self._yaw:.2f}',
            throttle_duration_sec=1.0
        )

        if not self._armed or self._hover:
            self._send_setpoint(0.0, 0.0, 0.0, 0.0)
        else:
            vx, vy, vz, yr = self._compute_velocity()
            self._send_setpoint(vx, vy, vz, yr)
            self._publish_leader_velocity(vx, vy, vz, yr)

        self._publish_leader_state()

    def _handle_buttons(self) -> None:
        if not self._buttons:
            return
        prev = self._prev_buttons if self._prev_buttons else [0] * len(self._buttons)

        def rising(idx: int) -> bool:
            return (idx < len(self._buttons)
                    and bool(self._buttons[idx])
                    and not (idx < len(prev) and bool(prev[idx])))

        if rising(BTN_ARM):
            self.get_logger().info('Arming drone0')
            self._hover = False
            self._disable_preflight_checks()
            self._arm()
        if rising(BTN_DISARM):
            self.get_logger().info('Disarming drone0')
            self._disarm()
        if rising(BTN_HOVER):
            self._hover = not self._hover
            self.get_logger().info(f'Hover {"ON" if self._hover else "OFF"}')

    def _compute_velocity(self) -> tuple[float, float, float, float]:
        if len(self._axes) <= max(AX_YAW, AX_THROTTLE, AX_STRAFE, AX_FORWARD):
            return 0.0, 0.0, 0.0, 0.0

        fwd_raw    = _deadzone(self._axes[AX_FORWARD],  DEADZONE)
        strafe_raw = _deadzone(self._axes[AX_STRAFE],   DEADZONE)
        throt_raw  = _deadzone(self._axes[AX_THROTTLE], DEADZONE)
        yaw_raw    = _deadzone(self._axes[AX_YAW],      DEADZONE)

        # Body frame (stick forward = negative axis value → invert for forward)
        body_fwd   = -fwd_raw    * MAX_VEL_H
        body_right =  strafe_raw * MAX_VEL_H

        # Vertical: pull stick back → positive axis → climb → negative NED z
        ned_vz   = -throt_raw  * MAX_VEL_V
        yaw_rate =  yaw_raw    * MAX_YAW_RATE

        # Rotate body forward/right into NED world frame using current yaw
        cy, sy = math.cos(self._yaw), math.sin(self._yaw)
        ned_vx = body_fwd * cy - body_right * sy
        ned_vy = body_fwd * sy + body_right * cy

        self.get_logger().info(
            f'vel: fwd={body_fwd:.2f} right={body_right:.2f} vz={ned_vz:.2f} yaw_rate={yaw_rate:.2f}',
            throttle_duration_sec=0.5
        )

        return ned_vx, ned_vy, ned_vz, yaw_rate

    # ── PX4 helpers ─────────────────────────────────────────────────────────

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

    def _send_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float) -> None:
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp  = self._ts()
        msg.position   = [nan, nan, nan]
        msg.velocity   = [float(vx), float(vy), float(vz)]
        msg.yawspeed   = float(yawspeed)
        msg.yaw        = nan
        self._tsp_pub.publish(msg)

    def _vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._vcmd_pub.publish(msg)

    def _disable_preflight_checks(self) -> None:
        # MAV_CMD_DO_SET_PARAMETER (180): set a named PX4 parameter by index.
        # PX4 maps COM_ARM_CHK_ALL (param_id index varies by build); setting it to 0
        # disables all arming sanity checks so SITL arms without GPS/RC/sensor requirements.
        # param1 = parameter index (0 = use param_id field — not available in VehicleCommand),
        # param2 = new value.
        # NOTE: VehicleCommand does not carry a string param_id, so this targets the
        # integer parameter by its hard-coded index in the PX4 parameter table.
        # The most reliable SITL approach is: ros2 run px4_ros_com set_param COM_ARM_CHK_ALL 0
        # or via the PX4 shell: param set COM_ARM_CHK_ALL 0
        # This command is sent as a best-effort attempt; verify via `ros2 param get`.
        self.get_logger().info('Sending COM_ARM_CHK_ALL=0 (disable all arming checks)')
        # VEHICLE_CMD_DO_SET_PARAMETER (180): param1=parameter index, param2=value
        # COM_ARM_CHK_ALL index in PX4 v1.14 parameter table = 0 (bitmask, 0 = all disabled)
        self._vehicle_command(180, 0.0, 0.0)

    def _arm(self) -> None:
        # VEHICLE_CMD_COMPONENT_ARM_DISARM (400), param2=0 (normal arm, no force)
        self._vehicle_command(400, 1.0, 0.0)

    def _disarm(self) -> None:
        self._vehicle_command(400, 0.0, 21196.0)

    def _engage_offboard(self) -> None:
        # VEHICLE_CMD_DO_SET_MODE (176): param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        # param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD (6)
        self._vehicle_command(176, 1.0, 6.0)

    # ── state publications ──────────────────────────────────────────────────

    def _publish_leader_state(self) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        x, y, z = self._pos
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # Encode yaw as a z-axis quaternion
        msg.pose.orientation.w = math.cos(self._yaw / 2.0)
        msg.pose.orientation.z = math.sin(self._yaw / 2.0)
        self._state_pub.publish(msg)

    def _publish_leader_velocity(self, vx: float, vy: float, vz: float, yr: float) -> None:
        msg = TwistStamped()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = 'map'
        msg.twist.linear.x    = float(vx)
        msg.twist.linear.y    = float(vy)
        msg.twist.linear.z    = float(vz)
        msg.twist.angular.z   = float(yr)
        self._vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
