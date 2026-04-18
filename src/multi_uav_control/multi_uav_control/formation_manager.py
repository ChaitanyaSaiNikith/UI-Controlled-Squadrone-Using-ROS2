#!/usr/bin/env python3
"""
formation_manager.py — Computes per-follower target positions from the leader.

Publishes target PoseStamped for each follower at 20 Hz.

KEY: Each PX4 SITL instance sets its local NED origin at its own spawn position.
This means follower i's NED origin is offset from the leader's NED origin by its
Gazebo spawn distance. The launch script spawns followers at Gazebo y = i*3m
(North in ENU → NED x = 3*i metres).

Formation_manager publishes targets in FOLLOWER's NED frame by subtracting each
follower's spawn offset from the world-NED target. This converts the absolute
world target into the correct target for each follower's local NED frame.

Parameter: spawn_north_m (default 3.0) — Gazebo y spacing between followers
           in metres. Must match the POSE_Y step in launch_multi_sitl.sh.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

# Formation offsets in leader BODY frame (metres).
#   x = forward(+) / behind(-)    y = right(+) / left(-)
# First N entries used for num_followers = N (up to 4).
FORMATIONS = {
    'V':       [(-5.0, -4.0, 0.0), (-5.0,  4.0, 0.0),
                (-10.0, -8.0, 0.0), (-10.0,  8.0, 0.0)],
    'line':    [(-5.0,  0.0, 0.0), (-10.0,  0.0, 0.0),
                (-15.0,  0.0, 0.0), (-20.0,  0.0, 0.0)],
    'diamond': [( 0.0, -6.0, 0.0), ( 0.0,  6.0, 0.0),
                (-6.0, -3.0, 0.0), (-6.0,  3.0, 0.0)],
    'square':  [(-3.0, -3.0, 0.0), (-3.0,  3.0, 0.0),
                (-9.0, -3.0, 0.0), (-9.0,  3.0, 0.0)],
    'circle':  None,  # sentinel — computed dynamically in _loop
}
DEFAULT_FORMATION = 'V'


def _px4_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class FormationManager(Node):
    def __init__(self):
        super().__init__('formation_manager')

        qos = _px4_qos()

        self.declare_parameter('num_followers', 2)
        # Metres between each follower's Gazebo spawn position (y axis = NED north).
        # Must match POSE_Y step in launch_multi_sitl.sh (default: i*3 → spacing=3).
        self.declare_parameter('spawn_north_m', 3.0)

        self._num_followers: int = self.get_parameter('num_followers').value
        self._spawn_north_m: float = self.get_parameter('spawn_north_m').value

        self._leader_pos: tuple[float, float, float] | None = None
        self._leader_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._leader_yaw: float = 0.0
        self._formation: str = DEFAULT_FORMATION

        # ── subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._leader_pos_cb, qos)
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._leader_att_cb, qos)
        self.create_subscription(
            String, '/formation_type', self._formation_cb, 10)

        # ── /num_followers publisher (VOLATILE for rosbridge) ────────────────
        nf_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._nf_pub = self.create_publisher(Int32, '/num_followers', nf_qos)
        self.create_timer(3.0, self._publish_num_followers)

        # ── per-follower target publishers ───────────────────────────────────
        self._target_pubs: dict[int, rclpy.publisher.Publisher] = {}
        for drone_id in range(1, self._num_followers + 1):
            self._target_pubs[drone_id] = self.create_publisher(
                PoseStamped, f'/drone{drone_id}/target_position', 10)

        # ── fleet position broadcast ─────────────────────────────────────────
        self._follower_world_pos: dict[int, tuple[float, float, float]] = {}
        for drone_id in range(1, self._num_followers + 1):
            self.create_subscription(
                VehicleLocalPosition,
                f'/drone{drone_id}/fmu/out/vehicle_local_position_v1',
                lambda msg, did=drone_id: self._follower_pos_cb(msg, did),
                qos)
        self._fleet_pub = self.create_publisher(Float32MultiArray, '/fleet_world_positions', 10)

        self.create_timer(1.0 / 20.0, self._loop)
        self.get_logger().info(
            f'FormationManager started — formation={self._formation} '
            f'num_followers={self._num_followers} '
            f'spawn_north_m={self._spawn_north_m}')

    def _publish_num_followers(self) -> None:
        msg = Int32()
        msg.data = self._num_followers
        self._nf_pub.publish(msg)

    # ── callbacks ───────────────────────────────────────────────────────────

    def _leader_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._leader_pos = (msg.x, msg.y, msg.z)
        self._leader_vel = (msg.vx, msg.vy, msg.vz)

    def _leader_att_cb(self, msg: VehicleAttitude) -> None:
        q = msg.q
        if len(q) == 4:
            w, x, y, z = q
            self._leader_yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def _formation_cb(self, msg: String) -> None:
        name = msg.data.strip()
        if name in FORMATIONS:
            self._formation = name
            self.get_logger().info(f'Formation → {name}')
        else:
            self.get_logger().warn(
                f"Unknown formation '{name}'. Valid: {list(FORMATIONS.keys())}")

    def _follower_pos_cb(self, msg: VehicleLocalPosition, drone_id: int) -> None:
        """Cache follower position converted to world NED frame."""
        self._follower_world_pos[drone_id] = (
            msg.x + self._spawn_north_m * drone_id,
            msg.y,
            msg.z,
        )

    def _publish_fleet_positions(self) -> None:
        """Broadcast all drone world-NED positions. Index 0=leader, 1..N=followers."""
        if self._leader_pos is None:
            return
        if len(self._follower_world_pos) < self._num_followers:
            return  # wait until every follower has reported at least once
        lx, ly, lz = self._leader_pos
        data: list[float] = [lx, ly, lz]
        for did in range(1, self._num_followers + 1):
            data.extend(self._follower_world_pos[did])
        msg = Float32MultiArray()
        msg.data = data
        self._fleet_pub.publish(msg)

    # ── helpers ─────────────────────────────────────────────────────────────

    def _spawn_offset(self, drone_id: int) -> tuple[float, float, float]:
        """
        Each follower i is spawned at Gazebo y = drone_id * spawn_north_m (North in ENU).
        ENU North → NED x (North). So follower i's NED frame is offset from world NED
        (leader's frame) by (spawn_north_m * drone_id, 0, 0).

        Subtracting this from a world-NED target converts it to the follower's NED frame.
        """
        return (self._spawn_north_m * drone_id, 0.0, 0.0)

    def _publish_target(self, drone_id: int, now,
                        world_tx: float, world_ty: float, world_tz: float) -> None:
        """Convert world-NED target to follower's local NED frame and publish."""
        sx, sy, sz = self._spawn_offset(drone_id)
        tx = world_tx - sx
        ty = world_ty - sy
        tz = world_tz - sz

        msg = PoseStamped()
        msg.header.stamp    = now
        msg.header.frame_id = 'map'
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = tz
        msg.pose.orientation.w = 1.0
        self._target_pubs[drone_id].publish(msg)

        self.get_logger().info(
            f'drone{drone_id}: world=({world_tx:.2f},{world_ty:.2f},{world_tz:.2f}) '
            f'follower_frame=({tx:.2f},{ty:.2f},{tz:.2f})',
            throttle_duration_sec=2.0)

    # ── publish loop ────────────────────────────────────────────────────────

    def _loop(self) -> None:
        if self._leader_pos is None:
            return

        lx, ly, lz = self._leader_pos
        now = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Leader: x={lx:.2f} y={ly:.2f} z={lz:.2f} '
            f'yaw={math.degrees(self._leader_yaw):.1f}°',
            throttle_duration_sec=2.0)

        # Rotate body-frame offsets → world NED using leader yaw
        cy = math.cos(self._leader_yaw)
        sy = math.sin(self._leader_yaw)

        # Circle: dynamic, N-follower ring centred on/behind leader
        if self._formation == 'circle':
            radius = 7.0
            for drone_id in range(1, self._num_followers + 1):
                angle = math.pi + 2 * math.pi * (drone_id - 1) / self._num_followers
                ox = radius * math.cos(angle)
                oy = radius * math.sin(angle)
                rot_x = ox * cy - oy * sy
                rot_y = ox * sy + oy * cy
                self._publish_target(drone_id, now, lx + rot_x, ly + rot_y, lz)
            self._publish_fleet_positions()
            return

        offsets = FORMATIONS[self._formation]
        for drone_id in range(1, self._num_followers + 1):
            ox, oy, oz = offsets[drone_id - 1]
            rot_x = ox * cy - oy * sy
            rot_y = ox * sy + oy * cy
            self._publish_target(drone_id, now,
                                 lx + rot_x,
                                 ly + rot_y,
                                 lz + oz)

        self._publish_fleet_positions()


def main(args=None):
    rclpy.init(args=args)
    node = FormationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
