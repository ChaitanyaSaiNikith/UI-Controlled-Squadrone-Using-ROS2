"""
state_monitor.py

Subscribes to leader and follower local positions plus formation targets,
then prints a formatted formation-health summary every second.

Subscriptions:
  /fmu/out/vehicle_local_position_v1          — leader position (no namespace)
  /drone1/fmu/out/vehicle_local_position_v1   — follower 1 position
  /drone2/fmu/out/vehicle_local_position_v1   — follower 2 position
  /drone1/target_position                     — follower 1 formation target
  /drone2/target_position                     — follower 2 formation target
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition

NUM_FOLLOWERS = 2


def _px4_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _dist(a: tuple, b: tuple) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')

        qos = _px4_qos()

        # Position state: leader + followers
        self._pos: dict[str, tuple[float, float, float] | None] = {
            'leader': None,
            **{f'drone{i}': None for i in range(1, NUM_FOLLOWERS + 1)},
        }
        # Formation targets for followers
        self._target: dict[int, tuple[float, float, float] | None] = {
            i: None for i in range(1, NUM_FOLLOWERS + 1)
        }

        # Leader (no namespace)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            lambda msg: self._pos_cb('leader', msg), qos)

        # Followers
        for i in range(1, NUM_FOLLOWERS + 1):
            ns = f'drone{i}'
            self.create_subscription(
                VehicleLocalPosition,
                f'/{ns}/fmu/out/vehicle_local_position_v1',
                lambda msg, name=ns: self._pos_cb(name, msg), qos)
            self.create_subscription(
                PoseStamped,
                f'/{ns}/target_position',
                lambda msg, idx=i: self._target_cb(idx, msg), 10)

        self.create_timer(1.0, self._print_status)
        self.get_logger().info('StateMonitor started')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _pos_cb(self, name: str, msg: VehicleLocalPosition) -> None:
        self._pos[name] = (msg.x, msg.y, msg.z)

    def _target_cb(self, drone_id: int, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._target[drone_id] = (p.x, p.y, p.z)

    # ── status print ─────────────────────────────────────────────────────────

    def _print_status(self) -> None:
        lines = ['── Formation Status ─────────────────────────────']

        lp = self._pos['leader']
        if lp:
            lines.append(f'  leader : x={lp[0]:7.2f}  y={lp[1]:7.2f}  z={lp[2]:7.2f}')
        else:
            lines.append('  leader : no data')

        for i in range(1, NUM_FOLLOWERS + 1):
            fp = self._pos[f'drone{i}']
            tgt = self._target[i]

            if fp:
                pos_str = f'x={fp[0]:7.2f}  y={fp[1]:7.2f}  z={fp[2]:7.2f}'
            else:
                pos_str = 'no data'

            if fp and tgt:
                dist = _dist(fp, tgt)
                dist_str = f'  dist_to_target={dist:.2f}m'
                health = 'OK' if dist < 1.0 else 'CATCHING UP' if dist < 3.0 else 'FAR'
                dist_str += f'  [{health}]'
            elif tgt is None:
                dist_str = '  (no target yet)'
            else:
                dist_str = '  (no position yet)'

            lines.append(f'  drone{i} : {pos_str}{dist_str}')

        lines.append('─────────────────────────────────────────────────')
        self.get_logger().info('\n' + '\n'.join(lines))

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
