# Multi-UAV Hierarchical Control System

## Project Overview
ROS 2 based hierarchical control interface for multi-UAV squadrons. A human operator controls a leader UAV via a web UI (virtual joystick), and N follower UAVs autonomously maintain formation. Tested with 1 leader + 4 followers (scalable to any N).

## Tech Stack
- **ROS 2** (Humble) on Ubuntu 22.04
- **PX4 SITL** (v1.17.0-alpha1)
- **Gazebo Harmonic** (gz sim 8.11.0)
- **Python 3** for all ROS 2 nodes
- **HTML/JS + rosbridge_websocket** for web UI (`web/web_ui.html`)
- **px4_msgs** commit `51e6678` — MUST use this version (see px4_msgs section)
- **ASUS ROG Strix G17**: Ryzen 9, RTX 3070, 16GB RAM

## Current Working State
- Web UI virtual joystick controls leader drone ✓
- ARM / DISARM / TAKEOFF ALL / LAND ALL buttons in web UI ✓
- Dynamic drone count — web UI auto-configures for any N followers ✓
- N followers auto-arm when leader arms ✓
- Followers hover until leader reaches 3m altitude, then follow in formation ✓
- 5 formation types switchable mid-flight (V, line, diamond, square, circle) ✓
- Formations computed in leader **body frame** and yaw-rotated into world NED ✓
- All followers auto-land when leader disarms ✓
- 2D drone position map in web UI updates live ✓

## Architecture

### Layer 1: Web UI (`web/web_ui.html`)
- Virtual joystick (left: throttle+yaw, right: forward+strafe)
- ARM, DISARM, TAKEOFF, LAND ALL buttons
- Formation selector: V, Line, ◇, □, ⬤ (circle)
- Live 2D drone position map
- Connects via **rosbridge_websocket** on port 9090
- Subscribes to `/num_followers` (Int32) on connect → calls `setupDroneUI(n)` dynamically
- `_followerSubs` is a dict (not array) to prevent duplicate subscriptions across reconnects
- `resizeMap()` only called on window resize — NOT inside `drawMap()` (caused canvas reset every frame)

### Layer 2: ROS 2 Nodes
- **leader_teleop.py**: Reads physical joystick → publishes Twist to `/offboard_velocity_cmd`, Bool to `/arm_message`. Has `joy_received` guard — only publishes after first Joy message arrives.
- **leader_offboard.py**: Subscribes to `/offboard_velocity_cmd` + `/cmd_takeoff` + `/cmd_land` → sends PX4 offboard setpoints. Rotates body-frame velocity to world NED using current yaw.
- **formation_manager.py**: Subscribes to leader position+velocity+attitude → computes yaw-rotated body-frame offsets → publishes `/droneX/target_position` at 20 Hz. Predicts leader position 0.3s ahead to cancel follower latency lag.
- **follower_controller.py**: Position-setpoint controller per follower. Auto-arms with leader, auto-lands when leader disarms. Uses PX4 position control (not velocity PD).
- **state_monitor.py**: Prints formation health status every second.

### Layer 3: PX4 SITL + Gazebo
- N+1 PX4 instances (1 leader + N followers), fully dynamic via `NUM_FOLLOWERS` env var
- Leader has **NO namespace**; followers use `/droneX/` namespace
- NED coordinate system: x=North, y=East, z=Down (negative z = above ground)

## Key Topic Naming Convention
- Leader (instance 0): `/fmu/in/trajectory_setpoint`, `/fmu/out/vehicle_local_position_v1` — **no namespace prefix**
- Followers (instance 1+): `/droneX/fmu/in/trajectory_setpoint`, `/droneX/fmu/out/vehicle_local_position_v1`
- Status topics use `_v2` suffix: `/fmu/out/vehicle_status_v2`, `/droneX/fmu/out/vehicle_status_v2`
- Attitude topic has NO suffix: `/fmu/out/vehicle_attitude`

## Inter-Node Topics
| Topic | Type | Published by | Subscribed by |
|---|---|---|---|
| `/offboard_velocity_cmd` | Twist | web UI / leader_teleop | leader_offboard |
| `/arm_message` | Bool | web UI | leader_offboard |
| `/cmd_takeoff` | Bool | web UI | leader_offboard |
| `/cmd_land` | Bool | web UI | leader_offboard |
| `/formation_type` | String | web UI | formation_manager |
| `/num_followers` | Int32 | formation_manager (every 3s) | web UI |
| `/droneX/target_position` | PoseStamped | formation_manager | follower_controller |
| `/leader_state` | PoseStamped | leader_offboard | (web UI, optional) |

## PX4 Offboard Control Pattern — CRITICAL

### QoS Profile (ALL PX4 topics)
```python
qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5
)
```
- **rosbridge only supports VOLATILE** QoS — never use TRANSIENT_LOCAL on topics the web UI subscribes to

### Follower Offboard Sequence (velocity mode)
```
1. Stream OffboardControlMode(velocity=True) + zero velocity setpoints for 50 cycles at 50 Hz
2. Send VEHICLE_CMD_DO_SET_MODE (176, param1=1.0, param2=6.0) to engage offboard
3. Wait for leader to arm, then send VEHICLE_CMD_COMPONENT_ARM_DISARM (400, param1=1.0, param2=21196.0)
4. PD controller: vel = Kp*(target - pos) - Kd*current_vel (Kp=0.5, Kd=0.3, max_h=4 m/s)
```

**NOTE — position-mode offboard does NOT work reliably for follower instances in PX4 SITL multi-instance.**
Follower instances (spawned via `./bin/px4 -i X`) fail to engage position-mode offboard — drones scatter to random positions. Use velocity=True heartbeat permanently, same as the leader.

### Leader Offboard (velocity mode)
```python
# OffboardControlMode: position=False, velocity=True
# TrajectorySetpoint: position=[nan,nan,nan], velocity=[vx,vy,vz], yawspeed=val
# Force arm in SITL: command 400, param1=1.0, param2=21196.0
# Disarm in SITL: command 400, param1=0.0, param2=21196.0
```

## Formation System

### CRITICAL — NED Frame Mismatch Between Instances

**Each PX4 SITL instance sets its NED origin at its own Gazebo spawn position.**

The launch script spawns followers at `POSE_Y = i * 3` (Gazebo Y = North in ENU = NED x).
- Leader NED origin = world (0, 0, 0)
- Follower 1 NED origin = world (3, 0, 0)
- Follower 2 NED origin = world (6, 0, 0), etc.

Formation_manager computes targets in world NED (leader's frame). These must be converted to each follower's local NED frame by **subtracting the spawn offset**:

```python
# In formation_manager._publish_target():
spawn_offset_x = spawn_north_m * drone_id   # NED north
target_in_follower_NED = (world_tx - spawn_offset_x, world_ty, world_tz)
```

`spawn_north_m` defaults to 3.0 (matching `i*3` in launch_multi_sitl.sh). Override with `SPAWN_NORTH_M=<val>` env var if you change the spawn spacing.

### Coordinate Frame
All formation offsets are in **leader body frame**:
- `x` = forward(+) / behind(-)
- `y` = right(+) / left(-)
- `z` = down(+) / up(-)

Rotated into world NED in `formation_manager._loop()` using leader yaw:
```python
cy, sy = math.cos(yaw), math.sin(yaw)
rot_x = ox * cy - oy * sy   # NED north
rot_y = ox * sy + oy * cy   # NED east
```

### Formation Offsets (body frame, metres)
```python
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
# First N offsets used based on num_followers
# Circle: radius=7m, evenly spaced, angle=π+(2π*(i-1)/N) so arc trails behind leader
```

## Dynamic Drone Count
- `NUM_FOLLOWERS` env var drives everything: `export NUM_FOLLOWERS=4 && ros2 launch ...`
- `SPAWN_NORTH_M` env var sets follower spawn spacing (default 3.0m, must match launch_multi_sitl.sh)
- `formation_control.launch.py` reads both env vars and passes as ROS params to formation_manager
- Web UI receives `/num_followers` (Int32, published every 3s, VOLATILE QoS) and calls `setupDroneUI(n)`

## Leader Takeoff/Land
- **TAKEOFF**: `leader_offboard._takeoff_mode=True` → climb at -1.5 m/s (NED up) until `z < -5.0`
- **LAND ALL**: `leader_offboard._landing_mode=True` → descend at 1.5 m/s until `z > -0.5`, then force-disarm (param2=21196.0 required in SITL)
- `_landing_mode` checked before the armed guard in the loop (so descent continues even after leader disarms)
- Followers detect leader disarm via `vehicle_status_v2` → set `_landing=True` → command ground-level position setpoint → force-disarm at z > -0.5

## Workspace Structure
```
~/squadron_ros2_ws/
├── web/
│   └── web_ui.html               # Browser control interface
├── scripts/
│   ├── CLAUDE.md
│   ├── launch_multi_sitl.sh      # Dynamic N followers via CLI arg
│   └── switch_formation.sh
└── src/
    ├── px4_msgs/                 # Commit 51e6678 — DO NOT UPDATE
    └── multi_uav_control/
        ├── multi_uav_control/
        │   ├── leader_teleop.py
        │   ├── leader_offboard.py
        │   ├── formation_manager.py
        │   ├── follower_controller.py
        │   └── state_monitor.py
        ├── launch/
        │   └── formation_control.launch.py
        ├── package.xml
        └── setup.py
```

## Launch Sequence
```bash
# Terminal 1 — PX4 + Gazebo + DDS agent (wait for "All drones ready")
cd ~/squadron_ros2_ws && ./scripts/launch_multi_sitl.sh 4   # 4 followers

# Terminal 2 — ROS 2 nodes
cd ~/squadron_ros2_ws && source install/setup.bash
export NUM_FOLLOWERS=4 SPAWN_NORTH_M=3.0 && ros2 launch multi_uav_control formation_control.launch.py

# Browser — open web/web_ui.html, connect to ws://localhost:9090
# Press TAKEOFF → followers auto-arm and join formation
# Switch formation with buttons
# Press LAND ALL to land everything
```

## Multi-Instance SITL Launch (CRITICAL)

### Leader (instance 0)
- Launch: `cd ~/PX4-Autopilot && make px4_sitl gz_x500`
- **Only** method that works for offboard control — running binary directly fails for leader
- Topics have **no namespace**: `/fmu/in/...`, `/fmu/out/...`

### Followers (instance 1+)
```bash
cd ~/PX4-Autopilot/build/px4_sitl_default
source rootfs/gz_env.sh
PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 PX4_UXRCE_DDS_NS=droneX \
PX4_GZ_MODEL_POSE="0,Y" ./bin/px4 -i X -d .
```
- `-d .` flag is **critical** — NOT `-d etc`
- `PX4_UXRCE_DDS_NS` is the correct env var (NOT `UXRCE_DDS_NS`)
- **Single shared MicroXRCEAgent on port 8888** for all drones
- Topics: `/droneX/fmu/in/...`, `/droneX/fmu/out/...`

## px4_msgs Version (CRITICAL)
- **MUST use commit `51e6678`**
- Latest GitHub version has a 1-byte payload mismatch → `RTPS_READER_HISTORY payload size` errors → all status callbacks fail silently
- Fix: `cp -r ~/px4_ros2_ws/src/px4_msgs ~/squadron_ros2_ws/src/px4_msgs && colcon build`

## Important Notes
- **NED frame**: x=North, y=East, z=Down. Negative z = above ground. Takeoff target = -5.0m.
- **Arming**: `vehicle_status_v2` arming_state == 2 means ARMED
- **Leader velocity rotation**: body-frame FLU → world NED using current yaw before publishing setpoint
- **Bash scripts sourcing ROS 2**: use `set -eo pipefail` NOT `set -euo pipefail` — ROS 2 setup.bash has unbound variables
- Physical joystick (Kreo, Xbox mode): axis 1 = left stick Y (throttle), axis 4 = right stick Y (forward). `joy_received` guard prevents flooding zeros when no physical joystick connected.
- Web UI joystick: left stick Y up = positive vz (ascend in NED), left stick Y down = negative vz (descend). Confirmed correct.
