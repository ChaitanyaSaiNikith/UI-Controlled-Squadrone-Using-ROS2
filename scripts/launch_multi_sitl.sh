#!/usr/bin/env bash
# launch_multi_sitl.sh
# Full 3-drone SITL startup sequence (leader + N followers).
#
# Step 1: Launch leader via make px4_sitl gz_x500 (starts Gazebo)
# Step 2: Launch single MicroXRCEAgent on port 8888 (all drones share it)
# Step 3: Spawn follower instances from build dir, 10s apart
# Step 4: Wait 15s for stabilisation, then print ready message
#
# Usage: ./launch_multi_sitl.sh [N]   (default N=2 followers)
#
# Press Ctrl+C to cleanly stop everything.

set -eo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
NUM_FOLLOWERS=${1:-2}
PX4_AUTOPILOT="${HOME}/PX4-Autopilot"
PX4_BUILD="${PX4_AUTOPILOT}/build/px4_sitl_default"
XRCE_CMD="MicroXRCEAgent"
DDS_PORT=8888
LEADER_WAIT=20       # seconds for Gazebo to fully load
FOLLOWER_WAIT=10     # seconds between follower spawns
STABILISE_WAIT=15    # seconds after last spawn before "ready"

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
log()  { echo -e "${GREEN}[SITL]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERR ]${NC} $*" >&2; }
step() { echo -e "\n${BOLD}${CYAN}▶ $*${NC}"; }

# ── Preflight checks ─────────────────────────────────────────────────────────
if ! [[ "$NUM_FOLLOWERS" =~ ^[1-9][0-9]*$ ]]; then
    err "NUM_FOLLOWERS must be a positive integer, got: '$NUM_FOLLOWERS'"
    exit 1
fi
[[ -d "$PX4_BUILD" ]]              || { err "PX4 build dir not found: $PX4_BUILD"; exit 1; }
command -v "$XRCE_CMD" &>/dev/null || { err "MicroXRCEAgent not found in PATH"; exit 1; }
command -v gz &>/dev/null          || { err "gz (Gazebo) not found in PATH"; exit 1; }

# ── Cleanup trap ─────────────────────────────────────────────────────────────
CHILD_PIDS=()

cleanup() {
    echo ""
    log "Shutting down — killing all child processes..."
    for pid in "${CHILD_PIDS[@]}"; do
        kill -SIGTERM "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${CHILD_PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && kill -SIGKILL "$pid" 2>/dev/null || true
    done
    pkill -SIGTERM -f "px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -SIGTERM -f "MicroXRCEAgent"            2>/dev/null || true
    pkill -SIGTERM -f "gz sim"                    2>/dev/null || true
    pkill -SIGTERM -f "gzserver"                  2>/dev/null || true
    sleep 1
    log "Cleanup done."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ── Source ROS 2 ──────────────────────────────────────────────────────────────
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    log "Sourced ROS 2 Humble"
else
    warn "/opt/ros/humble/setup.bash not found — skipping"
fi

# ─────────────────────────────────────────────────────────────────────────────
# STEP 1 — Leader drone
# ─────────────────────────────────────────────────────────────────────────────
step "Step 1 — Launching leader drone (make px4_sitl gz_x500)"
log "This starts Gazebo and spawns the leader. Topics at /fmu/... (no namespace)."

(
    cd "${PX4_AUTOPILOT}"
    exec make px4_sitl gz_x500 \
        > "${TMPDIR:-/tmp}/px4_leader.log" 2>&1
) &
LEADER_PID=$!
CHILD_PIDS+=("$LEADER_PID")
log "Leader PID ${LEADER_PID}  log: ${TMPDIR:-/tmp}/px4_leader.log"
log "Waiting ${LEADER_WAIT}s for Gazebo to fully load..."
sleep "${LEADER_WAIT}"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2 — Single DDS agent (all drones share port 8888)
# ─────────────────────────────────────────────────────────────────────────────
step "Step 2 — Launching MicroXRCEAgent on port ${DDS_PORT}"
log "All drones (leader + followers) communicate through this single agent."

(
    exec "${XRCE_CMD}" udp4 -p "${DDS_PORT}" \
        > "${TMPDIR:-/tmp}/xrce_agent.log" 2>&1
) &
XRCE_PID=$!
CHILD_PIDS+=("$XRCE_PID")
log "XRCE PID ${XRCE_PID}  log: ${TMPDIR:-/tmp}/xrce_agent.log"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3 — Spawn followers
# ─────────────────────────────────────────────────────────────────────────────
step "Step 3 — Spawning ${NUM_FOLLOWERS} follower(s)"

# Source Gazebo environment (required for followers to find gz models/plugins)
GZ_ENV="${PX4_BUILD}/rootfs/gz_env.sh"
if [[ -f "$GZ_ENV" ]]; then
    source "$GZ_ENV"
    log "Sourced gz_env.sh"
else
    warn "gz_env.sh not found at $GZ_ENV — Gazebo models may not load correctly"
fi

for (( i=1; i<=NUM_FOLLOWERS; i++ )); do
    DRONE_NS="drone${i}"
    POSE_Y=$(( i * 3 ))

    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "Spawning follower ${i}  ns=${DRONE_NS}  pose=(0, ${POSE_Y})  port=${DDS_PORT}"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    (
        cd "${PX4_BUILD}"
        export PX4_SIM_MODEL=gz_x500
        export GZ_IP=127.0.0.1
        export PX4_UXRCE_DDS_NS="${DRONE_NS}"
        export PX4_GZ_MODEL_POSE="0,${POSE_Y}"
        exec ./bin/px4 -i "${i}" -d . \
            > "${TMPDIR:-/tmp}/px4_drone${i}.log" 2>&1
    ) &
    FOLLOWER_PID=$!
    CHILD_PIDS+=("$FOLLOWER_PID")
    log "  drone${i} PID ${FOLLOWER_PID}  log: ${TMPDIR:-/tmp}/px4_drone${i}.log"

    if (( i < NUM_FOLLOWERS )); then
        log "  Waiting ${FOLLOWER_WAIT}s before next follower..."
        sleep "${FOLLOWER_WAIT}"
    fi
done

# ─────────────────────────────────────────────────────────────────────────────
# STEP 4 — Wait for stabilisation
# ─────────────────────────────────────────────────────────────────────────────
step "Step 4 — Waiting ${STABILISE_WAIT}s for all drones to stabilise..."
sleep "${STABILISE_WAIT}"

# ── Final summary ─────────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}${GREEN}  All drones ready!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${CYAN}DDS Agent:${NC}  port ${DDS_PORT}  (shared by all drones)"
echo ""
echo -e "  ${CYAN}Leader:${NC}     /fmu/in/...   /fmu/out/...   [no namespace]"
for (( i=1; i<=NUM_FOLLOWERS; i++ )); do
    echo -e "  ${CYAN}Follower ${i}:${NC}  /drone${i}/fmu/in/...   /drone${i}/fmu/out/..."
done
echo ""
echo -e "  ${CYAN}Logs:${NC}"
echo -e "    Leader:  ${TMPDIR:-/tmp}/px4_leader.log"
for (( i=1; i<=NUM_FOLLOWERS; i++ )); do
    echo -e "    drone${i}:  ${TMPDIR:-/tmp}/px4_drone${i}.log"
done
echo -e "    XRCE:    ${TMPDIR:-/tmp}/xrce_agent.log"
echo ""
echo -e "  ${BOLD}Now run:${NC}"
echo -e "    ${CYAN}export NUM_FOLLOWERS=${NUM_FOLLOWERS} && ros2 launch multi_uav_control formation_control.launch.py${NC}"
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
log "Press Ctrl+C to stop all instances."

# Stay alive
wait
