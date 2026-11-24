#!/usr/bin/env bash
# Fast-Drone-250: sensor/VIO startup and optional propeller-off controller test.
set -eo pipefail
MODE=${1:---diagnostic}
case "$MODE" in
  --diagnostic|--ground-ctrl) ;;
  *) echo "Usage: bash $0 [--diagnostic|--ground-ctrl]"; exit 2;;
esac
ROOT="$HOME/Fast-Drone-250"
CFG="$ROOT/src/realflight_modules/VINS-Fusion/config/fast_drone_250.yaml"
cd "$ROOT"
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311
OUT="$HOME/fast250_vins_diagnostics/hover_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$OUT"
PIDS=()
LABELS=()
start() {
  local label=$1; shift
  echo "[START] $label -> $OUT/$label.log"
  setsid "$@" >"$OUT/$label.log" 2>&1 &
  PIDS+=("$!"); LABELS+=("$label")
}
cleanup() {
  local rc=$? i p
  trap - EXIT INT TERM
  echo "[STOP] Stopping only processes started by this script"
  for ((i=${#PIDS[@]}-1;i>=0;i--)); do
    p=${PIDS[i]}
    kill -INT -- "-$p" 2>/dev/null || true
    # Let rosbag close and roslaunch shut down its children.
    for ((j=0;j<50;j++)); do
      kill -0 -- "-$p" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 -- "-$p" 2>/dev/null; then
      kill -TERM -- "-$p" 2>/dev/null || true
    fi
    wait "$p" 2>/dev/null || true
  done
  echo "[LOG] $OUT"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
check_children() {
  local i
  for ((i=0;i<${#PIDS[@]};i++)); do
    if ! kill -0 "${PIDS[i]}" 2>/dev/null; then
      echo "[ERROR] ${LABELS[i]} exited; inspect $OUT/${LABELS[i]}.log" >&2
      tail -30 "$OUT/${LABELS[i]}.log" >&2 || true
      return 1
    fi
  done
}
wait_topic() {
  local topic=$1
  echo "[WAIT] $topic"
  if ! timeout 20s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    echo "[ERROR] No message on $topic" >&2; exit 1
  fi
  check_children
}
# Validate paths, config and launch resolution without changing any files.
for f in "$CFG" "$(dirname "$CFG")/left.yaml" "$(dirname "$CFG")/right.yaml"; do
  test -r "$f" || { echo "[ERROR] Missing $f"; exit 1; }
  cp -a "$f" "$OUT/"
done
if ! grep -Eq '^estimate_extrinsic:[[:space:]]*0([[:space:]]|$)' "$CFG"; then
  echo '[ERROR] Flight config is not using fixed extrinsics. Review the YAML.'; exit 1
fi
roslaunch --files bringup mavros_imu200.launch > "$OUT/mavros_launch_files.txt"
roslaunch --files realsense2_camera rs_camera.launch > "$OUT/camera_launch_files.txt"
roslaunch --files vins fast_drone_250_purevio.launch > "$OUT/vins_launch_files.txt"
if [[ "$MODE" == --ground-ctrl ]]; then
  roslaunch --files px4ctrl run_ctrl.launch > "$OUT/px4ctrl_launch_files.txt"
fi
if pgrep -x QGroundControl >/dev/null || pgrep -x qgroundcontrol >/dev/null; then
  echo '[ERROR] Close QGroundControl first to avoid serial port contention.'; exit 1
fi
if timeout 3s rosnode list > "$OUT/initial_nodes.txt" 2>/dev/null; then
  echo '[INFO] Reusing existing ROS Master.'
  if grep -Eq '^/(mavros(/.*)?|camera/realsense2_camera(_manager)?|vins_fusion|px4ctrl)$' "$OUT/initial_nodes.txt"; then
    echo '[ERROR] Existing flight-stack nodes detected. Stop them in their original terminals first.'
    cat "$OUT/initial_nodes.txt"; exit 1
  fi
else
  start roscore roscore
  for ((i=0;i<20;i++)); do
    if timeout 2s rosnode list >/dev/null 2>&1; then break; fi
    check_children; sleep 1
  done
  timeout 3s rosnode list >/dev/null
fi
start mavros roslaunch bringup mavros_imu200.launch
wait_topic /mavros/state
wait_topic /mavros/imu/data_raw
# Check actual FCU connection and disarmed status before proceeding.
python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
rospy.init_node('hover_start_fcu_check', anonymous=True, disable_signals=True)
s = rospy.wait_for_message('/mavros/state', State, timeout=10)
print('FCU connected:', s.connected, 'armed:', s.armed, 'mode:', s.mode)
if not s.connected or s.armed:
    raise SystemExit('FCU must be connected and disarmed.')
PY
start camera roslaunch realsense2_camera rs_camera.launch \
  enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true \
  infra_width:=640 infra_height:=480 infra_fps:=30
wait_topic /camera/infra1/image_rect_raw
wait_topic /camera/infra2/image_rect_raw
# Record only selected diagnostic topics, never compressedDepth or all topics.
start rosbag rosbag record -O "$OUT/hover.bag" \
  /global_odom /vins_fusion/odometry /mavros/imu/data_raw /mavros/imu/data \
  /mavros/state /mavros/rc/in /mavros/setpoint_raw/attitude /debugPx4ctrl /rosout
start vins roslaunch vins fast_drone_250_purevio.launch
wait_topic /vins_fusion/odometry
wait_topic /global_odom
if [[ "$MODE" == --ground-ctrl ]]; then
  echo '[GROUND ONLY] Remove all propellers, keep FCU disarmed, and review the VIO output.'
  echo 'The previous position-jump fault has NOT been resolved by this script.'
  read -r -p 'Type GROUND to start PX4Ctrl for ground diagnostics only: ' answer
  [[ "$answer" == GROUND ]] || { echo 'Cancelled.'; exit 1; }
  check_children
  python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
rospy.init_node('hover_start_ground_check', anonymous=True, disable_signals=True)
s = rospy.wait_for_message('/mavros/state', State, timeout=5)
if not s.connected or s.armed:
    raise SystemExit('FCU must remain connected and disarmed.')
PY
  start px4ctrl roslaunch px4ctrl run_ctrl.launch
else
  echo '[INFO] Diagnostic mode: PX4Ctrl not started.'
fi
printf '\n[LOG] %s\n' "$OUT"
echo '[INFO] Ctrl+C stops this script and its own child processes; no arm/takeoff command is sent.'
echo "[INFO] Inspect VINS log: tail -f '$OUT/vins.log'"
while true; do check_children; sleep 2; done
