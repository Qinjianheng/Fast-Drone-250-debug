#!/usr/bin/env bash
set -eo pipefail

ROOT="$HOME/Fast-Drone-250"

source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11312

MODE="${1:-A}"

TD=0.0
unset VIO_DIAG_FULL_RATE

case "$MODE" in
    A) EST_TD=1 ;;
    B) EST_TD=0 ;;
    C) EST_TD=0; TD=-0.019526609 ;;
    D)
        EST_TD=0
        TD=-0.019526609
        export VIO_DIAG_FULL_RATE=1
        ;;
    *) echo "Usage: $0 A|B|C|D"; exit 1 ;;
esac

RATE="${2:-1.0}"

BAG=$(cat "$HOME/fast250_vins_diagnostics/current_raw_bag.txt")

test -f "$BAG"

NODES=$(rosnode list)

if echo "$NODES" | grep -Eq \
    '^/(vins_fusion|px4ctrl|mavros|camera/realsense2_camera)$'; then
    echo "ERROR: conflicting nodes on replay master."
    exit 1
fi

CFG="$ROOT/src/realflight_modules/VINS-Fusion/config"

RUN="$(dirname "$BAG")/replay_${MODE}_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RUN/output"

cp "$CFG/fast_drone_250.yaml" "$RUN/"
cp "$CFG/left.yaml" "$CFG/right.yaml" "$RUN/"

# A/B 使用相同的原始空间外参
sed -i -E \
    's/^estimate_extrinsic:.*/estimate_extrinsic: 0/' \
    "$RUN/fast_drone_250.yaml"

sed -i -E \
    "s/^estimate_td:.*/estimate_td: $EST_TD/" \
    "$RUN/fast_drone_250.yaml"

sed -i -E \
    "s/^td:.*/td: $TD/" \
    "$RUN/fast_drone_250.yaml"

sed -i \
    "s|^output_path:.*|output_path: \"$RUN/output\"|" \
    "$RUN/fast_drone_250.yaml"

cp "$(rospack find vins)/launch/fast_drone_250_purevio.launch" \
    "$RUN/replay.launch"

sed -i \
    "s|args=\"[^\"]*\"|args=\"$RUN/fast_drone_250.yaml\"|" \
    "$RUN/replay.launch"

# 使用消息原始 header 时间戳
rosparam set /use_sim_time false

echo "===== Replay $MODE ====="
echo "Bag: $BAG"
echo "Output: $RUN"

grep -E '^(estimate_extrinsic|estimate_td|td):' \
    "$RUN/fast_drone_250.yaml"

PID=""

cleanup() {
    if [ -n "$PID" ]; then
        kill -INT -- "-$PID" 2>/dev/null || true
        wait "$PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid roslaunch "$RUN/replay.launch" \
    > "$RUN/vins.log" 2>&1 &

PID=$!

for ((i=0;i<20;i++)); do
    kill -0 "$PID"

    if rosnode list | grep -qx '/vins_fusion'; then
        break
    fi

    sleep 1
done

rosnode list | grep -qx '/vins_fusion'

sleep 2

rosbag play -r "$RATE" --delay=2 "$BAG" --topics \
    /camera/infra1/image_rect_raw \
    /camera/infra2/image_rect_raw \
    /mavros/imu/data_raw \
    2>&1 | tee "$RUN/replay.log"

sleep 2

echo "===== Replay completed ====="
echo "$RUN"

