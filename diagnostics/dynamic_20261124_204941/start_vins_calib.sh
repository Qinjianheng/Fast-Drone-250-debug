#!/usr/bin/env bash
set -eo pipefail

ROOT="$HOME/Fast-Drone-250"

source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11311

CFG="$ROOT/src/realflight_modules/VINS-Fusion/config"
CAL="$HOME/fast250_calibration/dynamic_$(date +%Y%m%d_%H%M%S)"

# 检查 ROS 节点
NODES=$(rosnode list)

if echo "$NODES" | grep -qx '/px4ctrl'; then
    echo "[ERROR] PX4Ctrl is running."
    exit 1
fi

if echo "$NODES" | grep -qx '/vins_fusion'; then
    echo "[ERROR] VINS is already running."
    exit 1
fi

# 检查传感器及飞控状态
python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import Image, Imu

rospy.init_node("calibration_precheck", anonymous=True)

s = rospy.wait_for_message(
    "/mavros/state", State, timeout=10
)

if s.armed or not s.connected:
    raise SystemExit("ERROR: FCU armed or disconnected")

print("[OK] FCU connected and disarmed")

for topic, typ in [
    ("/mavros/imu/data_raw", Imu),
    ("/camera/infra1/image_rect_raw", Image),
    ("/camera/infra2/image_rect_raw", Image)
]:
    rospy.wait_for_message(topic, typ, timeout=10)
    print("[OK]", topic)
PY

# 创建独立标定目录
mkdir -p "$CAL/output"

# 复制实际飞行配置
cp "$CFG/fast_drone_250.yaml" "$CAL/"
cp "$CFG/left.yaml" "$CFG/right.yaml" "$CAL/"

# 保存原始参数
cp "$CAL/fast_drone_250.yaml" "$CAL/original.yaml"

# 开启在线外参估计
sed -i -E \
    's/^estimate_extrinsic:.*/estimate_extrinsic: 1/' \
    "$CAL/fast_drone_250.yaml"

# 开启时间偏移估计
sed -i -E \
    's/^estimate_td:.*/estimate_td: 1/' \
    "$CAL/fast_drone_250.yaml"

# 设置独立输出目录
sed -i \
    "s|^output_path:.*|output_path: \"$CAL/output\"|" \
    "$CAL/fast_drone_250.yaml"

# 复制实际飞行使用的 launch
cp "$(rospack find vins)/launch/fast_drone_250_purevio.launch" \
    "$CAL/calibration.launch"

sed -i \
    "s|args=\"[^\"]*\"|args=\"$CAL/fast_drone_250.yaml\"|" \
    "$CAL/calibration.launch"

echo "$CAL" > "$HOME/fast250_calibration/current_run.txt"

echo "===== Calibration configuration ====="

grep -E '^(estimate_extrinsic|estimate_td|td|output_path):' \
    "$CAL/fast_drone_250.yaml"

echo "===== Calibration directory ====="
echo "$CAL"

echo "===== Dynamic calibration ====="
echo "Remove propellers. Keep FCU disarmed."

# 启动当前版本的 VINS
roslaunch "$CAL/calibration.launch" 2>&1 | tee "$CAL/vins.log"

