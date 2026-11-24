#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash

echo "[set_imu200] Waiting for MAVROS connection..."

# 最多等待60秒，超时则退出
if ! timeout 60 bash -c '
  until timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null \
    | grep -q "connected: True"; do
    sleep 1
  done
'; then
  echo "[set_imu200] ERROR: MAVROS connection timeout"
  exit 1
fi

echo "[set_imu200] MAVROS connected"

# 等待消息频率设置服务
python3 -c 'import rospy; rospy.wait_for_service("/mavros/set_message_interval", timeout=20)'

request_rate() {
  local msg_id="$1"
  local result

  result=$(rosservice call /mavros/set_message_interval \
    "message_id: ${msg_id}
message_rate: 200.0") || {
      echo "[set_imu200] ERROR: request failed for ${msg_id}"
      return 1
    }

  if ! echo "$result" | grep -q "success: True"; then
    echo "[set_imu200] ERROR: request rejected for ${msg_id}"
    echo "$result"
    return 1
  fi

  echo "[set_imu200] Message ${msg_id}: 200 Hz requested"
}

request_rate 31
request_rate 105

echo "[set_imu200] Both requests accepted"
