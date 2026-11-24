#!/bin/bash
set -e

cd "$HOME/Fast-Drone-250"
source devel/setup.bash

echo "===== Starting MAVROS and RealSense ====="
roslaunch bringup mavros_realsense.launch &
sleep 8

echo "===== Starting VINS-Fusion ====="
roslaunch vins fast_drone_250_purevio.launch &
sleep 5

echo "===== Starting PX4Ctrl ====="
roslaunch px4ctrl run_ctrl.launch &

echo "===== System started ====="
wait
