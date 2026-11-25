#!/usr/bin/env bash
# Run each case in a fresh ROS and Gazebo instance from a sourced overlay.
set -euo pipefail

case_name="${1:-static}"
port="${2:-11463}"
work="$(mktemp -d /tmp/prometheus-m1-test.XXXXXX)"
mkdir -p "$work/.gazebo" "$work/.ros/log"
export HOME="$work" ROS_HOME="$work/.ros" ROS_LOG_DIR="$work/.ros/log"
export ROS_MASTER_URI="http://127.0.0.1:$port" ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI="http://127.0.0.1:$((port + 1000))"
unset ROS_HOSTNAME DISPLAY WAYLAND_DISPLAY
export LIBGL_ALWAYS_SOFTWARE=1
dynamic=false
[[ "$case_name" != dynamic ]] || dynamic=true
scan=scan
odom=odom
[[ "$case_name" != scan_loss ]] || scan=test_scan
[[ "$case_name" != odom_loss ]] || odom=test_odom
echo "Logs: $work"
roslaunch prometheus_quadruped_sim standalone.launch dynamic_obstacle:="$dynamic" \
    guard_scan:="$scan" guard_odom:="$odom" > "$work/launch.log" 2>&1 &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" || true' EXIT
timeout 15 bash -c 'until rosparam get /use_sim_time >/dev/null 2>&1; do sleep 0.2; done'
rosrun prometheus_quadruped_sim m1_check.py "$case_name" 2>&1 | tee "$work/result.log"
