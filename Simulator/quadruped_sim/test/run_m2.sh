#!/usr/bin/env bash
# Run one fresh four-dog trial from a sourced quadruped overlay.
set -euo pipefail
port="${1:-11561}"
work="$(mktemp -d /tmp/prometheus-m2-test.XXXXXX)"
mkdir -p "$work/.gazebo" "$work/.ros/log"
export HOME="$work" ROS_HOME="$work/.ros" ROS_LOG_DIR="$work/.ros/log"
export ROS_MASTER_URI="http://127.0.0.1:$port" ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI="http://127.0.0.1:$((port + 1000))"
unset ROS_HOSTNAME DISPLAY WAYLAND_DISPLAY
script_dir="$(dirname "$(realpath "$0")")"
echo "Logs: $work"
roslaunch prometheus_quadruped_sim standalone.launch robot_count:=4 >"$work/launch.log" 2>&1 &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" || true' EXIT
timeout 15 bash -c 'until rosparam get /use_sim_time >/dev/null 2>&1; do sleep 0.2; done'
timeout 130 /usr/bin/python3 "$script_dir/m2_check.py" --cancel-one 2>&1 | tee "$work/result.log"
