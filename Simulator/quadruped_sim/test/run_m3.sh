#!/usr/bin/env bash
# Source devel_quadruped/setup.bash before running.
set -euo pipefail
case_name="${1:-gait}"
port="${2:-11630}"
case "$case_name" in
  gait|static) count=1; dynamic=false ;;
  dynamic) count=1; dynamic=true ;;
  team) count=4; dynamic=false ;;
  *) echo "Usage: $0 {gait|static|dynamic|team} [ROS_PORT]" >&2; exit 2 ;;
esac
work="$(mktemp -d /tmp/prometheus-m3-test.XXXXXX)"
mkdir -p "$work/.gazebo" "$work/.ros/log"
export HOME="$work" ROS_HOME="$work/.ros" ROS_LOG_DIR="$work/.ros/log"
export ROS_MASTER_URI="http://127.0.0.1:$port" ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI="http://127.0.0.1:$((port + 1000))"
unset ROS_HOSTNAME DISPLAY WAYLAND_DISPLAY
export LIBGL_ALWAYS_SOFTWARE=1
script_dir="$(dirname "$(realpath "$0")")"
echo "Logs: $work"
roslaunch prometheus_quadruped_sim standalone.launch agent_backend:=physical \
  robot_count:="$count" dynamic_obstacle:="$dynamic" \
  a1_mesh_dir:="${A1_MESH_DIR:-}" >"$work/launch.log" 2>&1 &
pid=$!
trap 'kill -INT "$pid" 2>/dev/null || true; wait "$pid" || true' EXIT
timeout 15 bash -c 'until rosparam get /use_sim_time >/dev/null 2>&1; do sleep 0.2; done'
case "$case_name" in
  gait) command=(/usr/bin/python3 "$script_dir/m3_check.py") ;;
  static|dynamic) command=(rosrun prometheus_quadruped_sim m1_check.py "$case_name" --physical) ;;
  team) command=(/usr/bin/python3 "$script_dir/m2_check.py" --physical) ;;
esac
timeout 150 "${command[@]}" 2>&1 | tee "$work/result.log"
