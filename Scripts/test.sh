#!/bin/bash
# === 启动仿真环境 ===

# 指定显示器（机载电脑的主显示器一般是 :0）
export DISPLAY=:0

# 确保 xhost 权限允许当前用户访问 X Server
xhost +local:$(whoami)

# 后台启动 ROS 仿真，不受 SSH 终端影响
nohup roslaunch prometheus_gazebo sitl_outdoor_1uav_P450.launch > /tmp/ros_sim.log 2>&1 &