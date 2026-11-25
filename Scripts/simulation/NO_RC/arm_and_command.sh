#!/bin/bash

# 解锁无人机
gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 0
arming: true 
px4_mode: \"\"
control_state: \"\""' 

# 暂停 3 秒
sleep 3
echo "pub_command_control"
# 进入命令模式
gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true 
px4_mode: \"\"
control_state: \"COMMAND_CONTROL\""' 

sleep 3

echo "Switch back to MANUAL_CONTROL (RC_POS_CONTROL)"

# 3) 切换回 MANUAL_CONTROL → 无人机内部自动变为 RC_POS_CONTROL=1
gnome-terminal --window -e 'rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: \"\"
cmd: 3
arming: true
px4_mode: \"\"
control_state: \"MANUAL_CONTROL\""' 