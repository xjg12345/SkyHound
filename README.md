# 一架 P450 D435i + 一台物理四足：共场、起飞与 EGO 飞行

仅用于本机 PX4 SITL / Gazebo 仿真。使用 Ubuntu 图形桌面和 5 个 Bash 终端（A–E）。开始前停止之前的无人机和四足仿真；所有终端连接同一个本地 ROS master。使用仓库已有的无人机工作空间 `devel/` 和 PX4 SITL 构建。不要运行四足 `standalone.launch`，也不要运行末尾会切换控制模式的 `arm_and_command.sh`。

实测范围：无相机 P450 未解锁时的物理四足共场导航已通过；本页的 **D435i + 物理四足 + 起飞与 EGO 飞行组合未完成端到端验证**。首次按顺序单独验证四足和无人机，不同时给两者发目标。

## 1. 终端 A：编译四足，启动 P450 D435i、Gazebo 和无人机 RViz

```bash
cd /home/xjg/Prometheus
source /opt/ros/noetic/setup.bash
catkin_make --source Simulator/quadruped_sim --build build/quadruped_sim \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_quadruped" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -j4
```

编译通过后，在 A 继续执行。此命令会打开 Gazebo 和无人机 RViz，并在需要时启动本地 ROS master；保留 A 运行。

```bash
source /home/xjg/Prometheus/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export GAZEBO_MASTER_URI=http://localhost:11345
PX4="$HOME/prometheus_px4"
source "$PX4/Tools/setup_gazebo.bash" "$PX4" "$PX4/build/amovlab_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4:$PX4/Tools/sitl_gazebo"
export PATH="$PX4/build/amovlab_sitl_default/bin:$PATH"
MODELS="$PWD/Simulator/gazebo_simulator/gazebo_models"
export GAZEBO_MODEL_PATH="$MODELS/uav_models:$MODELS/sensor_models:$MODELS/scene_models:$GAZEBO_MODEL_PATH"
roslaunch prometheus_gazebo sitl_p450_d435i.launch \
  vehicle:=p450_D435i uav1_id:=1 dynamic_obstacles_enable:=false
```

## 2. 终端 B：启动无人机控制器

看到 Gazebo 中出现 P450 后执行，保留 B。

```bash
source /home/xjg/Prometheus/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
roslaunch prometheus_uav_control uav_control_main_outdoor.launch uav_id:=1
```

## 3. 终端 C：启动深度建图和 EGO

两组进程共用终端 C。按 `Ctrl+C` 或其中一组退出时，清理两组进程。

```bash
source /home/xjg/Prometheus/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
bash -c '
  roslaunch prometheus_gazebo depth_to_octomap.launch uav_id:=1 & octo=$!
  roslaunch ego_planner sitl_ego_planner_basic_octomap.launch uav_id:=1 & ego=$!
  trap "exit 130" INT TERM
  trap "kill -TERM $octo $ego 2>/dev/null; wait $octo $ego 2>/dev/null" EXIT
  wait -n
'
```

## 4. 终端 D：在现有 Gazebo 注入一台物理四足

```bash
source /home/xjg/Prometheus/devel/setup.bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash --extend
export ROS_MASTER_URI=http://localhost:11311
rostopic echo -n 1 /uav1/mavros/state
```

看到 `connected: True`、`armed: False` 后执行，保留 D：

```bash
roslaunch prometheus_quadruped_sim attach.launch \
  robot_count:=1 agent_backend:=physical uav_model:=p450_D435i_0 rviz:=true
```

等到 `dog1 ready`。Gazebo 里应同时出现 `p450_D435i_0` 和 `dog1`；此时会多开一个四足 RViz，`Fixed Frame` 为 `dog_map`，不同于无人机的 `world`。

## 5. 终端 E：先验证四足到点停车

```bash
source /home/xjg/Prometheus/devel/setup.bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash --extend
export ROS_MASTER_URI=http://localhost:11311
rosservice call /gazebo/get_world_properties
rostopic echo -n 1 /dog1/physical_ready
```

确认 `data: True` 再发目标。该目标是 `dog_map` 下的共场坐标。

```bash
rosrun prometheus_quadruped_sim send_goal.py \
  --robot dog1 --x -4.3 --y -4.4 --yaw 0 --timeout 90
rostopic echo -n 1 /dog1/cmd_vel
```

看 Gazebo 中的迈步与停车、四足 RViz 中的路径、终端的 `SUCCEEDED` 和最终零速命令。若要在行走时检查关节和足端，可在发目标之前运行 `rostopic echo /dog1/joint_states` 或 `rostopic echo /dog1/feet/fl`，另一次试跑时观察；监听命令可用 `Ctrl+C` 停止。四足停稳后再进行下一节。

## 6. 终端 E：检查数据链并起飞悬停

逐条执行，不要整段连发。先确认 SITL 已连接、定位有效、规划器订阅了目标。`timeout` 超时没有输出时，应先检查对应启动终端。

```bash
timeout 10s rostopic echo -n 1 /uav1/mavros/state
timeout 10s rostopic echo -n 1 /uav1/prometheus/state/odom_valid
timeout 10s rostopic echo -n 1 /uav1/camera/depth/color/points/header
rostopic info /uav1/prometheus/motion_planning/goal
```

确认 `connected: True`、`armed: False`、`odom_valid` 为 `True` 后解锁：

```bash
rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup '{cmd: 0, arming: true}'
rostopic echo -n 1 /uav1/mavros/state
```

只有实际看到 `armed: True`，才切入命令模式。控制器将请求 `OFFBOARD`，初始指令 `Init_Pos_Hover` 默认升到起飞点上方约 1.5 m。

```bash
rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup '{cmd: 3, control_state: COMMAND_CONTROL}'
rostopic echo -n 1 /uav1/prometheus/control_state
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/mavros/local_position/pose/pose/position
```

等到 `control_state: 2`、`failsafe: False`、`mode: "OFFBOARD"`，并看到 Gazebo 中无人机升起、稳定悬停。失败时先检查 B 的定位、模式和解锁日志，不要继续发航点。

## 7. 终端 E：先验证短程轨迹，再验证柱体绕障

先查看建图数据是否更新；无人机 RViz 的点云应能显示前方柱体。只有看见障碍被建图后，才尝试柱体绕障。

```bash
timeout 10s rostopic echo -n 1 /uav1/octomap_point_cloud_centers/header

# 短程飞行检查：默认从原点附近起飞，先去 (1, 0, 1.5)。
rostopic pub -1 /uav1/prometheus/motion_planning/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}'
```

Gazebo 中到达后再做下一步。短目标只验证规划与跟踪，不算避障验收。下一个**候选**目标 `(6, 0, 1.5)` 的直线经过场景中 `cylinder1`（中心 XY 为 `(4.5,0)`，高度 3 m）。先在无人机 RViz 确认柱体已入图且目标点未落入占据区，再执行：

```bash
timeout 20s rostopic echo -n 1 /uav1/planning/bspline > /tmp/p450-ego-bspline.yaml &
rostopic pub -1 /uav1/prometheus/motion_planning/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 6.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}'
rostopic echo -n 1 /uav1/mavros/local_position/pose/pose/position
```

判断是否真正绕障：C 有规划输出，RViz 轨迹绕开柱体，Gazebo 中飞机实际避开柱体并到达目标附近。收到一条 B 样条消息本身不能证明避障（也可能是停车轨迹）。若路径穿过柱体、地图不更新或飞机未按轨迹飞行，不再发新目标；在下方空旷时按下一节降落。这里用 CLI 明确指定高度，因为当前 EGO 会直接使用 `PoseStamped` 的 `z`，不应假定 RViz 的 `2D Nav Goal` 会自动保留悬停高度。

## 8. 终端 E：降落并退出

在 Gazebo 确认无人机下方空旷，再通过 Prometheus 发降落命令：

```bash
rostopic pub -1 /uav1/prometheus/command prometheus_msgs/UAVCommand \
  '{Agent_CMD: 3, Control_Level: 1}'
rostopic echo /uav1/mavros/state
```

观察实际落地并等待 `armed: False`，随后 `Ctrl+C` 停止状态监听。不要在空中手动上锁。按 **D → C → B → A** 的顺序各按一次 `Ctrl+C`：先移除四足，再停止 EGO/建图、无人机控制器，最后关闭 Gazebo、无人机 RViz 和本次启动的 ROS master。D435i 服务端若报 RealSense/渲染错误而退出，这次 EGO 共场不成立；无相机 P450 可用于四足回归，但不能替代本页深度避障测试。
