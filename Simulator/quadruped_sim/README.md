# Prometheus 四足导航

`proxy` 是默认的导航回归后端。Gazebo 中显示橙灰色四足外壳，腿固定在机身上随底盘移动，不会迈步；原碰撞体、激光和导航参数不变。不需要下载网格模型。`physical` 使用 12 个力矩控制关节和独立足端接触。两种后端使用相同的目标接口，并支持独立场景和注入已有 Gazebo。物理后端的通过项和限制见 [M3 验收报告](M3_RESULTS.md)。

已经看到蓝色方块时，退出当前四足启动，再用原命令启动；共场只需重启 `attach.launch`，不用重启 P450。

## 观看物理四足

先完成下文的编译，再在 Ubuntu 桌面终端启动：

```bash
source ~/Prometheus/devel_quadruped/setup.bash
roslaunch prometheus_quadruped_sim standalone.launch agent_backend:=physical gui:=true rviz:=true
```

在第二个终端加载同一环境，再发送目标。不要同时启动代理场景；四台物理狗可加 `robot_count:=4`。

```bash
source ~/Prometheus/devel_quadruped/setup.bash
rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x 5 --y 0 --timeout 90
```

`/dog1/joint_states` 是实际关节位置，`/dog1/feet/{fl,fr,rl,rr}` 是四只脚的接触，`/dog1/physical_ready` 和 `/dog1/physical_fault` 是控制器状态。跌倒后不自动恢复，须退出后重启场景或重新注入。

### 可选：宇树 A1 官方外观

在仓库根目录运行。脚本用 `curl` 下载固定版本的宇树 `unitree_ros` A1 网格、纹理及 BSD-3-Clause 许可证到本机缓存；不需要编译官方仓库或安装宇树 SDK。

```bash
source ~/Prometheus/devel_quadruped/setup.bash
A1_MESH_DIR="$(bash Simulator/quadruped_sim/scripts/fetch_unitree_a1.sh)" &&
  roslaunch prometheus_quadruped_sim standalone.launch \
    agent_backend:=physical a1_mesh_dir:="$A1_MESH_DIR" gui:=true rviz:=true
```

与 P450 共场时，先按下文启动 P450，在原来的 `attach.launch` 命令加上 `agent_backend:=physical a1_mesh_dir:="$A1_MESH_DIR"`，其中 `A1_MESH_DIR` 也要在注入终端设置。网格路径必须能被 Gazebo 服务端和桌面客户端共同访问。去掉 `a1_mesh_dir` 即恢复包内外观。

这只复用官方 A1 的显示资源，沿用 M3 的关节、碰撞体、足端接触、雷达和步态控制；导航及控制器不是宇树实现。无界面步态复跑：`A1_MESH_DIR="$A1_MESH_DIR" bash Simulator/quadruped_sim/test/run_m3.sh gait`。

## 四狗独立场景：启动图形界面

先停掉之前的单狗仿真。在 Ubuntu 桌面终端加载下面的环境并启动（首次使用或从 M1 升级后，先执行下方编译命令）：

```bash
source ~/Prometheus/devel_quadruped/setup.bash
roslaunch prometheus_quadruped_sim standalone.launch robot_count:=4 gui:=true rviz:=true
```

Gazebo 会显示四个底盘；RViz 可以查看四条全局路线。等终端出现 `dog1, dog2, dog3, dog4 ready`，在第二个终端给四台发送不同目标：

```bash
source ~/Prometheus/devel_quadruped/setup.bash
rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x 5 --y -3.6 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog2 --x 5 --y -1.65 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog3 --x 5 --y 1.65 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog4 --x 5 --y 3.6 &
wait
```

RViz 的 `2D Nav Goal` 默认只发给 `dog1`；其余用 `--robot` 指定，避免目标发错。

## 与已启动的 P450 共场

1. 退出四狗独立场景。按平时的方法启动 `Scripts/simulation/ego_planner/ego_planner_p450_d435i.sh`，使用默认 `planning_easy.world`，并保持 `dynamic_obstacles_enable:=false`。
2. 待 P450 模型出现在 Gazebo 中，在同一 ROS master 的另一个终端注入四台狗。不要再次运行 `standalone.launch`。

```bash
source ~/Prometheus/devel/setup.bash
source ~/Prometheus/devel_quadruped/setup.bash --extend
roslaunch prometheus_quadruped_sim attach.launch rviz:=true
```

在已加载这两个工作空间的终端，可用 `rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x -4.3 --y -4.4` 发第一个共场目标。`dog2` 至 `dog4` 的演示目标分别为 `(-4.3, -2.2)`、`(-4.3, 0.4)`、`(-4.3, 4.7)`。`Ctrl+C` 退出注入终端会移除本次生成的狗，不停止 P450 或原有 Gazebo。不戴 D435i 的标准 P450 使用 `uav_model:=p450_0`。

共场入口校验现有 P450、柱体集合、柱体位置及出生点，从 `planning_easy.world` 中的柱体碰撞几何和运行时稳定位置生成共享 `/dog_map`。默认读取 `config/planning_easy.yaml`；不把独立演示场景的地图用于 P450。共场不重新设置 `/use_sim_time`。

## 编译与启动

环境：Ubuntu 20.04、ROS Noetic、Gazebo Classic 11。依赖见 `package.xml`，可从仓库根目录用 `rosdep install --from-paths Simulator/quadruped_sim --ignore-src -r -y` 检查和安装。独立运行无需加载 Prometheus 原有 `devel/`。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C "$HOME/Prometheus" --source Simulator/quadruped_sim \
  --build build/quadruped_sim \
  -DCATKIN_DEVEL_PREFIX="$HOME/Prometheus/devel_quadruped" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$HOME/Prometheus/devel_quadruped/setup.bash"
roslaunch prometheus_quadruped_sim standalone.launch
```

默认 headless。启动前确认不会复用正在运行的无人机 ROS master；隔离运行时，在启动与发目标的两个终端设置同一个独立的 `ROS_MASTER_URI`、`GAZEBO_MASTER_URI`。需要观察时可传 `gui:=true rviz:=true`；没有可用 GPU/X11 的环境可执行 `env -u DISPLAY -u WAYLAND_DISPLAY LIBGL_ALWAYS_SOFTWARE=1 roslaunch ...`。

另一终端加载同一 overlay，发送 `dog_map` 下的目标：

```bash
source "$HOME/Prometheus/devel_quadruped/setup.bash"
rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x 5 --y 0 --yaw 0 --timeout 90
```

机器狗出生于 `(-5, 0)`，会绕过原点处的盒体。动态演示使用 `dynamic_obstacle:=true`，重新启动场景后发送同一目标。CLI 返回码：`0` 到达，`1` action 非成功终态，`2` 未就绪或参数错误，`124` 超时或 Ctrl+C 已请求取消（终端显示取消是否确认）。RViz 的 `2D Nav Goal` 发送到 `/dog1/move_base_simple/goal`。

## 实现契约

- `worlds/demo.world` 是静态几何唯一来源：`bringup.py` 将其中轴对齐碰撞盒栅格化后发布锁存 `/dog_map`，无需维护 PGM 或再起 `map_server`。启动确认 Gazebo spawn、`/dog1/scan`、`/dog1/odom`、TF 和出生点一致后才发布地图。
- TF：`dog_map → dog1/odom → dog1/base_footprint → dog1/base_link → dog1/laser_link`。`odom` 是仿真真值，XY 和航向与 Gazebo 世界一致。原无人机启动脚本和规划器未改动。
- `/dog1/cmd_vel_nav` 经 Python 节点限幅及校验后进入 `/dog1/cmd_vel`。激光、里程计或 TF 过期时置零；Gazebo 插件另用单调时钟执行 250 ms 超时，覆盖保护节点进程退出。`/dog1/contacts` 发布 Gazebo 接触信息。
- `proxy` 使用 Navfn + DWA。`physical` 使用 Navfn + `GaitLocalPlanner`，低速跟踪全局路径、检查预测外廓，并在目标容差内保持零速后才报告到达。单狗 RViz 中的 DWA 局部路径显示仅适用于 `proxy`；两种后端都可看 Navfn 全局路径。
- 动态障碍在狗到达指定 X 位置后横穿，不写入静态地图。局部代价地图用激光标记与射线清除；两种局部规划器都不预测障碍的未来轨迹。

## 复跑验收

在仓库根目录、已加载 overlay 的终端运行：

```bash
for scenario in empty static dynamic wall cancel pause scan_loss odom_loss watchdog; do
  bash Simulator/quadruped_sim/test/run_m1.sh "$scenario" || break
done
```

每个用例启动独立的 headless Gazebo，并输出临时日志目录与单行 JSON。测试读取 Gazebo 位姿、速度、碰撞和策略执行结果；动态用例另外断言移动障碍曾被局部地图标记，离开后原占据栅格被清除。`scan_loss` 与 `odom_loss` 通过切断指定保护输入注入故障。

运行 `bash Simulator/quadruped_sim/test/run_m2.sh` 复跑四狗独立导航和取消隔离。在注入 P450 后，用 `/usr/bin/python3 Simulator/quadruped_sim/test/m2_check.py --attach --cancel-one` 复跑共场导航。不可与手动发目标同时运行。
