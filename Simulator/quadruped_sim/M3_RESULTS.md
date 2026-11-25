# M3 物理四足验收

使用 ROS Noetic、Gazebo Classic 11.15.1、ODE、Catkin、C++17 和 Python 3.8。依赖见 [`package.xml`](package.xml)：`gazebo_dev`、`gazebo_ros`、`gazebo_plugins`、`tf2_ros`、Navfn、move_base；物理局部规划插件另使用 `nav_core`、`pluginlib`、`costmap_2d`、`tf2_geometry_msgs` 与 `nav_msgs`，DWA 保留给代理。默认物理后端使用包内生成的 SDF 和关节力矩插件，无外部网格、CHAMP 或厂商 SDK 依赖；未验收 CHAMP。里程计是 Gazebo 真值，不是定位算法。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C ~/Prometheus --source Simulator/quadruped_sim \
  --build build/quadruped_sim \
  -DCATKIN_DEVEL_PREFIX="$HOME/Prometheus/devel_quadruped" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source ~/Prometheus/devel_quadruped/setup.bash
# 在仓库根目录执行，每个用例启动独立实例。
bash Simulator/quadruped_sim/test/run_m3.sh gait 11644
bash Simulator/quadruped_sim/test/run_m3.sh static 11647
bash Simulator/quadruped_sim/test/run_m3.sh dynamic 11648
bash Simulator/quadruped_sim/test/run_m3.sh team 11649
bash Simulator/quadruped_sim/test/run_m1.sh static 11640
```

共场复跑：先启动 P450 和 `planning_easy.world`，待 MAVROS 连接后，在同一 ROS master 的另一个终端执行：

```bash
source ~/Prometheus/devel/setup.bash
source ~/Prometheus/devel_quadruped/setup.bash --extend
roslaunch prometheus_quadruped_sim attach.launch agent_backend:=physical uav_model:=p450_0
```

然后从仓库根目录运行 `/usr/bin/python3 Simulator/quadruped_sim/test/m2_check.py --attach --physical`。标准 D435i 的模型名不同，使用默认 `uav_model`。无图形测试可用 `test/p450_headless.launch vehicle:=p450`；需要可在 ROS 中找到的 PX4 SITL 构建、Gazebo 插件和模型路径，以及现有 UAV 控制、MAVROS、EGO Planner 环境。

## 原始基线

| 用例 | 实测 |
| --- | --- |
| 物理模型、站立、直行 | 通过：站立 3 仿真秒的漂移 0.00011 m；直行 6 仿真秒前进 1.703 m、横移 0.019 m；全过程最大横滚或俯仰 0.053 rad |
| 转向、断导航指令停车 | 通过：正向累计 3.311 rad；停止发布导航指令后，4 仿真秒观察窗口的后半段平面漂移 0.0094 m |
| 关节与足端证据 | 通过：12 关节状态；8 个大腿或小腿关节的角度变化超过 0.42 rad，四足均在站立时触地、在直行中出现触地和离地；脚和躯干无异常接触 |
| 跌倒保护 | 通过：测试脚本外力扰动等效地设置翻倒姿态；即使持续送非零导航输入，仍记录 `physical_fault=true`、`physical_ready=false` 和最终零速命令 |
| `proxy` 静态绕障 | 通过：action 成功，实走 10.408 m，终点误差 0.194 m，37.70 仿真秒，无非地面接触 |
| 物理单狗静态绕障 | 未通过：原配置与物理专用轨迹预测配置均在 90 秒墙钟内未返回成功。后者的采样位姿曾到盒体另一侧 `(1.889, 1.347)`，不能作为到达证据 |
| 物理单狗动态横穿 | 未通过：初次和调参后均在 120 秒墙钟内未返回成功，不能确认整段避障或动态栅格清除验收 |
| 四台物理狗独立导航 | 仅接口通过：四台生成且地图就绪；两轮均在 100 秒内未收齐四台动作终态，导航未通过 |
| 无相机 P450 共场 | 仅接口通过：四狗注入，四目标试验超时，导航未通过。退出后仅清理了狗，`p450_0` 仍在，MAVROS 仍连接，`/use_sim_time` 仍为 `false`；无人机未解锁 |
| P450 D435i 共场 | 启动未通过：本轮 headless 的 RealSense Gazebo 插件在相机指针断言处使 `gzserver` 退出。物理狗与 D435i、深度图、OctoMap 和 EGO 数据链的完整回归未验证 |

初版落地偏移约 0.95 m；最终改用 SDF 中的前后镜像屈膝和低增益关节控制。运行时控制器只施加关节力矩；跌倒试验直接设置翻倒姿态，该调用只存在于验收脚本，不参与运动。

代表性日志目录：步态 `/tmp/prometheus-m3-test.KvmiQa`，调参后静态 `/tmp/prometheus-m3-test.1sTRWB`、动态 `/tmp/prometheus-m3-test.nABo5U`、四机 `/tmp/prometheus-m3-test.4l7MQp`，P450 `/tmp/prometheus-m3-p450-plain.srhnqR`。

### 宇树 A1 外观接入回归

- 资源：宇树 `unitree_ros` 提交 `157ff950351e5f535e03c5d85a16b7d594b3dbcc`，BSD-3-Clause。仅下载 A1 网格、纹理和许可证；不运行厂商控制器，不改变 `proxy` 默认行为。
- 复跑：加载四足 overlay 后，执行 `A1_MESH_DIR="$(bash Simulator/quadruped_sim/scripts/fetch_unitree_a1.sh)"`，再执行 `A1_MESH_DIR="$A1_MESH_DIR" bash Simulator/quadruped_sim/test/run_m3.sh gait`。
- 通过：移除 `<visual>` 后新旧 SDF 完全一致；无界面步态直行 1.688 m、转向 3.427 rad，四足均记录到触地和离地，无异常接触，跌倒保护通过。
- 原物理外观静态基线复跑：未通过，90 墙钟秒结束时为 `(2.065, -0.725)`，无接触；日志多次出现 DWA 无可行轨迹。A1 外观下的自主导航和 P450 共场未验证；桌面渲染未验证。

依赖/能力边界：未验证坡地、楼梯、真机、GUI 实际渲染、激光全方位盲区或多机窄道互斥。物理后端 250 ms 独立命令看门狗已实现，但单独杀死保护节点的物理后端试验未验证。碰撞检查覆盖躯干与四足传感器，不覆盖其余腿段。

## 物理导航改版

未找到可直接接入本仓库 ROS 1 Noetic `move_base` 的宇树官方导航组件。保留 Navfn、激光局部代价地图与速度保护，以 `GaitLocalPlanner` 取代物理后端的 DWA；代理配置不变。插件跟踪全局路径，检查未来 0.7 秒的采样外廓，按原速、半速或原地转向选可行命令；全部不可行便交由 `move_base` 停车及处理失败，不承诺主动寻找替代路线或预测动态轨迹。为避免反复重置到点判定，关闭周期性全局重规划，保留 `move_base` 的失败重试机制。

停稳判据：在目标位置与朝向容差内持续发零速，且最近里程计年龄不超过 0.25 秒、平面速度低于 0.025 m/s、偏航角速度低于 0.025 rad/s，连续满足 0.8 ROS 秒才报告成功。零速站姿增加关节速度阻尼，未改 SDF、惯量、碰撞体或运动步态参数。验收仍从独立 Gazebo 真值检查停车，未放宽原有门槛。

早期版本只等固定时间，单狗静态未过停车；动态和四狗独立分别通过过一轮。无相机 P450 共场已生成四狗并收齐终态，但 `dog1` 的停稳窗口漂移指标为 0.150，超出 0.025（该指标取平移米数和偏航弧度的最大值），该轮未通过，日志 `/tmp/prometheus-m3-p450-nav.Jsmug9`。增加站姿阻尼后的独立步态通过：直行 1.658 m、转向 3.433 rad、停车后段漂移 0.0020 m；四足交替触地、无异常接触、跌倒保护通过，日志 `/tmp/prometheus-m3-test.XjCGYp`。

| 改版复测 | 结果 | 日志目录 |
| --- | --- | --- |
| 单狗静态（11941） | 通过：34.11 仿真秒，终点误差 0.029 m，实走 11.140 m，停车速度指标峰值 0.0012，无异常接触 | `/tmp/prometheus-m3-test.w0tXiy` |
| 单狗动态（11942） | 通过既有用例：37.11 仿真秒，终点误差 0.052 m，停车速度指标峰值 0.0021，动态栅格清除、无异常接触；矩形估算最小间隙 0 m，未证明正安全余量 | `/tmp/prometheus-m3-test.0Phqen` |
| 四狗独立（11943） | 四台均到点停稳，终点误差 0.040–0.096 m，停稳漂移指标最大 0.0009，无异常接触 | `/tmp/prometheus-m3-test.iYPsfU` |
| 无相机 P450 共场（11945） | 四台均到点停稳，终点误差 0.038–0.175 m，停稳漂移指标最大 0.0004，无异常接触；P450 保持未解锁，停止注入后仅移除狗 | `/tmp/prometheus-m3-p450-nav.Rr5Gzq` |
| `proxy` 静态回归 | 通过：终点误差 0.192 m，无非地面接触 | `/tmp/prometheus-m1-test.dxCwJx` |

单狗停车速度指标是平移速度（m/s）与偏航角速度（rad/s）数值的最大值。上述为单轮通过，不代表任意布局下的成功率。改版未验证 D435i 物理共场、联合飞行、四物理狗取消隔离、桌面渲染和 A1 外观下的自主导航。复跑使用本文开头的命令，端口可替换为表中端口。
