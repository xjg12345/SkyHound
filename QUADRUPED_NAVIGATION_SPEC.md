# Prometheus 四足机器人自主避障导航实现规格

适用环境：Ubuntu 20.04、ROS Noetic、Gazebo Classic 11。**直接运行请跳到[第 5 节：仿真运行手册](#5-仿真运行手册)**。M1/M2 已通过代理后端验收；M3 已通过平地步态、跌倒保护及独立场景的单狗静态、动态、四狗到点停车测试。动态场景未证明有正的避障余量，物理四足与无人机联合飞行导航未验证。第 6 节保留验收标准，实测结果见包内 `M1_RESULTS.md`、`M2_RESULTS.md` 和 `M3_RESULTS.md`。

## 1. 目标和交付标准

将“无人狗（四组机器人）”按 **4 台可独立接收目标、独立规划的四足机器人** 理解，实例名为 `dog1` 至 `dog4`，也支持只启动 1 台。用户通过 RViz 或命令行发送地面目标位姿，机器人自行绕开障碍，到点后停住。单独运行不依赖 PX4、MAVROS、EGO Planner 或 GPU；联动模式复用已有无人机 Gazebo 场景。

按三个里程碑交付：

| 里程碑 | 要交付的能力 | 通过条件 |
| --- | --- | --- |
| M1：单机最小闭环 | 带碰撞体及二维激光的代理底盘，目标 → 规划 → 避障 → 停车 | 一个命令启动，跑通单机静态和动态障碍基本用例 |
| M2：四机与无人机共场 | 四套独立导航，支持狗单独运行或注入已有 Gazebo | 四个目标独立完成，命名和 TF 不串，原无人机启动链无回归 |
| M3：真正的物理四足 | 接入有腿部关节、足端接触及步态控制的模型 | 通过步态、导航和跌倒保护验证；最终功能不能仅以代理模型验收 |

第一版只覆盖平坦地面；不要求楼梯、视觉 SLAM、真机部署或无人机给狗下发任务。不要把联动、四足步态和导航算法同时作为第一次闭环的前置条件。

## 2. 仓库事实与接入点

| 现有路径 | 已核对事实 | 实现时的处理 |
| --- | --- | --- |
| [`Scripts/simulation/NO_RC/arm_and_command.sh`](Scripts/simulation/NO_RC/arm_and_command.sh) | 给 `/uav1/prometheus/setup` 发解锁、`COMMAND_CONTROL`，最后还会切回 `MANUAL_CONTROL` | 保持原样；需要 EGO 自动飞行时，单独确认当前控制模式 |
| [`Scripts/simulation/NO_RC/arm_and_command_3uav.sh`](Scripts/simulation/NO_RC/arm_and_command_3uav.sh) | 发送三台无人机的解锁和模式指令；不启动世界 | 不是 Gazebo 或四机狗的启动入口 |
| [`Scripts/simulation/ego_planner/ego_planner_p450_d435i.sh`](Scripts/simulation/ego_planner/ego_planner_p450_d435i.sh) | 分别启动 `roscore`、单机 Gazebo、UAV 控制、深度转 OctoMap 和 EGO Planner | 先照常启动无人机，再启动只注入狗的入口；不复制这些节点 |
| [`Simulator/gazebo_simulator/launch_uav_with_sensor/sitl_p450_d435i.launch`](Simulator/gazebo_simulator/launch_uav_with_sensor/sitl_p450_d435i.launch) | 有 `world` 参数，默认 `planning_easy.world`；Gazebo 启动时设 `/use_sim_time=false`；`dynamic_obstacles_enable` 默认关闭 | 共场沿用主仿真的时间设置，确保地图与实际世界一致 |
| [`planning_easy.world`](Simulator/gazebo_simulator/gazebo_worlds/planning_worlds/planning_easy.world) 与 [`run_dynamic_obstacles.py`](Simulator/gazebo_simulator/scripts/run_dynamic_obstacles.py) | 多个柱体有碰撞体和 `planar_move` 插件；运动脚本读取固定 world 的 `commandTopic` 并周期发布速度 | 首轮共场保持开关关闭；动态实验另用可重放的障碍脚本，不直接复用整组柱体的运动 |

本机已确认存在 Gazebo 11.15.1、`move_base`、`dwa_local_planner`、`map_server` 以及 Gazebo 平面运动、激光、碰撞插件的库文件；已安装库不等于模型配置已验证。当前四足模型由 `Simulator/quadruped_sim` 生成，既可用 `proxy` 跑导航，也可用 `physical` 观察关节驱动的步态。`Modules/global_planner_ugv/` 不是运行依赖。

## 3. 导航架构与 ROS 接口

```text
RViz / CLI → /dogN/move_base (MoveBaseAction)
共享 /dog_map + 本机 /dogN/scan + 本机 TF
    → Navfn 全局规划器 + 局部规划器（proxy: DWA；physical: GaitLocalPlanner）
    → /dogN/cmd_vel_nav → 安全限速与超时停车 → /dogN/cmd_vel
    → 模型运动后端：代理底盘 / 四足步态
```

全局规划器显式使用 `navfn/NavfnROS`。`proxy` 保留 `dwa_local_planner/DWAPlannerROS`；`physical` 使用包内 `GaitLocalPlanner`，低速跟踪 Navfn 路径，按局部代价地图检查预测外廓，到点后保持零速再报告成功。它不预测移动障碍轨迹，也不是多机协调算法。声明 `navfn` 等直接依赖，不假定已装整个 `navigation` 元包。四台复用配置模板，每台拥有独立命名空间、参数树、costmap、action 和模型实体。只共享 `/dog_map`、ROS 主节点及 Gazebo 世界；无需每台运行一份 `map_server`。

| 接口 | 类型或约定 |
| --- | --- |
| `/dogN/move_base` | `move_base_msgs/MoveBaseAction`；目标包含位置与朝向，`frame_id=dog_map`；CLI 用 action client 查询终态、取消或超时 |
| `/dogN/scan` | `sensor_msgs/LaserScan`；传感器帧 `dogN/laser_link`；检查真实射线、有正确时间戳，至少 10 Hz 起步 |
| `/dogN/odom` | `nav_msgs/Odometry`；`header.frame_id=dogN/odom`、`child_frame_id=dogN/base_footprint`；确保 DWA 订阅这一路 |
| `/dogN/cmd_vel_nav` → `/dogN/cmd_vel` | `geometry_msgs/Twist`；最终速度仅一个发布者；后端仅消费最终话题，不接裸 `/cmd_vel` |
| `/dog_map` | `nav_msgs/OccupancyGrid`，地图 frame 为 `dog_map`；显式核对 map_server 输出及每台静态层订阅，不使用无人机的 `/map` |

每台 TF 为 `dog_map → dogN/odom → dogN/base_footprint → dogN/base_link → dogN/laser_link`，帧名不加前导 `/`。若后端返回世界坐标位姿，则 `dog_map → dogN/odom` 可用恒等变换，前提是地图坐标与世界坐标数字一致；若后端以初始机器人位姿置零，则该变换应包含出生点平移与航向，不能再让里程计重复包含出生偏移。移动前在非零出生点校验位置和朝向。`odom → base_footprint` 只允许一个发布者；不能同时启用运动插件、真值桥和 EKF 发布同一条 TF。RViz Fixed Frame 用 `dog_map`；单机版把 `2D Nav Goal` 工具的发布话题设为 `/dog1/move_base_simple/goal`，四机通过带机器人编号的 CLI 指定目标，避免默认 `/move_base_simple/goal` 被误用。

### 模型后端

- **M1/M2 基线**：新增一个有实心碰撞体、惯量、二维 ray 激光及碰撞 sensor 的动态 SDF 模型。优先使用本机 `libgazebo_ros_planar_move.so` 和 `libgazebo_ros_laser.so`；通过 `robotNamespace`、命令和里程计 topic、frame 参数实现隔离。先用单模型实测插件是否发布 odom/TF 及其坐标语义，再确定是否需要自编 TF 广播器。若平面运动插件的 Z、接触或停车行为不满足测试，可换受支持的差速底盘做导航基线。不得用周期调用 `set_model_state` 传送机器人来伪造导航或避障。
- **M3 物理四足**：优先验证支持 **ROS 1 Noetic + Gazebo Classic 11** 的 CHAMP 仿真方案；确认目标提交的模型、网格资源、许可、关节控制器、启动入口及 `Twist` 输入，锁定 commit 并记入依赖清单。可换已在本机验证的 Go1/A1 等方案，但不要默认上游 Docker、SDK 或 ROS 2 示例可移植。先在其独立场景验证上电站立、低速直走、转弯、停车，再接导航接口；根据实际 controller 校正关节映射、出生高度、体轴方向和雷达安装位姿。导航基线可先用 Gazebo 真值里程计，日报/测试报告明确标注仿真真值；需要算法定位时再用 EKF / AMCL 替换，不能把真值基线当作定位成果。

## 4. 地图、避障与参数基线

**地图可复现。** 先交付一个约 `16 × 12 m` 的自包含平地 world，地面边界和盒体布局由同一份 YAML 生成或校验地图与 world，避免建立两份独立坐标。地图初始分辨率 `0.05 m`，例如 `origin: [-8.0, -6.0, 0.0]`，生成 `320 × 240` 的占据图；校验 PGM 行方向反转、边界和 grid 坐标。首个静态用例：从 `(-5, 0)` 去 `(5, 0)`，在 `(0, 0)` 放置 `1.2 × 2.0 m` 的障碍，绕行通道宽度按实测足端外廓及膨胀余量预留。可控横穿物放在世界中但不烘焙进静态地图。共场地图从实际 world 的地面碰撞体生成或人工复核：需处理模型与碰撞体姿态叠加、旋转、低矮或悬空物的取舍；无法可靠解析的 `include` 或网格碰撞应明示并校验，不能静默忽略。注入模式要求显式传 `map_yaml`；检查场景名/版本或地图标志物是否对齐，不能用演示图直接给 `planning_easy.world` 导航。

**静态与动态分层。** 全局 costmap 用静态层和膨胀层；局部 rolling costmap 用激光障碍层和膨胀层，明确配置 `marking`、`clearing`、`obstacle_range`、`raytrace_range`，后者应略大并在雷达量程内。户外空白方向的 `Inf` 是否作为清除射线取决于驱动约定，必须通过移动障碍移走后的清除用例确认。如果长期堵住静态最短路线导致只会等待，再启用全局实时障碍层或增加显式重规划；动态障碍清除未验证前不默认启用。四足腿部可能遮挡雷达，需正确选装高度，并验证前后左右及近距离盲区。单层雷达看不见的障碍不在基线覆盖范围内。

| 参数 | 起始值与调整依据 |
| --- | --- |
| 最大线速度、角速度 | `0.30 m/s`、`0.70 rad/s`；先禁用侧移 `vy=0`，限制倒车；物理步态按实测稳定速度再调整 |
| 线加速度 | `0.4 m/s²` 起步；用实测减速、控制延迟和传感器延迟核对停车距离 |
| 局部 costmap、更新频率 | `5 × 5 m`，分辨率 `0.05 m`，更新 `10 Hz`；控制 `10 Hz`、全局重规划 `1 Hz` 起步 |
| 预测时域 | DWA `sim_time=1.7 s` 起步；根据窄通道和振荡实测调整，不默认大幅加采样数 |
| 目标容差 | 平面 `0.25 m`、航向 `0.35 rad`；到点后稳定停车 `2 s` |
| footprint、膨胀 | 使用真实躯干与步态中足端扫掠范围的保守多边形，不照搬代理尺寸；`footprint_padding=0.03 m`、`inflation_radius=0.45 m` 起步，结合通道宽度和实际最小间距调整 |

**失效时停车。** 安全节点对速度限幅、丢失指令或 scan/odom/TF 过期（`0.5 s` 起步）发零速度，忽略异常值；另做后端命令超时或验证插件确实自带失联停车，覆盖安全节点自身崩溃。启动时未就绪只发零速，不能透传上一次的非零命令；仿真暂停或 `/clock` 回跳后清除缓存，恢复前要等新数据。时间戳新鲜度以 ROS 时钟判断，进程断流用墙钟或单调钟监视，避免仿真暂停后恢复带出旧速度。提供取消、不可达和执行超时的终态；默认不盲目触发清图加原地旋转的恢复动作。物理四足检测到失稳或跌倒后进入后端定义的安全停止状态，不自动继续追逐原目标。

**四机交互。** 其他狗通过激光进入局部 costmap。先验收非冲突路线和宽通道交会，不把 DWA 当作多机协调算法。若需要共享单车道或容易相互锁死的交叉口，做可配置路口互斥区：进入前请求通行，按先来先服务等待，确认完整外廓退出后释放；等待的位置不得挡住对侧出口。机器人失联但仍占用区域时不能仅凭超时放行，需确认区域已清空。没有可抢占路线或等待超过限时就报告失败并停车。用显式互斥处理死锁，再考虑更换局部规划器；不要靠提高最大速度解决。

## 5. 仿真运行手册

**第一次建议跑「一架 P450 + 四台 `proxy`」**，可在同一个 Gazebo 里看到无人机与四台橙灰色四足外观的代理底盘。想看真实关节迈步时，先单独启动一台 `physical`。下面三种场景任选其一；切换前先停旧场景。同一场仿真的各终端须使用相同的 `ROS_MASTER_URI`；不要一边运行四足 `standalone.launch`，一边在同一 ROS master 启动无人机。

### 5.1 首次编译四足包

四足包使用独立的 `devel_quadruped/`，不会覆盖无人机的 `devel/`。无人机部分假设现有 Prometheus、PX4 SITL 和 MAVROS 已安装、编译。

```bash
cd /home/xjg/Prometheus
source /opt/ros/noetic/setup.bash
# 缺依赖时执行：
# rosdep install --from-paths Simulator/quadruped_sim --ignore-src -r -y
catkin_make --source Simulator/quadruped_sim --build build/quadruped_sim \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_quadruped" -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$PWD/devel_quadruped/setup.bash"
```

### 5.2 单独运行一架 P450 D435i：Gazebo + RViz + EGO Planner

在 Ubuntu 桌面终端执行。这个脚本会打开多个 GNOME Terminal 标签页，启动 ROS master、Gazebo、UAV 控制、深度转 OctoMap 和 EGO Planner。默认打开 Gazebo 和无人机 RViz，不需要再起第二个 `roscore`。

```bash
source /opt/ros/noetic/setup.bash
source /home/xjg/Prometheus/devel/setup.bash

# 如果现有终端已配置 PX4，可以跳过这段环境设置。
PX4="$HOME/prometheus_px4"
source "$PX4/Tools/setup_gazebo.bash" "$PX4" "$PX4/build/amovlab_sitl_default"
export ROS_PACKAGE_PATH="$PX4:$PX4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export PATH="$PX4/build/amovlab_sitl_default/bin:$PATH"
MODELS=/home/xjg/Prometheus/Simulator/gazebo_simulator/gazebo_models
export GAZEBO_MODEL_PATH="$MODELS/uav_models:$MODELS/sensor_models:$MODELS/scene_models:$GAZEBO_MODEL_PATH"

cd /home/xjg/Prometheus/Scripts/simulation/ego_planner
bash ego_planner_p450_d435i.sh
```

在另一个终端加载无人机 `devel/setup.bash`，用 `rostopic echo -n 1 /uav1/mavros/state` 确认 `connected: True`。**只看模型或跑四足共场时，不必解锁无人机。**

需要按现有脚本做单机解锁和模式切换时，仅在仿真中执行：

```bash
source /home/xjg/Prometheus/devel/setup.bash
cd /home/xjg/Prometheus/Scripts/simulation/NO_RC
bash arm_and_command.sh
```

该脚本最后会切回 `MANUAL_CONTROL`，不适合需要持续使用 EGO 指令的演示。做 EGO 仿真时，可在 MAVROS 已连接后，用以下两条代替脚本。先执行解锁，确认 MAVROS 的 `armed: True`，再执行第二条；确认控制节点进入 `COMMAND_CONTROL` 后，再在**无人机 RViz** 用 `2D Nav Goal` 指定目标。

```bash
rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup '{cmd: 0, arming: true}'
rostopic pub -1 /uav1/prometheus/setup prometheus_msgs/UAVSetup '{cmd: 3, control_state: COMMAND_CONTROL}'
```

**三架无人机单独运行（可选）**：停止单机栈，在已加载上述环境的桌面终端运行 `bash /home/xjg/Prometheus/Scripts/simulation/ego_planner/ego_planner_p450_d435i_depth_swarm.sh`。等 `/uav1`、`/uav2`、`/uav3` 对应的 MAVROS `state` 都显示已连接，再在另一个已加载无人机环境的终端运行 `bash /home/xjg/Prometheus/Scripts/simulation/NO_RC/arm_and_command_3uav.sh`。它只对已启动的三机解锁和切换模式；三机飞行与四足共场不属于这里的已验证用例。

### 5.3 单独运行四足：Gazebo + 四足 RViz

**终端 A：启动场景。** 在桌面终端选择下面一个命令。独立启动会自动启动 ROS master（如果当前未运行）及 Gazebo。默认不弹窗，所以要显式加 `gui:=true rviz:=true`。

```bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash

# 一台代理：中间盒体静态绕障。
roslaunch prometheus_quadruped_sim standalone.launch \
  robot_count:=1 agent_backend:=proxy gui:=true rviz:=true

# 四台代理：四狗导航演示。要切换到本例，先 Ctrl+C 停掉上一例。
roslaunch prometheus_quadruped_sim standalone.launch \
  robot_count:=4 agent_backend:=proxy gui:=true rviz:=true

# 一台物理四足：观察真实关节和足端，并试跑独立场景自主导航。
roslaunch prometheus_quadruped_sim standalone.launch \
  robot_count:=1 agent_backend:=physical gui:=true rviz:=true
```

单狗场景还可加 `dynamic_obstacle:=true` 观看横穿避障；该开关只适用于一台狗。要在无桌面环境跑独立场景，把 `gui`、`rviz` 都设为 `false`。

**终端 B：发目标。** 等终端 A 出现 `dog1 ready`（四台时为 `dog1, dog2, dog3, dog4 ready`）。以下坐标以 `dog_map` 为准，单位是米，`--yaw` 为弧度。单狗与四狗命令二选一。

```bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash

# 单狗场景，从 (-5, 0) 到 (5, 0)。
rosrun prometheus_quadruped_sim send_goal.py \
  --robot dog1 --x 5 --y 0 --yaw 0 --timeout 90

# 四狗场景，并发发送四个独立目标。
rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x 5 --y -3.6 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog2 --x 5 --y -1.65 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog3 --x 5 --y 1.65 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog4 --x 5 --y 3.6 &
wait
```

### 5.4 一架 P450 与四台四足共用一个 Gazebo

**先按第 5.2 节启动 P450。** 停止之前的四足独立场景，保持 P450 使用默认的 `planning_easy.world`，且 `dynamic_obstacles_enable:=false`。等 Gazebo 中看到 P450、MAVROS 已连接，再执行下面的注入命令。不要启动第二个 Gazebo，也不要修改 `/use_sim_time`。

**注入终端：**

```bash
source /home/xjg/Prometheus/devel/setup.bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash --extend
roslaunch prometheus_quadruped_sim attach.launch \
  robot_count:=4 agent_backend:=proxy rviz:=true \
  map_yaml:=/home/xjg/Prometheus/Simulator/quadruped_sim/config/planning_easy.yaml
```

`attach.launch` 没有 `gui` 参数：它复用无人机的 Gazebo 窗口，只额外开启四足 RViz。如果已启动的是无相机标准 P450，在注入命令末尾加 `uav_model:=p450_0`；默认是 `p450_D435i_0`。使用 `agent_backend:=physical` 时，无相机 P450 未解锁条件下的四目标共场导航已通过一轮；D435i 物理共场及无人机飞行中的联合导航未验证。

**目标终端：** 等注入终端显示四狗就绪，再发共场专用坐标，不能直接套用第 5.3 节的目标。

```bash
source /home/xjg/Prometheus/devel/setup.bash
source /home/xjg/Prometheus/devel_quadruped/setup.bash --extend
rosrun prometheus_quadruped_sim send_goal.py --robot dog1 --x -4.3 --y -4.4 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog2 --x -4.3 --y -2.2 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog3 --x -4.3 --y 0.4 &
rosrun prometheus_quadruped_sim send_goal.py --robot dog4 --x -4.3 --y 4.7 &
wait
```

退出时先在**注入终端**按 `Ctrl+C`，它只清理本次生成的狗，P450 和 Gazebo 仍运行。然后按需停止无人机启动标签页，最后停止 `roscore`。独立场景则直接在启动终端按 `Ctrl+C`，该场景的 Gazebo 会一起退出。

### 5.5 界面、就绪检查和复跑

| 窗口 | 用途 |
| --- | --- |
| **Gazebo** | 看三维仿真。`proxy` 是橙灰色四足外观，腿不会迈步；`physical` 才有可动关节和足端接触。在模型树里找 `dog1` 到 `dog4`；共场时同一窗口还可看到 P450。 |
| **无人机 RViz** | `Fixed Frame` 为 `world`，看深度建图、无人机位置和规划轨迹。`2D Nav Goal` 发布到 `/uav1/prometheus/motion_planning/goal`。 |
| **四足 RViz** | `Fixed Frame` 为 `dog_map`，看占据图、激光和导航路径。`2D Nav Goal` 默认只给 `dog1`；点击并拖动可指定位置和朝向，其他狗用命令行的 `--robot` 指定。 |

没看到窗口先确认是在 Ubuntu **图形桌面**，并检查独立场景是否传了 `gui:=true rviz:=true`。如果 Gazebo 和 RViz 已打开但没有四足地图，先看启动终端是否显示就绪或报错，再在加载了相应环境的终端检查：

```bash
echo "${ROS_MASTER_URI:-http://localhost:11311}"
rosservice call /gazebo/get_world_properties
rostopic echo -n 1 /dog_map/header
rostopic echo -n 1 /dog1/odom/header
# 仅 physical：
rostopic echo -n 1 /dog1/physical_ready
```

命令行发目标后应看到 action 终态，返回码 `0` 才表示成功。若 `physical` 跌倒锁存，须退出后重启或重新注入。D435i 无界面运行曾在 RealSense 渲染插件处失败，应优先用桌面运行，不要把能打开 RViz 等同于已通过完整无人机链路验收。

无界面复跑（在仓库根目录执行，使用隔离的 ROS/Gazebo 端口）：

```bash
cd /home/xjg/Prometheus
source devel_quadruped/setup.bash
bash Simulator/quadruped_sim/test/run_m1.sh static
bash Simulator/quadruped_sim/test/run_m2.sh
bash Simulator/quadruped_sim/test/run_m3.sh gait
```

更多用例和限制见 [M1 报告](Simulator/quadruped_sim/M1_RESULTS.md)、[M2 报告](Simulator/quadruped_sim/M2_RESULTS.md)、[M3 报告](Simulator/quadruped_sim/M3_RESULTS.md)。不要与手动发目标同时运行验收脚本。

## 6. 验收矩阵与效果指标

下表是建议门槛，不是已有成绩。使用固定起终点、world、种子及障碍触发条件，动态物体从相同状态开始，以固定仿真节拍或同一位置触发横穿；不得依赖墙钟启动时刻随机相位。

| 用例 | 通过条件 |
| --- | --- |
| M1 单机冒烟 | 一个命令启动；Gazebo 只有一个 `gzserver`；出生点 TF 与地图对齐；向空地发目标后到达并停车 |
| M1 单机静态绕障 | 直线被盒体挡住仍能绕行；10 次至少 9 次在 90 s 内 `SUCCEEDED`，所有运行无狗体碰撞 |
| M1 动态横穿 | 10 次至少 9 次在 120 s 内到达；横穿时减速等待或绕行，障碍离开后恢复，无碰撞且局部地图不残留障碍 |
| M1 失败与取消 | 发送墙内目标、途中取消、停止 scan 或里程计输入，分别检查拒绝或 `ABORTED`、`PREEMPTED`、停车；断流后 `0.7 s` 内最终速度命令归零，另测实体停止距离 |
| M2 四机 | 每轮四个不同目标；至少 5 轮，至少 4 轮四机全部成功，未达者必须报告原因且停住。所有测试无狗间或狗体对障碍碰撞，单独取消某台不能让其他台收到取消或速度命令 |
| M2 共场回归 | 对原单机无人机场景做不带狗/带狗对比，检查 PX4、UAV 控制、深度和 EGO 话题仍工作；启动、结束狗进程时不重启 Gazebo、不改变无人机时间设置 |
| M3 物理四足 | 站立、直行、转向、停车均稳定，并以物理后端重新执行单机静态、动态以及四机用例；至少做一次共场回归。通过 Gazebo 足端接触与关节状态证明实际走动，而非平移模型 |

成功同时满足 action `SUCCEEDED`、终点实际误差不超过上述容差，并在到点后 `2 s` 内保持稳定停车。测量使用 Gazebo 真值及接触数据做独立评估，不只读规划器内部状态；碰撞排除正常足端/底盘与地面接触，近距按机器人外廓与障碍碰撞几何计算而不是雷达中心到障碍距离。记录每台成功率、耗时、实走路径长度、静态试验 `实走长度 / 起始可行全局路径长度`、最小净距、碰撞数、停走次数，以及 Gazebo 实时率。建议静态场景的路径长度比中位数 `≤1.3`，开阔横穿用例净距 `≥0.15 m`；若门槛不达，先通过 rosbag 回看 `scan/TF/odom/costmap/速度/action 状态` 再调参，一次只改一组参数并保留前后对照。对外不要把代理基线与物理四足的结果混在一个成功率中。

## 7. 交给 AI Coding 的执行指令

> 阅读本规格及第 2 节当前仓库文件。先仅实现 M1，提交可编译的新 Catkin 包、独立 headless 启动、固定场景、目标 CLI、停车保护与实际运行证据；不要改现有无人机脚本或替换其规划器。先查 ROS 包、Gazebo 插件和 frame 语义，再写配置。M1 通过后实现 M2，先四狗独立场景，再注入已启动的 P450 Gazebo；最后验证 M3 的物理模型和步态，保留 `proxy` 作为导航回归后端。每个里程碑输出依赖清单、可重跑命令、测试结果与未通过项目；没有运行或只验证了接口的功能，必须分别标为“未验证”或“仅接口通过”。
