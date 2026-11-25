# M1 Headless 实测记录

环境：Ubuntu 20.04、ROS Noetic、Gazebo Classic 11.15.1。独立 Catkin 编译生成 `libquadruped_proxy.so`；测试启动独立 ROS master 与 Gazebo，未加载 UAV 工作区。目标 CLI 另行向 `(-3.5, 0)` 发目标，实际返回 `dog1 SUCCEEDED`。

| 用例 | 结果 |
| --- | --- |
| 空地目标 | 成功，5.31 仿真秒，终点误差 0.214 m |
| 静态绕障 | 10/10 通过，37.60–37.81 仿真秒。代表性一轮终点误差 0.188 m，路径 10.417 m |
| 动态横穿 | 初始十轮均到点，9/10 通过全部断言，已通过轮次用时 40.10–42.30 仿真秒。第十轮未在原来的稀疏采样点看到占据标记，虽到点但计为测试未通过 |
| 障碍内目标 | `ABORTED`，未碰撞 |
| 主动取消 | `PREEMPTED`，0.085 s 内速度归零，底盘又走了 0.011 m |
| 暂停物理 | 0.385 s 内速度归零，恢复前取消目标 |
| 激光断流 | 0.487 s 内速度归零，底盘又走了 0.109 m |
| 里程计断流 | 0.497 s 内速度归零，底盘又走了 0.112 m |
| 保护节点退出 | Gazebo 后端超时停车，测试起点至检测到停车约 0.122 m |

上述用例均未检测到非地面接触。速度归零时间用墙钟测量，导航耗时用仿真时钟测量。碰撞检测来自 Gazebo 接触话题；位姿、路径和停车验证来自 Gazebo 真值。`action` 数值 `2 / 3 / 4` 分别为 `PREEMPTED / SUCCEEDED / ABORTED`。断流与后端超时用例的 `PREEMPTED` 来自测试主动清理目标，不代表系统会自行取消目标。

代表性原始输出：

```jsonl
{"action": 3, "case": "empty", "contacts": 0, "error_m": 0.214, "path_m": 1.288, "sim_s": 5.31, "wall_s": 7.327, "yaw_error_rad": 0.067}
{"action": 3, "box_pass_abs_y_m": 1.631, "case": "static", "contacts": 0, "error_m": 0.188, "path_m": 10.417, "sim_s": 37.71, "wall_s": 39.77, "yaw_error_rad": 0.282}
{"action": 3, "box_pass_abs_y_m": 1.629, "case": "dynamic", "contacts": 0, "dynamic_cells_cleared": true, "error_m": 0.196, "path_m": 10.41, "sim_s": 40.21, "wall_s": 42.26, "yaw_error_rad": 0.266}
{"action": 4, "case": "wall", "contacts": 0, "sim_s": 3.21, "wall_s": 5.228}
{"action": 2, "case": "cancel", "contacts": 0, "sim_s": 0.62, "stop_distance_m": 0.011, "wall_s": 2.634, "zero_command_wall_s": 0.085}
{"action": 2, "case": "pause", "contacts": 0, "sim_s": 0.6, "wall_s": 3.003, "zero_command_wall_s": 0.385}
{"action": 2, "case": "scan_loss", "contacts": 0, "sim_s": 1.1, "stop_distance_m": 0.109, "wall_s": 3.103, "zero_command_wall_s": 0.487}
{"action": 2, "case": "odom_loss", "contacts": 0, "sim_s": 1.1, "stop_distance_m": 0.112, "wall_s": 3.117, "zero_command_wall_s": 0.497}
{"action": 2, "case": "watchdog", "contacts": 0, "sim_s": 1.08, "stop_distance_m": 0.122, "wall_s": 3.095}
```

六个静态轮次记录了实走路径与初始 Navfn 路径的长度比，结果为 0.979–0.980；到达容差允许狗在精确目标之前停车。两个动态轮次按碰撞外廓的平面投影计算横穿物最小采样净距，分别为 0.255 m 和 0.258 m（约 20 Hz）；这两轮超过 0.15 m 建议值，不能外推到没有测距的轮次。

将检测区域扩展到激光可命中的障碍表面后，补测一轮通过：40.21 仿真秒到点，终点误差 0.194 m，最小采样净距 0.270 m，无非地面接触，确认占据标记与清除。此补测不替换原始十轮的失败记录。

复跑命令见 [使用说明](README.md)。此报告没有验证 RViz 实际渲染、真实四足步态、四机或无人机共场。
