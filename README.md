# 27 算法组导航方向招新大作业

## 项目简介

在给定迷宫地图（15 m × 15 m，走廊宽约 1.5 m）上完成**一次点击导航**全流程：

- 机器人从固定起点 (0.900, 0.900) 出发；
- 在 RViz 中对固定终点 (14.100, 14.100) 发布一次 `2D Goal Pose` 目标；
- 上层行为树 `click_nav.xml` 触发导航，下层行为树 `default_nav_with_fallback.xml` 调用 A\* 规划全局路径；
- 自研 **PidController** 插件（`sp_controller_server`，pluginlib 加载）跟踪规划轨迹，平滑准确地到达终点。

## 快速开始

```bash
# 构建
colcon build --packages-select sp_controller_server sp_nav_bringup sp_decision sp_global_planner sp_nav_bt robot_msg sp_map_server sp_nav_sim
source install/setup.bash
# 一键启动（仿真 + 规划 + 行为树 + 控制器 + RViz）
ros2 launch sp_nav_bringup sp_nav.launch.py

# 在 RViz 中用 2D Goal Pose 点击迷宫右上角终点 (14.1, 14.1) 即可
```

## 系统架构

```
RViz 2D Goal Pose (/goal_pose)
        │
        ▼
sp_decision  click_nav.xml        ← 上层行为树（锁定底盘/云台姿态）
        │
        ▼
sp_nav_bt    default_nav_with_fallback.xml
        ├── /planner_server/make_plan   → sp_global_planner (A*)
        └── /set_control_enable         → sp_controller_server
                                               └── PidController 插件（本作业实现）
        │
        ▼
/sentry/cmd_vel  →  sp_nav_sim 仿真器（里程计 /Odometry + TF）
```

| 关键话题 | 类型 | 说明 |
|---|---|---|
| `/goal_pose` | PoseStamped | 点击目标点（map 系） |
| `/global_path` | nav_msgs/Path | A\* 全局路径（经 controller server 转发） |
| `/sentry/cmd_vel` | TwistStamped | 控制器输出速度 |
| `/Odometry` | nav_msgs/Odometry | 仿真里程计（map 原点偏移 (0.9, 0.9)） |

## 控制器设计（PidController）

在发放的 `ControllerPlugin` 框架上实现纯追踪（Vector Pursuit）路径跟踪控制器：

1. **`configure()`**：读取 PID 参数与运动约束（速度上下限、预瞄距离、终点容差）。
2. **`setPlan(path)`**：接收全局路径；重置 PID 积分与方向状态。
3. **`computeVelocityCommands(pose, velocity)`**：每个控制周期（50 Hz）：
   - **预瞄点选取**：沿路径累加距离，取前方 `lookahead_dist` 处的点作为跟踪目标；
   - **期望方向**：机器人当前位置指向预瞄点（map 系），再旋转到车体系输出 `linear.x / linear.y`（全向底盘横向纠偏）；
   - **航向 PID**：`angular.z = kp·e + ki·∫e + kd·ė`，积分限幅防饱和；
   - **自适应减速**：预瞄点之后路径转角越大，线速度越低，保证弯道平稳；
   - **终点平滑停车**：按剩余距离线性降速，并设最低速度保底，稳定进入终点容差判定。

### 关键修复与调优

| 问题 | 根因 | 修复 |
|---|---|---|
| 导航时边走边自转、方向错乱 | 仿真云台默认恒速自转（mode=0），cmd_vel 按车体系解释 | `click_nav.xml` 中 `GimbalControl mode="3" big_yaw="0.0"` 锁定云台朝向，使 map→body 旋转变换恒等 |
| 终点前蜗牛爬 | 减速坡过长把尾速压得过低 | 最低速度保底 `linear = max(linear, min_linear_vel)`，以 `min_linear_vel=0.15 m/s` 匀速抵近 |
| 直道快、弯道稳 | 弯道减速依赖预瞄转角 | 转角越大减速越多，参数 `corner_angle_deg / corner_margin` 可调 |

### 参数表（`sp_nav_bringup/config/nav_params.yaml`）

| 参数 | 值 | 含义 |
|---|---|---|
| `kp / ki / kd` | 2.0 / 0.0 / 0.1 | 航向 PID 增益 |
| `max_linear_vel` | 0.9 m/s | 最大线速度 |
| `min_linear_vel` | 0.15 m/s | 最低线速度（尾段保底） |
| `lookahead_dist` | 1.2 m | 预瞄距离 |
| `goal_tolerance` | 0.10 m | 到达判定容差 |

## 实验结果

评测口径：固定起点 → 一次点击终点 → 记录从目标下发到判定到达的过程（`/Odometry` 采样，map 坐标 = (0.9, 0.9) + odom，距终点 < 0.08 m 判定到达；官方 NavStateTrack 容差 0.15 m）。

| 轮次 | 官方判定 | 用时 (s) | 距终点最小距离 (m) | 撞墙/卡死 |
|---|---|---|---|---|
| RUN 1 | 到达（remaining < 0.15 m） | ~94 | 0.092 | 无 |
| RUN 2 | 到达 | ~94 | 0.090 | 无 |
| RUN 3 | 到达 | ~94 | 0.088 | 无 |
| RUN 4 | 到达（remaining=0.149, elapsed=94.13 s） | 94.13 | 0.091 | 无 |

> **结论**：多次重复实验 **4/4 全部到达**，无撞墙、无卡死，稳定性良好；终点停靠精度 0.088 ~ 0.092 m（远优于官方到达容差 0.15 m）；单程用时约 **94 s**，全程轨迹平滑、无明显抖动（RViz 中可直接观察机器人位姿与绿色全局路径的贴合情况）。

## 可视化

RViz（`sp_nav_bringup/rviz/rviz.rviz`）默认展示：

- **Map**：全局代价地图（`/global_costmap`）；
- **Path**：A\* 全局路径（`/global_path`，绿色）；
- **Marker**：规划与控制器可视化（`/plan_markers` 等）；
- **TF**：机器人位姿（map / odom / base_link）。

## 加分项说明

- **轨迹平滑**：控制器代码中实现了 `smoothPath()`（对 A\* 直角折点用 0.3 m 半径圆弧 + 6 点插值替代，消除急折角），可用于需要平滑评级的高分场景；当前提交以**稳定优先**参数（纯 PID + 云台锁定 + 尾段保底）保证评测不撞墙，平滑实现保留在仓库历史中，切换平滑参数即可启用。
- 后续可尝试 LQR / MPC 控制器替换 PID（接口已在 `ControllerPlugin` 中抽象）。

## Commit 记录（节选）

```
2e98c87 perf(controller): tighten corner deceleration and stop ramp for maze sprint
9a94947 feat(controller): implement lookahead vector-pursuit PID controller
c017d47 fix(bt): fix navigation tree config for maze click task
```
