# otg_planner 架构说明

<p align="right">
  <a href="./ARCHITECTURE.md">简体中文</a> | <a href="./ARCHITECTURE.en.md">English</a>
</p>

## 1. 范围

`otg_planner` 现在已经是一个面向 Franka Panda 的 `ROS 2 Humble + Gazebo Sim` 关节空间规划与执行系统。

它不再只是一个固定 A/B 往返运动 demo。当前系统已经提供：

- 外部关节空间目标输入
- 带碰撞约束的全局路径规划
- 基于 `Ruckig` 的在线局部执行
- 动态障碍监控
- `hold / wait / replan` 恢复机制

系统仍然工作在关节空间，不是笛卡尔规划器；运行时规划环路也不依赖 MoveIt。

## 2. 主要组件

### 2.1 启动与仿真

文件：

- `otg_planner/launch/simulation.launch.py`

职责：

- 启动 Gazebo Sim
- 发布 `/clock`
- 生成 Panda 机器人
- 启动 `robot_state_publisher`
- 激活 `joint_state_broadcaster`
- 激活 `panda_arm_controller`
- 启动规划节点
- 按需启动 demo 客户端
- 按需启动障碍场景发布器

### 2.2 规划节点

文件：

- `otg_planner/planner_server.py`

节点：

- `/otg_planner_node`

职责：

- 从 `/otg/goal_trajectory` 接收外部目标
- 将目标 waypoint 规范化到 Panda 关节顺序
- 从 `/joint_states` 读取当前运动状态
- 规划带碰撞约束的关节路径
- 每个控制周期执行一次 `Ruckig` 更新
- 向 `/joint_trajectory` 流式发送单点命令
- 监控障碍 clearance
- 在执行、等待和重规划状态之间切换

### 2.3 规划核心

文件：

- `otg_planner/planning_core.py`

职责：

- Panda 关节限制和动力学约束
- 简化前向运动学
- sphere-chain 机器人近似
- capsule 障碍物近似
- clearance 计算
- 碰撞有效性检查
- `RRT-Connect` 规划
- shortcut 路径压缩
- 路径重采样

### 2.4 ROS 工具层

文件：

- `otg_planner/ros_utils.py`

职责：

- 按 Panda 关节顺序重排数组
- 从 `JointState` 提取有序位置和速度
- 将 marker 转换为 capsule
- 将规划路径转换为预览轨迹
- 构造单点控制轨迹
- 在活动路径上选择前瞻目标

### 2.5 障碍场景发布器

文件：

- `otg_planner/obstacle_scenario_node.py`

节点：

- `/obstacle_scenario_publisher`

职责：

- 在 `/otg/obstacles` 上发布 `MarkerArray` 障碍物
- 支持 `static`、`moving` 和 `mixed` 三种场景
- 在场景结束后清空障碍物

### 2.6 Demo 客户端

文件：

- `otg_planner/demo_client_node.py`

节点：

- `/otg_demo_client_node`

职责：

- 交替发布 A/B 关节目标
- 等待规划器目标订阅建立
- 通过 `/joint_states` 监控执行进度

### 2.7 兼容入口

文件：

- `otg_planner/ruckig_node.py`

用途：

- 兼容包装，内部转发到 `planner_server.main`

## 3. 运行时数据流

![Runtime Dataflow](docs/images/runtime_dataflow.svg)

图源文件：`docs/images/runtime_dataflow.mmd`、`docs/images/runtime_dataflow.dot`

## 4. 内部规划与执行逻辑

### 4.1 全局几何层

对于每个请求目标：

1. 按 Panda 关节顺序重排目标关节
2. 获取最新关节位置
3. 用 `plan_joint_path` 从当前状态到每个 waypoint 做规划
4. 对路径做 shortcut
5. 以受限关节步长对路径重采样

这一层回答的问题是：

- 应该沿哪条无碰撞关节路径前进

它不负责最终的在线时间参数化。

### 4.2 局部最优执行层

旧行为是一次性发布整条预览轨迹，并依赖静态定时。

当前行为不同：

1. 从 `/joint_states` 读取当前位置、速度和估计加速度
2. 在当前路径上选择一个前瞻点
3. 把当前状态和目标点送入 `Ruckig`
4. 为下一个控制时域发布单点轨迹命令
5. 每个控制周期重复执行

这意味着实际执行始终基于机器人当前真实状态，而不是盲目的开环时间假设。

在每个控制周期内，相对于当前状态、局部目标以及配置好的速度 / 加速度 / jerk 约束，这一步是局部时间最优的。

## 5. 碰撞模型

### 5.1 机器人模型

Panda 被近似成一条沿简化运动学链布置的球串。

目的：

- 快速 clearance 检查
- 保守碰撞检测
- 规划侧几何有效性判断

### 5.2 障碍物模型

输入的 marker 会被转换成 capsule。

当前支持的 marker 类型：

- `CYLINDER`
- `SPHERE`

每个障碍 capsule 会和每个机器人球体计算距离，从而得到：

- 当前状态 clearance
- 路径 clearance

## 6. 恢复策略

当前运行时安全逻辑分三层：

### 6.1 正常执行

- 沿当前几何路径前进
- 持续流式发送 `Ruckig` 生成的单点命令

### 6.2 提前重规划

如果剩余路径进入重规划 margin，规划器不会等到硬失败后再处理，而是提前切换到重规划。

目的：

- 避免机器人已经走进坏状态之后才开始重新规划

### 6.3 等待可恢复窗口

如果当前机器人状态已经进入 clearance 违规区域：

1. 切换到 `WAITING_CLEARANCE`
2. 发布 hold 命令
3. 持续监控 clearance
4. 仅在当前状态重新变得可恢复时尝试重规划
5. 找到新路径后返回 `EXECUTING`

这正是动态障碍场景下，相比旧版“立即中止”策略更稳定的关键改动。

## 7. 规划器状态机

![Planner State Machine](docs/images/planner_state_machine.svg)

图源文件：`docs/images/planner_state_machine.mmd`、`docs/images/planner_state_machine.dot`

## 8. Topics 与接口

### 输入

- `/otg/goal_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/otg/cancel` (`std_srvs/srv/Trigger`)
- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/otg/obstacles` (`visualization_msgs/msg/MarkerArray`)

### 输出

- `/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/otg/planned_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- `/otg/planner_state` (`std_msgs/msg/String`)
- `/otg/goal_result` (`std_msgs/msg/String`)
- `/otg/min_clearance` (`std_msgs/msg/Float32`)
- `/otg/goal_joint` (`sensor_msgs/msg/JointState`)

### 控制器映射

launch 时，`/joint_trajectory` 会被重映射到：

- `/panda_arm_controller/joint_trajectory`

## 9. 关键文件

- `otg_planner/planner_server.py`
- `otg_planner/planning_core.py`
- `otg_planner/ros_utils.py`
- `otg_planner/demo_client_node.py`
- `otg_planner/obstacle_scenario_node.py`
- `otg_planner/launch/simulation.launch.py`
- `otg_planner/urdf/panda.urdf.xacro`

## 10. 已验证内容

在当前工作区中，已经验证：

- 包可以成功构建
- 测试可以通过
- Gazebo 可以端到端启动
- 控制器链路处于 active
- 外部目标可以被规划并执行
- 规划器可以看到障碍发布器的消息
- 动态障碍会触发 `WAITING_CLEARANCE` 和 `REPLANNING`
- 执行期间控制器会收到流式单点命令

## 11. 当前限制

当前策略相比旧 demo 已经稳定得多，但仍然不能保证在任意动态障碍场景下最终一定成功。

具体来说：

- 规划器目前只工作在关节空间
- 碰撞检测是近似模型
- 系统还不会预测障碍物的时间窗
- 持续振荡的障碍物仍可能导致反复重规划
- 目前还没有专门的“退回安全区域”行为

## 12. 推荐下一步

最值得继续加强的方向，是加入真正的安全退让策略：

- 当 `WAITING_CLEARANCE` 持续存在时，移动到一个已验证安全的姿态集合
- 从这个安全区域重新规划，而不是一直原地等待

这会把当前的“hold and retry”恢复策略，提升成更强的“retreat, wait, then recover”策略。
