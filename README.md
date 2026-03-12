# otg_planner

<p align="right">
  <a href="./README.md">简体中文</a> | <a href="./README.en.md">English</a>
</p>

`otg_planner` 是一个面向 Franka Panda 的 `ROS 2 Humble + Gazebo Sim` 关节空间规划与执行系统。

它已经不再是最初的固定 A/B 往返 demo，而是包含完整闭环的规划栈：

- 外部 `JointTrajectory` 目标输入
- 基于 `RRT-Connect` 的关节空间无碰撞路径规划
- 基于 `Ruckig` 的在线局部最优执行
- 动态障碍监控、等待清障与自动重规划
- Gazebo + `ros2_control` 端到端仿真链路
- 可选 demo 客户端与障碍场景发布器

架构说明：

- 结构、状态机、数据流与执行逻辑见 [ARCHITECTURE.md](./ARCHITECTURE.md)

## 1. 构建

```bash
cd /home/zoel/ruckig_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select otg_planner
source install/setup.bash
```

## 2. 启动仿真

```bash
source /opt/ros/humble/setup.bash
source /home/zoel/ruckig_ws/install/setup.bash
ros2 launch otg_planner simulation.launch.py
```

默认启动行为：

- 启动 Gazebo Sim 并生成 Panda 机械臂
- 激活 `joint_state_broadcaster` 和 `panda_arm_controller`
- 启动 `/otg_planner_node`
- 启动自动在 Pose A / Pose B 间切换的 demo 客户端

## 3. 手动发送目标

规划器提供以下接口：

- topic: `/otg/goal_trajectory`
- topic: `/otg/planned_trajectory`
- topic: `/otg/planner_state`
- topic: `/otg/goal_result`
- topic: `/otg/min_clearance`
- service: `/otg/cancel`

示例关节目标：

```bash
ros2 topic pub --once /otg/goal_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7],
  points: [
    {
      positions: [1.2, 0.1, -0.4, -1.6, 0.6, 2.1, -0.3]
    }
  ]
}"
```

规划器会将 `points[].positions` 作为关节空间 waypoint，内部完成重规划并发布控制器轨迹。

取消当前目标：

```bash
ros2 service call /otg/cancel std_srvs/srv/Trigger
```

## 4. Demo 控制

降低速度或减小演示幅度：

```bash
ros2 launch otg_planner simulation.launch.py \
  planner_speed_scale:=0.25 \
  planner_path_scale:=0.7
```

关闭 demo，改为手动控制规划器：

```bash
ros2 launch otg_planner simulation.launch.py enable_demo:=false
```

## 5. 障碍重规划演示

启动带障碍发布器的仿真：

```bash
ros2 launch otg_planner simulation.launch.py \
  enable_obstacle_scenario:=true \
  obstacle_scenario:=moving
```

支持的场景：

- `static`
- `moving`
- `mixed`

规划器会持续监控 `/otg/obstacles`，当当前路径变得不安全时自动等待或重规划。

## 6. 一键健康检查

在仿真运行期间的另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/zoel/ruckig_ws/install/setup.bash
bash /home/zoel/ruckig_ws/src/otg_planner/scripts/check_sim_health.sh
```

脚本会检查：

- 关键 ROS 节点是否存在
- 控制器是否处于 active
- `/clock` 和 `/joint_states` 是否在发布
- 规划器是否在发布轨迹
- 机械臂是否实际发生了运动

## 7. 项目特性

- 系统运行在关节空间，不是笛卡尔任务空间规划器
- 运行时规划环路不依赖 MoveIt
- 碰撞检测使用保守的 sphere-chain 机械臂模型和 capsule 障碍模型
- 动态障碍下的执行策略是 `WAITING_CLEARANCE -> REPLANNING -> EXECUTING`
