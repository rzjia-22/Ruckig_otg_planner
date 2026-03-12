# otg_planner

<p align="right">
  <a href="./README.md">简体中文</a> | <a href="./README.en.md">English</a>
</p>

`otg_planner` is a joint-space planning and execution stack for the Franka Panda on `ROS 2 Humble + Gazebo Sim`.

It is no longer the original fixed A/B motion demo. The current project provides a complete closed-loop planning stack:

- external `JointTrajectory` goal input
- collision-aware joint-space planning with `RRT-Connect`
- `Ruckig`-based online locally time-optimal execution
- dynamic obstacle monitoring, wait-and-recover behavior, and automatic replanning
- end-to-end Gazebo + `ros2_control` simulation
- optional demo client and obstacle scenario publishers

Architecture details:

- see [ARCHITECTURE.en.md](./ARCHITECTURE.en.md) for the runtime graph, topic graph, state machine, and execution logic

## 1. Build

```bash
cd /home/zoel/ruckig_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select otg_planner
source install/setup.bash
```

## 2. Run Simulation

```bash
source /opt/ros/humble/setup.bash
source /home/zoel/ruckig_ws/install/setup.bash
ros2 launch otg_planner simulation.launch.py
```

Default launch behavior:

- starts Gazebo Sim and spawns Panda
- activates `joint_state_broadcaster` and `panda_arm_controller`
- starts `/otg_planner_node`
- starts the demo client that alternates between Pose A and Pose B

## 3. Send Your Own Goal

The planner exposes:

- topic: `/otg/goal_trajectory`
- topic: `/otg/planned_trajectory`
- topic: `/otg/planner_state`
- topic: `/otg/goal_result`
- topic: `/otg/min_clearance`
- service: `/otg/cancel`

Example joint-space goal:

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

The planner treats `points[].positions` as joint-space waypoints, replans internally, and publishes the resulting controller trajectory.

Cancel the current goal:

```bash
ros2 service call /otg/cancel std_srvs/srv/Trigger
```

## 4. Demo Controls

Slow the robot down or reduce the demo motion amplitude:

```bash
ros2 launch otg_planner simulation.launch.py \
  planner_speed_scale:=0.25 \
  planner_path_scale:=0.7
```

Disable the demo client and control the planner manually:

```bash
ros2 launch otg_planner simulation.launch.py enable_demo:=false
```

## 5. Obstacle Replanning Demo

Start the simulation with obstacle publishing enabled:

```bash
ros2 launch otg_planner simulation.launch.py \
  enable_obstacle_scenario:=true \
  obstacle_scenario:=moving
```

Supported scenarios:

- `static`
- `moving`
- `mixed`

The planner monitors `/otg/obstacles` and automatically waits or replans if the active path becomes unsafe.

## 6. One-Click Health Check

Run in another terminal while the simulation is active:

```bash
source /opt/ros/humble/setup.bash
source /home/zoel/ruckig_ws/install/setup.bash
bash /home/zoel/ruckig_ws/src/otg_planner/scripts/check_sim_health.sh
```

The script verifies:

- required ROS nodes exist
- controllers are active
- `/clock` and `/joint_states` are publishing
- the planner is publishing trajectories
- the arm is actually moving

## 7. Project Characteristics

- The system plans in joint space, not Cartesian task space.
- The runtime planning loop does not depend on MoveIt.
- Collision checks use a conservative sphere-chain robot model and capsule obstacle model.
- The dynamic-obstacle execution policy is `WAITING_CLEARANCE -> REPLANNING -> EXECUTING`.
