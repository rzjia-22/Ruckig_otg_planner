"""ROS topic/service planner for joint-space planning and execution."""

from __future__ import annotations

import threading
import time
import traceback
from typing import List, Optional, Sequence, Tuple

import rclpy
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray

from .planning_core import (
    DEFAULT_POINT_A,
    PANDA_JOINT_NAMES,
    PANDA_MAX_ACCELERATION,
    PANDA_MAX_JERK,
    PANDA_MAX_VELOCITY,
    clamp_joints,
    joint_distance,
    min_robot_capsule_clearance,
    path_min_clearance,
    plan_joint_path,
    resample_path,
    shortcut_path,
    within_joint_limits,
)
from .ros_utils import (
    build_joint_state,
    build_single_point_trajectory,
    build_timed_waypoints,
    joint_motion_from_msg,
    marker_array_to_capsules,
    path_lookahead_target,
    reorder_positions,
    timed_waypoints_to_trajectory,
    within_goal_tolerance,
)


class JointPathPlannerNode(Node):
    """Plan and execute collision-aware joint-space paths."""

    def __init__(self):
        super().__init__('otg_planner_node')

        self._data_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None
        self._goal_generation = 0
        self._queued_goal_waypoints: Optional[List[List[float]]] = None
        self._cancel_requested = False

        self.declare_parameter('control_cycle', 0.05)
        self.declare_parameter('command_horizon', 0.10)
        self.declare_parameter('speed_scale', 0.35)
        self.declare_parameter('planning_max_iterations', 3500)
        self.declare_parameter('planning_step_size', 0.25)
        self.declare_parameter('planning_edge_resolution', 0.08)
        self.declare_parameter('planning_clearance_margin', 0.03)
        self.declare_parameter('planning_sphere_resolution', 0.08)
        self.declare_parameter('path_resample_step', 0.12)
        self.declare_parameter('goal_bias', 0.2)
        self.declare_parameter('planner_seed', 7)
        self.declare_parameter('joint_state_timeout', 1.5)
        self.declare_parameter('obstacle_timeout', 0.5)
        self.declare_parameter('replan_check_period', 0.2)
        self.declare_parameter('replan_clearance_buffer', 0.05)
        self.declare_parameter('hard_stop_clearance_margin', 0.0)
        self.declare_parameter('lookahead_distance', 0.35)
        self.declare_parameter('max_replans', 20)
        self.declare_parameter('default_goal_tolerance', 0.05)
        self.declare_parameter('default_path_tolerance', 1.0)
        self.declare_parameter('goal_timeout_slack', 1.0)
        self.declare_parameter('clearance_wait_timeout', 10.0)
        self.declare_parameter('stall_timeout', 2.5)
        self.declare_parameter('stall_epsilon', 0.01)
        self.declare_parameter('hold_position', DEFAULT_POINT_A)

        self.control_cycle = max(0.01, float(self.get_parameter('control_cycle').value))
        self.command_horizon = max(
            self.control_cycle,
            float(self.get_parameter('command_horizon').value),
        )
        self.speed_scale = min(1.0, max(0.05, float(self.get_parameter('speed_scale').value)))
        self.planning_max_iterations = int(self.get_parameter('planning_max_iterations').value)
        self.planning_step_size = float(self.get_parameter('planning_step_size').value)
        self.planning_edge_resolution = float(
            self.get_parameter('planning_edge_resolution').value
        )
        self.planning_clearance_margin = float(
            self.get_parameter('planning_clearance_margin').value
        )
        self.planning_sphere_resolution = float(
            self.get_parameter('planning_sphere_resolution').value
        )
        self.path_resample_step = float(self.get_parameter('path_resample_step').value)
        self.goal_bias = float(self.get_parameter('goal_bias').value)
        self.planner_seed = int(self.get_parameter('planner_seed').value)
        self.joint_state_timeout = float(self.get_parameter('joint_state_timeout').value)
        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout').value)
        self.replan_check_period = float(self.get_parameter('replan_check_period').value)
        self.replan_clearance_buffer = float(
            self.get_parameter('replan_clearance_buffer').value
        )
        self.hard_stop_clearance_margin = float(
            self.get_parameter('hard_stop_clearance_margin').value
        )
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_replans = int(self.get_parameter('max_replans').value)
        self.default_goal_tolerance = float(
            self.get_parameter('default_goal_tolerance').value
        )
        self.default_path_tolerance = float(
            self.get_parameter('default_path_tolerance').value
        )
        self.goal_timeout_slack = float(self.get_parameter('goal_timeout_slack').value)
        self.clearance_wait_timeout = float(
            self.get_parameter('clearance_wait_timeout').value
        )
        self.stall_timeout = float(self.get_parameter('stall_timeout').value)
        self.stall_epsilon = float(self.get_parameter('stall_epsilon').value)
        self.hold_position = clamp_joints(self.get_parameter('hold_position').value)
        self.replan_clearance_margin = (
            self.planning_clearance_margin + self.replan_clearance_buffer
        )

        self.latest_joint_positions: Optional[List[float]] = None
        self.latest_joint_velocities: Optional[List[float]] = None
        self.latest_joint_accelerations: Optional[List[float]] = None
        self.latest_joint_wall_time = 0.0
        self._last_velocity_sample: Optional[Tuple[List[float], float]] = None
        self.latest_obstacles = []
        self.latest_obstacle_wall_time = 0.0
        self.ruckig = Ruckig(len(PANDA_JOINT_NAMES), self.command_horizon)

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.command_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            command_qos,
        )
        self.planned_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/otg/planned_trajectory',
            10,
        )
        self.state_pub = self.create_publisher(String, '/otg/planner_state', 10)
        self.result_pub = self.create_publisher(String, '/otg/goal_result', 10)
        self.clearance_pub = self.create_publisher(Float32, '/otg/min_clearance', 10)
        self.goal_pub = self.create_publisher(JointState, '/otg/goal_joint', 10)

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 20)
        self.create_subscription(MarkerArray, '/otg/obstacles', self._obstacles_cb, 20)
        self.create_subscription(
            JointTrajectory,
            '/otg/goal_trajectory',
            self._goal_trajectory_cb,
            20,
        )
        self.create_service(Trigger, '/otg/cancel', self._cancel_cb)

        self._publish_state('IDLE')
        self._publish_result('Planner started and waiting for /otg/goal_trajectory.')
        self.get_logger().info(
            'Joint path planner started. '
            f'cycle={self.control_cycle:.2f}s speed_scale={self.speed_scale:.2f} '
            f'clearance_margin={self.planning_clearance_margin:.3f} '
            f'replan_margin={self.replan_clearance_margin:.3f}'
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        motion = joint_motion_from_msg(msg, PANDA_JOINT_NAMES)
        if motion is None:
            return
        positions, velocities = motion
        sample_time = time.monotonic()
        accelerations = [0.0] * len(PANDA_JOINT_NAMES)

        if self._last_velocity_sample is not None:
            previous_velocities, previous_time = self._last_velocity_sample
            dt = sample_time - previous_time
            if dt > 1e-4:
                accelerations = [
                    (velocities[index] - previous_velocities[index]) / dt
                    for index in range(len(PANDA_JOINT_NAMES))
                ]

        with self._data_lock:
            self.latest_joint_positions = positions
            self.latest_joint_velocities = velocities
            self.latest_joint_accelerations = accelerations
            self.latest_joint_wall_time = time.monotonic()
        self._last_velocity_sample = (list(velocities), sample_time)

    def _obstacles_cb(self, msg: MarkerArray) -> None:
        with self._data_lock:
            self.latest_obstacles = marker_array_to_capsules(msg)
            self.latest_obstacle_wall_time = time.monotonic()

    def _goal_trajectory_cb(self, msg: JointTrajectory) -> None:
        self.get_logger().info(
            f'Received goal trajectory with {len(msg.points)} point(s).'
        )
        goal_waypoints, error = self._normalize_goal_waypoints(msg)
        if goal_waypoints is None:
            self._publish_state('REJECTED')
            self._publish_result(f'Rejected goal: {error}')
            self.get_logger().warning(error)
            return

        is_preempting = False
        with self._goal_lock:
            is_preempting = self._worker_thread is not None and self._worker_thread.is_alive()
            self._goal_generation += 1
            self._queued_goal_waypoints = goal_waypoints
            self._cancel_requested = False

            if self._worker_thread is None or not self._worker_thread.is_alive():
                self._worker_thread = threading.Thread(
                    target=self._worker_loop,
                    daemon=True,
                )
                self._worker_thread.start()

        if is_preempting:
            self._publish_state('PREEMPTING')
            self._publish_hold_position(
                self._fresh_joint_positions() or self.hold_position
            )
            self.get_logger().info('Preempting current execution with a newer goal.')

        self._publish_result(
            f'Queued goal with {len(goal_waypoints)} waypoint(s).'
        )

    def _cancel_cb(self, _request, response):
        active_goal = False
        with self._goal_lock:
            active_goal = (
                self._queued_goal_waypoints is not None
                or self._worker_thread is not None and self._worker_thread.is_alive()
            )
            if active_goal:
                self._goal_generation += 1
                self._queued_goal_waypoints = None
                self._cancel_requested = True

        if active_goal:
            self._publish_state('CANCELING')
            self._publish_hold_position(
                self._fresh_joint_positions() or self.hold_position
            )
            self._publish_result('Cancel requested for the active goal.')
            response.success = True
            response.message = 'Canceled active or queued goal.'
        else:
            response.success = False
            response.message = 'No active or queued goal.'
        return response

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)

    def _publish_clearance(self, clearance: float) -> None:
        msg = Float32()
        msg.data = float(clearance)
        self.clearance_pub.publish(msg)

    def _publish_goal(self, positions: Sequence[float]) -> None:
        stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(build_joint_state(PANDA_JOINT_NAMES, positions, stamp))

    def _fresh_joint_positions(self) -> Optional[List[float]]:
        with self._data_lock:
            if self.latest_joint_positions is None:
                return None
            if time.monotonic() - self.latest_joint_wall_time > self.joint_state_timeout:
                return None
            return list(self.latest_joint_positions)

    def _fresh_joint_motion(self) -> Optional[Tuple[List[float], List[float], List[float]]]:
        with self._data_lock:
            if self.latest_joint_positions is None:
                return None
            if time.monotonic() - self.latest_joint_wall_time > self.joint_state_timeout:
                return None

            velocities = (
                list(self.latest_joint_velocities)
                if self.latest_joint_velocities is not None
                else [0.0] * len(PANDA_JOINT_NAMES)
            )
            accelerations = (
                list(self.latest_joint_accelerations)
                if self.latest_joint_accelerations is not None
                else [0.0] * len(PANDA_JOINT_NAMES)
            )
            return list(self.latest_joint_positions), velocities, accelerations

    def _fresh_obstacles(self):
        with self._data_lock:
            if time.monotonic() - self.latest_obstacle_wall_time > self.obstacle_timeout:
                return []
            return list(self.latest_obstacles)

    def _wait_for_joint_positions(self, timeout_seconds: float) -> Optional[List[float]]:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline and rclpy.ok():
            positions = self._fresh_joint_positions()
            if positions is not None:
                return positions
            time.sleep(0.02)
        return None

    def _wait_for_joint_motion(
        self,
        timeout_seconds: float,
    ) -> Optional[Tuple[List[float], List[float], List[float]]]:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline and rclpy.ok():
            motion = self._fresh_joint_motion()
            if motion is not None:
                return motion
            time.sleep(0.02)
        return None

    def _normalize_goal_waypoints(
        self,
        msg: JointTrajectory,
    ) -> Tuple[Optional[List[List[float]]], str]:
        if not msg.points:
            return None, 'Goal trajectory does not contain any points.'

        if set(msg.joint_names) != set(PANDA_JOINT_NAMES):
            return None, 'Goal trajectory joint names do not match the Panda arm.'

        waypoints: List[List[float]] = []
        for point in msg.points:
            ordered = reorder_positions(
                msg.joint_names,
                point.positions,
                PANDA_JOINT_NAMES,
            )
            if ordered is None:
                return None, 'Failed to reorder a goal waypoint to Panda joint order.'
            if not within_joint_limits(ordered):
                return None, 'One or more waypoints exceed the Panda joint limits.'
            if not waypoints or joint_distance(waypoints[-1], ordered) > 1e-5:
                waypoints.append(ordered)

        if not waypoints:
            return None, 'The goal trajectory did not contain any distinct waypoints.'
        return waypoints, ''

    def _plan_path(
        self,
        start: Sequence[float],
        goal_waypoints: Sequence[Sequence[float]],
        obstacles,
    ) -> Tuple[Optional[List[List[float]]], str]:
        path: List[List[float]] = [list(start)]
        current = list(start)

        for index, waypoint in enumerate(goal_waypoints):
            if joint_distance(current, waypoint) <= 1e-5:
                continue

            segment = plan_joint_path(
                current,
                waypoint,
                obstacles,
                clearance_margin=self.planning_clearance_margin,
                max_iterations=self.planning_max_iterations,
                step_size=self.planning_step_size,
                edge_resolution=self.planning_edge_resolution,
                sphere_resolution=self.planning_sphere_resolution,
                goal_bias=self.goal_bias,
                seed=self.planner_seed + index,
            )
            if segment is None:
                return None, f'Failed to plan a path to waypoint {index + 1}.'

            segment = shortcut_path(
                segment,
                obstacles,
                clearance_margin=self.planning_clearance_margin,
                edge_resolution=self.planning_edge_resolution,
                sphere_resolution=self.planning_sphere_resolution,
                attempts=100,
                seed=self.planner_seed + index,
            )
            path.extend(segment[1:])
            current = list(waypoint)

        return resample_path(path, max_joint_step=self.path_resample_step), ''

    def _publish_hold_position(self, positions: Sequence[float]) -> None:
        trajectory = build_single_point_trajectory(
            PANDA_JOINT_NAMES,
            positions,
            time_from_start=self.command_horizon,
            velocities=[0.0] * len(PANDA_JOINT_NAMES),
            accelerations=[0.0] * len(PANDA_JOINT_NAMES),
        )
        self.command_pub.publish(trajectory)

    def _publish_otg_command(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        accelerations: Sequence[float],
    ) -> None:
        trajectory = build_single_point_trajectory(
            PANDA_JOINT_NAMES,
            positions,
            time_from_start=self.command_horizon,
            velocities=velocities,
            accelerations=accelerations,
        )
        self.command_pub.publish(trajectory)

    def _publish_planned_path_preview(self, path: Sequence[Sequence[float]]):
        timed_waypoints = build_timed_waypoints(
            path,
            PANDA_MAX_VELOCITY,
            velocity_scale=self.speed_scale,
            min_segment_time=self.command_horizon,
        )
        trajectory = timed_waypoints_to_trajectory(PANDA_JOINT_NAMES, timed_waypoints)
        self.planned_trajectory_pub.publish(trajectory)
        return timed_waypoints

    def _path_clearance(
        self,
        current_positions: Sequence[float],
        active_path: Sequence[Sequence[float]],
        obstacles,
    ) -> float:
        if not obstacles or not active_path:
            return float('inf')

        return path_min_clearance(
            [list(point) for point in active_path],
            obstacles,
            edge_resolution=self.planning_edge_resolution,
            sphere_resolution=self.planning_sphere_resolution,
        )

    def _current_clearance(
        self,
        current_positions: Sequence[float],
        obstacles,
    ) -> float:
        if not obstacles:
            return float('inf')

        return min_robot_capsule_clearance(
            current_positions,
            obstacles,
            resolution=self.planning_sphere_resolution,
        )

    def _compute_ruckig_step(
        self,
        current_positions: Sequence[float],
        current_velocities: Sequence[float],
        current_accelerations: Sequence[float],
        target_positions: Sequence[float],
    ) -> Tuple[Optional[Tuple[List[float], List[float], List[float]]], str]:
        input_state = InputParameter(len(PANDA_JOINT_NAMES))
        output_state = OutputParameter(len(PANDA_JOINT_NAMES))

        input_state.current_position = [float(value) for value in current_positions]
        input_state.current_velocity = [float(value) for value in current_velocities]
        input_state.current_acceleration = [float(value) for value in current_accelerations]
        input_state.target_position = [float(value) for value in target_positions]
        input_state.target_velocity = [0.0] * len(PANDA_JOINT_NAMES)
        input_state.target_acceleration = [0.0] * len(PANDA_JOINT_NAMES)

        velocity_scale = min(1.0, max(0.05, self.speed_scale))
        dynamic_scale = max(0.25, velocity_scale)
        input_state.max_velocity = [value * velocity_scale for value in PANDA_MAX_VELOCITY]
        input_state.max_acceleration = [
            value * dynamic_scale for value in PANDA_MAX_ACCELERATION
        ]
        input_state.max_jerk = [value * dynamic_scale for value in PANDA_MAX_JERK]

        result = self.ruckig.update(input_state, output_state)
        if result not in (Result.Working, Result.Finished):
            return None, f'Ruckig update failed with result {result!s}.'

        next_state = (
            [float(value) for value in output_state.new_position],
            [float(value) for value in output_state.new_velocity],
            [float(value) for value in output_state.new_acceleration],
        )
        return next_state, ''

    def _hold_until_plan_available(
        self,
        final_goal: Sequence[float],
        generation: int,
    ) -> Tuple[
        Optional[Tuple[List[float], List[float], List[float]]],
        Optional[List[List[float]]],
        str,
    ]:
        deadline = time.monotonic() + self.clearance_wait_timeout
        last_error = 'Timed out while waiting for a recoverable replan window.'
        while rclpy.ok() and time.monotonic() < deadline:
            generation_state = self._generation_state(generation)
            if generation_state != 'active':
                return None, None, 'Goal became inactive while waiting for replanning.'

            current_motion = self._fresh_joint_motion()
            if current_motion is None:
                time.sleep(self.control_cycle)
                continue

            current_positions, _, _ = current_motion
            obstacles = self._fresh_obstacles()
            current_clearance = self._current_clearance(current_positions, obstacles)
            self._publish_clearance(current_clearance)
            self._publish_hold_position(current_positions)

            if current_clearance >= self.replan_clearance_margin:
                replanned_path, error = self._plan_path(
                    current_positions,
                    [final_goal],
                    obstacles,
                )
                if replanned_path is not None:
                    return current_motion, replanned_path, ''
                last_error = error

            time.sleep(self.control_cycle)
        return None, None, last_error

    def _generation_state(self, generation: int) -> str:
        with self._goal_lock:
            if generation == self._goal_generation:
                return 'active'
            if self._cancel_requested and self._queued_goal_waypoints is None:
                return 'canceled'
            return 'preempted'

    def _worker_loop(self) -> None:
        try:
            while rclpy.ok():
                with self._goal_lock:
                    if self._queued_goal_waypoints is None:
                        self._worker_thread = None
                        return
                    goal_waypoints = [list(point) for point in self._queued_goal_waypoints]
                    generation = self._goal_generation
                    self._queued_goal_waypoints = None
                    self._cancel_requested = False

                self._execute_goal(goal_waypoints, generation)
        except Exception as exc:  # pragma: no cover - runtime diagnostics
            self.get_logger().error(
                f'Planner worker crashed: {exc}\n{traceback.format_exc()}'
            )
            with self._goal_lock:
                self._worker_thread = None

    def _execute_goal(
        self,
        goal_waypoints: Sequence[Sequence[float]],
        generation: int,
    ) -> None:
        self.get_logger().info(
            f'Executing goal generation={generation} waypoint_count={len(goal_waypoints)}'
        )
        self._publish_state('PLANNING')

        current_motion = self._wait_for_joint_motion(self.joint_state_timeout)
        if current_motion is None:
            self._publish_state('ABORTED')
            self._publish_result('Aborted goal: no fresh /joint_states available.')
            self.get_logger().warning('No fresh /joint_states available while planning.')
            return
        current_positions, current_velocities, current_accelerations = current_motion

        if self._generation_state(generation) != 'active':
            self.get_logger().info('Goal became inactive before planning completed.')
            return

        final_goal = list(goal_waypoints[-1])
        self._publish_goal(final_goal)

        obstacles = self._fresh_obstacles()
        active_path, error = self._plan_path(current_positions, goal_waypoints, obstacles)
        if active_path is None:
            self._publish_state('ABORTED')
            self._publish_result(f'Aborted goal: {error}')
            self.get_logger().warning(f'Planning failed: {error}')
            return

        self.get_logger().info(
            f'Planned path with {len(active_path)} sampled waypoints.'
        )
        timed_waypoints = self._publish_planned_path_preview(active_path)
        trajectory_duration = (
            timed_waypoints[-1].time_from_start if timed_waypoints else self.command_horizon
        )
        execution_start = time.monotonic()
        replans = 0
        last_replan_check = 0.0

        goal_tolerances = {
            joint_name: self.default_goal_tolerance for joint_name in PANDA_JOINT_NAMES
        }
        best_goal_distance = joint_distance(current_positions, final_goal)
        last_progress_time = time.monotonic()
        path_cursor = 0

        self._publish_state('EXECUTING')

        while rclpy.ok():
            generation_state = self._generation_state(generation)
            if generation_state == 'preempted':
                self._publish_state('PREEMPTED')
                self._publish_result('Goal preempted by a newer request.')
                self.get_logger().info('Execution preempted.')
                return
            if generation_state == 'canceled':
                self._publish_state('CANCELED')
                self._publish_result('Goal canceled.')
                self.get_logger().info('Execution canceled.')
                return

            current_motion = self._fresh_joint_motion()
            if current_motion is None:
                self._publish_state('ABORTED')
                self._publish_result('Aborted goal: lost fresh /joint_states during execution.')
                self.get_logger().warning('Lost fresh /joint_states during execution.')
                return
            current_positions, current_velocities, current_accelerations = current_motion

            goal_distance = joint_distance(current_positions, final_goal)
            if goal_distance + self.stall_epsilon < best_goal_distance:
                best_goal_distance = goal_distance
                last_progress_time = time.monotonic()

            if within_goal_tolerance(
                current_positions,
                final_goal,
                PANDA_JOINT_NAMES,
                goal_tolerances,
            ):
                self._publish_state('SUCCEEDED')
                self._publish_result('Goal reached successfully.')
                self.get_logger().info('Goal reached successfully.')
                return

            now = time.monotonic()
            elapsed = now - execution_start
            if elapsed > trajectory_duration + self.goal_timeout_slack:
                if now - last_progress_time <= self.stall_timeout and replans < self.max_replans:
                    self._publish_state('REPLANNING')
                    self._publish_hold_position(current_positions)
                    replans += 1
                    self.get_logger().info(
                        f'Execution exceeded preview duration. Replanning attempt {replans}.'
                    )
                    obstacles = self._fresh_obstacles()
                    replanned_path, error = self._plan_path(
                        current_positions,
                        [final_goal],
                        obstacles,
                    )
                    if replanned_path is None:
                        self._publish_state('ABORTED')
                        self._publish_hold_position(current_positions)
                        self._publish_result(f'Aborted goal: {error}')
                        self.get_logger().warning(f'Replanning failed: {error}')
                        return

                    active_path = replanned_path
                    timed_waypoints = self._publish_planned_path_preview(active_path)
                    trajectory_duration = (
                        timed_waypoints[-1].time_from_start
                        if timed_waypoints
                        else self.command_horizon
                    )
                    execution_start = now
                    last_progress_time = now
                    best_goal_distance = goal_distance
                    path_cursor = 0
                    self._publish_state('EXECUTING')
                else:
                    self._publish_state('ABORTED')
                    self._publish_hold_position(current_positions)
                    self._publish_result('Aborted goal: goal tolerance violated by timeout.')
                    self.get_logger().warning('Goal tolerance timeout.')
                    return

            if now - last_replan_check >= self.replan_check_period:
                obstacles = self._fresh_obstacles()
                current_clearance = self._current_clearance(current_positions, obstacles)
                clearance = self._path_clearance(
                    current_positions,
                    active_path[path_cursor:],
                    obstacles,
                )
                monitored_clearance = min(current_clearance, clearance)
                self._publish_clearance(monitored_clearance)

                if current_clearance < self.hard_stop_clearance_margin:
                    if replans >= self.max_replans:
                        self._publish_state('ABORTED')
                        self._publish_hold_position(current_positions)
                        self._publish_result(
                            'Aborted goal: current state entered an unrecoverable '
                            'clearance violation.'
                        )
                        self.get_logger().warning('Current state clearance violation.')
                        return

                    self._publish_state('WAITING_CLEARANCE')
                    self._publish_hold_position(current_positions)
                    self.get_logger().warning(
                        'Current state clearance violated. '
                        'Holding position until the scene clears.'
                    )
                    recovered_motion, replanned_path, error = self._hold_until_plan_available(
                        final_goal,
                        generation,
                    )
                    if recovered_motion is None or replanned_path is None:
                        self._publish_state('ABORTED')
                        self._publish_hold_position(current_positions)
                        self._publish_result(f'Aborted goal: {error}')
                        self.get_logger().warning(f'Clearance recovery failed: {error}')
                        return

                    current_positions, current_velocities, current_accelerations = recovered_motion
                    replans += 1
                    self._publish_state('REPLANNING')
                    self.get_logger().info(
                        f'Clearance recovered. Replanning attempt {replans}.'
                    )
                    active_path = replanned_path
                    timed_waypoints = self._publish_planned_path_preview(active_path)
                    trajectory_duration = (
                        timed_waypoints[-1].time_from_start
                        if timed_waypoints
                        else self.command_horizon
                    )
                    execution_start = time.monotonic()
                    last_progress_time = execution_start
                    best_goal_distance = joint_distance(current_positions, final_goal)
                    path_cursor = 0
                    self._publish_state('EXECUTING')
                    last_replan_check = execution_start
                    continue

                if clearance < self.replan_clearance_margin:
                    if replans >= self.max_replans:
                        self._publish_state('ABORTED')
                        self._publish_hold_position(current_positions)
                        self._publish_result(
                            'Aborted goal: obstacle invalidated the path and '
                            'max replans was reached.'
                        )
                        return

                    self._publish_state('REPLANNING')
                    self._publish_hold_position(current_positions)
                    replans += 1
                    self.get_logger().info(
                        f'Path entered replan margin. Replanning attempt {replans}.'
                    )
                    recovered_motion, replanned_path, error = self._hold_until_plan_available(
                        final_goal,
                        generation,
                    )
                    if recovered_motion is None or replanned_path is None:
                        self._publish_state('ABORTED')
                        self._publish_hold_position(current_positions)
                        self._publish_result(f'Aborted goal: {error}')
                        self.get_logger().warning(f'Replanning failed: {error}')
                        return

                    current_positions, current_velocities, current_accelerations = recovered_motion
                    active_path = replanned_path
                    timed_waypoints = self._publish_planned_path_preview(active_path)
                    trajectory_duration = (
                        timed_waypoints[-1].time_from_start
                        if timed_waypoints
                        else self.command_horizon
                    )
                    execution_start = now
                    last_progress_time = now
                    best_goal_distance = goal_distance
                    path_cursor = 0
                    self._publish_state('EXECUTING')

                last_replan_check = now

            target_positions, target_index = path_lookahead_target(
                active_path,
                current_positions,
                self.lookahead_distance,
                min_index=path_cursor,
            )
            path_cursor = max(path_cursor, target_index)
            next_state, error = self._compute_ruckig_step(
                current_positions,
                current_velocities,
                current_accelerations,
                target_positions,
            )
            if next_state is None:
                self._publish_state('ABORTED')
                self._publish_hold_position(current_positions)
                self._publish_result(f'Aborted goal: {error}')
                self.get_logger().warning(error)
                return

            next_positions, next_velocities, next_accelerations = next_state
            self._publish_otg_command(
                next_positions,
                next_velocities,
                next_accelerations,
            )

            time.sleep(self.control_cycle)

    def destroy_node(self):
        """Stop the planner node."""
        with self._goal_lock:
            self._goal_generation += 1
            self._queued_goal_waypoints = None
            self._cancel_requested = True
            worker = self._worker_thread
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None) -> None:
    """Run the planner node."""
    rclpy.init(args=args)
    node = JointPathPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
