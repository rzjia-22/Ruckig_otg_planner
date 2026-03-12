"""ROS-facing helpers for the Panda joint planner."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .planning_core import Capsule, joint_distance, nearest_path_index


@dataclass(frozen=True)
class TimedWaypoint:
    """A joint target paired with its cumulative time from start."""

    time_from_start: float
    positions: List[float]


def joint_positions_from_msg(
    msg: JointState,
    expected_joint_names: Sequence[str],
) -> Optional[List[float]]:
    """Extract joint positions in a fixed joint order."""
    if not msg.name or not msg.position:
        return None

    index_by_name = {name: index for index, name in enumerate(msg.name)}
    ordered_positions: List[float] = []

    for joint_name in expected_joint_names:
        if joint_name not in index_by_name:
            return None
        ordered_positions.append(float(msg.position[index_by_name[joint_name]]))

    return ordered_positions


def joint_motion_from_msg(
    msg: JointState,
    expected_joint_names: Sequence[str],
) -> Optional[Tuple[List[float], List[float]]]:
    """Extract ordered positions and velocities from a JointState."""
    positions = joint_positions_from_msg(msg, expected_joint_names)
    if positions is None:
        return None

    if not msg.velocity:
        return positions, [0.0] * len(expected_joint_names)

    velocities = reorder_positions(
        msg.name,
        msg.velocity,
        expected_joint_names,
    )
    if velocities is None:
        velocities = [0.0] * len(expected_joint_names)
    return positions, velocities


def reorder_positions(
    joint_names: Sequence[str],
    positions: Sequence[float],
    expected_joint_names: Sequence[str],
) -> Optional[List[float]]:
    """Reorder a joint list to match the planner joint order."""
    if len(joint_names) != len(positions):
        return None

    index_by_name = {name: index for index, name in enumerate(joint_names)}
    ordered_positions: List[float] = []

    for joint_name in expected_joint_names:
        if joint_name not in index_by_name:
            return None
        ordered_positions.append(float(positions[index_by_name[joint_name]]))

    return ordered_positions


def _quat_to_axis(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return [0.0, 0.0, 1.0]

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rotation = [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]
    return [rotation[0][2], rotation[1][2], rotation[2][2]]


def marker_array_to_capsules(msg: MarkerArray) -> List[Capsule]:
    """Convert RViz/Gazebo markers into planner capsules."""
    capsules: List[Capsule] = []

    for marker in msg.markers:
        if marker.action == Marker.DELETEALL:
            return []
        if marker.action == Marker.DELETE:
            continue
        if marker.type not in (Marker.CYLINDER, Marker.SPHERE):
            continue

        center = [
            float(marker.pose.position.x),
            float(marker.pose.position.y),
            float(marker.pose.position.z),
        ]
        radius = max(1e-3, float(marker.scale.x) * 0.5)

        if marker.type == Marker.SPHERE:
            axis = [0.0, 0.0, 1.0]
            length = radius * 2.0
        else:
            axis = _quat_to_axis(
                float(marker.pose.orientation.x),
                float(marker.pose.orientation.y),
                float(marker.pose.orientation.z),
                float(marker.pose.orientation.w),
            )
            length = max(radius * 2.0, float(marker.scale.z))

        cylinder_half = max(0.0, 0.5 * length - radius)
        point_a = (
            center[0] - cylinder_half * axis[0],
            center[1] - cylinder_half * axis[1],
            center[2] - cylinder_half * axis[2],
        )
        point_b = (
            center[0] + cylinder_half * axis[0],
            center[1] + cylinder_half * axis[1],
            center[2] + cylinder_half * axis[2],
        )
        capsules.append(
            Capsule(
                a=point_a,
                b=point_b,
                radius=radius,
                obstacle_id=f'{marker.ns}:{marker.id}',
            )
        )

    return capsules


def build_timed_waypoints(
    path: Sequence[Sequence[float]],
    max_velocity: Sequence[float],
    velocity_scale: float,
    min_segment_time: float,
) -> List[TimedWaypoint]:
    """Assign conservative timing to a path."""
    if not path:
        return []

    limited_scale = min(1.0, max(0.05, float(velocity_scale)))
    timed_waypoints: List[TimedWaypoint] = []
    elapsed = 0.0

    for index in range(1, len(path)):
        previous = path[index - 1]
        current = path[index]
        segment_time = 0.0

        for joint_index, limit in enumerate(max_velocity):
            per_joint_time = abs(current[joint_index] - previous[joint_index]) / max(
                1e-3,
                float(limit) * limited_scale,
            )
            segment_time = max(segment_time, per_joint_time)

        elapsed += max(float(min_segment_time), segment_time)
        timed_waypoints.append(
            TimedWaypoint(
                time_from_start=elapsed,
                positions=[float(value) for value in current],
            )
        )

    if not timed_waypoints:
        timed_waypoints.append(
            TimedWaypoint(
                time_from_start=max(float(min_segment_time), 1e-3),
                positions=[float(value) for value in path[0]],
            )
        )

    return timed_waypoints


def timed_waypoints_to_trajectory(
    joint_names: Sequence[str],
    timed_waypoints: Sequence[TimedWaypoint],
) -> JointTrajectory:
    """Create a JointTrajectory message from timed waypoints."""
    trajectory = JointTrajectory()
    trajectory.header.stamp.sec = 0
    trajectory.header.stamp.nanosec = 0
    trajectory.joint_names = list(joint_names)

    for waypoint in timed_waypoints:
        point = JointTrajectoryPoint()
        point.positions = list(waypoint.positions)
        point.time_from_start = Duration(seconds=waypoint.time_from_start).to_msg()
        trajectory.points.append(point)

    return trajectory


def build_single_point_trajectory(
    joint_names: Sequence[str],
    positions: Sequence[float],
    time_from_start: float,
    velocities: Optional[Sequence[float]] = None,
    accelerations: Optional[Sequence[float]] = None,
) -> JointTrajectory:
    """Create a single-point command trajectory."""
    trajectory = JointTrajectory()
    trajectory.header.stamp.sec = 0
    trajectory.header.stamp.nanosec = 0
    trajectory.joint_names = list(joint_names)

    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in positions]
    if velocities is not None:
        point.velocities = [float(value) for value in velocities]
    if accelerations is not None:
        point.accelerations = [float(value) for value in accelerations]
    point.time_from_start = Duration(seconds=max(1e-3, float(time_from_start))).to_msg()
    trajectory.points.append(point)
    return trajectory


def build_joint_state(
    joint_names: Sequence[str],
    positions: Sequence[float],
    stamp_msg=None,
) -> JointState:
    """Create a JointState message in a fixed order."""
    msg = JointState()
    if stamp_msg is not None:
        msg.header.stamp = stamp_msg
    msg.name = list(joint_names)
    msg.position = [float(value) for value in positions]
    return msg


def sample_timed_waypoints(
    start_positions: Sequence[float],
    timed_waypoints: Sequence[TimedWaypoint],
    elapsed: float,
) -> List[float]:
    """Sample a timed path using linear interpolation."""
    if not timed_waypoints:
        return [float(value) for value in start_positions]

    previous_time = 0.0
    previous_positions = [float(value) for value in start_positions]

    for waypoint in timed_waypoints:
        if elapsed <= waypoint.time_from_start:
            duration = max(1e-6, waypoint.time_from_start - previous_time)
            alpha = min(1.0, max(0.0, (elapsed - previous_time) / duration))
            return [
                previous_positions[index]
                + alpha * (waypoint.positions[index] - previous_positions[index])
                for index in range(len(previous_positions))
            ]

        previous_time = waypoint.time_from_start
        previous_positions = list(waypoint.positions)

    return list(timed_waypoints[-1].positions)


def path_lookahead_target(
    path: Sequence[Sequence[float]],
    current_positions: Sequence[float],
    lookahead_distance: float,
    min_index: int = 0,
) -> Tuple[List[float], int]:
    """Return a path point ahead of the current state by a joint-space distance."""
    if not path:
        return [float(value) for value in current_positions], 0

    start_index = min(max(0, int(min_index)), len(path) - 1)
    nearest_offset = nearest_path_index(path[start_index:], current_positions)
    target_index = start_index + nearest_offset
    accumulated = joint_distance(current_positions, path[target_index])

    while target_index < len(path) - 1 and accumulated < float(lookahead_distance):
        accumulated += joint_distance(path[target_index], path[target_index + 1])
        target_index += 1

    return [float(value) for value in path[target_index]], target_index


def duration_msg_to_seconds(duration_msg: DurationMsg) -> float:
    """Convert a ROS duration message into floating seconds."""
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def tolerance_map(
    tolerances,
    expected_joint_names: Sequence[str],
    default_position_tolerance: float,
) -> Dict[str, float]:
    """Build a per-joint position tolerance lookup."""
    lookup = {name: float(default_position_tolerance) for name in expected_joint_names}
    for tolerance in tolerances:
        name = str(tolerance.name)
        if name in lookup and float(tolerance.position) > 0.0:
            lookup[name] = float(tolerance.position)
    return lookup


def within_goal_tolerance(
    actual: Sequence[float],
    goal: Sequence[float],
    expected_joint_names: Sequence[str],
    tolerance_by_joint: Dict[str, float],
) -> bool:
    """Check whether a goal has been reached."""
    for index, joint_name in enumerate(expected_joint_names):
        if abs(float(actual[index]) - float(goal[index])) > tolerance_by_joint[joint_name]:
            return False
    return True


def max_tracking_error(
    actual: Sequence[float],
    desired: Sequence[float],
    expected_joint_names: Sequence[str],
    tolerance_by_joint: Dict[str, float],
) -> float:
    """Return the maximum normalized tracking error."""
    worst = 0.0
    for index, joint_name in enumerate(expected_joint_names):
        tolerance = max(1e-6, tolerance_by_joint[joint_name])
        error = abs(float(actual[index]) - float(desired[index])) / tolerance
        worst = max(worst, error)
    return worst
