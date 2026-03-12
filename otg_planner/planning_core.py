"""Joint-space planning primitives for the Panda arm."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import List, Optional, Sequence, Tuple


PANDA_JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

PANDA_JOINT_LIMITS: List[Tuple[float, float]] = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]

PANDA_MAX_VELOCITY = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]
PANDA_MAX_ACCELERATION = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
PANDA_MAX_JERK = [7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0]

DEFAULT_POINT_A = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
DEFAULT_POINT_B = [1.4, 0.4, -0.3, -1.2, 0.5, 2.2, -0.8]

PANDA_JOINT_ORIGINS: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = [
    ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, -0.316, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.0825, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((-0.0825, 0.384, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.088, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
]
PANDA_TOOL_OFFSET = (0.0, 0.0, 0.107)
SPHERE_CHAIN_RADII = [0.09, 0.085, 0.075, 0.07, 0.065, 0.06, 0.055, 0.05]


@dataclass(frozen=True)
class Capsule:
    """Capsule approximation used for obstacle checks."""

    a: Tuple[float, float, float]
    b: Tuple[float, float, float]
    radius: float
    obstacle_id: str = ''


@dataclass
class _TreeNode:
    """RRT tree node."""

    state: List[float]
    parent: Optional[int]


def clamp_joints(joints: Sequence[float]) -> List[float]:
    """Clamp joints to the Panda limits."""
    clamped = []
    for index, value in enumerate(joints):
        lower, upper = PANDA_JOINT_LIMITS[index]
        clamped.append(min(upper, max(lower, float(value))))
    return clamped


def within_joint_limits(joints: Sequence[float]) -> bool:
    """Return whether a configuration lies inside the joint limits."""
    if len(joints) != len(PANDA_JOINT_NAMES):
        return False

    for index, value in enumerate(joints):
        lower, upper = PANDA_JOINT_LIMITS[index]
        if float(value) < lower or float(value) > upper:
            return False
    return True


def interpolate_joints(start: Sequence[float], goal: Sequence[float], alpha: float) -> List[float]:
    """Linearly interpolate between two joint configurations."""
    return [
        float(start[index]) + alpha * (float(goal[index]) - float(start[index]))
        for index in range(len(start))
    ]


def joint_distance(start: Sequence[float], goal: Sequence[float]) -> float:
    """Euclidean distance in joint space."""
    return math.sqrt(
        sum((float(goal[index]) - float(start[index])) ** 2 for index in range(len(start)))
    )


def joint_distance_linf(start: Sequence[float], goal: Sequence[float]) -> float:
    """Maximum absolute joint delta."""
    return max(
        abs(float(goal[index]) - float(start[index]))
        for index in range(len(start))
    )


def _vector_add(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [float(a[index]) + float(b[index]) for index in range(len(a))]


def _vector_sub(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [float(a[index]) - float(b[index]) for index in range(len(a))]


def _vector_scale(vector: Sequence[float], scalar: float) -> List[float]:
    return [float(value) * scalar for value in vector]


def _vector_dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[index]) * float(b[index]) for index in range(len(a)))


def _vector_norm(vector: Sequence[float]) -> float:
    return math.sqrt(_vector_dot(vector, vector))


def _mat_mul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [
            sum(float(a[row][k]) * float(b[k][col]) for k in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def _mat_vec_mul(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> List[float]:
    return [
        sum(float(matrix[row][col]) * float(vector[col]) for col in range(3))
        for row in range(3)
    ]


def _identity_matrix() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _rx(angle: float) -> List[List[float]]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [1.0, 0.0, 0.0],
        [0.0, cosine, -sine],
        [0.0, sine, cosine],
    ]


def _ry(angle: float) -> List[List[float]]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ]


def _rz(angle: float) -> List[List[float]]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    return _mat_mul(_mat_mul(_rz(yaw), _ry(pitch)), _rx(roll))


def forward_link_positions(joints: Sequence[float]) -> List[Tuple[float, float, float]]:
    """Compute link origins for a simplified Panda model."""
    if len(joints) != len(PANDA_JOINT_NAMES):
        raise ValueError('Expected 7 Panda joint values.')

    rotation = _identity_matrix()
    translation = [0.0, 0.0, 0.0]
    positions: List[Tuple[float, float, float]] = [tuple(translation)]

    for index, joint in enumerate(joints):
        origin_xyz, origin_rpy = PANDA_JOINT_ORIGINS[index]
        translation = _vector_add(translation, _mat_vec_mul(rotation, origin_xyz))
        rotation = _mat_mul(rotation, _rpy_matrix(*origin_rpy))
        rotation = _mat_mul(rotation, _rz(float(joint)))
        positions.append(tuple(translation))

    tool_position = _vector_add(translation, _mat_vec_mul(rotation, PANDA_TOOL_OFFSET))
    positions.append(tuple(tool_position))
    return positions


def build_sphere_chain(
    joints: Sequence[float],
    resolution: float = 0.08,
) -> List[Tuple[Tuple[float, float, float], float]]:
    """Approximate the robot with a sphere chain."""
    positions = forward_link_positions(joints)
    spheres: List[Tuple[Tuple[float, float, float], float]] = []

    for index in range(len(positions) - 1):
        start = positions[index]
        goal = positions[index + 1]
        delta = _vector_sub(goal, start)
        segment_length = _vector_norm(delta)
        steps = max(1, int(math.ceil(segment_length / max(resolution, 1e-3))))
        radius = SPHERE_CHAIN_RADII[min(index, len(SPHERE_CHAIN_RADII) - 1)]

        for step in range(steps + 1):
            alpha = step / steps
            center = interpolate_joints(start, goal, alpha)
            spheres.append((tuple(center), radius))

    return spheres


def point_to_segment_distance(
    point: Sequence[float],
    seg_a: Sequence[float],
    seg_b: Sequence[float],
) -> float:
    """Compute the minimum distance from a point to a line segment."""
    segment = _vector_sub(seg_b, seg_a)
    denominator = _vector_dot(segment, segment)
    if denominator <= 1e-12:
        return _vector_norm(_vector_sub(point, seg_a))

    ratio = _vector_dot(_vector_sub(point, seg_a), segment) / denominator
    ratio = min(1.0, max(0.0, ratio))
    closest = _vector_add(seg_a, _vector_scale(segment, ratio))
    return _vector_norm(_vector_sub(point, closest))


def sphere_to_capsule_clearance(
    center: Sequence[float],
    radius: float,
    capsule: Capsule,
) -> float:
    """Compute signed clearance between a sphere and a capsule."""
    center_distance = point_to_segment_distance(center, capsule.a, capsule.b)
    return center_distance - float(radius) - float(capsule.radius)


def min_robot_capsule_clearance(
    joints: Sequence[float],
    capsules: Sequence[Capsule],
    resolution: float = 0.08,
) -> float:
    """Return the minimum clearance between the robot and capsules."""
    if not capsules:
        return float('inf')

    minimum = float('inf')
    for center, radius in build_sphere_chain(joints, resolution=resolution):
        for capsule in capsules:
            clearance = sphere_to_capsule_clearance(center, radius, capsule)
            if clearance < minimum:
                minimum = clearance
    return minimum


def is_state_valid(
    joints: Sequence[float],
    obstacles: Sequence[Capsule],
    clearance_margin: float,
    resolution: float = 0.08,
) -> bool:
    """Check limits and capsule clearance for a configuration."""
    if not within_joint_limits(joints):
        return False
    clearance = min_robot_capsule_clearance(joints, obstacles, resolution=resolution)
    return clearance >= clearance_margin


def is_segment_valid(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Sequence[Capsule],
    clearance_margin: float,
    edge_resolution: float,
    sphere_resolution: float = 0.08,
) -> bool:
    """Check a straight-line edge in joint space."""
    steps = max(1, int(math.ceil(joint_distance(start, goal) / max(edge_resolution, 1e-3))))
    for step in range(steps + 1):
        alpha = step / steps
        sample = interpolate_joints(start, goal, alpha)
        if not is_state_valid(
            sample,
            obstacles,
            clearance_margin=clearance_margin,
            resolution=sphere_resolution,
        ):
            return False
    return True


def path_min_clearance(
    path: Sequence[Sequence[float]],
    obstacles: Sequence[Capsule],
    edge_resolution: float,
    sphere_resolution: float = 0.08,
) -> float:
    """Return the minimum clearance along a path."""
    if not path:
        return float('inf')
    if not obstacles:
        return float('inf')

    minimum = float('inf')
    for index in range(len(path)):
        current = min_robot_capsule_clearance(
            path[index],
            obstacles,
            resolution=sphere_resolution,
        )
        minimum = min(minimum, current)
        if index == len(path) - 1:
            continue

        steps = max(
            1,
            int(
                math.ceil(
                    joint_distance(path[index], path[index + 1])
                    / max(edge_resolution, 1e-3)
                )
            ),
        )
        for step in range(1, steps):
            alpha = step / steps
            sample = interpolate_joints(path[index], path[index + 1], alpha)
            current = min_robot_capsule_clearance(
                sample,
                obstacles,
                resolution=sphere_resolution,
            )
            minimum = min(minimum, current)
    return minimum


def _sample_random_configuration(rng: random.Random) -> List[float]:
    return [
        rng.uniform(lower, upper)
        for lower, upper in PANDA_JOINT_LIMITS
    ]


def _nearest_node_index(tree: Sequence[_TreeNode], sample: Sequence[float]) -> int:
    return min(
        range(len(tree)),
        key=lambda index: joint_distance(tree[index].state, sample),
    )


def _steer_towards(
    start: Sequence[float],
    goal: Sequence[float],
    step_size: float,
) -> List[float]:
    distance = joint_distance(start, goal)
    if distance <= step_size:
        return list(goal)
    alpha = step_size / max(distance, 1e-9)
    return interpolate_joints(start, goal, alpha)


def _extend_tree(
    tree: List[_TreeNode],
    target: Sequence[float],
    obstacles: Sequence[Capsule],
    clearance_margin: float,
    step_size: float,
    edge_resolution: float,
    sphere_resolution: float,
) -> Tuple[Optional[int], bool]:
    nearest_index = _nearest_node_index(tree, target)
    nearest_state = tree[nearest_index].state
    new_state = _steer_towards(nearest_state, target, step_size)

    if not is_segment_valid(
        nearest_state,
        new_state,
        obstacles,
        clearance_margin=clearance_margin,
        edge_resolution=edge_resolution,
        sphere_resolution=sphere_resolution,
    ):
        return None, False

    tree.append(_TreeNode(state=list(new_state), parent=nearest_index))
    new_index = len(tree) - 1

    if joint_distance(new_state, target) <= 1e-6:
        return new_index, True

    if joint_distance(new_state, target) <= step_size and is_segment_valid(
        new_state,
        target,
        obstacles,
        clearance_margin=clearance_margin,
        edge_resolution=edge_resolution,
        sphere_resolution=sphere_resolution,
    ):
        tree.append(_TreeNode(state=list(target), parent=new_index))
        return len(tree) - 1, True

    return new_index, False


def _connect_tree(
    tree: List[_TreeNode],
    target: Sequence[float],
    obstacles: Sequence[Capsule],
    clearance_margin: float,
    step_size: float,
    edge_resolution: float,
    sphere_resolution: float,
) -> Tuple[Optional[int], bool]:
    last_index: Optional[int] = None
    while True:
        new_index, reached = _extend_tree(
            tree,
            target,
            obstacles,
            clearance_margin=clearance_margin,
            step_size=step_size,
            edge_resolution=edge_resolution,
            sphere_resolution=sphere_resolution,
        )
        if new_index is None:
            return last_index, False
        last_index = new_index
        if reached:
            return new_index, True


def _reconstruct_branch(tree: Sequence[_TreeNode], node_index: int) -> List[List[float]]:
    branch: List[List[float]] = []
    current_index: Optional[int] = node_index
    while current_index is not None:
        node = tree[current_index]
        branch.append(list(node.state))
        current_index = node.parent
    branch.reverse()
    return branch


def _merge_solution(
    start_tree: Sequence[_TreeNode],
    start_index: int,
    goal_tree: Sequence[_TreeNode],
    goal_index: int,
) -> List[List[float]]:
    start_branch = _reconstruct_branch(start_tree, start_index)
    goal_branch = _reconstruct_branch(goal_tree, goal_index)
    goal_branch.reverse()
    return start_branch + goal_branch[1:]


def plan_joint_path(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Sequence[Capsule],
    clearance_margin: float = 0.02,
    max_iterations: int = 3000,
    step_size: float = 0.25,
    edge_resolution: float = 0.08,
    sphere_resolution: float = 0.08,
    goal_bias: float = 0.2,
    seed: int = 7,
) -> Optional[List[List[float]]]:
    """Plan a collision-aware joint path with direct-connect + RRT-Connect."""
    start_state = clamp_joints(start)
    goal_state = clamp_joints(goal)

    if not is_state_valid(
        start_state,
        obstacles,
        clearance_margin=clearance_margin,
        resolution=sphere_resolution,
    ):
        return None
    if not is_state_valid(
        goal_state,
        obstacles,
        clearance_margin=clearance_margin,
        resolution=sphere_resolution,
    ):
        return None

    if is_segment_valid(
        start_state,
        goal_state,
        obstacles,
        clearance_margin=clearance_margin,
        edge_resolution=edge_resolution,
        sphere_resolution=sphere_resolution,
    ):
        return [start_state, goal_state]

    rng = random.Random(seed)
    tree_a = [_TreeNode(state=start_state, parent=None)]
    tree_b = [_TreeNode(state=goal_state, parent=None)]
    tree_a_is_start = True

    for _ in range(max_iterations):
        if rng.random() < goal_bias:
            sample = goal_state if tree_a_is_start else start_state
        else:
            sample = _sample_random_configuration(rng)

        new_index, _ = _extend_tree(
            tree_a,
            sample,
            obstacles,
            clearance_margin=clearance_margin,
            step_size=step_size,
            edge_resolution=edge_resolution,
            sphere_resolution=sphere_resolution,
        )
        if new_index is not None:
            connect_index, connected = _connect_tree(
                tree_b,
                tree_a[new_index].state,
                obstacles,
                clearance_margin=clearance_margin,
                step_size=step_size,
                edge_resolution=edge_resolution,
                sphere_resolution=sphere_resolution,
            )
            if connected and connect_index is not None:
                if tree_a_is_start:
                    return _merge_solution(tree_a, new_index, tree_b, connect_index)
                return _merge_solution(tree_b, connect_index, tree_a, new_index)

        tree_a, tree_b = tree_b, tree_a
        tree_a_is_start = not tree_a_is_start

    return None


def shortcut_path(
    path: Sequence[Sequence[float]],
    obstacles: Sequence[Capsule],
    clearance_margin: float = 0.02,
    edge_resolution: float = 0.08,
    sphere_resolution: float = 0.08,
    attempts: int = 120,
    seed: int = 7,
) -> List[List[float]]:
    """Shortcut a path while preserving validity."""
    shortcut = [list(point) for point in path]
    if len(shortcut) < 3:
        return shortcut

    rng = random.Random(seed)
    for _ in range(attempts):
        if len(shortcut) < 3:
            break
        left = rng.randint(0, len(shortcut) - 3)
        right = rng.randint(left + 2, len(shortcut) - 1)
        if is_segment_valid(
            shortcut[left],
            shortcut[right],
            obstacles,
            clearance_margin=clearance_margin,
            edge_resolution=edge_resolution,
            sphere_resolution=sphere_resolution,
        ):
            shortcut = shortcut[:left + 1] + shortcut[right:]

    return shortcut


def resample_path(
    path: Sequence[Sequence[float]],
    max_joint_step: float = 0.12,
) -> List[List[float]]:
    """Densify a path for execution."""
    if not path:
        return []

    dense_path: List[List[float]] = [list(path[0])]
    for index in range(len(path) - 1):
        start = path[index]
        goal = path[index + 1]
        steps = max(
            1,
            int(
                math.ceil(
                    joint_distance_linf(start, goal) / max(max_joint_step, 1e-3)
                )
            ),
        )
        for step in range(1, steps + 1):
            alpha = step / steps
            dense_path.append(interpolate_joints(start, goal, alpha))
    return dense_path


def nearest_path_index(path: Sequence[Sequence[float]], current: Sequence[float]) -> int:
    """Return the index of the nearest path state."""
    return min(
        range(len(path)),
        key=lambda index: joint_distance(path[index], current),
    )
