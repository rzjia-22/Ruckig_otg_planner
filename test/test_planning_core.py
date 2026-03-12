"""Unit tests for the joint-space planning core."""

from otg_planner.planning_core import (
    Capsule,
    DEFAULT_POINT_A,
    DEFAULT_POINT_B,
    joint_distance_linf,
    min_robot_capsule_clearance,
    plan_joint_path,
    resample_path,
)


def test_plan_without_obstacles_returns_direct_path():
    path = plan_joint_path(DEFAULT_POINT_A, DEFAULT_POINT_B, [])

    assert path is not None
    assert len(path) == 2
    assert path[0] == DEFAULT_POINT_A
    assert path[-1] == DEFAULT_POINT_B


def test_resample_path_limits_joint_step():
    path = resample_path([DEFAULT_POINT_A, DEFAULT_POINT_B], max_joint_step=0.05)

    assert len(path) > 2
    for index in range(len(path) - 1):
        assert joint_distance_linf(path[index], path[index + 1]) <= 0.050001


def test_static_obstacle_reduces_clearance():
    obstacle = Capsule(
        a=(0.18, 0.10, 0.445),
        b=(0.18, 0.10, 0.795),
        radius=0.08,
        obstacle_id='static',
    )

    clearance = min_robot_capsule_clearance(DEFAULT_POINT_B, [obstacle])

    assert clearance < 0.20


def test_planner_finds_detour_around_static_obstacle():
    obstacle = Capsule(
        a=(-0.1, -0.2, 0.25),
        b=(-0.1, -0.2, 0.55),
        radius=0.07,
        obstacle_id='static',
    )

    path = plan_joint_path(
        DEFAULT_POINT_A,
        DEFAULT_POINT_B,
        [obstacle],
        clearance_margin=0.03,
        max_iterations=5000,
        step_size=0.25,
        edge_resolution=0.08,
        sphere_resolution=0.08,
        goal_bias=0.2,
        seed=11,
    )

    assert path is not None
    assert len(path) >= 2
