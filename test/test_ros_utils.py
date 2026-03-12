"""Unit tests for ROS-facing planner helpers."""

from sensor_msgs.msg import JointState

from otg_planner.ros_utils import joint_motion_from_msg, path_lookahead_target


def test_joint_motion_from_msg_reorders_positions_and_velocities():
    msg = JointState()
    msg.name = ['joint_b', 'joint_a']
    msg.position = [2.0, 1.0]
    msg.velocity = [0.2, 0.1]

    motion = joint_motion_from_msg(msg, ['joint_a', 'joint_b'])

    assert motion is not None
    positions, velocities = motion
    assert positions == [1.0, 2.0]
    assert velocities == [0.1, 0.2]


def test_path_lookahead_target_advances_along_path():
    path = [
        [0.0, 0.0],
        [0.1, 0.0],
        [0.2, 0.0],
        [0.5, 0.0],
    ]

    target, target_index = path_lookahead_target(
        path,
        current_positions=[0.05, 0.0],
        lookahead_distance=0.22,
    )

    assert target_index == 2
    assert target == [0.2, 0.0]
