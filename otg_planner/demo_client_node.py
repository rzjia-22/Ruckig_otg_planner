"""Demo publisher that alternates between two Panda joint goals."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .planning_core import DEFAULT_POINT_A, DEFAULT_POINT_B, PANDA_JOINT_NAMES


class DemoGoalClientNode(Node):
    """Publish alternating joint-space goals to the planner."""

    def __init__(self):
        super().__init__('otg_demo_client_node')

        self.declare_parameter('point_a', DEFAULT_POINT_A)
        self.declare_parameter('point_b', DEFAULT_POINT_B)
        self.declare_parameter('startup_delay', 2.0)
        self.declare_parameter('pause_between_goals', 0.5)
        self.declare_parameter('path_scale', 1.0)
        self.declare_parameter('first_target', 'point_b')

        self.point_a = [float(value) for value in self.get_parameter('point_a').value]
        self.point_b = [float(value) for value in self.get_parameter('point_b').value]
        self.path_scale = max(0.05, float(self.get_parameter('path_scale').value))
        self.pause_between_goals = max(
            0.0,
            float(self.get_parameter('pause_between_goals').value),
        )
        self.next_target = str(self.get_parameter('first_target').value)

        self.goal_pub = self.create_publisher(JointTrajectory, '/otg/goal_trajectory', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        self._latest_joint_positions = None
        self._last_sent_target = None

        self.start_timer = self.create_timer(
            max(0.1, float(self.get_parameter('startup_delay').value)),
            self._kickoff,
        )
        self._delayed_timer = None

        self.get_logger().info(f'Demo client started. first_target={self.next_target}')

    def _joint_state_cb(self, msg: JointState) -> None:
        if set(msg.name) != set(PANDA_JOINT_NAMES):
            return
        index_by_name = {name: index for index, name in enumerate(msg.name)}
        self._latest_joint_positions = [
            float(msg.position[index_by_name[name]])
            for name in PANDA_JOINT_NAMES
        ]

    def _kickoff(self) -> None:
        self.start_timer.cancel()
        self._send_next_goal()

    def _schedule_once(self, delay_seconds: float, callback) -> None:
        if self._delayed_timer is not None:
            self._delayed_timer.cancel()
            self.destroy_timer(self._delayed_timer)
            self._delayed_timer = None

        def _wrapped() -> None:
            if self._delayed_timer is not None:
                self._delayed_timer.cancel()
                self.destroy_timer(self._delayed_timer)
                self._delayed_timer = None
            callback()

        self._delayed_timer = self.create_timer(max(0.05, delay_seconds), _wrapped)

    def _target_positions(self):
        if self.next_target == 'point_a':
            return list(self.point_a)
        return [
            self.point_a[index]
            + self.path_scale * (self.point_b[index] - self.point_a[index])
            for index in range(len(self.point_a))
        ]

    def _send_next_goal(self) -> None:
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for planner goal subscription...')
            self._schedule_once(0.5, self._send_next_goal)
            return

        target = self._target_positions()
        self._last_sent_target = list(target)

        msg = JointTrajectory()
        msg.joint_names = list(PANDA_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = list(target)
        msg.points.append(point)

        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published demo goal: {self.next_target}')

        self._schedule_once(max(1.0, self.pause_between_goals + 2.0), self._check_goal_progress)

    def _check_goal_progress(self) -> None:
        if self._latest_joint_positions is None or self._last_sent_target is None:
            self._schedule_once(0.5, self._check_goal_progress)
            return

        max_error = max(
            abs(self._latest_joint_positions[index] - self._last_sent_target[index])
            for index in range(len(self._last_sent_target))
        )
        if max_error > 0.08:
            self._schedule_once(0.5, self._check_goal_progress)
            return

        self.next_target = 'point_a' if self.next_target == 'point_b' else 'point_b'
        self._schedule_once(self.pause_between_goals, self._send_next_goal)


def main(args=None) -> None:
    """Run the demo publisher."""
    rclpy.init(args=args)
    node = DemoGoalClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
