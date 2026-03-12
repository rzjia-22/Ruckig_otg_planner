"""Publish simple obstacle scenarios for planner demos."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleScenarioPublisher(Node):
    """Publish static or moving obstacle markers."""

    def __init__(self):
        super().__init__('obstacle_scenario_publisher')

        self.declare_parameter('scenario', 'static')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('duration', 12.0)
        self.declare_parameter('appear_delay', 2.0)

        self.scenario = str(self.get_parameter('scenario').value)
        self.rate = max(1.0, float(self.get_parameter('rate').value))
        self.duration = max(1.0, float(self.get_parameter('duration').value))
        self.appear_delay = max(0.0, float(self.get_parameter('appear_delay').value))

        self.publisher = self.create_publisher(MarkerArray, '/otg/obstacles', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(
            f'Obstacle scenario started. scenario={self.scenario} '
            f'rate={self.rate:.1f}Hz duration={self.duration:.1f}s'
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _publish_clear(self) -> None:
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self.publisher.publish(msg)

    def _build_capsule_marker(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        radius: float,
        length: float,
        marker_id: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'otg_obstacles'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)

        marker.scale.x = float(radius * 2.0)
        marker.scale.y = float(radius * 2.0)
        marker.scale.z = float(length)

        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        return marker

    def _publish_static(self, _elapsed: float) -> None:
        msg = MarkerArray()
        msg.markers.append(
            self._build_capsule_marker(
                x=0.18,
                y=0.10,
                z=0.62,
                yaw=0.0,
                radius=0.08,
                length=0.35,
                marker_id=1,
            )
        )
        self.publisher.publish(msg)

    def _publish_moving(self, elapsed: float) -> None:
        phase = elapsed * 0.8
        msg = MarkerArray()
        msg.markers.append(
            self._build_capsule_marker(
                x=0.15 + 0.05 * math.cos(phase),
                y=0.25 * math.sin(phase),
                z=0.62,
                yaw=0.4 * math.sin(phase),
                radius=0.08,
                length=0.35,
                marker_id=1,
            )
        )
        self.publisher.publish(msg)

    def _publish_mixed(self, elapsed: float) -> None:
        if elapsed < 5.0:
            self._publish_static(elapsed)
        else:
            self._publish_moving(elapsed)

    def _on_timer(self) -> None:
        elapsed = self._elapsed()

        if elapsed < self.appear_delay:
            self._publish_clear()
            return

        active_elapsed = elapsed - self.appear_delay
        if active_elapsed > self.duration:
            self._publish_clear()
            self.get_logger().info('Scenario completed. Clearing obstacles and stopping node.')
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.scenario == 'static':
            self._publish_static(active_elapsed)
        elif self.scenario == 'moving':
            self._publish_moving(active_elapsed)
        else:
            self._publish_mixed(active_elapsed)


def main(args=None) -> None:
    """Run the obstacle scenario publisher."""
    rclpy.init(args=args)
    node = ObstacleScenarioPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
