#!/usr/bin/env python3
"""Select free frontier goals and supervise one asynchronous Nav2 action."""

import math
import time

from autonomy_common.navigation import NavigationTask
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from turtlebot3_explore.frontiers import Grid


class Explore(Node):
    """Explore with bounded retries and explicit ownership of each goal."""

    def __init__(self):
        super().__init__('explore_node')
        defaults = {
            'info_radius': 1.0, 'frontier_visit_threshold': 0.3,
            'max_attempts': 3, 'timer_interval': 2.0,
            'min_frontier_size': 5, 'minimum_goal_distance': 0.3,
            'clearance_radius': 0.2, 'free_threshold': 25,
            'distance_weight': 0.5, 'navigation_timeout': 120.0,
            'retry_cooldown': 10.0, 'map_frame': 'map', 'robot_frame': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        for name in ('info_radius', 'frontier_visit_threshold', 'timer_interval',
                     'navigation_timeout'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name in ('minimum_goal_distance', 'clearance_radius', 'distance_weight', 'retry_cooldown'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be nonnegative and finite')
        if self.max_attempts < 1 or self.min_frontier_size < 1 or not 0 <= self.free_threshold <= 100:
            raise ValueError('invalid retry, cluster-size or occupancy threshold')
        self.grid = None
        self.visited_frontiers = []
        self.frontier_selection_attempts = {}
        self.retry_after = {}
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation = NavigationTask(
            self.action_client, NavigateToPose.Goal, self.navigation_timeout,
            self.navigation_finished, notify=self.get_logger().warning)
        self.frontier_pub = self.create_publisher(MarkerArray, 'frontiers_individuales', 10)
        self.timer = self.create_timer(self.timer_interval, self.exploration_cycle)
        self.watchdog = self.create_timer(0.2, self.navigation.tick)
        self.stopping = False

    def map_callback(self, message):
        """Accept only complete maps in the configured global frame."""
        try:
            if message.header.frame_id != self.map_frame:
                raise ValueError(f'expected frame {self.map_frame!r}')
            self.grid = Grid(message, self.free_threshold)
        except ValueError as exc:
            self.get_logger().warning(f'Map discarded: {exc}', throttle_duration_sec=5.0)

    def round_coord(self, coord):
        return tuple(round(float(value), 2) for value in coord)

    def filter_frontiers(self, coords):
        """Apply retry limits even before the first successful goal."""
        now = time.monotonic()
        return [coord for coord in coords
                if self.frontier_selection_attempts.get(self.round_coord(coord), 0) < self.max_attempts
                and now >= self.retry_after.get(self.round_coord(coord), 0.0)
                and not any(math.dist(coord, old) < self.frontier_visit_threshold
                            for old in self.visited_frontiers)]

    def exploration_cycle(self):
        if self.stopping or self.navigation.busy or self.grid is None:
            return
        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose', throttle_duration_sec=10.0)
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warning('Robot transform unavailable', throttle_duration_sec=5.0)
            return
        position = transform.transform.translation
        current = (position.x, position.y)
        if not all(math.isfinite(value) for value in current):
            self.get_logger().warning('Discarding nonfinite robot pose', throttle_duration_sec=5.0)
            return
        grid = self.grid
        coords = self.filter_frontiers(grid.candidates(self.min_frontier_size, self.clearance_radius))
        coords = [coord for coord in coords if math.dist(coord, current) >= self.minimum_goal_distance]
        self.publish_frontiers(coords)
        if not coords:
            self.get_logger().info('No eligible frontier in the current map', throttle_duration_sec=10.0)
            return
        selected = max(coords, key=lambda coord: grid.information_gain(coord, self.info_radius)
                       - self.distance_weight * math.dist(coord, current))
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = selected
        yaw = math.atan2(selected[1]-current[1], selected[0]-current[0])
        goal.pose.orientation = Quaternion(z=math.sin(yaw/2), w=math.cos(yaw/2))
        key = self.round_coord(selected)
        self.frontier_selection_attempts[key] = self.frontier_selection_attempts.get(key, 0) + 1
        if not self.navigation.begin(goal):
            self.frontier_selection_attempts[key] -= 1

    def navigation_finished(self, outcome):
        coord = (outcome.pose.pose.position.x, outcome.pose.pose.position.y)
        if outcome.status == 'SUCCEEDED':
            self.visited_frontiers.append(coord)
        else:
            self.retry_after[self.round_coord(coord)] = time.monotonic() + self.retry_cooldown
        self.get_logger().info(f'Navigation {outcome.status}: {outcome.detail}')

    def publish_frontiers(self, coords):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, (x, y) in enumerate(coords):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'frontiers', index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = x, y
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.r = marker.color.a = 1.0
            markers.markers.append(marker)
        self.frontier_pub.publish(markers)

    def stop(self):
        """Cancel this mission's action without shutting down the Nav2 stack."""
        self.stopping = True
        self.navigation.cancel()
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and self.navigation.busy and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.navigation.busy:
            self.get_logger().error('Cancellation unconfirmed; verify the base is stopped')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Explore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
