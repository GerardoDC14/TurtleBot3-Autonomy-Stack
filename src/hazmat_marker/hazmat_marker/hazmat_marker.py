"""Record robot observation locations in the map frame."""

from collections import deque
from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401: register PoseStamped transformations
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray


class HazmatMarkerNode(Node):
    """Annotate receipt-time observations; do not infer an object's position."""

    def __init__(self):
        super().__init__('hazmat_marker_node')
        for name, value in (('map_frame', 'map'), ('max_pose_age', 0.5),
                            ('min_separation', 0.5), ('max_observations', 200)):
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if (not math.isfinite(self.max_pose_age) or self.max_pose_age <= 0
                or not math.isfinite(self.min_separation) or self.min_separation < 0
                or self.max_observations < 1):
            raise ValueError('invalid observation limits')
        self.latest_odom = None
        self.observations = deque(maxlen=self.max_observations)
        self.next_id = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'hazmat_markers', qos)
        self.hazmat_sub = self.create_subscription(
            String, 'hazmat_publisher', self.hazmat_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, message):
        """Cache pose only; stale detection flags cannot generate markers."""
        self.latest_odom = message

    def hazmat_callback(self, message):
        name = message.data.strip()
        if not name or name == 'No detections' or self.latest_odom is None:
            return
        odom = self.latest_odom
        stamp = rclpy.time.Time.from_msg(odom.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0 <= age <= self.max_pose_age:
            self.get_logger().warning('Observation discarded: odometry is stale or in the future',
                                      throttle_duration_sec=5.0)
            return
        observation = PoseStamped()
        observation.header = deepcopy(odom.header)
        observation.pose = deepcopy(odom.pose.pose)
        try:
            # No blocking wait: the executor must remain available for TF updates.
            mapped = self.tf_buffer.transform(observation, self.map_frame)
        except TransformException as exc:
            self.get_logger().warning(f'Observation discarded: {exc}', throttle_duration_sec=5.0)
            return
        coord = (mapped.pose.position.x, mapped.pose.position.y)
        if not all(math.isfinite(value) for value in coord):
            return
        if any(old_name == name and math.dist(coord, (pose.pose.position.x, pose.pose.position.y))
               <= self.min_separation for _, old_name, pose in self.observations):
            return
        self.observations.append((self.next_id, name, mapped))
        self.next_id = (self.next_id + 1) % 2147483647
        self.publish_observations()

    def publish_observations(self):
        """Publish a bounded, latched snapshot with distinct text/sphere keys."""
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, name, pose in self.observations:
            for namespace, marker_type in (('hazmat_labels', Marker.TEXT_VIEW_FACING),
                                           ('hazmat_observations', Marker.SPHERE)):
                marker = Marker()
                marker.header = deepcopy(pose.header)
                marker.ns, marker.id = namespace, index
                marker.type, marker.action = marker_type, Marker.ADD
                marker.pose.position = deepcopy(pose.pose.position)
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                if marker_type == Marker.TEXT_VIEW_FACING:
                    marker.pose.position.z += 1.0
                    marker.scale.z, marker.color.r = 0.2, 1.0
                    marker.text = f'Observed: {name}'
                else:
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
                    marker.color.g = 1.0
                markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = HazmatMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
