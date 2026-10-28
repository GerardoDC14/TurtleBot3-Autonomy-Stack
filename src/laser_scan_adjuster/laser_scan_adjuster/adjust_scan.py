"""Republish valid scans, optionally reducing their sampling density."""

from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def adjust_scan(message, max_samples=0):
    """Decimate rays together with their angles, times and intensities.

    A zero limit preserves every ray. Short scans are never padded. Invalid
    geometry raises ValueError; NaN/inf range returns retain their sensor meaning.
    """
    count = len(message.ranges)
    if count == 0:
        raise ValueError('empty scan')
    if max_samples < 0 or max_samples == 1:
        raise ValueError('max_samples must be zero or at least two')
    if message.intensities and len(message.intensities) != count:
        raise ValueError('intensities must be empty or match ranges')
    values = (message.angle_min, message.angle_max, message.angle_increment,
              message.time_increment, message.scan_time)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite scan geometry')
    if message.time_increment < 0 or message.scan_time < 0:
        raise ValueError('negative scan timing')
    if count > 1 and message.angle_increment == 0:
        raise ValueError('zero angular increment')
    last_angle = message.angle_min + (count - 1) * message.angle_increment
    tolerance = max(1e-5, abs(message.angle_increment) * 0.01)
    if not math.isclose(message.angle_max, last_angle, abs_tol=tolerance):
        raise ValueError('angular span does not match the measured ray count')
    result = deepcopy(message)
    stride = math.ceil(count / max_samples) if max_samples else 1
    result.ranges = message.ranges[::stride]
    if message.intensities:
        result.intensities = message.intensities[::stride]
    result.angle_increment = message.angle_increment * stride
    result.time_increment = message.time_increment * stride
    result.angle_max = message.angle_min + (len(result.ranges) - 1) * result.angle_increment
    return result


class LaserScanAdjuster(Node):
    """Keep the original scan geometry unless decimation is requested."""

    def __init__(self):
        super().__init__('laser_scan_adjuster')
        self.declare_parameter('max_samples', 0)
        self.max_samples = self.get_parameter('max_samples').value
        if self.max_samples < 0 or self.max_samples == 1:
            raise ValueError('max_samples must be zero or at least two')
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            LaserScan, 'adjusted_scan', qos_profile_sensor_data)

    def scan_callback(self, message):
        """Drop malformed input rather than manufacturing measurements."""
        try:
            self.publisher.publish(adjust_scan(message, self.max_samples))
        except ValueError as exc:
            self.get_logger().warning(f'Scan discarded: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanAdjuster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
