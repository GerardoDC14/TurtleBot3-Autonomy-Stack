"""Exercise the actual sensor, POI and mechanism callbacks with message doubles."""

from collections import deque
from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest

import yaml

from support import ROOT, Logger, Point, PoseStamped, Publisher, callbacks


class ScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adjust = staticmethod(callbacks(
            'src/laser_scan_adjuster/laser_scan_adjuster/adjust_scan.py', {'math': math})['adjust_scan'])

    def scan(self, count=360, intensities=True):
        return NS(ranges=list(range(count)), intensities=list(range(count)) if intensities else [],
                  angle_min=0.0, angle_increment=math.pi/180, angle_max=(count-1)*math.pi/180,
                  time_increment=0.001, scan_time=0.36)

    def test_default_preserves_all_measurements(self):
        original = self.scan()
        self.assertEqual(self.adjust(original).ranges, original.ranges)

    def test_decimation_keeps_intensities_angles_and_times_aligned(self):
        original = self.scan()
        result = self.adjust(original, max_samples=227)
        self.assertEqual(result.ranges, original.ranges[::2])
        self.assertEqual(result.intensities, original.intensities[::2])
        self.assertAlmostEqual(result.angle_max, original.angle_min+(len(result.ranges)-1)*result.angle_increment)
        self.assertAlmostEqual(result.time_increment, 0.002)
        self.assertEqual(len(original.ranges), 360)

    def test_short_scans_are_never_padded(self):
        self.assertEqual(self.adjust(self.scan(2), 227).ranges, [0, 1])

    def test_invalid_empty_or_inconsistent_input_is_rejected(self):
        for message in (self.scan(0), self.scan()):
            if message.ranges:
                message.angle_max += 0.1
            with self.assertRaises(ValueError):
                self.adjust(message)
        message = self.scan()
        message.intensities.pop()
        with self.assertRaises(ValueError):
            self.adjust(message)

    def test_negative_increment_and_missing_intensities_are_supported(self):
        message = self.scan(10, intensities=False)
        message.angle_increment *= -1
        message.angle_max *= -1
        result = self.adjust(message, 4)
        self.assertEqual(result.ranges, [0, 3, 6, 9])
        self.assertEqual(result.intensities, [])

    def test_invalid_range_returns_are_not_fabricated(self):
        message = self.scan(3)
        message.ranges = [math.inf, math.nan, 0.1]
        result = self.adjust(message)
        self.assertTrue(math.isinf(result.ranges[0]))
        self.assertTrue(math.isnan(result.ranges[1]))


class POITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scope = callbacks('src/turtlebot3_poi_navigation/turtlebot3_poi_navigation/poi_manager.py',
                          {'math': math, 'Path': Path, 'os': os, 'tempfile': tempfile, 'yaml': yaml})
        cls.POI, cls.Manager = scope['POI'], scope['POIManager']

    def node(self):
        node = object.__new__(self.Manager)
        node.get_logger = lambda: Logger()
        node.map_frame = 'map'
        node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1)))
        node.marker_pub = Publisher()
        node.pois = {'lab': self.POI('lab', 1.0, 2.0, 0.5)}
        return node

    def test_markers_do_not_move_stored_destinations(self):
        node = self.node()
        for _ in range(20):
            node.publish_markers()
        self.assertEqual(node.pois['lab'].position.z, 0.0)
        self.assertEqual(node.marker_pub.messages[-1].markers[-1].pose.position.z, 0.2)

    def test_deleted_pois_clear_old_markers(self):
        node = self.node()
        node.publish_markers()
        node.pois.clear()
        node.publish_markers()
        self.assertEqual(len(node.marker_pub.messages[-1].markers), 1)
        self.assertEqual(node.marker_pub.messages[-1].markers[0].action, 3)

    def test_invalid_pois_are_rejected(self):
        for values in (('', 0, 0, 1), ('lab', math.nan, 0, 1), ('lab', 0, 0, -1)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.POI(*values)

    def test_storage_round_trip_and_invalid_file_leave_existing_state(self):
        node = self.node()
        with tempfile.TemporaryDirectory(dir=ROOT.parent) as folder:
            path = Path(folder) / 'subdir' / 'pois.yaml'
            self.assertTrue(node.save_pois(path))
            node.pois.clear()
            node.load_pois(path)
            self.assertEqual(node.pois['lab'].position.x, 1.0)
            path.write_text('broken: [', encoding='utf-8')
            node.load_pois(path)
            self.assertIn('lab', node.pois)


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __sub__(self, other):
        return NS(nanoseconds=int((self.seconds-other.seconds)*1e9))

    @staticmethod
    def from_msg(stamp):
        return FakeTime(stamp.sec+stamp.nanosec/1e9)


class HazmatTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.MarkerNode = callbacks('src/hazmat_marker/hazmat_marker/hazmat_marker.py',
            {'math': math, 'deque': deque, 'rclpy': NS(time=NS(Time=FakeTime)),
             'TransformException': RuntimeError})['HazmatMarkerNode']

    def node(self):
        node = object.__new__(self.MarkerNode)
        node.get_logger = lambda: Logger()
        node.get_clock = lambda: NS(now=lambda: FakeTime(10.0))
        node.max_pose_age, node.min_separation, node.map_frame = 0.5, 0.5, 'map'
        node.observations, node.next_id = deque(maxlen=2), 0
        node.marker_pub = Publisher()
        node.latest_odom = NS(header=NS(frame_id='odom', stamp=NS(sec=10, nanosec=0)),
                              pose=NS(pose=PoseStamped().pose))
        self.transforms = []

        def transform(pose, frame):
            self.transforms.append((deepcopy(pose), frame))
            pose = deepcopy(pose)
            pose.header.frame_id = frame
            pose.pose.position.x += 5.0
            return pose
        node.tf_buffer = NS(transform=transform)
        return node

    def test_observation_uses_timestamped_transform_and_unique_marker_keys(self):
        node = self.node()
        node.hazmat_callback(NS(data='flammable'))
        self.assertEqual(self.transforms[0][0].header.frame_id, 'odom')
        self.assertEqual(self.transforms[0][0].header.stamp.sec, 10)
        markers = node.marker_pub.messages[-1].markers[1:]
        self.assertEqual(markers[0].header.frame_id, 'map')
        self.assertEqual(markers[0].pose.position.x, 5.0)
        self.assertNotEqual((markers[0].ns, markers[0].id), (markers[1].ns, markers[1].id))

    def test_odometry_alone_cannot_repeat_old_detection(self):
        node = self.node()
        node.hazmat_callback(NS(data='flammable'))
        node.odom_callback(node.latest_odom)
        node.hazmat_callback(NS(data='flammable'))
        self.assertEqual(len(node.marker_pub.messages), 1)

    def test_stale_odometry_is_discarded(self):
        node = self.node()
        node.latest_odom.header.stamp.sec = 8
        node.hazmat_callback(NS(data='flammable'))
        self.assertEqual(node.marker_pub.messages, [])

    def test_missing_transform_is_discarded(self):
        node = self.node()
        node.tf_buffer.transform = lambda *args: (_ for _ in ()).throw(RuntimeError('no TF'))
        node.hazmat_callback(NS(data='flammable'))
        self.assertEqual(node.marker_pub.messages, [])

    def test_snapshots_are_bounded(self):
        node = self.node()
        for index in range(4):
            node.hazmat_callback(NS(data=f'class-{index}'))
        self.assertEqual(len(node.observations), 2)
        self.assertEqual(len(node.marker_pub.messages[-1].markers), 5)


class MotorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from motor_position_controller.limits import clamp_position
        cls.Motor = callbacks(
            'src/motor_position_controller/motor_position_controller/motor_position_controller_node.py',
            {'clamp_position': clamp_position, 'Float32': lambda **values: NS(**values)})['MotorPositionControllerNode']

    def node(self):
        node = object.__new__(self.Motor)
        node.get_logger = lambda: Logger()
        node.limits = ((-1, 1), (-2, 2))
        node.positions, node.enabled = [0.0, 0.0], [False, False]
        node.lock = threading.Lock()
        node.publishers_by_axis = [Publisher(), Publisher()]
        return node

    def test_startup_does_not_command_hardware(self):
        node = self.node()
        node.timer_callback()
        self.assertTrue(all(not publisher.messages for publisher in node.publishers_by_axis))

    def test_both_axes_clamp_to_their_own_bounds(self):
        node = self.node()
        node.set_position('both', 5.0)
        node.timer_callback()
        self.assertEqual([p.messages[-1].data for p in node.publishers_by_axis], [1, 2])

    def test_invalid_command_cannot_partially_update_axes(self):
        node = self.node()
        with self.assertRaises(ValueError):
            node.set_position('both', math.nan)
        self.assertEqual(node.enabled, [False, False])
