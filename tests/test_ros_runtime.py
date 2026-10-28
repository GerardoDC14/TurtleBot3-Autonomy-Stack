"""Run installed executables against a fake Nav2 server; no robot is connected."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

from support import ROOT

try:
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    HAS_ROS = True
except ImportError:
    if os.environ.get('AUTONOMY_REQUIRE_ROS') == '1':
        raise
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, 'requires a built ROS 2 Humble workspace')
class ROSRuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=['--ros-args', '-p', 'interactive:=false'])
        self.node = Node('test_nav2_server')
        self.mode, self.requests, self.finished, self.canceled = 'success', 0, 0, 0
        self.server = ActionServer(
            self.node, NavigateToPose, 'navigate_to_pose', self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(OccupancyGrid, 'map', qos)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = message.info.height = 12
        message.info.resolution = 0.2
        message.info.origin.position.x = message.info.origin.position.y = -1.2
        message.info.origin.orientation.w = 1.0
        message.data = [-1] * 144
        for row in range(3, 9):
            for col in range(3, 9):
                message.data[row*12+col] = 0
        # Publish once before launching clients to exercise late map subscription.
        self.publisher.publish(message)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.process = None
        self.log = tempfile.TemporaryFile(mode='w+b', dir=ROOT.parent)

    def tearDown(self):
        self.stop_client()
        self.server.destroy()
        self.executor.shutdown(timeout_sec=10.0)
        self.thread.join(timeout=10.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.log.close()

    def goal(self, request):
        self.requests += 1
        if self.mode == 'reject_once' and self.requests == 1:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        result = NavigateToPose.Result()
        if self.mode == 'abort':
            handle.abort()
        elif self.mode == 'hold':
            deadline = time.monotonic() + 8.0
            while rclpy.ok() and not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                handle.canceled()
                self.canceled += 1
            else:
                handle.abort()
        else:
            handle.succeed()
        self.finished += 1
        return result

    def launch_client(self, implementation, timeout=5.0, attempts=3):
        package, executable = (('explore_cpp', 'explore_node') if implementation == 'cpp'
                               else ('turtlebot3_explore', 'explore'))
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        command = [str(path), '--ros-args', '-p', 'timer_interval:=0.1',
                   '-p', 'retry_cooldown:=0.0', '-p', 'minimum_goal_distance:=0.0',
                   '-p', 'clearance_radius:=0.0', '-p', f'navigation_timeout:={timeout}',
                   '-p', f'max_attempts:={attempts}']
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)

    def stop_client(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=5.0)
            self.process = None

    def wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process and self.process.poll() is not None:
                break
            time.sleep(0.02)
        self.log.seek(0)
        self.fail('ROS condition timed out. Client output:\n' +
                  self.log.read().decode('utf-8', errors='replace')[-6000:])

    def reset_counts(self):
        self.requests = self.finished = self.canceled = 0

    def test_success_is_not_selected_again(self):
        for implementation in ('python', 'cpp'):
            with self.subTest(implementation=implementation):
                self.reset_counts()
                self.mode = 'success'
                self.launch_client(implementation)
                self.wait_for(lambda: self.finished >= 1)
                time.sleep(0.8)
                self.assertEqual(self.requests, 1)
                self.stop_client()

    def test_rejection_allows_retry(self):
        for implementation in ('python', 'cpp'):
            with self.subTest(implementation=implementation):
                self.reset_counts()
                self.mode = 'reject_once'
                self.launch_client(implementation)
                self.wait_for(lambda: self.finished >= 1)
                time.sleep(0.5)
                self.assertEqual(self.requests, 2)
                self.stop_client()

    def test_abort_respects_retry_limit_before_any_success(self):
        for implementation in ('python', 'cpp'):
            with self.subTest(implementation=implementation):
                self.reset_counts()
                self.mode = 'abort'
                self.launch_client(implementation, attempts=3)
                self.wait_for(lambda: self.finished >= 3)
                time.sleep(0.8)
                self.assertEqual(self.requests, 3)
                self.stop_client()

    def test_deadline_cancels_real_action(self):
        for implementation in ('python', 'cpp'):
            with self.subTest(implementation=implementation):
                self.reset_counts()
                self.mode = 'hold'
                self.launch_client(implementation, timeout=0.6, attempts=1)
                self.wait_for(lambda: self.canceled == 1)
                time.sleep(0.4)
                self.assertEqual(self.requests, 1)
                self.stop_client()

    def test_shutdown_cancels_owned_action(self):
        for implementation in ('python', 'cpp'):
            with self.subTest(implementation=implementation):
                self.reset_counts()
                self.mode = 'hold'
                self.launch_client(implementation, timeout=120.0)
                self.wait_for(lambda: self.requests == 1)
                # Allow the accepted handle to reach the client before interrupting.
                time.sleep(0.3)
                self.process.send_signal(signal.SIGINT)
                self.process.wait(timeout=5.0)
                self.wait_for(lambda: self.canceled == 1)
                self.assertEqual(self.process.returncode, 0)
                self.stop_client()

    def test_installed_packages_and_model_import_and_initialize(self):
        from hazmat_marker.hazmat_marker import HazmatMarkerNode
        from joint_state_publisher_custom.joint_state_publisher import JointStatePublisher
        from laser_scan_adjuster.adjust_scan import LaserScanAdjuster
        from motor_position_controller.motor_position_controller_node import MotorPositionControllerNode
        from turtlebot3_poi_navigation.poi_manager import POIManager
        nodes = []
        try:
            for cls in (HazmatMarkerNode, JointStatePublisher, LaserScanAdjuster,
                        MotorPositionControllerNode, POIManager):
                nodes.append(cls())
            self.assertEqual(nodes[3].limits, ((-3.14, 3.14), (-3.14, 3.14)))
        finally:
            for node in reversed(nodes):
                node.destroy_node()
