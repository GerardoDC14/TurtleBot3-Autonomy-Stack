"""Exercise action ownership with independently completed asynchronous futures."""

from concurrent.futures import Future
from types import SimpleNamespace as NS
import unittest

import support  # noqa: F401
from autonomy_common.navigation import NavigationTask


class Client:
    def __init__(self):
        self.ready = True
        self.requests = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        future = Future()
        self.requests.append((goal, future))
        return future


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return self.cancel_future


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.client = Client()
        self.outcomes, self.messages = [], []
        self.now = 0.0
        self.task = NavigationTask(
            self.client, lambda: NS(pose=None), 2.0, self.outcomes.append,
            clock=lambda: self.now, notify=self.messages.append)
        self.pose = NS(x=1.0)

    def accept(self):
        handle = Handle()
        self.client.requests[-1][1].set_result(handle)
        return handle

    def test_success_returns_owned_copy_once(self):
        self.assertTrue(self.task.begin(self.pose))
        self.pose.x = 99.0
        handle = self.accept()
        handle.result_future.set_result(NS(status=4))
        self.assertFalse(self.task.busy)
        self.assertEqual(self.outcomes[0].pose.x, 1.0)
        self.assertEqual(self.outcomes[0].status, 'SUCCEEDED')

    def test_rejected_goal_releases_slot(self):
        self.task.begin(self.pose)
        self.client.requests[-1][1].set_result(Handle(accepted=False))
        self.assertFalse(self.task.busy)
        self.assertEqual(self.outcomes[0].status, 'FAILED')
        self.assertTrue(self.task.begin(self.pose))

    def test_unavailable_server_does_not_submit(self):
        self.client.ready = False
        self.assertFalse(self.task.begin(self.pose))
        self.assertEqual(self.client.requests, [])

    def test_overlapping_goal_is_refused(self):
        self.task.begin(self.pose)
        self.assertFalse(self.task.begin(NS(x=2)))
        self.assertEqual(len(self.client.requests), 1)

    def test_timeout_before_acceptance_cancels_late_handle(self):
        self.task.begin(self.pose)
        self.now = 3.0
        self.task.tick()
        handle = self.accept()
        self.assertEqual(handle.cancels, 1)
        self.assertTrue(self.task.busy)
        self.assertFalse(self.task.begin(self.pose))

    def test_cancel_acknowledgement_waits_for_terminal_result(self):
        self.task.begin(self.pose)
        handle = self.accept()
        self.task.cancel()
        handle.cancel_future.set_result(NS(goals_canceling=[object()]))
        self.assertTrue(self.task.busy)
        self.task.cancel()
        self.assertEqual(handle.cancels, 1)
        handle.result_future.set_result(NS(status=5))
        self.assertFalse(self.task.busy)
        self.assertEqual(self.outcomes[0].status, 'CANCELED')

    def test_refused_cancellation_keeps_slot(self):
        self.task.begin(self.pose)
        handle = self.accept()
        self.task.cancel()
        handle.cancel_future.set_result(NS(goals_canceling=[]))
        self.assertTrue(self.task.busy)
        self.assertTrue(self.messages)

    def test_result_transport_failure_holds_and_cancels(self):
        self.task.begin(self.pose)
        handle = self.accept()
        handle.result_future.set_exception(RuntimeError('transport lost'))
        self.assertTrue(self.task.busy)
        self.assertEqual(handle.cancels, 1)
        self.assertEqual(self.outcomes, [])

    def test_abort_allows_next_goal(self):
        self.task.begin(self.pose)
        self.accept().result_future.set_result(NS(status=6))
        self.assertEqual(self.outcomes[0].status, 'FAILED')
        self.assertTrue(self.task.begin(self.pose))

    def test_timeout_is_requested_once(self):
        self.task.begin(self.pose)
        handle = self.accept()
        self.now = 3.0
        for _ in range(5):
            self.task.tick()
        self.assertEqual(handle.cancels, 1)

    def test_invalid_timeouts_are_rejected(self):
        for value in (0.0, -1.0, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                NavigationTask(self.client, lambda: NS(), timeout=value)

    def test_uncertain_submission_holds_the_mission(self):
        def fail(goal):
            raise RuntimeError('transport interrupted')
        self.client.send_goal_async = fail
        self.task.begin(self.pose)
        self.assertTrue(self.task.busy)
        self.assertTrue(self.task.cancel_requested)
        self.assertFalse(self.task.begin(self.pose))
