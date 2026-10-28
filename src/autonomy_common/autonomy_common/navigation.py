"""Nonblocking Nav2 task ownership independent of a ROS executor."""

from copy import deepcopy
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class Outcome:
    """One terminal result for the pose owned by this client."""

    pose: object
    status: str
    detail: str = ''


class NavigationTask:
    """Own one action until rejection or a terminal result confirms completion.

    Cancellation acknowledgement alone never frees the slot. A late acceptance
    of a timed-out request is canceled before another goal may be submitted.
    Timeouts use monotonic wall time, including while simulation is paused.
    """

    def __init__(self, client, goal_factory, timeout=120.0, on_finished=None,
                 clock=time.monotonic, notify=None):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('navigation timeout must be positive and finite')
        self.client = client
        self.goal_factory = goal_factory
        self.timeout = timeout
        self.on_finished = on_finished or (lambda outcome: None)
        self.clock = clock
        self.notify = notify or (lambda message: None)
        self.busy = False
        self.handle = None
        self.pose = None
        self.started_at = None
        self.cancel_requested = False
        self.cancel_sent = False
        self._generation = 0

    def begin(self, pose):
        """Submit only when the action server is ready and this slot is empty."""
        if self.busy or not self.client.server_is_ready():
            return False
        self._generation += 1
        generation = self._generation
        self.busy = True
        self.pose = deepcopy(pose)
        self.handle = None
        self.started_at = self.clock()
        self.cancel_requested = False
        self.cancel_sent = False
        goal = self.goal_factory()
        goal.pose = deepcopy(pose)
        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda done: self._accepted(done, generation))
        except Exception as exc:
            self.notify(f'Goal submission uncertain; mission held: {exc}')
            self.cancel()
        return True

    def _accepted(self, future, generation):
        if generation != self._generation or not self.busy:
            return
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._finish('FAILED', 'Goal rejected')
                return
            self.handle = handle
            result = handle.get_result_async()
            result.add_done_callback(lambda done: self._result(done, generation))
            if self.cancel_requested and self.busy:
                self._send_cancel()
        except Exception as exc:
            # A transport failure after submission leaves execution uncertain.
            # Keep ownership and request cancellation if a handle is available.
            self.notify(f'Goal response unavailable; mission held: {exc}')
            self.cancel()

    def _result(self, future, generation):
        if generation != self._generation or not self.busy:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.notify(f'Goal result unavailable; mission held: {exc}')
            self.cancel()
            return
        # action_msgs/GoalStatus terminal values: succeeded=4, canceled=5.
        outcome = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'FAILED'}.get(status)
        if outcome is None:
            self.notify(f'Unexpected nonterminal result {status}; mission held')
            self.cancel()
            return
        self._finish(outcome)

    def _finish(self, status, detail=''):
        outcome = Outcome(self.pose, status, detail)
        self.busy = False
        self.handle = None
        self.pose = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.on_finished(outcome)

    def tick(self):
        """Request cancellation at the deadline without blocking callbacks."""
        if self.busy and not self.cancel_requested:
            if self.clock() - self.started_at >= self.timeout:
                self.notify('Navigation deadline reached; cancellation requested')
                self.cancel()

    def cancel(self):
        """Remember cancellation even before the goal response has arrived."""
        if self.busy:
            self.cancel_requested = True
            self._send_cancel()

    def _send_cancel(self):
        if self.handle is None or self.cancel_sent:
            return
        self.cancel_sent = True
        try:
            future = self.handle.cancel_goal_async()
            future.add_done_callback(self._cancel_response)
        except Exception as exc:
            self.notify(f'Cancellation unavailable; mission held: {exc}')

    def _cancel_response(self, future):
        try:
            if not future.result().goals_canceling and self.busy:
                self.notify('Cancellation refused; waiting for the terminal result')
        except Exception as exc:
            self.notify(f'Cancellation response unavailable: {exc}')
