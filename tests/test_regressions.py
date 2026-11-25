"""Regressions for trajectory validation and command/fault ordering."""
import threading
import time

import numpy as np
import pytest

from piper_control.controller import RobotController
from piper_control.model import RobotModel
from piper_control.planning import Plan, Planner


class FeedbackBackend:
    def __init__(self):
        self.q = np.array([0., .5, -.5, 0., .2, 0.])
        self.connected = True

    def read(self):
        return {"positions": self.q.tolist(), "enabled": True,
                "connected": self.connected, "feedback_age": 0., "errors": []}

    def step(self, dt):
        pass

    def command(self, q, dq=None):
        self.q = np.asarray(q).copy()

    def close(self):
        pass


def test_short_quintic_segment_cannot_hide_its_velocity_peak():
    model = RobotModel()
    planner = Planner(model, max_velocity=.3)
    q = np.array([0., .5, -.5, 0., .2, 0.])
    displaced = q + [0.01, 0, 0, 0, 0, 0]
    plan = Plan(np.array([0., .001, .002, 1.]), np.array([q, displaced, q, q]))
    assert plan.sample(.0005)[1][0] == pytest.approx(18.75)
    with pytest.raises(ValueError, match="velocity"):
        planner.validate(plan)


def test_feedback_fault_invalidates_a_plan_still_being_prepared():
    backend = FeedbackBackend()
    controller = RobotController(backend, RobotModel())
    start, epoch = controller._prepare()
    pending = controller.planner.movej(start, start + [0.02, 0, 0, 0, 0, 0], duration=1)
    backend.connected = False
    controller._tick(time.monotonic(), .005)
    backend.connected = True
    with pytest.raises(RuntimeError, match="changed"):
        controller._submit(pending, "movej", epoch)
    assert controller.state()["motion"] is None


def test_stream_solver_fault_preserves_the_previous_successful_motion(monkeypatch):
    backend = FeedbackBackend()
    model = RobotModel()
    controller = RobotController(backend, model, goal_tolerance=.001)
    motion = controller.movej(backend.q + [0.02, 0, 0, 0, 0, 0], duration=1)
    controller._tick(controller._start + 1.01, .005)
    controller._tick(controller._start + 1.02, .005)
    assert controller.motion(motion["id"])["status"] == "succeeded"

    def fail_solver(*args):
        raise ValueError("injected stream solver fault")

    monkeypatch.setattr(model, "jacobian", fail_solver)
    controller.servo_twist([0, 0, .001], [0, 0, 0])
    controller.start()
    try:
        deadline = time.monotonic() + 2
        while "injected stream solver fault" not in controller.state()["errors"]:
            assert time.monotonic() < deadline, "control loop did not report solver failure"
            time.sleep(.005)
        assert controller.motion(motion["id"])["status"] == "succeeded"
    finally:
        controller.close()


def test_stop_can_interrupt_a_tcp_request_while_ik_is_running(monkeypatch):
    backend = FeedbackBackend()
    model = RobotModel()
    controller = RobotController(backend, model)
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    errors = []

    def slow_ik(*args):
        entered.set()
        assert release.wait(2), "test did not release inverse kinematics"
        return backend.q.copy()

    def send_pose():
        try:
            controller.servo_pose(model.pose(backend.q))
        except RuntimeError as error:
            errors.append(str(error))

    def stop():
        controller.stop()
        stopped.set()

    monkeypatch.setattr(model, "inverse", slow_ik)
    producer = threading.Thread(target=send_pose)
    stopper = threading.Thread(target=stop)
    producer.start()
    try:
        assert entered.wait(1)
        stopper.start()
        assert stopped.wait(.5), "TCP inverse kinematics blocked stop"
    finally:
        release.set()
        producer.join(timeout=2)
        if stopper.ident:
            stopper.join(timeout=2)
        controller.close()
    assert errors and "changed" in errors[0]
    assert controller._stream_time is None
