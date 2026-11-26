import time

import numpy as np
import pytest

from piper_control.backends.simulation import SimulationBackend
from piper_control.controller import RobotController
from piper_control.model import RobotModel


class StationaryFeedback:
    """Connected actuator whose measured position does not follow its commands."""

    def __init__(self):
        self.q = [0, 0.5, -0.5, 0, 0.2, 0]
        self.commands = []
        self.command_velocities = []

    def read(self):
        return {"positions": self.q, "velocities": [0] * 6, "efforts": [0] * 6,
                "enabled": True, "connected": True, "feedback_age": 0, "errors": []}

    def step(self, dt):
        pass

    def command(self, positions, velocities=None):
        self.commands.append(np.array(positions))
        self.command_velocities.append(np.array(velocities if velocities is not None else [0.] * 6))

    def close(self):
        pass


def test_invalid_path_has_no_execution_and_goal_uses_measured_feedback():
    backend = StationaryFeedback()
    controller = RobotController(backend, RobotModel(), goal_tolerance=0.005, goal_timeout=0.2)
    with pytest.raises(ValueError, match="unreachable"):
        controller.movel({"position": [10, 10, 10], "orientation": [0, 0, 0, 1]}, duration=1)
    assert not backend.commands and controller.state()["motion"] is None
    target = np.array(backend.q) + [0.08, 0, 0, 0, 0, 0]
    motion = controller.movej(target, duration=1)
    controller._tick(controller._start + 1.01, 0.005)
    assert controller.motion(motion["id"])["status"] == "running"
    controller._tick(controller._start + 1.21, 0.005)
    result = controller.motion(motion["id"])
    assert result["status"] == "aborted" and "tracking error" in result["error"]
    assert np.allclose(backend.commands[-1], target)
    assert np.allclose(backend.command_velocities[-1], 0)


def test_following_error_does_not_step_targets_at_command_boundaries():
    backend = StationaryFeedback()
    backend.initial_target = np.asarray(backend.q) + [0, 0, 0, .012, 0, 0]
    controller = RobotController(backend, RobotModel(), max_velocity=.08)
    held = backend.initial_target.copy()
    controller._tick(time.monotonic(), .005)
    motion = controller.movej(held + [.02, 0, 0, 0, 0, 0], duration=1)
    controller._tick(controller._start, .005)
    assert np.array_equal(backend.commands[-1], held)
    controller._tick(controller._start + .5, .005)
    last = backend.commands[-1].copy()
    assert last[3] == held[3]
    controller.stop()
    controller._tick(time.monotonic(), .005)
    assert controller.motion(motion["id"])["status"] == "cancelled"
    assert np.array_equal(backend.commands[-1], last)
    assert np.allclose(backend.command_velocities[-1], 0)

    controller.servo_joints(last + [0, .02, 0, 0, 0, 0])
    assert np.array_equal(controller._q, last)
    stream_start = controller._stream_time
    controller._tick(stream_start + .005, .005)
    current = backend.commands[-1].copy()
    assert np.max(np.abs(current-last)) <= .08*.005 + 1e-12
    assert current[3] == held[3]
    controller._tick(stream_start + .25, .005)
    assert controller.state()["mode"] == "idle"
    assert np.array_equal(backend.commands[-1], current)
    assert np.allclose(backend.command_velocities[-1], 0)

    controller.servo_twist([0, 0, 0], [0, 0, 0])
    assert np.array_equal(controller._q, current)
    controller._tick(controller._stream_time + .005, .005)
    assert np.array_equal(backend.commands[-1], current)
    controller.stop()
    controller._tick(time.monotonic(), .005)
    assert np.array_equal(backend.commands[-1], current)
    assert np.allclose(backend.command_velocities[-1], 0)


def test_enable_is_idempotent_during_an_active_motion(monkeypatch):
    backend = StationaryFeedback()
    controller = RobotController(backend, RobotModel())
    motion = controller.movej(np.asarray(backend.q) + [.04, 0, 0, 0, 0, 0], duration=1)
    controller._tick(controller._start + .5, .005)
    target = controller.state()["target_positions"]
    epoch = controller._epoch
    monkeypatch.setattr(backend, "enable", lambda value: pytest.fail("unexpected hardware enable"), raising=False)
    state = controller.enable()
    assert state["target_positions"] == target
    assert state["motion"]["id"] == motion["id"]
    assert state["motion"]["status"] == "running"
    assert state["mode"] == "movej"
    assert controller._plan is not None and controller._epoch == epoch


@pytest.mark.parametrize("during_motion", [False, True])
def test_feedback_outage_preserves_nominal_target_on_recovery(monkeypatch, during_motion):
    backend = StationaryFeedback()
    backend.initial_target = np.asarray(backend.q) + [0, .02, 0, .009, 0, 0]
    controller = RobotController(backend, RobotModel())
    if during_motion:
        controller.movej(backend.initial_target + [.04, 0, 0, 0, 0, 0], duration=1)
        controller._tick(controller._start + .5, .005)
    expected = controller._q.copy()
    read = backend.read
    monkeypatch.setattr(backend, "read", lambda: dict(read(), connected=False, feedback_age=.3))
    count = len(backend.commands)
    controller._tick(time.monotonic(), .005)
    assert controller.state()["mode"] == "fault"
    assert len(backend.commands) == count
    monkeypatch.setattr(backend, "read", read)
    controller._tick(time.monotonic(), .005)
    assert np.array_equal(backend.commands[-1], expected)
    assert np.array_equal(backend.command_velocities[-1], np.zeros(6))
    assert controller.state()["mode"] == "idle"
    if during_motion:
        assert controller.state()["motion"]["status"] == "aborted"


def test_stream_timeout_stop_and_physics_motion_success():
    model = RobotModel()
    backend = SimulationBackend(model)
    controller = RobotController(backend, model, goal_tolerance=0.001, feedback_timeout=1)
    try:
        controller.enable()
        initial = np.asarray(controller.state()["positions"])
        controller.servo_joints(initial + [0.1, 0, 0, 0, 0, 0])
        stream_start = controller._stream_time
        for i in range(1, 10):
            controller._tick(stream_start + i * 0.005, 0.005)
        assert controller.state()["target_positions"][0] > initial[0]
        before_timeout = controller.state()["target_positions"]
        controller._tick(stream_start + 0.25, 0.005)
        state = controller.state()
        assert state["mode"] == "idle"
        assert state["target_positions"] == before_timeout

        target = np.asarray(state["positions"]) + [0.03, 0, 0, 0, 0, 0]
        motion = controller.movej(target, duration=1)
        controller._tick(controller._start + 0.1, 0.005)
        before_stop = controller.state()["target_positions"]
        controller.stop()
        assert controller.motion(motion["id"])["status"] == "cancelled"
        stopped = controller.state()
        assert stopped["target_positions"] == before_stop

        target = np.asarray(stopped["positions"]) + [0.03, 0, 0, 0, 0, 0]
        motion = controller.movej(target, duration=1)
        start = controller._start
        for i in range(1, 801):
            controller._tick(start + i * 0.005, 0.005)
            if controller.motion(motion["id"])["status"] != "running":
                break
        assert controller.motion(motion["id"])["status"] == "succeeded"
        assert np.max(np.abs(np.asarray(controller.state()["positions"]) - target)) < 0.001
    finally:
        controller.close()
