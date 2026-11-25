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

    def read(self):
        return {"positions": self.q, "velocities": [0] * 6, "efforts": [0] * 6,
                "enabled": True, "connected": True, "feedback_age": 0, "errors": []}

    def step(self, dt):
        pass

    def command(self, positions, velocities=None):
        self.commands.append(np.array(positions))

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
    assert np.allclose(backend.commands[-1], backend.q)


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
        controller._tick(stream_start + 0.25, 0.005)
        state = controller.state()
        assert state["mode"] == "idle"
        assert np.allclose(state["target_positions"], state["positions"])

        target = np.asarray(state["positions"]) + [0.03, 0, 0, 0, 0, 0]
        motion = controller.movej(target, duration=1)
        controller._tick(controller._start + 0.1, 0.005)
        controller.stop()
        assert controller.motion(motion["id"])["status"] == "cancelled"
        stopped = controller.state()
        assert np.allclose(stopped["target_positions"], stopped["positions"])

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
