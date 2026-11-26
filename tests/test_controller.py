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


def test_per_joint_goal_tolerances_wait_for_strict_axes_and_report_the_failed_joint():
    backend = StationaryFeedback()
    tolerances = [.001, .001, .001, .005, .005, .005]
    controller = RobotController(backend, RobotModel(), goal_tolerance=.005,
                                 joint_goal_tolerances=tolerances, goal_timeout=.2, goal_settle_time=.05)
    target = np.array(backend.q)+[.002, 0, 0, 0, .003, 0]
    task = controller.movej(target, duration=1)
    start = controller._start
    controller._tick(start+1.01, .005)
    assert controller.motion(task['id'])['status'] == 'running'
    backend.q[0] = target[0]
    controller._tick(start+1.02, .005)
    controller._tick(start+1.08, .005)
    assert controller.motion(task['id'])['status'] == 'succeeded'
    assert controller.info()['joint_goal_tolerances'] == tolerances
    target[1] += .002
    task = controller.movej(target, duration=1)
    controller._tick(controller._start+1.21, .005)
    result = controller.motion(task['id'])
    assert result['status'] == 'aborted'
    assert 'joint2' in result['error'] and '0.0010 rad' in result['error']


@pytest.mark.parametrize('tolerances', [[.001]*5, [.001]*5+[0], [.001]*5+[float('nan')]])
def test_invalid_per_joint_goal_tolerances(tolerances):
    with pytest.raises(ValueError):
        RobotController(StationaryFeedback(), RobotModel(), joint_goal_tolerances=tolerances)


@pytest.mark.parametrize('kind', ['movel', 'movec'])
def test_tcp_goal_requires_cartesian_accuracy_and_restarts_dwell(kind):
    backend = StationaryFeedback()
    model = RobotModel()
    tracking = []
    backend.set_tracking_active = tracking.append
    controller = RobotController(backend, model, goal_tolerance=.02,
                                 tcp_position_tolerance=.0005)
    pose = model.pose(backend.q)
    via = model.pose(backend.q)
    pose['position'][2] += .002
    via['position'][0] += .001
    via['position'][2] += .001
    motion = (controller.movel(pose, duration=1) if kind == 'movel'
              else controller.movec(via, pose, duration=1))
    start = controller._start
    target = controller._plan.positions[-1].copy()
    assert np.max(abs(target-backend.q)) < controller.goal_tolerance
    controller._tick(start+1.01, .005)
    assert controller.motion(motion['id'])['status'] == 'running'
    assert controller.motion(motion['id'])['tcp_position_error'] == pytest.approx(.002, abs=1e-8)
    assert tracking[-1]  # Joint tolerance alone must not freeze correction.
    backend.q = target.copy()
    controller._tick(start+1.02, .005)
    controller._tick(start+1.12, .005)
    backend.q[0] += .004
    controller._tick(start+1.13, .005)
    assert tracking[-1]
    backend.q = target.copy()
    controller._tick(start+1.14, .005)
    controller._tick(start+1.26, .005)
    assert controller.motion(motion['id'])['status'] == 'running'
    controller._tick(start+1.30, .005)
    assert controller.motion(motion['id'])['status'] == 'succeeded'
    assert not tracking[-1]


def test_tcp_goal_timeout_and_following_joint_motion_have_distinct_tolerances():
    backend = StationaryFeedback()
    model = RobotModel()
    controller = RobotController(backend, model, goal_tolerance=.02, goal_timeout=.3,
                                 tcp_position_tolerance=.0005)
    pose = model.pose(backend.q)
    pose['position'][2] += .002
    motion = controller.movel(pose, duration=1)
    controller._tick(controller._start+1.31, .005)
    result = controller.motion(motion['id'])
    assert result['status'] == 'aborted'
    assert 'TCP position error' in result['error']
    motion = controller.movej(controller._q, duration=1)
    start = controller._start
    for t in (1.01, 1.10, 1.18):
        controller._tick(start+t, .005)
    assert controller.motion(motion['id'])['status'] == 'succeeded'


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


def test_goal_waits_for_continuous_position_and_velocity_dwell(monkeypatch):
    backend = StationaryFeedback()
    velocity = [0.] * 6
    read = backend.read
    monkeypatch.setattr(backend, "read", lambda: dict(read(), velocities=velocity.copy()))
    controller = RobotController(backend, RobotModel(), goal_tolerance=.005,
                                 goal_velocity_tolerance=.02, goal_settle_time=.15)
    motion = controller.movej(backend.q, duration=1)
    start = controller._start
    velocity[3] = .12  # Inside position tolerance, still physically moving.
    for t in (1., 1.1, 1.2):
        controller._tick(start+t, .005)
        assert controller.motion(motion['id'])['status'] == 'running'
    velocity[3] = 0.
    controller._tick(start+1.21, .005)
    controller._tick(start+1.30, .005)
    velocity[3] = -.03  # A brief reversal must restart the dwell.
    controller._tick(start+1.31, .005)
    velocity[3] = 0.
    controller._tick(start+1.32, .005)
    controller._tick(start+1.46, .005)
    assert controller.motion(motion['id'])['status'] == 'running'
    controller._tick(start+1.48, .005)
    assert controller.motion(motion['id'])['status'] == 'succeeded'
    assert np.allclose(backend.command_velocities[-1], 0)


@pytest.mark.parametrize('velocity', [[.03]*6, [], [float('nan')]*6])
def test_goal_velocity_failure_cannot_be_reported_as_success(monkeypatch, velocity):
    backend = StationaryFeedback()
    read = backend.read
    monkeypatch.setattr(backend, 'read', lambda: dict(read(), velocities=velocity))
    controller = RobotController(backend, RobotModel(), goal_timeout=.3)
    motion = controller.movej(backend.q, duration=1)
    controller._tick(controller._start+1.01, .005)
    controller._tick(controller._start+1.31, .005)
    result = controller.motion(motion['id'])
    assert result['status'] == 'aborted'
    assert ('settle' if velocity and np.isfinite(velocity[0]) else 'velocity feedback') in result['error']
    assert np.allclose(backend.commands[-1], backend.q)


def test_goal_dwell_cannot_bridge_a_long_control_gap():
    backend = StationaryFeedback()
    controller = RobotController(backend, RobotModel(), feedback_timeout=.1)
    motion = controller.movej(backend.q, duration=1)
    start = controller._start
    controller._tick(start+1.01, .005)
    controller._tick(start+1.3, .005)
    assert controller.motion(motion['id'])['status'] == 'running'
    for t in (1.36, 1.42, 1.46):
        controller._tick(start+t, .005)
    assert controller.motion(motion['id'])['status'] == 'succeeded'


def test_terminal_event_timestamp_is_not_client_poll_time(monkeypatch):
    backend = StationaryFeedback()
    controller = RobotController(backend, RobotModel())
    motion = controller.movej(backend.q, duration=1)
    monkeypatch.setattr(time, 'time', lambda: 1234.)
    monkeypatch.setattr(time, 'monotonic', lambda: 567.)
    controller.stop()
    monkeypatch.setattr(time, 'time', lambda: 1235.)
    result = controller.motion(motion['id'])
    assert result['status'] == 'cancelled'
    assert result['finished_timestamp'] == 1234.
    assert result['finished_monotonic_time'] == 567.


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
