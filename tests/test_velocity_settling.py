import math

import numpy as np
import pytest

from piper_control.controller import RobotController
from piper_control.model import RobotModel
from test_controller import StationaryFeedback


def test_velocity_window_distinguishes_sparse_pulses_from_continuous_motion(monkeypatch):
    backend = StationaryFeedback()
    velocity = [0.]*6
    read = backend.read
    monkeypatch.setattr(backend, 'read', lambda: dict(read(), velocities=velocity.copy()))
    controller = RobotController(backend, RobotModel(), goal_velocity_window=.5,
                                 goal_velocity_tolerance=.01, goal_settle_time=.15)
    task = controller.movej(backend.q, duration=1)
    start = controller._start
    for i in range(80):
        velocity[3] = .03 if i % 20 == 1 else 0.
        controller._tick(start+1+i*.01, .01)
        assert controller.state()['velocities'] == velocity
        if i < 50:
            assert controller.motion(task['id'])['status'] == 'running'
    assert controller.motion(task['id'])['status'] == 'succeeded'
    assert controller.info()['goal_velocity_window'] == .5
    # A new task must build a fresh window; continuous movement cannot pass it.
    task = controller.movej(backend.q, duration=1)
    start = controller._start
    velocity[3] = .03
    for i in range(100):
        controller._tick(start+1+i*.01, .01)
    result = controller.motion(task['id'])
    assert result['status'] == 'running'
    assert result['settling_speed'] == pytest.approx(.03)


def test_velocity_rms_is_time_weighted_and_resets_after_invalid_data_or_gap():
    controller = RobotController(StationaryFeedback(), RobotModel(), goal_velocity_window=.2)
    pulse = [.02]+[0.]*5
    assert math.isinf(controller._settling_speed(pulse, 0.))
    controller._settling_speed([0.]*6, .1)
    controller._settling_speed([0.]*6, .19)
    assert controller._settling_speed([0.]*6, .2) == pytest.approx(.02/np.sqrt(2))
    assert pulse == [.02]+[0.]*5
    assert math.isinf(controller._settling_speed([float('nan')]*6, .21))
    assert math.isinf(controller._settling_speed([0.]*6, .22))
    assert math.isinf(controller._settling_speed([0.]*6, .6))
    controller._settling_speed([0.]*6, .7)
    assert controller._settling_speed([0.]*6, .8) == pytest.approx(0.)


@pytest.mark.parametrize('window', [-.1, 6, float('nan'), float('inf')])
def test_velocity_window_requires_valid_bounds(window):
    with pytest.raises(ValueError, match='goal_velocity_window'):
        RobotController(StationaryFeedback(), RobotModel(), goal_velocity_window=window)


def test_velocity_window_and_dwell_must_fit_timeout():
    with pytest.raises(ValueError, match='goal_velocity_window'):
        RobotController(StationaryFeedback(), RobotModel(), goal_velocity_window=.5,
                        goal_settle_time=.2, goal_timeout=.6)
