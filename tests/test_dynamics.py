import json

import numpy as np
import pytest

from piper_control.backends.dynamics import BulletModel, DynamicsFeedforward
from piper_control.model import RobotModel


Q = [0., 1.3, -1.28634, 1.08, -.00754, 0.]


def test_dynamic_model_excludes_gravity_and_contains_stationary_axis_coupling():
    body = BulletModel(RobotModel())
    try:
        assert body.dynamic_efforts(Q, np.zeros(6), np.zeros(6)) == pytest.approx(np.zeros(6), abs=1e-12)
        slow = body.dynamic_efforts(Q, [1., 0, 0, 0, 0, 0], np.zeros(6))
        fast = body.dynamic_efforts(Q, [2., 0, 0, 0, 0, 0], np.zeros(6))
        # Centrifugal load grows with speed squared on a stationary shoulder.
        assert abs(slow[1]) > .1
        assert fast == pytest.approx(4*slow, abs=1e-10)
        accelerating = body.dynamic_efforts(Q, np.zeros(6), [10., 0, 0, 0, 0, 0])
        assert abs(accelerating[4]) > .2  # J1 acceleration loads stationary J5.
    finally:
        body.close()


def test_acceleration_bounded_and_stop_or_stall_clears_history():
    ff = DynamicsFeedforward(RobotModel(), scale=.5)
    try:
        for speed in [3., -3., 3.]:
            assert np.isfinite(ff.evaluate(Q, [speed, 0, 0, 0, 0, 0], .005)).all()
            assert max(map(abs, ff.last['acceleration'])) <= 50
            assert ff.last['acceleration_limited']
        assert ff.evaluate(Q, np.zeros(6), .005) == pytest.approx(np.zeros(6), abs=0)
        assert ff.last['acceleration'] == [0.]*6
        assert ff.evaluate(Q, [2., 0, 0, 0, 0, 0], .1) == pytest.approx(np.zeros(6), abs=0)
        ff.evaluate(Q, [2., 0, 0, 0, 0, 0], .005)
        assert ff.last['acceleration'] == [0.]*6
        assert abs(ff.last['torque'][1]) > .2
    finally:
        ff.close()


@pytest.mark.parametrize('settings', [{'scale': float('nan')}, {'scale': 1.1},
    {'acceleration_cutoff_hz': 0}, {'max_acceleration': float('inf')}])
def test_invalid_dynamics_settings_fail_before_world_is_created(settings):
    with pytest.raises(ValueError):
        DynamicsFeedforward(RobotModel(), **settings)


def test_feedforward_coupling_survives_stationary_axis_gate_and_total_torque_limit():
    from test_tracking import make_feedforward, Raw, Receiver, MOTOR

    ff = make_feedforward({'friction_scale': 0., 'velocity_remainder': False,
                           'dynamics': {'scale': 1.}})
    raw = Raw()
    try:
        for _ in range(100):
            effort = ff.evaluate(MOTOR, [2., 0, 0, 0, 0, 0], raw.read(), Receiver.value,
                                 raw.kp, raw.kd, .005, Q)
        assert abs(effort[1]) > .4
        effort = ff.evaluate(MOTOR, [2.05, 0, 0, 0, 0, 0], raw.read(), Receiver.value,
                             raw.kp, raw.kd, .005, Q)
        assert abs(effort[4]) > .1
        for speed in [3., -3., 3.]:
            effort = ff.evaluate(MOTOR, [speed]*6, raw.read(), Receiver.value,
                                 raw.kp, raw.kd, .005, Q)
            assert all(abs(t) <= limit for t, limit in zip(effort, ff.limits))
        assert ff.evaluate(MOTOR, [0.]*6, raw.read(), Receiver.value,
                           raw.kp, raw.kd, .005, Q) == [0.]*6
        json.dumps(ff.last, allow_nan=False)
    finally:
        ff.close()


def test_simulated_stationary_wrist_tracks_during_fast_base_sweep():
    from piper_control.backends.simulation import SimulationBackend
    from piper_control.planning import Planner

    model = RobotModel()
    end = np.array(Q)
    end[0] -= np.pi/4
    plan = Planner(model, np.pi).movej(Q, end, duration=.56)
    peaks = []
    for dynamics in (None, {'scale': 1.}):
        sim = SimulationBackend(model, initial_positions=Q, kp=[51, 64, 60, 32, 3, 3],
                                kd=[8, 12.12, 20, .53, .3, .08], dynamics=dynamics)
        try:
            sim.enable()
            for _ in range(500):
                sim.step(.005)
            errors = []
            for t in np.arange(0, plan.duration+.3, .005):
                q, v = plan.sample(t)
                sim.command(q, v)
                sim.step(.005)
                errors.append(abs(sim.read()['positions'][4]-q[4]))
            peaks.append(max(errors))
        finally:
            sim.close()
    assert peaks[0] > .03
    assert peaks[1] < .004 and peaks[1] < peaks[0]/8
