import pytest

from piper_control.backends.position_integral import PositionIntegral


def test_bias_is_rate_limited_bounded_and_does_not_chase_deadband():
    integral = PositionIntegral(gains=[1.]*6)
    for _ in range(500):
        before = integral.bias
        integral.advance([.1]*6, [0.]*6, .01)
        assert max(abs(a-b) for a, b in zip(before, integral.bias)) <= .00005000001
    assert integral.bias == pytest.approx([.02]*6)
    integral.advance([.0005]*6, [0.]*6, .01)
    assert integral.bias == pytest.approx([.02]*6)
    integral.advance([-.1]*6, [0.]*6, 10.)
    assert integral.bias == pytest.approx([.0199]*6)  # Long outage cannot create a jump.
    before = integral.bias
    with pytest.raises(ValueError):
        integral.advance([float('nan')]*6, [0.]*6, .005)
    assert integral.bias == before


def test_static_load_bias_converges_without_unbounded_windup():
    integral = PositionIntegral(gains=[1.]*6)
    # A settled inner servo has a constant 6 mrad load error. The outer loop
    # removes that error to its explicit encoder deadband, not to an ideal zero.
    for _ in range(1500):
        measured = [b-.006 for b in integral.bias]
        integral.advance([0.]*6, measured, .005)
    assert max(abs(b-.006) for b in integral.bias) < .00101


def test_fast_delayed_feedback_does_not_wind_up_then_static_load_converges():
    integral = PositionIntegral(gains=[2.]*6, deadband=.0002, max_velocity=.02)
    # A 10 ms measurement lag during a 2 rad/s sweep is not static load error.
    for direction in (-1, 1):
        for step in range(100):
            target = [direction*2*step*.005]+[0.]*5
            measured = [target[0]-direction*.02]+[-.003]*5
            integral.advance(target, measured, .005, [direction*2]+[0.]*5)
    assert integral.bias == (0.,)*6
    assert integral.velocity_gated
    for _ in range(1000):
        measured = [b-.003 for b in integral.bias]
        integral.advance([0.]*6, measured, .005, [0.]*6)
    assert not integral.velocity_gated
    assert max(abs(b-.003) for b in integral.bias) < .00021


@pytest.mark.parametrize('maximum', [0, -.01, float('nan'), float('inf')])
def test_velocity_gate_requires_a_valid_limit(maximum):
    with pytest.raises(ValueError):
        PositionIntegral(max_velocity=maximum)


def test_velocity_gate_does_not_treat_missing_velocity_as_stationary():
    integral = PositionIntegral(max_velocity=.02)
    with pytest.raises(ValueError, match='requires reference velocity'):
        integral.advance([0.]*6, [0.]*6, .005)


def test_restart_preserves_integral_and_mechanical_coordinates(tmp_path):
    from piper_control.backends.tracking import TrackingBackend
    from test_tracking import Raw, Receiver, Feedforward, Q, MOTOR

    settings = {'position_integral': {'gains': [1.]*6}}
    first = TrackingBackend(Raw(), settings, receiver=Receiver(), feedforward=Feedforward(),
                            state_path=tmp_path/'state.json')
    try:
        first._freeze_until = 0
        target = [q+.004 for q in Q]
        for _ in range(100):
            first.command(target)
        bias = first.integral.bias
        assert min(bias) > .001
        held = first.raw.sent[-1][0]
        first.hold()
    finally:
        first.close()
    raw = Raw()
    raw.initial_target = held
    second = TrackingBackend(raw, settings, receiver=Receiver(), feedforward=Feedforward(),
                             state_path=tmp_path/'state.json')
    try:
        assert second.initial_target == pytest.approx(target)
        assert second.integral.bias == bias
        second.command(second.initial_target)
        assert second.raw.sent[-1][0] == pytest.approx(held)
    finally:
        second.close()


def test_motor_motion_reduces_windup_without_hiding_unresolved_output_error():
    integral = PositionIntegral(gains=[2.]*6, deadband=.0002, max_velocity=.02,
                                motor_lead_limit=.003)
    integral.advance([.001]*6, [0.]*6, .005, [0.]*6, [0.]*6)
    before = integral.bias
    integral.advance([.001]*6, [0.]*6, .005, [0.]*6, [.001]*6)
    assert integral.bias == before  # Motor already reached the target.
    assert .0009 < integral.motor_lead[0] < .001
    # Arrival of the new output value must not double-count motor motion.
    integral.advance([.001]*6, [.001]*6, .005, [0.]*6, [.001]*6)
    assert integral.bias == before
    assert integral.motor_lead == (0.,)*6
    integral.advance([.001]*6, [.001]*6, .005, [0.]*6, [.02]*6)
    assert integral.motor_lead == pytest.approx([.003]*6)
    # A new fast move cannot retain a stale low-speed anchor.
    integral.advance([1.]*6, [.001]*6, .005, [.5]*6, [1.]*6)
    assert integral.motor_lead == (0.,)*6
    integral.advance([1.]*6, [1.]*6, .005, [0.]*6, [1.]*6)
    assert integral.motor_lead == (0.,)*6
    integral.advance([1.001]*6, [1.]*6, .005, [0.]*6, [1.001]*6)
    before = integral.bias
    for _ in range(1000):
        integral.advance([1.001]*6, [1.]*6, .005, [0.]*6, [1.001]*6)
    assert abs(integral.motor_lead[0]) < 1e-7
    assert integral.bias[0]-before[0] > .006


@pytest.mark.parametrize('limit', [-.001, .006, float('nan'), float('inf')])
def test_motor_lead_requires_bounded_configuration(limit):
    with pytest.raises(ValueError):
        PositionIntegral(motor_lead_limit=limit)


def test_motor_lead_requires_valid_feedback_without_changing_bias():
    integral = PositionIntegral(motor_lead_limit=.003)
    for motor in (None, [float('nan')]*6):
        with pytest.raises(ValueError):
            integral.advance([0.]*6, [0.]*6, .005, motor=motor)
    assert integral.bias == (0.,)*6
