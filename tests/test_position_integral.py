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
