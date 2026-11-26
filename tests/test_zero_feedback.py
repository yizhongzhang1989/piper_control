from dataclasses import replace
import math

import pytest

from piper_control.backends.motor_feedback import EncoderSample, ZeroBandReconstructor
from piper_control.backends.tracking import TrackingBackend
from test_tracking import Raw, Receiver, Feedforward, Q, MOTOR


def test_zero_band_crossing_uses_motor_continuity_and_preserves_raw_sample():
    fix = ZeroBandReconstructor(math.radians(.3))
    offset = (.001,)*6
    for q in [-.006, -.005, -.003, 0., .003, .005, .006]:
        reported = 0. if abs(q) < fix.band else q
        sample = EncoderSample((q+.001,)*6, (reported,)*6, (.05,)*6, .005)
        assert fix.correct(sample, offset).joints == pytest.approx((q,)*6)
        assert sample.joints == (reported,)*6
        assert fix.affected(sample) == (abs(q) < fix.band,)*6


def test_inference_is_bounded_and_does_not_hide_stale_or_inconsistent_feedback():
    fix = ZeroBandReconstructor(math.radians(.3))
    sample = EncoderSample((.006,)*6, (0.,)*6, (0.,)*6, .005)
    assert fix.correct(sample, (0.,)*6).joints == (fix.band,)*6
    with pytest.raises(RuntimeError, match='inconsistent'):
        fix.correct(replace(sample, motor=(.1,)*6), (0.,)*6)
    with pytest.raises(RuntimeError, match='Fresh'):
        fix.correct(replace(sample, age=1.), (0.,)*6)


def test_valid_anchor_survives_zero_readings_and_restart():
    fix = ZeroBandReconstructor(math.radians(.3))
    fix.observe(EncoderSample((.011,)*6, (.01,)*6, (0.,)*6, .005))
    zero = EncoderSample((.004,)*6, (0.,)*6, (0.,)*6, .005)
    fix.observe(zero)
    assert fix.correct(zero, (.004,)*6).joints == pytest.approx((.003,)*6)
    restored = ZeroBandReconstructor(fix.band)
    restored.restore(fix.checkpoint())
    assert restored.correct(zero, (.004,)*6).joints == pytest.approx((.003,)*6)
    assert restored.anchored == (True,)*6


@pytest.mark.parametrize('band', [0., -.001, .1, float('nan'), float('inf')])
def test_invalid_zero_band(band):
    with pytest.raises(ValueError):
        ZeroBandReconstructor(band)


def test_zero_feedback_cannot_relearn_coordinate_offset_or_wind_up_integral(tmp_path):
    receiver = Receiver()
    b = TrackingBackend(Raw(), {'feedback_zero_band': math.radians(.3),
                               'position_integral': {'gains': [1.]*6}},
                        receiver=receiver, feedforward=Feedforward(), state_path=tmp_path/'zero.json')
    try:
        b._freeze_until = 0
        offset = b.mapper.offset
        motor = list(MOTOR)
        motor[4] -= .003
        receiver.value = EncoderSample(tuple(motor), Q, (0.,)*6, .001)
        target = list(Q)
        target[4] -= .003
        for _ in range(100):
            b.command(target, [0, 0, 0, 0, .02, 0])
            b.step(.005)
        assert b.mapper.offset == offset
        assert b.integral.bias == (0.,)*6
        assert b.raw.sent[-1][0] == pytest.approx(motor)
        state = b.read()
        assert state['positions'][4] == pytest.approx(-.003)
        assert state['extra']['feedback_reconstruction']['raw_positions'][4] == 0.
        assert state['extra']['feedback_reconstruction']['inferred'][4]
        assert not state['extra']['motor_mapping']['observer_active_joints'][4]
        held = b.raw.sent[-1][0]
        b.hold()
        assert b.raw.held == held
    finally:
        b.close()
