from dataclasses import replace
import math

import pytest

from piper_control.backends.motor_feedback import EncoderSample, EncoderCache, ZeroBandReconstructor
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


def test_real_nonzero_feedback_inside_band_is_not_replaced_or_rejected():
    # Captured at the end of a 1 m/s arc: treating every |q| < 0.3 deg
    # as zero incorrectly clipped this valid reading, then faulted on it.
    fix = ZeroBandReconstructor(math.radians(.3))
    fix.offset = (-.0017544242052180553,)*6
    fix.anchored = (True,)*6
    sample = EncoderSample((.006,)*6, (.0050789081233034985,)*6, (0.,)*6, .005)
    assert not any(fix.affected(sample))
    assert fix.correct(sample, (0.,)*6).joints == sample.joints
    fix.observe(sample)
    assert fix.offset == pytest.approx((.006-.0050789081233034985,)*6)


def test_zero_transition_provenance_survives_interpolation_outside_band():
    import struct
    cache = EncoderCache()
    for stamp in [1., 1.01]:
        for i in range(6):
            motor_stamp = 1.005 if stamp == 1.01 and i == 0 else stamp
            cache.update(0x251+i, struct.pack('>hhi', 1000, 0, 10), motor_stamp)
        for i in range(3):
            cache.update(0x2A5+i, struct.pack('>ii', 1000 if stamp == 1. else 0, 2000), stamp)
    # The slowest motor group makes the common time 1.005 s.
    sample = cache.aligned_sample(1.011)
    sample.validate()
    fix = ZeroBandReconstructor(math.radians(.3))
    assert sample.joints[0] > fix.band
    assert fix.affected(sample) == (True, False)*3
    fix.observe(sample)
    assert fix.anchored == (False, True)*3


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
