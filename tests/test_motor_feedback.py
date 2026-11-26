import math
import socket
import struct

import pytest

from piper_control.backends.motor_feedback import (
    CoordinateMapper, EncoderCache, EncoderSample, SO_TIMESTAMPNS, receive_timestamp,
)


def fresh_cache():
    cache = EncoderCache()
    for i in range(6):
        cache.update(0x251+i, struct.pack(">hhi", -125, 500, -1000), 10.)
    for i in range(3):
        cache.update(0x2A5+i, struct.pack(">ii", -57000, -57000), 10.)
    return cache


def test_protocol_coordinates_have_different_units_and_signed_fields():
    sample = fresh_cache().sample(10.01)
    sample.validate()
    assert sample.motor == (-1.,)*6
    assert sample.velocity == (-.125,)*6
    assert sample.joints == pytest.approx((math.radians(-57),)*6)


def test_missing_or_stale_axis_never_becomes_valid_from_other_frames():
    cache = EncoderCache()
    for i in range(6):
        cache.update(0x251+i, struct.pack(">hhi", 0, 0, 0), 10.)
    with pytest.raises(RuntimeError, match="Fresh"):
        cache.sample(10.01).validate()
    cache = fresh_cache()
    for i in range(6):
        cache.update(0x251+i, struct.pack(">hhi", 0, 0, 0), 11.)
    with pytest.raises(RuntimeError, match="Fresh"):
        cache.sample(11.).validate()


def test_invalid_frames_cannot_refresh_stale_samples():
    cache = fresh_cache()
    for can_id, payload in [(0x80000251, bytes(8)), (0x40000251, bytes(8)),
                            (0x20000251, bytes(8)), (0x251, bytes(7)), (0x151, bytes(8))]:
        assert not cache.update(can_id, payload, 11.)
    assert cache.sample(11.).age == 1.
    with pytest.raises(RuntimeError, match="receiver failed"):
        cache.sample(10.01, "socket closed").validate()


def test_kernel_timestamp_retains_queue_delay():
    ancillary = [(socket.SOL_SOCKET, SO_TIMESTAMPNS, struct.pack("=qq", 100, 500_000_000))]
    assert receive_timestamp(ancillary, realtime_now=101., monotonic_now=50.) == 49.5
    with pytest.raises(RuntimeError, match="timestamp"):
        receive_timestamp([], 101., 50.)


def test_conversion_slew_and_roundtrip_preserve_nonzero_held_target():
    mapper = CoordinateMapper(offset=(.01,)*6)
    held = (1., 1.5, -1.2, .7, .2, -.1)
    assert mapper.to_motor(mapper.to_joints(held)) == pytest.approx(held, abs=1e-15)
    sample = EncoderSample((.03,)*6, (0.,)*6, (0.,)*6, .01)
    mapper.advance(sample, .005)
    assert mapper.offset == pytest.approx((.010025,)*6)
    before = mapper.offset
    mapper.advance(sample, 2.)
    assert max(a-b for a,b in zip(mapper.offset, before)) <= .0001 + 1e-15


def test_failed_observation_cannot_corrupt_previous_conversion():
    mapper = CoordinateMapper(offset=(.01,)*6)
    held = mapper.to_motor((.2,)*6)
    for sample in [EncoderSample((.1,)*6, (0.,)*6, (0.,)*6, .01),
                   EncoderSample((.02,)*6, (0.,)*6, (0.,)*6, .2),
                   EncoderSample((math.nan,)*6, (0.,)*6, (0.,)*6, .01)]:
        with pytest.raises(RuntimeError):
            mapper.advance(sample, .005)
        assert mapper.to_motor((.2,)*6) == held
    with pytest.raises(ValueError):
        mapper.to_motor((0.,)*5 + (math.nan,))


def test_spatial_phase_filters_position_quantization_without_velocity_lag():
    import numpy as np
    from piper_control.backends.motor_feedback import EncoderSample, MotorPhaseEstimator

    estimator = MotorPhaseEstimator(10.)
    raw_error, filtered_error = [], []
    velocity = .061
    for tick in range(600):
        now = tick*.005
        received_at = (tick//2)*.01
        age = now-received_at
        true = velocity*now
        measured = round(velocity*received_at, 3)
        sample = EncoderSample((measured,)*6, (true,)*6, (velocity,)*6, age, motor_age=(age,)*6)
        phase = estimator.update(sample, .005)[0]
        if now > .5:
            raw_error.append(np.sin(540*(measured+velocity*age))-np.sin(540*true))
            filtered_error.append(np.sin(540*phase)-np.sin(540*true))
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    assert rms(filtered_error) < .7*rms(raw_error)
    assert abs(np.mean(filtered_error)) < .01


def test_spatial_phase_rejects_stale_axis_without_changing_estimate():
    from piper_control.backends.motor_feedback import EncoderSample, MotorPhaseEstimator

    estimator = MotorPhaseEstimator()
    valid = EncoderSample((0.,)*6, (0.,)*6, (0.,)*6, .001, motor_age=(.001,)*6)
    before = estimator.update(valid, .005)
    stale = EncoderSample((1.,)*6, (1.,)*6, (0.,)*6, .001, motor_age=(.001,)*5+(.2,))
    with pytest.raises(RuntimeError):
        estimator.update(stale, .005)
    assert estimator.position == before
