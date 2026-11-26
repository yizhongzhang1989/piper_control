"""Receive PiPER motor/output encoder feedback without transmitting CAN frames.

The MIT servo and the public joint feedback use distinct position signals. Keep
their conversion at the hardware boundary; planners always use output joints.
"""
from dataclasses import dataclass, replace
from collections import deque
import math
import socket
import struct
import threading
import time


MOTOR_IDS = tuple(range(0x251, 0x257))
JOINT_IDS = tuple(range(0x2A5, 0x2A8))
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)


def receive_timestamp(ancillary, realtime_now, monotonic_now):
    """Convert Linux's kernel RX time; queued old packets must remain stale."""
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(data) >= 16:
            seconds, nanos = struct.unpack("=qq", data[:16])
            if seconds >= 0 and 0 <= nanos < 1_000_000_000:
                return monotonic_now + (seconds + nanos * 1e-9 - realtime_now)
    raise RuntimeError("Missing kernel timestamp on motor feedback")


@dataclass(frozen=True)
class EncoderSample:
    motor: tuple
    joints: tuple
    velocity: tuple
    age: float
    error: str | None = None
    motor_age: tuple = ()

    def validate(self, timeout=.1):
        if self.error:
            raise RuntimeError(f"Motor feedback receiver failed: {self.error}")
        if (not math.isfinite(self.age) or self.age < 0 or self.age > timeout
                or any(len(values) != 6 or any(v is None or not math.isfinite(v) for v in values)
                       for values in (self.motor, self.joints, self.velocity))):
            raise RuntimeError("Fresh motor and output encoder feedback is required")
        if self.motor_age and (len(self.motor_age) != 6 or any(
                not math.isfinite(v) or not 0 <= v <= timeout for v in self.motor_age)):
            raise RuntimeError('Fresh per-motor timestamps are required')


class EncoderCache:
    """Decode complete frames; every required frame has its own freshness clock."""

    def __init__(self):
        self.motor = [None] * 6
        self.joints = [None] * 6
        self.velocity = [None] * 6
        self.stamps = [None] * 9
        self.history = [deque(maxlen=32) for _ in self.stamps]

    def update(self, can_id, payload, stamp):
        # Reject extended, error and RTR frames even if their low ID bits match.
        if (can_id not in MOTOR_IDS + JOINT_IDS or len(payload) != 8
                or not math.isfinite(stamp)):
            return False
        index = can_id-MOTOR_IDS[0] if can_id in MOTOR_IDS else 6+can_id-JOINT_IDS[0]
        if self.stamps[index] is not None and stamp <= self.stamps[index]:
            return False
        if can_id in MOTOR_IDS:
            i = can_id - MOTOR_IDS[0]
            velocity, _current, position = struct.unpack(">hhi", payload)
            self.motor[i], self.velocity[i] = position * .001, velocity * .001
            self.stamps[i] = stamp
            self.history[i].append((stamp, self.motor[i], self.velocity[i]))
        else:
            i = can_id - JOINT_IDS[0]
            self.joints[2*i:2*i+2] = [math.radians(v * .001) for v in struct.unpack(">ii", payload)]
            self.stamps[6+i] = stamp
            self.history[6+i].append((stamp, *self.joints[2*i:2*i+2]))
        return True

    def sample(self, now, error=None):
        age = math.inf if any(t is None for t in self.stamps) else now - min(self.stamps)
        motor_age = tuple(math.inf if stamp is None else now-stamp for stamp in self.stamps[:6])
        return EncoderSample(tuple(self.motor), tuple(self.joints), tuple(self.velocity), age, error, motor_age)

    def aligned_sample(self, now, error=None):
        """Interpolate all nine groups at one past RX time; never extrapolate.

        At 3 rad/s, a 9 ms arrival skew alone creates a false 0.027 rad
        encoder offset. This removes that skew before estimating coordinates.
        RX alignment cannot remove unknown motor-internal sampling delays.
        """
        try:
            if any(t is None for t in self.stamps):
                raise ValueError('incomplete encoder history')
            stamp = min(self.stamps)
            values = []
            for history in self.history:
                if not history or history[0][0] > stamp:
                    raise ValueError('encoder history does not cover common time')
                left = history[0]
                value = None
                for right in history:
                    if right[0] == stamp:
                        value = right[1:]
                        break
                    if right[0] > stamp:
                        gap = right[0]-left[0]
                        if gap > .05:
                            raise ValueError('encoder interpolation gap exceeds 50 ms')
                        u = (stamp-left[0])/gap
                        value = tuple(a+u*(b-a) for a, b in zip(left[1:], right[1:]))
                        break
                    left = right
                if value is None:
                    raise ValueError('encoder interpolation would extrapolate')
                values.append(value)
            age = now-stamp
            return EncoderSample(tuple(v[0] for v in values[:6]),
                                 tuple(q for v in values[6:] for q in v),
                                 tuple(v[1] for v in values[:6]), age, error, (age,)*6)
        except ValueError as exc:
            return self.sample(now, error or str(exc))


class MotorFeedbackReceiver:
    """Linux SocketCAN receiver. This class has no command/transmit operation."""

    def __init__(self, interface):
        self._cache = EncoderCache()
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._error = None
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # Match EFF/RTR plus all standard ID bits. CAN_ERR_FLAG must not be
            # in this mask: Linux routes such filters to its error-frame list.
            # Error-frame reception is disabled unless CAN_RAW_ERR_FILTER is set.
            filters = b"".join(struct.pack("=II", i, 0xC00007FF) for i in MOTOR_IDS + JOINT_IDS)
            self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self._socket.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            self._socket.bind((interface,))
            self._socket.settimeout(.05)
        except Exception:
            self._socket.close()
            raise
        self._thread = threading.Thread(target=self._receive, name="piper-encoders", daemon=True)
        self._thread.start()

    def _receive(self):
        while not self._done.is_set():
            try:
                packet, ancillary, _flags, _address = self._socket.recvmsg(16, socket.CMSG_SPACE(16))
            except TimeoutError:
                continue
            except OSError as exc:
                if not self._done.is_set():
                    with self._lock:
                        self._error = str(exc)
                return
            if len(packet) != 16:
                continue
            can_id, length, payload = struct.unpack("=IB3x8s", packet)
            if length == 8:
                try:
                    stamp = receive_timestamp(ancillary, time.time(), time.monotonic())
                except RuntimeError as exc:
                    with self._lock:
                        self._error = str(exc)
                    return
                with self._lock:
                    self._cache.update(can_id, payload, stamp)

    def sample(self):
        with self._lock:
            return self._cache.sample(time.monotonic(), self._error)

    def sample_aligned(self):
        with self._lock:
            return self._cache.aligned_sample(time.monotonic(), self._error)

    def close(self):
        self._done.set()
        self._thread.join(timeout=.2)
        self._socket.close()


def _six(values):
    values = tuple(map(float, values))
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected six finite joint values")
    return values


class ZeroBandReconstructor:
    """Bounded motor-based position estimate inside a firmware zero-report band.

    PiPER firmware can report exactly zero over +/-0.3 degrees. An interpolated
    transition into that band is equally unsuitable for learning encoder offsets.
    Use the last valid motor/output conversion, never update it from these values,
    and expose the estimate as such. This cannot replace external metrology.
    """

    def __init__(self, band):
        self.band = float(band)
        if not math.isfinite(self.band) or not 0 < self.band <= .01:
            raise ValueError('Firmware zero band must be within (0, 0.01] rad')
        self.offset = (0.,)*6
        self.anchored = (False,)*6

    def observe(self, sample):
        """Anchor only on an output reading outside the censored interval."""
        sample.validate()
        offsets, anchored = list(self.offset), list(self.anchored)
        for i, hidden in enumerate(self.affected(sample)):
            if not hidden:
                value = sample.motor[i]-sample.joints[i]
                if abs(value) > .05:
                    raise RuntimeError('Zero-band anchor exceeds encoder conversion bound')
                offsets[i], anchored[i] = value, True
        self.offset, self.anchored = tuple(offsets), tuple(anchored)

    def restore(self, saved):
        offsets = _six(saved['offset'])
        anchored = tuple(saved['anchored'])
        if (saved['band'] != self.band or max(map(abs, offsets)) > .05
                or len(anchored) != 6 or any(type(v) is not bool for v in anchored)):
            raise ValueError('Invalid zero-band checkpoint')
        self.offset, self.anchored = offsets, anchored

    def checkpoint(self):
        return dict(band=self.band, offset=list(self.offset), anchored=list(self.anchored))

    def affected(self, sample):
        return tuple(abs(q) < self.band for q in sample.joints)

    def correct(self, sample, offset):
        sample.validate()
        offset = _six(offset)
        affected = self.affected(sample)
        values = list(sample.joints)
        for i, infer in enumerate(affected):
            if not infer:
                continue
            candidate = sample.motor[i]-(self.offset[i] if self.anchored[i] else offset[i])
            # Accommodate motor quantization and normal asynchronous arrivals,
            # but do not turn an arbitrary/malformed zero into a valid angle.
            margin = .002+abs(sample.velocity[i])*.025
            if abs(candidate) > self.band+margin:
                raise RuntimeError(f'Zero joint feedback is inconsistent with motor feedback on joint{i+1}')
            values[i] = max(-self.band, min(self.band, candidate))
        return replace(sample, joints=tuple(values))


class CoordinateMapper:
    """Bounded slow estimate of motor angle minus output joint angle.

    It is a live coordinate conversion, not a fixed zero calibration. A failed
    observation leaves the last conversion intact so a held actuator target can
    be preserved. Offset derivatives must not enter desired joint velocities.
    """

    def __init__(self, offset=(0.,)*6, *, max_offset=.05, rate=.005, cutoff_hz=1.):
        if not all(math.isfinite(v) and v > 0 for v in (max_offset, rate, cutoff_hz)):
            raise ValueError("Coordinate mapping bounds must be positive and finite")
        self.max_offset, self.rate, self.cutoff_hz = max_offset, rate, cutoff_hz
        self.offset = _six(offset)
        if max(map(abs, self.offset)) > max_offset:
            raise ValueError("Initial encoder offset exceeds the configured bound")

    def advance(self, sample, dt, active=(True,)*6):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Mapping timestep must be positive and finite")
        sample.validate()
        if len(active) != 6 or any(type(value) is not bool for value in active):
            raise ValueError('Coordinate update mask must contain six booleans')
        goal = tuple(m-q for m, q in zip(sample.motor, sample.joints))
        if max(map(abs, goal)) > self.max_offset:
            raise RuntimeError("Motor/output encoder difference exceeds the configured bound")
        # A scheduling outage must not be repaid as a large position step.
        dt = min(dt, .02)
        alpha = -math.expm1(-2 * math.pi * self.cutoff_hz * dt)
        step = self.rate * dt
        self.offset = tuple(o + max(-step, min(step, alpha*(g-o))) if moving else o
                            for o, g, moving in zip(self.offset, goal, active))

    def to_motor(self, joints):
        return tuple(q+o for q, o in zip(_six(joints), self.offset))

    def to_joints(self, motor):
        return tuple(q-o for q, o in zip(_six(motor), self.offset))


class MotorPhaseEstimator:
    """Dequantize spatial phase using measured velocity plus bounded-bandwidth correction.

    Position feedback is only 1 mrad; at omega=540 this is a 0.54 rad phase step.
    Predict to the current time from each motor's kernel RX stamp, propagate with
    measured velocity, then correct the slow error using position observations.
    This estimator changes only bounded spatial feedforward, never servo targets.
    """

    def __init__(self, cutoff_hz=10.):
        if not math.isfinite(cutoff_hz) or not 1 <= cutoff_hz <= 30:
            raise ValueError('Motor phase cutoff must be within 1..30 Hz')
        self.cutoff_hz = cutoff_hz
        self.position = None

    def update(self, sample, dt):
        sample.validate()
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Phase timestep must be positive and finite')
        dt = min(dt, .02)
        ages = sample.motor_age or (sample.age,)*6
        observed = tuple(q+v*age for q, v, age in zip(sample.motor, sample.velocity, ages))
        if self.position is None:
            self.position = observed
        else:
            alpha = -math.expm1(-2*math.pi*self.cutoff_hz*dt)
            predicted = tuple(q+v*dt for q, v in zip(self.position, sample.velocity))
            self.position = tuple(p+alpha*(o-p) for p, o in zip(predicted, observed))
        return self.position
