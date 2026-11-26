"""Receive PiPER motor/output encoder feedback without transmitting CAN frames.

The MIT servo and the public joint feedback use distinct position signals. Keep
their conversion at the hardware boundary; planners always use output joints.
"""
from dataclasses import dataclass
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

    def validate(self, timeout=.1):
        if self.error:
            raise RuntimeError(f"Motor feedback receiver failed: {self.error}")
        if (not math.isfinite(self.age) or self.age < 0 or self.age > timeout
                or any(len(values) != 6 or any(v is None or not math.isfinite(v) for v in values)
                       for values in (self.motor, self.joints, self.velocity))):
            raise RuntimeError("Fresh motor and output encoder feedback is required")


class EncoderCache:
    """Decode complete frames; every required frame has its own freshness clock."""

    def __init__(self):
        self.motor = [None] * 6
        self.joints = [None] * 6
        self.velocity = [None] * 6
        self.stamps = [None] * 9

    def update(self, can_id, payload, stamp):
        # Reject extended, error and RTR frames even if their low ID bits match.
        if (can_id not in MOTOR_IDS + JOINT_IDS or len(payload) != 8
                or not math.isfinite(stamp)):
            return False
        if can_id in MOTOR_IDS:
            i = can_id - MOTOR_IDS[0]
            velocity, _current, position = struct.unpack(">hhi", payload)
            self.motor[i], self.velocity[i] = position * .001, velocity * .001
            self.stamps[i] = stamp
        else:
            i = can_id - JOINT_IDS[0]
            self.joints[2*i:2*i+2] = [math.radians(v * .001) for v in struct.unpack(">ii", payload)]
            self.stamps[6+i] = stamp
        return True

    def sample(self, now, error=None):
        age = math.inf if any(t is None for t in self.stamps) else now - min(self.stamps)
        return EncoderSample(tuple(self.motor), tuple(self.joints), tuple(self.velocity), age, error)


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

    def close(self):
        self._done.set()
        self._thread.join(timeout=.2)
        self._socket.close()


def _six(values):
    values = tuple(map(float, values))
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected six finite joint values")
    return values


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

    def advance(self, sample, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Mapping timestep must be positive and finite")
        sample.validate()
        goal = tuple(m-q for m, q in zip(sample.motor, sample.joints))
        if max(map(abs, goal)) > self.max_offset:
            raise RuntimeError("Motor/output encoder difference exceeds the configured bound")
        # A scheduling outage must not be repaid as a large position step.
        dt = min(dt, .02)
        alpha = -math.expm1(-2 * math.pi * self.cutoff_hz * dt)
        step = self.rate * dt
        self.offset = tuple(o + max(-step, min(step, alpha*(g-o))) for o, g in zip(self.offset, goal))

    def to_motor(self, joints):
        return tuple(q+o for q, o in zip(_six(joints), self.offset))

    def to_joints(self, motor):
        return tuple(q-o for q, o in zip(_six(motor), self.offset))
