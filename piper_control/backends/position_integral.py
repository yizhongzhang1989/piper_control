"""Bounded slow position-error correction for residual load/friction bias."""
import math

from .motor_feedback import _six


class PositionIntegral:
    def __init__(self, gains=(0.,)*6, limit=.02, rate=.005, deadband=.001, bias=(0.,)*6,
                 max_velocity=None):
        self.gains, self.bias = _six(gains), _six(bias)
        self.limit, self.rate, self.deadband = map(float, (limit, rate, deadband))
        self.max_velocity = None if max_velocity is None else float(max_velocity)
        self.velocity_gated = False
        if (any(not 0 <= k <= 2 for k in self.gains)
                or not 0 < self.limit <= .03 or not 0 < self.rate <= .01
                or not .0001 <= self.deadband <= .01
                or max(map(abs, self.bias)) > self.limit
                or (self.max_velocity is not None and not 0 < self.max_velocity <= .5)):
            raise ValueError('Invalid bounded position integral configuration')

    def advance(self, desired, measured, dt, velocity=None):
        desired, measured = _six(desired), _six(measured)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Integral timestep must be positive and finite')
        if self.max_velocity is not None:
            if velocity is None:
                raise ValueError('Velocity-gated integral requires reference velocity')
            self.velocity_gated = max(map(abs, _six(velocity))) > self.max_velocity
            if self.velocity_gated:
                return
        dt = min(dt, .02)
        error = [a-b for a, b in zip(desired, measured)]
        # A deadband avoids chasing encoder resolution/static breakaway. Both
        # the correction and its rate are bounded, including after an outage.
        residual = [math.copysign(max(0., abs(e)-self.deadband), e) for e in error]
        step = self.rate*dt
        self.bias = tuple(max(-self.limit, min(self.limit, b+max(-step, min(step, k*e*dt))))
                          for b, k, e in zip(self.bias, self.gains, residual))

    def apply(self, positions):
        return tuple(q+b for q, b in zip(_six(positions), self.bias))
