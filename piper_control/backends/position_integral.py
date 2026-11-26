"""Bounded slow position-error correction for residual load/friction bias."""
import math

from .motor_feedback import _six


class PositionIntegral:
    def __init__(self, gains=(0.,)*6, limit=.02, rate=.005, deadband=.001, bias=(0.,)*6):
        self.gains, self.bias = _six(gains), _six(bias)
        self.limit, self.rate, self.deadband = map(float, (limit, rate, deadband))
        if (any(not 0 <= k <= 2 for k in self.gains)
                or not 0 < self.limit <= .03 or not 0 < self.rate <= .01
                or not .0001 <= self.deadband <= .01
                or max(map(abs, self.bias)) > self.limit):
            raise ValueError('Invalid bounded position integral configuration')

    def advance(self, desired, measured, dt):
        desired, measured = _six(desired), _six(measured)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Integral timestep must be positive and finite')
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
