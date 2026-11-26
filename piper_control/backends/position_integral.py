"""Bounded slow position-error correction for residual load/friction bias."""
import math

from .motor_feedback import _six


class PositionIntegral:
    def __init__(self, gains=(0.,)*6, limit=.02, rate=.005, deadband=.001, bias=(0.,)*6,
                 max_velocity=None, motor_lead_limit=0., motor_lead_time=.5):
        self.gains, self.bias = _six(gains), _six(bias)
        self.limit, self.rate, self.deadband = map(float, (limit, rate, deadband))
        self.max_velocity = None if max_velocity is None else float(max_velocity)
        self.velocity_gated = False
        self.motor_lead_limit = float(motor_lead_limit)
        self.motor_lead_time = float(motor_lead_time)
        self.motor_lead = (0.,)*6
        self._anchors = [None]*6
        if (any(not 0 <= k <= 2 for k in self.gains)
                or not 0 < self.limit <= .03 or not 0 < self.rate <= .01
                or not .0001 <= self.deadband <= .01
                or max(map(abs, self.bias)) > self.limit
                or (self.max_velocity is not None and not 0 < self.max_velocity <= .5)
                or not 0 <= self.motor_lead_limit <= .005
                or not .05 <= self.motor_lead_time <= 2.):
            raise ValueError('Invalid bounded position integral configuration')

    def advance(self, desired, measured, dt, velocity=None, motor=None):
        desired, measured = _six(desired), _six(measured)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Integral timestep must be positive and finite')
        dt = min(dt, .02)
        if self.motor_lead_limit:
            if motor is None:
                raise ValueError('Motor lead requires motor position feedback')
            motor = _six(motor)
        if self.max_velocity is not None:
            if velocity is None:
                raise ValueError('Velocity-gated integral requires reference velocity')
            self.velocity_gated = max(map(abs, _six(velocity))) > self.max_velocity
            if self.velocity_gated:
                self._anchors = [None]*6
                self.motor_lead = (0.,)*6
                return
        if self.motor_lead_limit:
            lead = []
            for j, (q, m) in enumerate(zip(measured, motor)):
                anchor = self._anchors[j]
                if anchor is None or abs(q-anchor[0]) > 1e-10:
                    value = 0.
                else:
                    value = (self.motor_lead[j]+m-anchor[1])*math.exp(-dt/self.motor_lead_time)
                self._anchors[j] = (q, m)
                lead.append(max(-self.motor_lead_limit, min(self.motor_lead_limit, value)))
            # Output-angle packets can remain unchanged after the motor moves.
            # Account for that motion only inside the slow integral; the public
            # feedback and completion measurements remain untouched. Every new
            # output value reanchors the estimate, and its excursion is bounded.
            # Decay prevents stale output or elastic motor movement from leaving
            # a permanent correction that hides an unresolved output error.
            self.motor_lead = tuple(lead)
            measured = tuple(q+d for q, d in zip(measured, lead))
        error = [a-b for a, b in zip(desired, measured)]
        # A deadband avoids chasing encoder resolution/static breakaway. Both
        # the correction and its rate are bounded, including after an outage.
        residual = [math.copysign(max(0., abs(e)-self.deadband), e) for e in error]
        step = self.rate*dt
        self.bias = tuple(max(-self.limit, min(self.limit, b+max(-step, min(step, k*e*dt))))
                          for b, k, e in zip(self.bias, self.gains, residual))

    def apply(self, positions):
        return tuple(q+b for q, b in zip(_six(positions), self.bias))
