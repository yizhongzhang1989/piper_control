"""Opt-in PiPER tracking adapter; the upstream driver remains the CAN writer.

Public targets use output-joint coordinates. Only this boundary converts them
to motor coordinates. Feedforward vanishes at rest so the existing persistent
driver hold can retain exactly the same actuator target after API shutdown.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

from .motor_feedback import CoordinateMapper, MotorFeedbackReceiver, MotorPhaseEstimator, ZeroBandReconstructor, _six
from .position_integral import PositionIntegral


class ReferenceFeedforward:
    """Reuse the fitted vendor friction model with desired, not noisy measured, velocity."""

    def __init__(self, arm_config, config_path, settings):
        from piper_x_driver import _core
        from piper_x_driver.protocol import MIT_GAIN_SCALE
        from piper_x_driver.friction import load_friction_model, SpatialHarmonic
        from piper_x_driver.gravity import load_gravity_model

        allowed = {"friction_scale", "friction_factors", "velocity_remainder", "spatial_harmonics",
                   "spatial_prediction", "spatial_decay", "torque_limits", "phase_filter_hz", "dynamics"}
        if set(settings) - allowed:
            raise ValueError(f"Unknown tracking settings: {sorted(set(settings)-allowed)}")
        self.scale = float(settings.get("friction_scale", .88))
        self.friction_factors = _six(settings.get('friction_factors', [1.]*6))
        self.prediction = float(settings.get("spatial_prediction", .005))
        self.decay = float(settings.get("spatial_decay", .2))
        self.remainder = settings.get("velocity_remainder", True)
        cutoff = float(settings.get('phase_filter_hz', 0.))
        self.phase = None if cutoff == 0 else MotorPhaseEstimator(cutoff)
        self.limits = _six(settings.get("torque_limits", [4, 4, 4, .8, .5, .5]))
        if (not isinstance(self.remainder, bool) or not 0 <= self.scale <= 1
                or any(not 0 <= x <= 2 for x in self.friction_factors)
                or not 0 <= self.prediction <= .02 or not .01 <= self.decay <= 1
                or any(not 0 < x <= 4 for x in self.limits)):
            raise ValueError("Invalid tracking feedforward bounds")
        resolve = lambda key: str((Path(config_path).parent / arm_config[key]).resolve())
        self.friction = load_friction_model(resolve("friction_config"))
        gravity = load_gravity_model(resolve("gravity_config"))
        self.static = [gravity.joint(j).static_friction for j in range(1, 7)]
        self.harmonics = []
        amplitudes = [0.] * 6
        for entry in settings.get("spatial_harmonics", []):
            if set(entry) - {"joint", "omega", "a", "b", "c", "d"}:
                raise ValueError("Unknown spatial harmonic setting")
            joint = entry["joint"]
            if type(joint) is not int or joint not in range(1, 7):
                raise ValueError("Spatial harmonic joint must be an integer in 1..6")
            values = {key: float(entry.get(key, 0)) for key in ("omega", "a", "b", "c", "d")}
            if not all(math.isfinite(x) for x in values.values()) or not 0 < values["omega"] <= 2000:
                raise ValueError("Invalid spatial harmonic")
            amplitudes[joint-1] += math.hypot(values["a"], values["b"]) + math.hypot(values["c"], values["d"])
            self.harmonics.append((joint-1, SpatialHarmonic.from_dict(values)))
        if max(amplitudes) > .8:
            raise ValueError("Spatial harmonic amplitude bound exceeds 0.8 Nm")
        self.encode = _core.encode_mit_command
        self.gain_scale = MIT_GAIN_SCALE
        self.last = {"torque": [0.]*6, "spatial_torque": [0.]*6, "clipped": [False]*6}
        self.dynamics = None
        if settings.get('dynamics') is not None:
            from piper_control.model import RobotModel
            from .dynamics import DynamicsFeedforward
            self.dynamics = DynamicsFeedforward(
                RobotModel(end_effector=arm_config.get('end_effector', 'gripper_with_wrench')),
                **settings['dynamics'])

    def evaluate(self, target, velocity, state, sample, kp, kd, dt=.005, reference=None):
        target, velocity, kp, kd = map(_six, (target, velocity, kp, kd))
        sample.validate()
        phase = self.phase.update(sample, dt) if self.phase else None
        desired = [v if abs(v) > 1e-6 else 0. for v in velocity]
        gravity = _six(state["extra"]["driver"]["gravity_torque"])
        ff = list(self.friction.torques(
            state["positions"], desired, target_velocities=desired, gravity_torques=gravity,
            static_frictions=self.static, active_impedances=[True]*6,
            friction_scale=self.scale, cogging_scale=self.scale,
        ))
        ff = [factor*f for factor, f in zip(self.friction_factors, ff)]
        friction = [f if v else 0. for f, v in zip(ff, desired)]
        if self.remainder:
            # Use the vendor's encoder as a pure calculation, never to transmit.
            # Preserve the legacy zero-velocity wire bias, including during hold.
            frames = self.encode(list(target), list(velocity), list(kp), list(kd), [0.]*6)
            for i, frame in enumerate(frames):
                data = frame["data"]
                wire_v = ((data[2] << 4) | (data[3] >> 4)) * 90/4095 - 45
                wire_d = (((data[5] << 4) | (data[6] >> 4)) * 10/4095 - 5) * self.gain_scale[i]
                ff[i] += wire_d * (velocity[i] - wire_v - 45/4095)
        spatial = [0.]*6
        for i, harmonic in self.harmonics:
            if not desired[i]:
                continue
            theta = (phase[i] + velocity[i]*self.prediction if phase is not None else
                     sample.motor[i] + velocity[i]*(sample.age+self.prediction))
            direction = math.tanh(velocity[i]/.008)
            spatial[i] += ((harmonic.cogging(theta) + direction*harmonic.modulation(theta))
                           * abs(direction) * math.exp(-(velocity[i]/self.decay)**2))
        requested = [f+s if v else 0. for f, s, v in zip(ff, spatial, desired)]
        if self.dynamics:
            if reference is None:
                raise ValueError('Dynamics feedforward requires output-joint reference positions')
            dynamic = self.dynamics.evaluate(reference, velocity, dt)
            requested = [f+d for f, d in zip(requested, dynamic.tolist())]
        if not all(math.isfinite(x) for x in requested):
            raise RuntimeError("Nonfinite tracking feedforward")
        torque = [max(-limit, min(limit, f)) for f, limit in zip(requested, self.limits)]
        self.last = {"torque": torque, "spatial_torque": spatial,
                     "friction_torque": friction, "friction_factors": self.friction_factors,
                     "clipped": [a != b for a, b in zip(requested, torque)],
                     "motor_phase": phase, "phase_filter_hz": self.phase.cutoff_hz if self.phase else 0.}
        if self.dynamics:
            self.last['dynamics'] = self.dynamics.last.copy()
        return torque

    def close(self):
        if self.dynamics:
            self.dynamics.close()


class TrackingBackend:
    """Coordinate conversion, reference feedforward and restart continuity.

    A checkpoint retains public coordinates across API restarts. The native
    driver's held target is always authoritative; a missing checkpoint can
    change the displayed nominal target but cannot change the actuator target.
    """

    def __init__(self, raw, settings, *, receiver=None, feedforward=None, state_path=None):
        from piper_control.hold_guardian import state_directory

        self.raw = raw
        self._command_lock = threading.RLock()
        self._holding = False
        self._tracking_active = True
        settings = dict(settings)
        zero_band = float(settings.pop('feedback_zero_band', 0.))
        self._reconstruction = None if zero_band == 0 else ZeroBandReconstructor(zero_band)
        mapping = dict(settings.pop('coordinate_mapping', {}))
        if set(mapping)-{'time_aligned', 'rate', 'cutoff_hz'}:
            raise ValueError('Unknown coordinate mapping setting')
        self._aligned_mapping = mapping.pop('time_aligned', False)
        if (type(self._aligned_mapping) is not bool
                or not .001 <= mapping.get('rate', .005) <= .2
                or not 1 <= mapping.get('cutoff_hz', 1.) <= 10):
            raise ValueError('Invalid coordinate mapping bounds')
        self._mapping_options = mapping
        integral_settings = settings.pop('position_integral', {})
        self.integral = PositionIntegral(**integral_settings)
        self._hold_kd = _six(raw.kd)
        self._moving_kd = _six(settings.pop('moving_kd', self._hold_kd))
        self._gain_rate = float(settings.pop('gain_ramp_rate', 2.))
        if (not math.isfinite(self._gain_rate) or not .1 <= self._gain_rate <= 5
                or any(not 0 <= x <= 100 for x in self._hold_kd + self._moving_kd)
                or max(abs(h-m) for h, m in zip(self._hold_kd, self._moving_kd)) > 2*self._gain_rate):
            raise ValueError('Tracking damping requires gains in 0..100 and a 0.1..5 ramp reaching hold within 2 s')
        self._dt = .005
        self._feedforward_dt = .005
        self._last_velocity = (0.,)*6
        self._mapping_active = (False,)*6
        self.receiver = receiver or MotorFeedbackReceiver(raw.config["can_port"])
        self._done = threading.Event()
        self._file_lock = threading.Lock()
        self._checkpoint_error = None
        self._pending = None
        self._thread = None
        self._path = Path(state_path) if state_path else state_directory() / f"tracking_{raw.config['can_port']}.json"
        self._identity = {"version": 1, "namespace": raw.namespace, "can_port": raw.config["can_port"],
                          "boot_id": Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        try:
            until = time.monotonic() + 1.
            while True:
                sample = self.receiver.sample()
                try:
                    sample.validate()
                    break
                except RuntimeError:
                    if time.monotonic() >= until:
                        raise
                    time.sleep(.005)
            initial = _six(raw.initial_target)
            offset = tuple(m-q for m, q in zip(sample.motor, sample.joints))
            try:
                saved = json.loads(self._path.read_text())
                if (all(saved.get(k) == v for k, v in self._identity.items())
                        and max(abs(a-b) for a, b in zip(_six(saved['actuator_target']), initial)) < 1e-8):
                    candidate = CoordinateMapper(saved['offset'], **self._mapping_options)
                    integral = PositionIntegral(**dict(integral_settings, bias=saved.get('tracking_bias', [0.]*6)))
                    if max(abs(a-b) for a, b in zip(integral.apply(candidate.to_motor(saved['nominal_target'])), initial)) < 1e-8:
                        offset = candidate.offset
                        self.integral = integral
                        if self._reconstruction and 'zero_feedback' in saved:
                            self._reconstruction.restore(saved['zero_feedback'])
            except (OSError, KeyError, ValueError, TypeError):
                pass
            self.mapper = CoordinateMapper(offset, **self._mapping_options)
            if self._reconstruction:
                self._reconstruction.observe(sample)
            enabled = raw.read()['enabled']
            corrected = tuple(q-b for q, b in zip(initial, self.integral.bias))
            self.initial_target = list(self.mapper.to_joints(corrected) if enabled else sample.joints)
            self._nominal = list(self.initial_target)
            self._actuator = list(initial)
            self.feedforward = feedforward or ReferenceFeedforward(
                raw.config, raw.workspace / 'config/arms' / f'{raw.arm_id}.yaml', settings)
            self._freeze_until = time.monotonic() + 6.
            self._observer_frozen = True
            self._queue_checkpoint()
            self._thread = threading.Thread(target=self._checkpoint_loop, name='piper-tracking-checkpoint', daemon=True)
            self._thread.start()
        except Exception:
            self.receiver.close()
            raise

    def __getattr__(self, name):
        return getattr(self.raw, name)

    def _queue_checkpoint(self):
        self._pending = dict(self._identity, offset=list(self.mapper.offset),
                             tracking_bias=list(self.integral.bias),
                             nominal_target=list(self._nominal), actuator_target=list(self._actuator))
        if self._reconstruction:
            self._pending['zero_feedback'] = self._reconstruction.checkpoint()

    def _checkpoint(self):
        with self._file_lock:
            payload = self._pending
            self._path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._path.with_suffix('.tmp')
            temporary.write_text(json.dumps(payload))
            temporary.replace(self._path)

    def _checkpoint_loop(self):
        while not self._done.wait(.1):
            try:
                self._checkpoint()
                self._checkpoint_error = None
            except OSError as exc:
                self._checkpoint_error = str(exc)

    def step(self, dt):
        self.raw.step(dt)
        with self._command_lock:
            if self._holding:
                return
            self._dt = min(.02, max(.00001, dt))
            self._feedforward_dt = dt
            # IK spline roundoff can produce a few microradians/s on a nominally
            # stationary axis. Do not let it unlock the faster aligned mapper.
            threshold = 1e-4 if self._aligned_mapping else 1e-6
            active = tuple(abs(v) > threshold for v in self._last_velocity)
            warming = time.monotonic() < self._freeze_until
            align = self._aligned_mapping and any(active) and not warming
            sample = self.receiver.sample_aligned() if align else self.receiver.sample()
            sample.validate()
            if self._reconstruction:
                if align or max(map(abs, sample.velocity)) <= .02:
                    self._reconstruction.observe(sample)
                inferred = self._reconstruction.affected(sample)
                active = tuple(moving and not estimated for moving, estimated in zip(active, inferred))
                sample = self._reconstruction.correct(sample, self.mapper.offset)
            # The legacy estimate freezes at high speed. Aligned histories can
            # follow coordinate changes without learning packet arrival skew.
            self._observer_frozen = (warming
                                    or (not self._aligned_mapping and max(map(abs, sample.velocity)) > .2)
                                    or not any(active))
            self._mapping_active = (False,)*6 if self._observer_frozen else active
            if not self._observer_frozen:
                # Following a stationary motor's quantized motion would move
                # its target again and cancel part of the inner position loop.
                # Keep the last load-dependent conversion during static hold.
                self.mapper.advance(sample, dt, active)

    def command(self, q, dq=None):
        with self._command_lock:
            return self._command(q, dq)

    def set_tracking_active(self, active):
        with self._command_lock:
            self._tracking_active = bool(active)
            if not active:
                self._last_velocity = (0.,)*6

    def _command(self, q, dq=None):
        q, dq = _six(q), _six([0.]*6 if dq is None else dq)
        if self._holding:
            if max(abs(a-b) for a, b in zip(q, self._nominal)) > 1e-10 or any(dq):
                raise RuntimeError('Hardware hold handoff is in progress')
            self.raw.command(self._actuator, [0.]*6, torque=[0.]*6)
            return
        sample = self.receiver.sample()
        sample.validate()
        if self._reconstruction:
            sample = self._reconstruction.correct(sample, self.mapper.offset)
        # Only a commanded moving axis needs its motion damping. The measured
        # velocity must not keep high damping active during a parking vibration.
        wanted = [h+(m-h)*math.tanh(abs(v)/.003)
                  for h, m, v in zip(self._hold_kd, self._moving_kd, dq)]
        self._ramp_damping(wanted, self._dt)
        state = self.raw.read()
        if self._reconstruction:
            state['positions'] = list(sample.joints)
        if (self._tracking_active and time.monotonic() >= self._freeze_until and state.get('command_ready', False)
                and state.get('connected', True) and state['enabled']):
            self.integral.advance(q, sample.joints, self._dt, dq)
        target = self.integral.apply(self.mapper.to_motor(q))
        kd = state['extra']['driver'].get('applied_kd', self.raw.kd)
        torque = self.feedforward.evaluate(target, dq, state, sample, self.raw.kp, kd, self._feedforward_dt, q)
        self.raw.command(target, dq, torque=torque)
        self._last_velocity = dq
        self._nominal, self._actuator = list(q), list(target)
        self._queue_checkpoint()

    def _ramp_damping(self, wanted, dt):
        step = self._gain_rate*min(.02, max(0., dt))
        self.raw.kd = [g+max(-step, min(step, w-g)) for g, w in zip(self.raw.kd, wanted)]

    def read(self):
        state = self.raw.read()
        sample = self.receiver.sample()
        raw_joints = sample.joints
        try:
            sample.validate()
            if self._reconstruction:
                inferred = self._reconstruction.affected(sample)
                sample = self._reconstruction.correct(sample, self.mapper.offset)
                state['positions'] = list(sample.joints)
                state['extra']['feedback_reconstruction'] = dict(
                    zero_band_rad=self._reconstruction.band, raw_positions=list(raw_joints),
                    inferred=list(inferred), positions=list(sample.joints),
                    anchored=list(self._reconstruction.anchored),
                    offset=list(self._reconstruction.offset), motor_resolution_rad=.001,
                    note='Inside firmware zero band: motor-based estimate. Unanchored axes use the saved mapping until a nonzero output is observed. Not independent output metrology.',
                )
        except RuntimeError as exc:
            state['connected'] = False
            state.setdefault('errors', []).append(str(exc))
        state['extra']['motor_mapping'] = dict(
            enabled=True, motor_positions=sample.motor, joint_positions=sample.joints,
            feedback_age=sample.age if math.isfinite(sample.age) else None,
            offset=list(self.mapper.offset), nominal_target=list(self._nominal),
            actuator_target=list(self._actuator), observer_frozen=self._observer_frozen,
            checkpoint_error=self._checkpoint_error, handoff_in_progress=self._holding,
            observer_active_joints=list(self._mapping_active),
            time_aligned=self._aligned_mapping, rate=self.mapper.rate, cutoff_hz=self.mapper.cutoff_hz,
        )
        state['extra']['tracking_feedforward'] = self.feedforward.last.copy()
        state['extra']['tracking_damping'] = dict(hold=self._hold_kd, moving=self._moving_kd,
                                                 command=list(self.raw.kd), ramp_rate=self._gain_rate)
        state['extra']['position_integral'] = dict(gains=self.integral.gains, bias=self.integral.bias,
                                                  limit=self.integral.limit, rate=self.integral.rate,
                                                  deadband=self.integral.deadband, active=self._tracking_active,
                                                  max_velocity=self.integral.max_velocity,
                                                  velocity_gated=self._tracking_active and self.integral.velocity_gated)
        return state

    def enable(self, enabled):
        state = self.raw.read()
        if enabled and state['enabled'] and state['command_ready']:
            return
        if enabled:
            sample = self.receiver.sample()
            sample.validate()
            self.mapper = CoordinateMapper(tuple(m-q for m, q in zip(sample.motor, sample.joints)), **self._mapping_options)
            self.integral.bias = (0.,)*6
            self._freeze_until = time.monotonic()+6.
            # The controller chose this same public joint snapshot before enable.
            target = self.mapper.to_motor(state['positions'])
            self.raw.enable(True, hold_target=target)
            self._nominal, self._actuator = list(state['positions']), list(target)
            self._queue_checkpoint()
        else:
            self.raw.enable(False)

    def hold(self):
        # Raw backend owns the actuator hold; never apply the offset twice.
        # The controller's keepalive thread is still running during handoff.
        # Freeze observers/integrators first; both publishers must send the same
        # actuator target even when the public target has a residual error.
        with self._command_lock:
            self._holding = True
            self._observer_frozen = True
            self._mapping_active = (False,)*6
            self._queue_checkpoint()
        try:
            self._checkpoint()
        except OSError as exc:
            self._checkpoint_error = str(exc)
        if not self.raw.read()['enabled']:
            self.raw.kd = list(self._hold_kd)
        while max(abs(a-b) for a, b in zip(self.raw.kd, self._hold_kd)) > 1e-10:
            with self._command_lock:
                self._ramp_damping(self._hold_kd, .005)
                self.raw.command(self._actuator, [0.]*6, torque=[0.]*6)
            time.sleep(.005)
        # A failed handoff may still have started another hold owner. Continue
        # the identical frozen keepalive until handoff can be retried.
        self.raw.hold()

    def close(self):
        self._done.set()
        if self._thread:
            self._thread.join(timeout=.5)
        try:
            self._checkpoint()
        finally:
            self.receiver.close()
            try:
                close = getattr(self.feedforward, 'close', None)
                if close:
                    close()
            finally:
                self.raw.close()
