"""Hardware boundary tests with fake feedback: no ROS publisher or CAN socket."""
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest
import yaml

from piper_control.backends.motor_feedback import EncoderSample
from piper_control.backends.tracking import ReferenceFeedforward, TrackingBackend


Q = (0., 1.53, -1.24, 1.35, 0., 0.)
OFFSET = (.001, -.02, -.008, .001, -.004, -.002)
MOTOR = tuple(q+o for q, o in zip(Q, OFFSET))


class Raw:
    config = {'can_port': 'fake_tracking'}
    namespace = '/test_tracking'
    kp, kd = [51, 64, 60, 32, 3, 3], [10, 12, 20, 1.6, .3, .08]

    def __init__(self):
        self.initial_target = list(MOTOR)
        self.sent = []
        self.enabled = True
        self.enable_calls = []
        self.held = None

    def read(self):
        return dict(positions=list(Q), enabled=self.enabled, command_ready=self.enabled,
                    connected=True, feedback_age=.001,
                    extra={'driver': {'applied_kd': self.kd, 'gravity_torque': [0, -7, -6, -.1, 0, 0]}})

    def step(self, dt):
        pass

    def command(self, q, dq=None, torque=None):
        self.sent.append((tuple(q), tuple(dq), tuple(torque)))

    def enable(self, enabled, hold_target=None):
        self.enable_calls.append((enabled, hold_target))
        self.enabled = enabled

    def hold(self):
        self.held = self.sent[-1][0]

    def close(self):
        pass


class Receiver:
    value = EncoderSample(MOTOR, Q, (0.,)*6, .001)

    def sample(self):
        return self.value

    def close(self):
        pass


class Feedforward:
    last = {}

    def evaluate(self, *args):
        return [0.]*6


def backend(tmp_path):
    return TrackingBackend(Raw(), {}, receiver=Receiver(), feedforward=Feedforward(),
                           state_path=tmp_path/'coordinates.json')


def test_stationary_encoder_changes_cannot_move_a_held_actuator_target(tmp_path):
    b = backend(tmp_path)
    try:
        b._freeze_until = 0
        b.command(Q)
        held = b.raw.sent[-1][0]
        offset = b.mapper.offset
        b.receiver.value = EncoderSample(tuple(m+.004 for m in MOTOR), Q, (0.,)*6, .001)
        for _ in range(100):
            b.step(.005)
            b.command(Q)
        assert b.raw.sent[-1][0] == held
        assert b.mapper.offset == offset
        assert b.read()['extra']['motor_mapping']['observer_frozen']
        # A moving axis can update, while all other axes keep their conversion.
        b.command(Q, [.02, 0, 0, 0, 0, 0])
        b.step(.005)
        assert b.mapper.offset[0] > offset[0]
        assert b.mapper.offset[1:] == offset[1:]
        b.command(Q)
        offset = b.mapper.offset
        b.step(.005)
        assert b.mapper.offset == offset
    finally:
        b.close()


@pytest.mark.parametrize('ending', ['succeeded', 'aborted', 'stop', 'watchdog'])
def test_terminal_command_freezes_residual_integral_until_next_motion(tmp_path, ending):
    from piper_control.controller import RobotController
    from piper_control.model import RobotModel

    b = TrackingBackend(Raw(), {'position_integral': {'gains': [1.]*6, 'deadband': .003}},
                        receiver=Receiver(), feedforward=Feedforward(), state_path=tmp_path/'hold.json')
    b._freeze_until = 0
    controller = RobotController(b, RobotModel(), goal_tolerance=.005, goal_timeout=.2)
    try:
        target = [q+(.004 if i == 0 else 0.) for i, q in enumerate(Q)]
        if ending == 'aborted':
            target[0] += .01
        if ending == 'watchdog':
            controller.servo_joints(target)
            begin = controller._stream_time
        else:
            controller.movej(target, duration=.5)
            begin = controller._start
        for i in range(1, 100 if ending != 'watchdog' else 30):
            controller._tick(begin+i*.005, .005)
        assert any(abs(x) > 0 for x in b.integral.bias)
        if ending == 'stop':
            controller.stop()
        controller._tick(begin+.71, .005)
        assert controller.state()['mode'] == 'idle'
        if ending in ('succeeded', 'aborted'):
            assert controller.state()['motion']['status'] == ending
        saved_bias, saved_target = b.integral.bias, b.raw.sent[-1][0]
        # Residual error and changing feedback used to accumulate integral after
        # completion, eventually breaking stiction seconds after "succeeded".
        b.receiver.value = EncoderSample(tuple(m-.002 for m in MOTOR),
                                         tuple(q-.002 for q in Q), (0.,)*6, .001)
        for i in range(400):
            controller._tick(begin+1+i*.005, .005)
        assert b.integral.bias == saved_bias
        assert b.raw.sent[-1][0] == saved_target
        assert not controller.state()['extra']['position_integral']['active']
        controller.movej([q+.015 for q in target], duration=1.)
        controller._tick(controller._start+.8, .005)
        assert b.integral.bias != saved_bias
        assert controller.state()['extra']['position_integral']['active']
    finally:
        controller.close()


def test_coordinate_target_and_restart_hold_do_not_double_apply_offset(tmp_path):
    first = backend(tmp_path)
    target = [v+.003 for v in Q]
    try:
        assert first.initial_target == pytest.approx(Q)
        first.command(target)
        first.hold()
        held = first.raw.held
        assert held == pytest.approx([q+o for q, o in zip(target, OFFSET)])
    finally:
        first.close()
    raw = Raw()
    raw.initial_target = held
    # Changed measurement must not recapture a different public goal on restart.
    receiver = Receiver()
    receiver.value = EncoderSample(tuple(m+.004 for m in MOTOR), Q, (0.,)*6, .001)
    second = TrackingBackend(raw, {}, receiver=receiver, feedforward=Feedforward(),
                             state_path=tmp_path/'coordinates.json')
    try:
        assert second.initial_target == pytest.approx(target)
        second.command(second.initial_target)
        second.hold()
        assert second.raw.held == pytest.approx(held)
    finally:
        second.close()


def test_stale_feedback_rejects_command_and_preserves_last_raw_hold(tmp_path):
    b = backend(tmp_path)
    try:
        b.command(Q)
        previous = b.raw.sent.copy()
        b.receiver.value = EncoderSample(MOTOR, Q, (0.,)*6, .2)
        with pytest.raises(RuntimeError, match='Fresh'):
            b.command([x+.01 for x in Q])
        assert b.raw.sent == previous
        b.hold()
        assert b.raw.held == pytest.approx(MOTOR)
    finally:
        b.close()


def test_fast_feedback_freezes_observer_and_repeated_enable_keeps_goal(tmp_path):
    b = backend(tmp_path)
    try:
        b._freeze_until = 0
        b.receiver.value = EncoderSample(tuple(m+.01 for m in MOTOR), Q, (1.,)*6, .001)
        b.step(.005)
        assert b.mapper.offset == pytest.approx(OFFSET)
        b.enable(True)
        assert not b.raw.enable_calls
        b.receiver.value = EncoderSample(MOTOR, Q, (0.,)*6, .001)
        b.raw.enabled = False
        b.enable(True)
        assert b.raw.enable_calls[-1][1] == pytest.approx(MOTOR)
    finally:
        b.close()


def test_checkpoint_failure_cannot_prevent_native_hold(tmp_path, monkeypatch):
    b = backend(tmp_path)
    try:
        b.command(Q)
        with monkeypatch.context() as patch:
            def fail():
                raise OSError('disk full')
            patch.setattr(b, '_checkpoint', fail)
            b.hold()
            assert b.raw.held == pytest.approx(MOTOR)
            assert b.read()['extra']['motor_mapping']['checkpoint_error'] == 'disk full'
    finally:
        b.close()


def test_corrupt_checkpoint_falls_back_without_actuator_jump(tmp_path):
    (tmp_path/'coordinates.json').write_text('{broken')
    b = backend(tmp_path)
    try:
        b.command(b.initial_target)
        assert b.raw.sent[-1][0] == pytest.approx(MOTOR)
    finally:
        b.close()


def make_feedforward(settings):
    pytest.importorskip('piper_x_driver._core')
    config_path = Path('vendor/Piper_workspace/config/arms/arm_2.yaml')
    return ReferenceFeedforward(yaml.safe_load(config_path.read_text()), config_path,
                                {k:v for k, v in settings.items() if k not in ('moving_kd', 'gain_ramp_rate', 'position_integral')})


def test_reference_feedforward_is_zero_at_rest_and_bounded_at_high_speed():
    ff = make_feedforward({'spatial_harmonics': [{'joint': 3, 'omega': 540, 'a': -.06, 'b': .45}]})
    raw = Raw()
    for speed in (0., 1e-8, -.01, .03, -.1, 1.8, -3.):
        effort = ff.evaluate(MOTOR, [speed]*6, raw.read(), Receiver.value, raw.kp, raw.kd)
        assert all(abs(f) <= limit for f, limit in zip(effort, ff.limits))
        if abs(speed) <= 1e-6:
            assert effort == [0.]*6
        if abs(speed) > 1:
            assert max(map(abs, ff.last['spatial_torque'])) < 1e-10
            assert not any(ff.last['clipped'])


@pytest.mark.parametrize('settings', [
    {'friction_scale': float('nan')}, {'velocity_remainder': 'true'},
    {'friction_factors': [1, 2.01, 1, 1, 1, 1]},
    {'friction_factors': [1, float('nan'), 1, 1, 1, 1]},
    {'spatial_harmonics': [{'joint': 3.0, 'omega': 540, 'b': .45}]},
    {'spatial_harmonics': [{'joint': 3, 'omega': 540, 'b': .9}]},
    {'spatial_harmonics': [{'joint': 3, 'omega': float('inf')}]},
])
def test_invalid_feedforward_configuration_rejected(settings):
    with pytest.raises(ValueError):
        make_feedforward(settings)


def test_joint_friction_factor_preserves_other_axes_and_stationary_hold():
    base = make_feedforward({'velocity_remainder': False})
    scaled = make_feedforward({'velocity_remainder': False, 'friction_factors': [1, 1.75, 1, 1, 1, 1]})
    raw = Raw()
    for speed in (-.03, 0., .03):
        a = base.evaluate(MOTOR, [speed]*6, raw.read(), Receiver.value, raw.kp, raw.kd)
        b = scaled.evaluate(MOTOR, [speed]*6, raw.read(), Receiver.value, raw.kp, raw.kd)
        assert b == pytest.approx([x*(1.75 if j == 1 else 1) for j,x in enumerate(a)])
        assert not any(scaled.last['clipped'])
        if speed == 0:
            assert b == [0.]*6


def test_gravity_override_uses_real_service_and_retains_source_file(tmp_path):
    from piper_control.backends.hardware import HardwareBackend

    raw = HardwareBackend.__new__(HardwareBackend)
    raw.gravity_scale, raw.friction_scale = .89, 0.
    raw._lock = threading.RLock()
    raw._control = {'mode': 'mit', 'gravity_scale_applied': 1., 'friction_scale_applied': 0.}
    raw._control_received = time.monotonic()
    raw._last_target, raw.initial_target = list(MOTOR), list(MOTOR)
    raw.config = {'end_effector': 'gripper_with_wrench'}
    raw.config_path = tmp_path/'arm.yaml'
    original = b'gravity_scale: 1\nfriction_scale: 0.88\n'
    raw.config_path.write_bytes(original)
    raw.read = lambda: {'enabled': True}
    raw._ConfigureArm = SimpleNamespace(Request=SimpleNamespace)
    raw._configure_client = object()
    sent, requested = [], []
    raw.command = lambda q: sent.append(list(q))

    def call(client, request, keepalive):
        requested.append(request)
        assert request.friction_scale == 0.  # A negative sentinel cannot change gravity.
        keepalive()
        raw.config_path.write_text('gravity_scale: 0.89\nfriction_scale: 0\n')
        raw._control['gravity_scale_applied'] = request.gravity_scale
        raw._control_received = time.monotonic()
        return SimpleNamespace(success=True)

    raw._call = call
    raw._apply_gravity_scale()
    assert raw.config_path.read_bytes() == original
    assert sent and all(q == list(MOTOR) for q in sent)
    assert raw._control['gravity_scale_applied'] == .89
    raw._apply_gravity_scale()
    assert len(requested) == 1  # Reconnecting to a matching controller changes nothing.


def test_candidate_profile_loads_all_bounded_harmonics():
    from piper_control.__main__ import load_control_profile
    profile = load_control_profile('piper_control/assets/tcp_tracking.yaml')
    ff = make_feedforward(profile['tracking'])
    assert {i for i, _ in ff.harmonics} == {0, 2}
    assert profile['gravity_scale'] == .89


def test_motion_damping_ramps_and_shutdown_restores_hold_gain(tmp_path):
    raw = Raw()
    raw.kd = list(raw.kd)
    raw.kd[3] = .53
    moving = list(raw.kd)
    moving[3] = 1.6
    b = TrackingBackend(raw, {'moving_kd': moving}, receiver=Receiver(), feedforward=Feedforward(),
                        state_path=tmp_path/'coordinates.json')
    try:
        prior = raw.kd[3]
        for _ in range(20):
            b.command(Q, [0., 0., 0., .04, 0., 0.])
            assert 0 < raw.kd[3]-prior <= .01000001
            prior = raw.kd[3]
        held = raw.sent[-1][0]
        count = len(raw.sent)
        b.hold()
        assert raw.kd[3] == pytest.approx(.53)
        assert raw.held == held
        assert all(q == held and dq == (0.,)*6 and torque == (0.,)*6
                   for q, dq, torque in raw.sent[count:])
    finally:
        b.close()


def test_handoff_freezes_integral_while_controller_keepalive_is_running(tmp_path):
    raw = Raw()
    raw.kd = [8., 12., 20., .53, .3, .08]
    moving = list(raw.kd)
    moving[3] = 1.6
    b = TrackingBackend(raw, {'moving_kd': moving, 'position_integral': {'gains': [1.]*6}},
                        receiver=Receiver(), feedforward=Feedforward(), state_path=tmp_path/'state.json')
    done = threading.Event()
    captured, failures = [], []
    original = raw.command

    def capture(q, dq=None, torque=None):
        if b._holding:
            captured.append((tuple(q), tuple(dq), b.integral.bias, b.mapper.offset))
        original(q, dq, torque)

    raw.command = capture
    target = [q+.004 for q in Q]

    def keepalive():
        while not done.is_set():
            try:
                b.step(.005)
                b.command(target)
            except Exception as exc:
                failures.append(exc)
            done.wait(.005)

    worker = threading.Thread(target=keepalive)
    try:
        b._freeze_until = 0
        b.command(target)
        raw.kd[3] = 1.2  # Force a ramp while the controller keeps publishing.
        worker.start()
        b.hold()
        assert not failures
        assert len(captured) > 10
        assert all(row == captured[0] for row in captured)
        assert raw.held == captured[0][0]
        with pytest.raises(RuntimeError, match='handoff'):
            b.command([q+.01 for q in target])
    finally:
        done.set()
        worker.join(timeout=1)
        b.close()
