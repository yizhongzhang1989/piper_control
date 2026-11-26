"""Constructor ordering regression tests without ROS, CAN, or a robot."""

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml

import piper_control.backends.hardware as hardware


@pytest.fixture
def startup(tmp_path, monkeypatch):
    folder = tmp_path / "config/arms"
    folder.mkdir(parents=True)
    config = dict(namespace="/fake_init", can_port="fake_initialization", end_effector="gripper_with_wrench",
                  gravity_scale=1., kp=[17, 16, 15, 4, 3, 3], kd=[5.99, 6.06, 3.16, .53, .3, .08])
    (folder / "arm_2.yaml").write_text(yaml.safe_dump(config))
    clock = SimpleNamespace(now=1000., tick=lambda: None)

    def sleep(dt):
        clock.now += dt
        clock.tick()

    monkeypatch.setattr(hardware, "time", SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep))
    real_open = Path.open
    lock = MagicMock()
    lock.fileno.return_value = 123

    def open_file(path, *args, **kwargs):
        if str(path) == "/tmp/piper_control_fake_initialization.lock":
            return lock
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", open_file)
    monkeypatch.setattr(hardware.fcntl, "flock", lambda *args: None)
    actual = [0., .5, -.5, .2, .2, 0.]
    nominal = [0., .5, -.5, .209, .2, 0.]
    commands, service_calls, guardian_calls, instances = [], [], [], []

    class Backend(hardware.HardwareBackend):
        problem = None
        repair_at = 1000.15
        disabled = False

        def _connect_ros(self):
            instances.append(self)
            self._node = SimpleNamespace(count_publishers=lambda _: 1, destroy_node=lambda: None)
            self._ConfigureArm = SimpleNamespace(Request=SimpleNamespace)
            self._configure_client = object()
            self.control = dict(mode="mit", mit_ready=True, position_hold_on_timeout=True,
                                mit_target=nominal, friction_scale_applied=.88)
            self.tick()
            clock.tick = self.tick

        def tick(self):
            problem = self.problem if clock.now < self.repair_at else None
            self._on_joints(SimpleNamespace(name=[f"joint{i}" for i in range(1, 7)], position=actual,
                                            velocity=[0.] * 6, effort=[0.] * 6))
            self._on_status(SimpleNamespace(
                ctrl_mode=1, mode_feedback=6, teach_status=2, feedback_age=[.001] * 11,
                err_code=0, arm_status=0, last_error="", motor_enabled=[not self.disabled] * 6,
                motor_enable_valid=[problem != "unknown_enable"] * 6))
            payload = dict(self.control)
            if problem == "missing_target":
                payload["mit_target"] = None
            if problem == "mode_disagreement":
                payload["mode"] = "point"
            self._on_control(SimpleNamespace(data=json.dumps(payload)))
            if problem == "stale_control":
                self._control_received -= .21

        def command(self, q, dq=None):
            commands.append((clock.now, list(q)))
            self._last_target = list(q)

        def _take_over_guardian(self):
            guardian_calls.append((clock.now, self.initial_target))

        def _call(self, client, request, timeout=5., keepalive=None):
            service_calls.append((clock.now, request.friction_scale))
            # Exercise the actual constructor's friction-profile keepalive path.
            if keepalive:
                keepalive()
            self.control["friction_scale_applied"] = -request.friction_scale - 4.
            self.tick()
            return SimpleNamespace(success=True)

    return SimpleNamespace(Backend=Backend, workspace=tmp_path, clock=clock, actual=actual,
                           nominal=nominal, commands=commands, services=service_calls,
                           guardians=guardian_calls, instances=instances)


@pytest.mark.parametrize("problem", ["unknown_enable", "stale_control", "missing_target", "mode_disagreement"])
@pytest.mark.parametrize("launch_driver", [False, True])
def test_initialization_waits_before_first_command_and_configuration(startup, problem, launch_driver):
    startup.Backend.problem = problem
    backend = startup.Backend(startup.workspace, timeout=.5, launch_driver=launch_driver, friction_scale=0.)
    try:
        assert backend.initial_target == startup.nominal
        assert backend.read()["enabled"] and backend.read()["command_ready"]
        assert startup.commands  # Includes a real _apply_friction_scale keepalive.
        assert all(t >= startup.Backend.repair_at and q == startup.nominal for t, q in startup.commands)
        assert all(t >= startup.Backend.repair_at for t, _ in startup.services + startup.guardians)
        assert startup.services[0][1] == -4.
    finally:
        backend.close()


@pytest.mark.parametrize("problem", ["unknown_enable", "stale_control", "missing_target", "mode_disagreement"])
def test_uncertain_handoff_times_out_without_publishing_or_configuring(startup, problem):
    startup.Backend.problem = problem
    startup.Backend.repair_at = float("inf")
    with pytest.raises(TimeoutError, match="Cannot establish initial target"):
        startup.Backend(startup.workspace, timeout=.3, launch_driver=True, friction_scale=0.)
    assert not startup.commands and not startup.services and not startup.guardians
    failed = startup.instances[-1]
    assert failed.initial_target is None and failed._last_target is None
    assert failed._closed


def test_confirmed_disabled_robot_starts_at_measured_position_without_keepalive(startup):
    startup.Backend.disabled = True
    backend = startup.Backend(startup.workspace, timeout=.3, launch_driver=True, friction_scale=0.)
    try:
        assert not backend.read()["enabled"]
        assert backend.initial_target == startup.actual
        assert backend.initial_target != startup.nominal  # Old MIT target belongs to an earlier enabled hold.
        assert not startup.commands
        assert startup.services
    finally:
        backend.close()
