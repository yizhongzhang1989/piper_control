"""ROS boundary to the unchanged, calibrated Piper_workspace C++ driver.

No CAN protocol or compensation is duplicated here. The driver intentionally
outlives the API process. Persistent MIT hold belongs to either the opt-in driver
watchdog or a ROS-only guardian. API shutdown never switches firmware modes.
"""

from __future__ import annotations

import copy
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

import yaml


class HardwareBackend:
    name = "hardware"

    def __init__(self, workspace, arm_id="arm_2", launch_driver=False, timeout=12.0,
                 kp=None, kd=None):
        self.workspace = Path(workspace).expanduser().resolve()
        self.arm_id = arm_id
        config_path = self.workspace / "config" / "arms" / f"{arm_id}.yaml"
        if not config_path.is_file():
            raise FileNotFoundError(f"Arm configuration not found: {config_path}")
        self.config = yaml.safe_load(config_path.read_text())
        self.namespace = self.config.get("namespace", f"/{arm_id}").rstrip("/")
        self.kp = list(map(float, self.config["kp"] if kp is None else kp))
        self.kd = list(map(float, self.config["kd"] if kd is None else kd))
        if len(self.kp) != 6 or not all(math.isfinite(v) and 0 <= v <= 500 for v in self.kp):
            raise ValueError("kp requires six finite values within the driver range 0..500 Nm/rad")
        if len(self.kd) != 6 or not all(math.isfinite(v) and (v == -1 or 0 <= v <= 100) for v in self.kd):
            raise ValueError("kd requires six finite values: -1 for automatic damping or 0..100 Nm/(rad/s)")
        self._lock = threading.RLock()
        self._positions = None
        self._velocities = []
        self._efforts = []
        self._joint_received = 0.0
        self._status_received = 0.0
        self._control_received = 0.0
        self._ready_since = None
        self._status = None
        self._control = {}
        self._closed = False
        self.driver_process = None
        self._writer_lock = None
        self._node = None
        self._context = None
        self._executor = None
        self._thread = None
        self._last_target = None
        self.initial_target = None
        try:
            self._connect_ros()
            lock_path = Path(f"/tmp/piper_control_{self.config['can_port']}.lock")
            self._writer_lock = lock_path.open("a+")
            try:
                fcntl.flock(self._writer_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError(f"Another piper_control owns {self.config['can_port']}") from exc
            if launch_driver:
                # Discover before launch so restarting the API reconnects to its
                # persistent driver instead of opening a second CAN writer.
                discovery_until = time.monotonic() + 1.0
                feedback_topic = f"{self.namespace}/joint_states_feedback"
                while (time.monotonic() < discovery_until and self._positions is None
                       and self._node.count_publishers(feedback_topic) == 0):
                    time.sleep(0.02)
                publisher_count = self._node.count_publishers(feedback_topic)
                if publisher_count > 1:
                    raise RuntimeError(f"Multiple driver publishers already exist at {feedback_topic}")
                if self._positions is None and publisher_count == 0:
                    self._launch_driver()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                state = self.read()
                if state["connected"] and self._control:
                    self._take_over_guardian()
                    if launch_driver and state["enabled"]:
                        self.command(self._last_target or state["positions"])
                        if self._control.get("mode") != "mit":
                            self._set_mode("mit")
                    self.initial_target = list(self._last_target or self._control.get("mit_target") or state["positions"])
                    break
                if self.driver_process and self.driver_process.poll() is not None:
                    raise RuntimeError(f"Driver exited; see {self.driver_log}")
                time.sleep(0.01)
            else:
                raise TimeoutError(
                    f"No fresh feedback at {self.namespace}; start the driver or use --launch-driver"
                )
        except Exception:
            self.close()
            raise

    def _connect_ros(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from piper_x_interfaces.msg import ArmStatus, MitCommand
            from piper_x_interfaces.srv import Enable, SetMode
        except ImportError as exc:
            raise RuntimeError(
                "Hardware requires ROS 2 Humble and piper_x_interfaces. "
                "Source scripts/env.sh before starting with Python 3.10."
            ) from exc
        self._rclpy = rclpy
        self._MitCommand, self._Enable, self._SetMode = MitCommand, Enable, SetMode
        self._context = Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node("piper_control_hardware", context=self._context)
        topic = lambda name: f"{self.namespace}/{name}"
        self._node.create_subscription(JointState, topic("joint_states_feedback"), self._on_joints, 1)
        self._node.create_subscription(ArmStatus, topic("arm_status"), self._on_status, 1)
        self._node.create_subscription(String, topic("control_state"), self._on_control, 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self._node.create_publisher(MitCommand, topic("command/mit"), qos)
        self._enable_client = self._node.create_client(Enable, topic("enable_srv"))
        self._mode_client = self._node.create_client(SetMode, topic("set_mode"))
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, name="piper-feedback", daemon=True)
        self._thread.start()

    def _spin(self):
        while not self._closed and self._context.ok():
            self._executor.spin_once(timeout_sec=0.05)

    def _launch_driver(self):
        executable = self.workspace / "install/piper_x_driver/lib/piper_x_driver/driver"
        if not executable.is_file():
            raise FileNotFoundError(f"Build the vendor driver first: {executable}")
        log_dir = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "piper_control"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.driver_log = log_dir / f"driver_{self.arm_id}.log"
        args = [str(executable), "--ros-args", "-r", f"__ns:={self.namespace}",
                "-p", f"arm_id:={self.arm_id}", "-p", f"config_file:={self.workspace / 'config/arms'}",
                "-p", "position_hold_on_timeout:=true"]
        with self.driver_log.open("a") as output:
            self.driver_process = subprocess.Popen(
                args, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        (log_dir / f"driver_{self.arm_id}.pid").write_text(str(self.driver_process.pid) + "\n")

    def _on_joints(self, msg):
        indexes = {name: index for index, name in enumerate(msg.name)}
        if any(f"joint{i}" not in indexes for i in range(1, 7)):
            return
        order = [indexes[f"joint{i}"] for i in range(1, 7)]
        if not msg.position or len(msg.position) <= max(order):
            return
        with self._lock:
            self._positions = [float(msg.position[i]) for i in order]
            self._velocities = [float(msg.velocity[i]) for i in order] if len(msg.velocity) > max(order) else []
            self._efforts = [float(msg.effort[i]) for i in order] if len(msg.effort) > max(order) else []
            self._joint_received = time.monotonic()

    def _on_status(self, msg):
        with self._lock:
            now = time.monotonic()
            ready = msg.ctrl_mode == 1 and msg.mode_feedback == 6 and msg.teach_status != 1
            if not ready:
                self._ready_since = None
            elif self._ready_since is None or now - self._status_received >= .2:
                self._ready_since = now
            self._status = msg
            self._status_received = now

    def _on_control(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._control = payload
            self._control_received = time.monotonic()

    def read(self):
        with self._lock:
            now = time.monotonic()
            age = now - self._joint_received if self._joint_received else math.inf
            status_age = now - self._status_received if self._status_received else math.inf
            status = self._status
            errors = []
            if status is not None:
                segments = list(status.feedback_age[:3])
                can_age = max(segments) + status_age if all(v >= 0 for v in segments) else math.inf
                age = max(age, can_age)
                if status.err_code:
                    errors.append(f"Firmware error 0x{status.err_code:04x}")
                if status.arm_status:
                    errors.append(f"Firmware arm status {status.arm_status}")
                if status.last_error:
                    errors.append(status.last_error)
            enabled = bool(status and all(status.motor_enabled) and all(status.motor_enable_valid) and status_age < 1)
            # Match the existing driver's target/feedback watchdog at 0.2 s.
            connected = self._positions is not None and age < 0.2 and status_age < 0.2
            command_ready = bool(connected and enabled and self._ready_since is not None
                                 and now - self._ready_since >= .1)
            return {
                "positions": list(self._positions or []), "velocities": list(self._velocities),
                "efforts": list(self._efforts), "enabled": enabled, "connected": connected,
                "feedback_age": age if math.isfinite(age) else None, "errors": errors,
                "command_ready": command_ready,
                "extra": {"can_port": self.config["can_port"], "namespace": self.namespace,
                          "firmware": None if status is None else {
                              "ctrl_mode": int(status.ctrl_mode), "mode_feedback": int(status.mode_feedback),
                              "teach_status": int(status.teach_status), "arm_status": int(status.arm_status),
                              "err_code": int(status.err_code), "command_ready": command_ready,
                          },
                          "command_gains": {"kp": list(self.kp), "kd": list(self.kd)},
                          "driver": copy.deepcopy(self._control), "status_age": status_age if math.isfinite(status_age) else None,
                          "driver_pid": self.driver_process.pid if self.driver_process else None},
            }

    def command(self, q, dq=None):
        values = list(map(float, q))
        velocity = [0.0] * 6 if dq is None else list(map(float, dq))
        if len(values) != 6 or len(velocity) != 6 or not all(math.isfinite(v) for v in values + velocity):
            raise ValueError("command requires six finite joint positions and velocities")
        if self._closed:
            raise RuntimeError("Hardware backend is closed")
        message = self._MitCommand()
        message.position, message.velocity = values, velocity
        message.kp, message.kd = self.kp, self.kd
        message.torque = [0.0] * 6
        self._publisher.publish(message)
        self._last_target = values

    @staticmethod
    def _call(client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"Driver service unavailable: {client.srv_name}")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"Driver service timeout: {client.srv_name}")
        return future.result()

    def enable(self, enabled):
        request = self._Enable.Request()
        request.enable_request = bool(enabled)
        response = self._call(self._enable_client, request)
        if not response.enable_response:
            raise RuntimeError(getattr(response, "message", "Driver enable failed"))
        if enabled:
            state = self.read()
            target = list(state["positions"])
            self.command(target)
            with self._lock:
                self._ready_since = None
            self._set_mode("mit")
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                self.command(target)
                if self.read()["command_ready"]:
                    return
                time.sleep(.005)
            firmware = self.read()["extra"]["firmware"]
            raise RuntimeError(
                f"Enabled but external MIT mode did not remain ready for 100 ms within 2 s: {firmware}"
            )

    def _set_mode(self, mode):
        request = self._SetMode.Request()
        request.mode = mode
        response = self._call(self._mode_client, request)
        if not response.success:
            raise RuntimeError(response.message)

    def hold(self):
        """Hand off a fixed MIT target without changing firmware modes."""
        state = self.read()
        if not state["enabled"]:
            return
        if not state["connected"]:
            raise RuntimeError("Cannot confirm MIT hold without fresh hardware feedback")
        target = self._last_target or self._control.get("mit_target") or state["positions"]
        if self._control.get("position_hold_on_timeout"):
            deadline = time.monotonic() + 2.0
            started = time.monotonic()
            while time.monotonic() < deadline:
                self.command(target)
                time.sleep(.005)
                with self._lock:
                    observed = self._control.get("mit_target")
                    if (self._control_received > started and self._control.get("mode") == "mit"
                            and observed and all(abs(a-b) < 1e-8 for a,b in zip(target, observed))):
                        return
            raise RuntimeError("Driver did not confirm persistent MIT hold")
        from piper_control.hold_guardian import state_directory
        session = state_directory() / "guardians" / uuid.uuid4().hex
        session.mkdir(parents=True)
        spec_path = session / "spec.json"
        spec_path.write_text(json.dumps({"namespace": self.namespace, "can_port": self.config["can_port"],
                                        "positions": target, "kp": self.kp, "kd": self.kd}))
        with (session / "guardian.log").open("a") as output:
            process = subprocess.Popen([sys.executable, "-m", "piper_control.hold_guardian", "--spec", str(spec_path)],
                                       stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.command(target)
            if (session / "ready").exists():
                return
            if process.poll() is not None:
                raise RuntimeError(f"MIT guardian exited; see {session / 'guardian.log'}")
            time.sleep(.005)
        raise RuntimeError(f"MIT guardian handoff unconfirmed; API must stay running: {session}")

    def _take_over_guardian(self):
        from piper_control.hold_guardian import active_path
        pointer = active_path(self.config["can_port"])
        if not pointer.exists():
            return
        spec_path = Path(json.loads(pointer.read_text())["spec"])
        session = spec_path.parent
        if (session / "done").exists():
            return
        spec = json.loads(spec_path.read_text())
        if spec["namespace"].rstrip("/") != self.namespace:
            raise RuntimeError("Guardian namespace differs from selected robot")
        target = spec["positions"]
        # Both publishers send the identical desired position during handoff.
        for _ in range(10):
            self.command(target)
            time.sleep(.005)
        (session / "stop").touch()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.command(target)
            if (session / "done").exists():
                return
            time.sleep(.005)
        raise RuntimeError(f"Previous MIT guardian did not acknowledge stop: {session}")

    def step(self, dt):
        pass

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._executor:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node:
            self._node.destroy_node()
        if self._context and self._context.ok():
            self._context.shutdown()
        if self._writer_lock:
            self._writer_lock.close()


Hardware = HardwareBackend
