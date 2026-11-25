"""Exercise persistent hold and reconnect using ROS publishers, never CAN."""

import json
from pathlib import Path
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("piper_x_interfaces")


def test_legacy_guardian_keeps_desired_target_and_hands_back(tmp_path, monkeypatch):
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from piper_x_interfaces.msg import ArmStatus, MitCommand
    from piper_x_interfaces.srv import Enable, SetMode
    from piper_control.backends.hardware import HardwareBackend
    from piper_control.hold_guardian import active_path

    monkeypatch.setenv("ROS_DOMAIN_ID", "129")
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    config = yaml.safe_load(Path("vendor/Piper_workspace/config/arms/arm_2.yaml").read_text())
    config["can_port"] = "fake_handoff"
    folder = tmp_path / "workspace/config/arms"
    folder.mkdir(parents=True)
    (folder / "arm_2.yaml").write_text(yaml.safe_dump(config))
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fake_handoff_driver", namespace="/arm_2", context=context)
    joint_pub = node.create_publisher(JointState, "joint_states_feedback", 1)
    status_pub = node.create_publisher(ArmStatus, "arm_status", 1)
    control_pub = node.create_publisher(String, "control_state", 1)
    actual = [0., .5, -.5, .2, .2, 0.]
    desired = actual.copy()
    desired[3] += .005  # Model a steady PD error; handoff must not compound it.
    received = []
    teaching = [False]

    def enable(request, response):
        response.enable_response = True
        return response

    def mode(request, response):
        response.success = request.mode == "mit"
        return response

    node.create_service(Enable, "enable_srv", enable)
    node.create_service(SetMode, "set_mode", mode)

    def command(msg):
        received.append((time.monotonic(), list(msg.position), list(msg.velocity)))

    node.create_subscription(MitCommand, "command/mit", command,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def feedback():
        joints = JointState()
        joints.name = [f"joint{i}" for i in range(1, 7)]
        joints.position, joints.velocity, joints.effort = actual, [0.] * 6, [0.] * 6
        joint_pub.publish(joints)
        status = ArmStatus()
        status.motor_enabled, status.motor_enable_valid = [True] * 6, [True] * 6
        status.feedback_age = [.001] * 11
        status.ctrl_mode, status.mode_feedback = 1, 6
        if teaching[0]:
            status.ctrl_mode, status.teach_status = 2, 1
        status_pub.publish(status)
        control = String()
        control.data = json.dumps({"mode": "mit", "mit_ready": True,
                                   "mit_target": received[-1][1] if received else actual})
        control_pub.publish(control)

    node.create_timer(.01, feedback)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    backend = replacement = None
    try:
        backend = HardwareBackend(tmp_path / "workspace", timeout=3)
        backend.enable(True)
        assert backend.read()["command_ready"]
        time.sleep(.02)
        received.clear()
        backend.command(desired)
        backend.hold()
        assert backend.read()["command_ready"]
        assert backend.read()["extra"]["firmware"]["ctrl_mode"] == 1
        teaching[0] = True
        time.sleep(.03)
        assert not backend.read()["command_ready"]
        teaching[0] = False
        time.sleep(.03)
        assert not backend.read()["command_ready"]
        time.sleep(.12)
        assert backend.read()["command_ready"]
        session = Path(json.loads(active_path("fake_handoff").read_text())["spec"]).parent
        assert (session / "ready").exists()
        backend.close()
        before = len(received)
        time.sleep(.15)
        assert len(received) > before + 5
        assert all(q == desired and dq == [0.] * 6 for _, q, dq in received[before:])
        replacement = HardwareBackend(tmp_path / "workspace", timeout=3)
        assert (session / "done").exists()
        assert replacement._last_target == desired
        assert all(q == desired for _, q, _ in received)
    finally:
        if backend:
            backend.close()
        if replacement:
            replacement.close()
        for session in (tmp_path / "state/piper_control/guardians").glob("*"):
            (session / "stop").touch()
            until = time.monotonic() + 2
            while not (session / "done").exists() and time.monotonic() < until:
                time.sleep(.01)
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        context.shutdown()
