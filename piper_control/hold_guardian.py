"""Persistent ROS-only fixed-position hold for an unmodified legacy driver.

CLI: python -m piper_control.hold_guardian --spec SESSION/spec.json
Spec: namespace, positions[6], kp[6], kd[6], optional can_port. The session
directory contains ready/stop/done IPC files. This process never opens CAN,
changes modes, enables/disables motors, or generates a moving target.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time


def state_directory():
    return Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "piper_control"


def active_path(can_port):
    if not can_port or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in can_port):
        raise ValueError("Invalid CAN interface name")
    return state_directory() / f"guardian_{can_port}.json"


def run(spec_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from piper_x_interfaces.msg import MitCommand
    from std_msgs.msg import String

    spec_path = Path(spec_path).resolve()
    session = spec_path.parent
    spec = json.loads(spec_path.read_text())
    for key in ("positions", "kp", "kd"):
        values = spec[key]
        if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"{key} requires six finite values")
    if not all(-12.5 <= float(v) <= 12.5 for v in spec["positions"]):
        raise ValueError("Position outside MIT protocol range")
    if not all(0 <= float(v) <= 500 for v in spec["kp"]):
        raise ValueError("kp outside driver range")
    if not all(float(v) == -1 or 0 <= float(v) <= 100 for v in spec["kd"]):
        raise ValueError("kd outside driver range")
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("piper_position_guardian", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    namespace = spec["namespace"].rstrip("/")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(MitCommand, f"{namespace}/command/mit", qos)
    message = MitCommand()
    message.position = list(map(float, spec["positions"]))
    message.velocity = [0.0] * 6
    message.kp, message.kd = list(map(float, spec["kp"])), list(map(float, spec["kd"]))
    message.torque = [0.0] * 6
    confirmed = False

    def feedback(msg):
        nonlocal confirmed
        data = json.loads(msg.data)
        target = data.get("mit_target")
        confirmed = bool(data.get("mode") == "mit" and target and len(target) == 6
                         and all(abs(a - b) < 1e-8 for a, b in zip(target, message.position)))

    node.create_subscription(String, f"{namespace}/control_state", feedback, 1)
    if spec.get("can_port"):
        pointer = active_path(spec["can_port"])
        pointer.parent.mkdir(parents=True, exist_ok=True)
        temporary = pointer.with_suffix(".tmp")
        temporary.write_text(json.dumps({"spec": str(spec_path)}))
        temporary.replace(pointer)
    next_tick = time.monotonic()
    try:
        while not (session / "stop").exists():
            executor.spin_once(timeout_sec=0.0)
            publisher.publish(message)
            if confirmed and not (session / "ready").exists():
                (session / "ready").write_text(json.dumps({"pid": os.getpid(), "confirmed": True}))
            next_tick += .005
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -.02:
                next_tick = time.monotonic()
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        context.shutdown()
        (session / "done").write_text("stopped\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", required=True)
    run(parser.parse_args().spec)
