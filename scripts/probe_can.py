#!/usr/bin/env python3
"""Passive SocketCAN inspection. This program never transmits a CAN frame."""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
import socket
import struct
import time
from zoneinfo import ZoneInfo


def probe(interface, duration=2.0):
    frames, stamps, counts = {}, {}, Counter()
    started = time.monotonic()
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
        receiver.bind((interface,))
        receiver.settimeout(0.1)
        while time.monotonic() - started < duration:
            try:
                raw = receiver.recv(16)
            except socket.timeout:
                continue
            can_id, length, data = struct.unpack("=IB3x8s", raw)
            can_id &= 0x1FFFFFFF
            frames[can_id] = data[:length]
            stamps[can_id] = time.monotonic()
            counts[can_id] += 1
    now = time.monotonic()
    positions = []
    for can_id in range(0x2A5, 0x2A8):
        data = frames.get(can_id)
        positions.extend([math.radians(v * .001) for v in struct.unpack(">ii", data)] if data and len(data) == 8 else [None, None])
    velocities = []
    enabled = []
    for index in range(6):
        data = frames.get(0x251 + index)
        velocities.append(struct.unpack(">h", data[:2])[0] * .001 if data else None)
        data = frames.get(0x261 + index)
        enabled.append(bool(data[5] & 0x40) if data and len(data) >= 6 else None)
    raw_status = frames.get(0x2A1)
    status = None
    if raw_status and len(raw_status) == 8:
        status = dict(zip(["ctrl_mode", "arm_status", "mode_feedback", "teach_status", "motion_status", "trajectory_num"], raw_status[:6]))
        status["err_code"] = struct.unpack(">H", raw_status[6:])[0]
    command_ids = set(range(0x150, 0x160)) | {0x470, 0x471, 0x472, 0x474, 0x475, 0x477, 0x479, 0x47A, 0x47B, 0x47C, 0x47D}
    expected = [0x2A5, 0x2A6, 0x2A7, 0x2A8, 0x2A1] + list(range(0x261, 0x267))
    return {
        "timestamp": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "interface": interface, "duration": now - started, "transmitted_frames": 0,
        "received_frames": sum(counts.values()), "positions": positions,
        "positions_deg": [math.degrees(v) if v is not None else None for v in positions],
        "velocities": velocities, "motor_enabled": enabled, "status": status,
        "feedback_age": {f"0x{i:03X}": now - stamps[i] if i in stamps else None for i in expected},
        "observed_command_frames": {f"0x{i:03X}": counts[i] for i in sorted(command_ids) if counts[i]},
        "frame_rates_hz": {f"0x{i:03X}": count / (now - started) for i, count in sorted(counts.items())},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("duration must be positive")
    text = json.dumps(probe(args.interface, args.duration), indent=2)
    print(text)
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(text + "\n")
