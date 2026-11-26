#!/usr/bin/env python3
"""Passive Piper hold diagnostics; never transmits CAN or changes robot state.

Example: python scripts/diagnose_hold.py --can can1 --seconds 10 --output hold.json
Optional: --audio hold.wav --max-speed-rms 0.01 --max-current-ac-rms 0.02

Quality checks use observed Piper X rates: 100 Hz high-speed feedback and
200 Hz MIT commands, >=90% of expected samples/window coverage, and no gap
longer than 100 ms. Missing data never becomes a zero-vibration result.
Audio is optional pulse/S16_LE/mono/48 kHz. Its 80--84 Hz power is digital
dBFS, not calibrated dB SPL; audio/CAN timing and microphone gain are uncalibrated.
Exit 0 means valid acquisition and compliance with explicitly supplied limits;
it does not independently certify silence, stability, or motion safety.
"""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
import wave
from zoneinfo import ZoneInfo

import numpy as np
from scipy.signal import welch


CAN_FRAME = struct.Struct("=IB3x8s")
FEEDBACK_IDS = tuple(range(0x251, 0x257))
MIT_IDS = tuple(range(0x15A, 0x160))
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
MIN_COVERAGE = .9
MAX_GAP_S = .1
FEEDBACK_HZ, MIT_HZ = 100., 200.
AUDIO_HZ = 48000


def decode_high_speed(payload):
    """Signed big-endian i16/i16/i32, each scaled by .001 (rad/s, A, rad)."""
    if len(payload) != 8:
        raise ValueError("high-speed feedback requires exactly eight bytes")
    return tuple(value * .001 for value in struct.unpack(">hhi", payload))


def kernel_timestamp_ns(ancillary):
    """SO_TIMESTAMPNS is a kernel software RX timestamp on CLOCK_REALTIME."""
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
            # Linux old timespec uses native longs; the time64 ABI uses two i64.
            layout = "=qq" if len(data) >= 16 else "=ii"
            size = struct.calcsize(layout)
            if len(data) < size:
                break
            seconds, nanos = struct.unpack(layout, data[:size])
            if seconds >= 0 and 0 <= nanos < 1_000_000_000:
                return seconds * 1_000_000_000 + nanos
    raise ValueError("missing or invalid SO_TIMESTAMPNS ancillary data")


def timing_summary(timestamps, duration, expected_hz):
    """Count/window checks also catch a short burst that happens to have high Hz."""
    stamps = np.asarray(timestamps, dtype=np.int64)
    count = len(stamps)
    span = (int(stamps[-1]) - int(stamps[0])) / 1e9 if count > 1 else 0.
    gaps = np.diff(stamps) / 1e9
    errors = []
    minimum_count = max(20, math.ceil(duration * expected_hz * MIN_COVERAGE))
    if count < minimum_count:
        errors.append(f"only {count} samples; need at least {minimum_count}")
    if span < duration * MIN_COVERAGE:
        errors.append(f"sample coverage {span:.3f}s is below {duration * MIN_COVERAGE:.3f}s")
    if len(gaps) and np.any(gaps <= 0):
        errors.append("kernel timestamps are not strictly increasing")
    max_gap = float(np.max(gaps)) if len(gaps) else None
    if max_gap is not None and max_gap > MAX_GAP_S:
        errors.append(f"maximum receive gap {max_gap:.3f}s exceeds {MAX_GAP_S}s")
    return {
        "count": count, "expected_rate_hz": expected_hz,
        "sample_rate_hz": (count - 1) / span if span > 0 else None,
        "count_per_window_hz": count / duration,
        "coverage_seconds": span, "max_gap_seconds": max_gap,
        "first_timestamp_ns": int(stamps[0]) if count else None,
        "last_timestamp_ns": int(stamps[-1]) if count else None,
        "quality_errors": errors,
    }


def summarize(feedback, commands, duration, max_speed_rms=None, max_current_ac_rms=None):
    """Pure statistics on captured rows, also used by offline regression tests."""
    joints, errors = {}, []
    for index in range(6):
        name = f"joint{index + 1}"
        rows = feedback[index]
        stats = timing_summary([row[0] for row in rows], duration, FEEDBACK_HZ)
        stats["feedback_can_id"] = f"0x{FEEDBACK_IDS[index]:03X}"
        stats.update(speed_rms_rad_s=None, current_mean_a=None,
                     current_ac_rms_a=None, current_peak_to_peak_a=None,
                     high_speed_position_peak_to_peak_rad=None)
        if rows:
            values = np.asarray([row[1:] for row in rows], dtype=float)
            speed, current, position = values.T
            stats.update(
                speed_rms_rad_s=float(np.sqrt(np.mean(speed ** 2))),
                current_mean_a=float(np.mean(current)),
                current_ac_rms_a=float(np.sqrt(np.mean((current - np.mean(current)) ** 2))),
                current_peak_to_peak_a=float(np.ptp(current)),
                high_speed_position_peak_to_peak_rad=float(np.ptp(position)),
            )
        errors.extend(f"{name} feedback: {error}" for error in stats["quality_errors"])
        for key, limit in (("speed_rms_rad_s", max_speed_rms), ("current_ac_rms_a", max_current_ac_rms)):
            value = stats[key]
            if limit is not None and value is not None and value > limit:
                errors.append(f"{name} {key}={value:.6g} exceeds {limit:.6g}")
        mit = timing_summary([row[0] for row in commands[index]], duration, MIT_HZ)
        payload_counts = Counter(row[1].hex() for row in commands[index])
        mit.update(can_id=f"0x{MIT_IDS[index]:03X}",
                   unique_payload_count=len(payload_counts), payload_counts=dict(sorted(payload_counts.items())))
        stats["mit"] = mit
        errors.extend(f"{name} MIT: {error}" for error in mit["quality_errors"])
        joints[name] = stats
    return joints, errors


def analyze_audio(path, duration):
    with wave.open(str(path), "rb") as source:
        rate, channels, width = source.getframerate(), source.getnchannels(), source.getsampwidth()
        if (rate, channels, width, source.getcomptype()) != (AUDIO_HZ, 1, 2, "NONE"):
            raise ValueError("audio must be uncompressed mono S16_LE at 48000 Hz")
        pcm = source.readframes(source.getnframes())
    samples = np.frombuffer(pcm, dtype="<i2")
    seconds = len(samples) / rate
    errors = []
    if seconds < duration * MIN_COVERAGE or seconds < .5:
        errors.append(f"audio only covers {seconds:.3f}s; require >=90% of window and >=0.5s")
    power = None
    if len(samples) >= rate // 2:
        # PSD integrates to mean-square full-scale-normalized PCM. A full-scale
        # sine entirely in the band measures approximately -3.01 dBFS here.
        freqs, density = welch(samples.astype(float) / 32768., fs=rate,
                               window="hann", nperseg=min(len(samples), 2 * rate),
                               detrend="constant", scaling="density")
        band = (freqs >= 80.) & (freqs <= 84.)
        power = float(np.sum(density[band]) * (freqs[1] - freqs[0]))
    return {
        "path": str(path), "sample_rate_hz": rate, "sample_count": len(samples),
        "duration_seconds": seconds, "band_hz": [80, 84],
        "band_power_fs_squared": power,
        "band_power_dbfs": 10 * math.log10(power) if power is not None and power > 0 else None,
        "zero_power_dbfs_note": "null for zero power denotes -infinity dBFS, not a calibrated noise floor",
        "all_zero_samples": bool(len(samples) and not np.any(samples)),
        "timing_and_level_note": "Uncalibrated microphone gain and audio/CAN timing; dBFS is not dB SPL. The recording start is not sample-synchronized to CAN.",
        "quality_errors": errors,
    }


def finish_audio(recorder):
    """Close the WAV, accepting arecord's exit 1 only after our own SIGINT."""
    errors = []
    interrupted_here = False
    if recorder.poll() is None:
        try:
            recorder.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass  # Exited between poll and signal; do not accept exit 1.
        else:
            interrupted_here = True
    try:
        _, stderr = recorder.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        recorder.kill()
        _, stderr = recorder.communicate()
        errors.append("arecord did not close the WAV after SIGINT")
    accepted_codes = (0, -signal.SIGINT, 128 + signal.SIGINT)
    if recorder.returncode not in accepted_codes and not (interrupted_here and recorder.returncode == 1):
        errors.append(f"arecord exited {recorder.returncode}: {stderr.decode(errors='replace')[:1000]}")
    return errors


def collect(interface, duration, audio_path=None):
    feedback, commands = [[] for _ in range(6)], [[] for _ in range(6)]
    failures = Counter()
    errors = []
    recorder = None
    started = None
    audio_start_ns = None
    capture_start_ns = None
    elapsed = 0.
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
            receiver.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            # Standard data frames only. No send calls, bus setup, or ROS imports.
            mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            filters = b"".join(struct.pack("=II", can_id, mask) for can_id in FEEDBACK_IDS + MIT_IDS)
            receiver.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            receiver.bind((interface,))
            if audio_path:
                audio_path = Path(audio_path).expanduser().resolve()
                audio_path.parent.mkdir(parents=True, exist_ok=True)
                audio_start_ns = time.time_ns()
                recorder = subprocess.Popen(
                    ["arecord", "-q", "-D", "pulse", "-r", str(AUDIO_HZ), "-c", "1",
                     "-f", "S16_LE", "-t", "wav", str(audio_path)],
                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                )
            capture_start_ns, started = time.time_ns(), time.monotonic()
            deadline = started + duration
            while time.monotonic() < deadline:
                receiver.settimeout(min(.1, max(.001, deadline - time.monotonic())))
                try:
                    raw, ancillary, flags, _ = receiver.recvmsg(CAN_FRAME.size, socket.CMSG_SPACE(16))
                except socket.timeout:
                    continue
                if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or len(raw) != CAN_FRAME.size:
                    failures["truncated CAN frame or ancillary data"] += 1
                    continue
                can_id, length, payload = CAN_FRAME.unpack(raw)
                if can_id & (socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                    continue
                if can_id not in FEEDBACK_IDS + MIT_IDS:
                    continue
                if length != 8:
                    failures[f"0x{can_id:03X} has DLC {length}, expected 8"] += 1
                    continue
                try:
                    timestamp = kernel_timestamp_ns(ancillary)
                except ValueError as error:
                    failures[str(error)] += 1
                    continue
                if timestamp < capture_start_ns:
                    continue  # Discard the socket queue predating this window.
                if can_id in FEEDBACK_IDS:
                    feedback[can_id - FEEDBACK_IDS[0]].append((timestamp, *decode_high_speed(payload)))
                else:
                    commands[can_id - MIT_IDS[0]].append((timestamp, payload))
    except KeyboardInterrupt:
        errors.append("capture interrupted")
    except (OSError, ValueError) as error:
        errors.append(f"capture failed: {error}")
    finally:
        if started is not None:
            elapsed = time.monotonic() - started
        if recorder:
            errors.extend(finish_audio(recorder))
    errors.extend(f"{name}: {count} frame(s)" for name, count in failures.items())
    if elapsed < duration * MIN_COVERAGE:
        errors.append(f"capture lasted only {elapsed:.3f}s of requested {duration:.3f}s")
    return feedback, commands, {
        "duration_seconds": elapsed, "capture_start_realtime_ns": capture_start_ns,
        "audio_process_start_realtime_ns": audio_start_ns,
        "timestamp_source": "SO_TIMESTAMPNS kernel software receive timestamp (CLOCK_REALTIME), not motor sampling time",
        "transmitted_can_frames": 0,
    }, errors


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a finite nonnegative number")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--can", default="can1", help="existing SocketCAN interface; never configured by this tool")
    parser.add_argument("--seconds", type=nonnegative, default=10., help="capture duration (recommend >=1 s)")
    parser.add_argument("--output", type=Path, required=True, help="JSON report, including acquisition failures")
    parser.add_argument("--audio", type=Path, help="optional WAV to record with arecord -D pulse")
    parser.add_argument("--max-speed-rms", type=nonnegative, help="per-joint RMS velocity limit in rad/s")
    parser.add_argument("--max-current-ac-rms", type=nonnegative, help="per-joint AC RMS current limit in A")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    feedback, commands, capture, errors = collect(args.can, args.seconds, args.audio)
    joints, joint_errors = summarize(feedback, commands, args.seconds,
                                     args.max_speed_rms, args.max_current_ac_rms)
    errors.extend(joint_errors)
    audio = None
    if args.audio:
        try:
            audio = analyze_audio(args.audio.expanduser().resolve(), args.seconds)
            errors.extend(f"audio: {error}" for error in audio["quality_errors"])
        except (OSError, ValueError, EOFError, wave.Error) as error:
            errors.append(f"audio analysis failed: {error}")
    report = {
        "timestamp": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "can_interface": args.can, "requested_seconds": args.seconds, "capture": capture,
        "quality_requirements": {"feedback_expected_hz": FEEDBACK_HZ, "mit_expected_hz": MIT_HZ,
                                 "minimum_count_fraction": MIN_COVERAGE, "minimum_window_coverage": MIN_COVERAGE,
                                 "minimum_samples_per_channel": 20, "maximum_gap_seconds": MAX_GAP_S},
        "limits": {"max_speed_rms_rad_s": args.max_speed_rms, "max_current_ac_rms_a": args.max_current_ac_rms},
        "position_note": "0x251..0x256 signed high-speed motor position is scaled by 0.001 rad; it is not the 0x2A5..0x2A7 joint-angle encoding.",
        "pass_scope": "capture quality and explicitly requested numerical limits only",
        "joints": joints, "audio": audio, "passed": not errors, "errors": errors,
    }
    args.output.expanduser().parent.mkdir(parents=True, exist_ok=True)
    args.output.expanduser().write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({"passed": report["passed"], "output": str(args.output), "errors": errors}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
