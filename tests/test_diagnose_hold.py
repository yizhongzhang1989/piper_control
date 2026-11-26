"""Offline CAN/WAV fixtures only: this module never opens a device."""
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
from unittest.mock import Mock
import wave

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location("diagnose_hold", Path(__file__).parents[1] / "scripts/diagnose_hold.py")
diagnose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnose)


def capture_rows():
    feedback = [[(1_000_000_000 + index * 10_000_000, .005, 1.0 + .01 * (-1) ** index, -1.2)
                 for index in range(100)] for _ in range(6)]
    commands = [[(1_000_000_000 + index * 5_000_000, b"\0" * 8)
                 for index in range(200)] for _ in range(6)]
    return feedback, commands


def test_signed_protocol_statistics_and_limits():
    assert diagnose.decode_high_speed(struct.pack(">hhi", -17, -203, -123456)) == pytest.approx((-.017, -.203, -123.456))
    assert diagnose.kernel_timestamp_ns([(socket.SOL_SOCKET, diagnose.SO_TIMESTAMPNS,
                                          struct.pack("=qq", 123, 456))]) == 123_000_000_456
    with pytest.raises(ValueError, match="SO_TIMESTAMPNS"):
        diagnose.kernel_timestamp_ns([])
    feedback, commands = capture_rows()
    commands[3][-1] = (commands[3][-1][0], b"\1" * 8)
    joints, errors = diagnose.summarize(feedback, commands, 1., .01, .02)
    assert not errors
    assert joints["joint4"]["speed_rms_rad_s"] == pytest.approx(.005)
    assert joints["joint4"]["current_ac_rms_a"] == pytest.approx(.01)
    assert joints["joint4"]["current_peak_to_peak_a"] == pytest.approx(.02)
    assert joints["joint4"]["mit"]["unique_payload_count"] == 2
    assert joints["joint4"]["sample_rate_hz"] == pytest.approx(100)
    assert any("exceeds" in error for error in diagnose.summarize(feedback, commands, 1., .001, None)[1])


def test_missing_or_short_capture_is_not_zero_vibration():
    feedback, commands = capture_rows()
    feedback[3] = []
    commands[1] = commands[1][:10]
    joints, errors = diagnose.summarize(feedback, commands, 1.)
    assert joints["joint4"]["speed_rms_rad_s"] is None
    assert any("joint4 feedback" in error for error in errors)
    assert any("joint2 MIT" in error for error in errors)
    burst = list(range(100))
    quality = diagnose.timing_summary(burst, 1., 100)
    assert quality["quality_errors"]  # A full count over a tiny burst is insufficient.


def test_audio_band_power_has_digital_full_scale_units(tmp_path):
    path = tmp_path / "tone.wav"
    t = np.arange(48000) / 48000
    pcm = np.round(.25 * np.sin(2 * np.pi * 82 * t) * 32768).astype("<i2")
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(48000)
        output.writeframes(pcm.tobytes())
    audio = diagnose.analyze_audio(path, 1.)
    assert not audio["quality_errors"]
    assert audio["band_power_dbfs"] == pytest.approx(10 * np.log10(.25 ** 2 / 2), abs=.01)
    assert "not dB SPL" in audio["timing_and_level_note"]
    assert diagnose.analyze_audio(path, 10.)["quality_errors"]


@pytest.mark.parametrize("initial_status,exit_code,fails", [
    (None, 1, False),  # Running arecord exits 1 in response to our SIGINT.
    (1, 1, True),  # A prior acquisition failure must remain a failure.
    (None, 0, False),
    (None, 2, True),
])
def test_audio_exit_status_requires_our_sigint(initial_status, exit_code, fails):
    recorder = Mock(returncode=exit_code)
    recorder.poll.return_value = initial_status
    recorder.communicate.return_value = (b"", b"")
    errors = diagnose.finish_audio(recorder)
    assert bool(errors) == fails
    if initial_status is None:
        recorder.send_signal.assert_called_once_with(diagnose.signal.SIGINT)
    else:
        recorder.send_signal.assert_not_called()
    recorder.communicate.assert_called_once_with(timeout=3)


def test_audio_exit_one_not_accepted_when_sigint_cannot_be_sent():
    recorder = Mock(returncode=1)
    recorder.poll.return_value = None
    recorder.send_signal.side_effect = ProcessLookupError
    recorder.communicate.return_value = (b"", b"premature exit")
    assert diagnose.finish_audio(recorder) == ["arecord exited 1: premature exit"]


def test_audio_failure_to_close_wav_remains_failure():
    recorder = Mock(returncode=-diagnose.signal.SIGKILL)
    recorder.poll.return_value = None
    recorder.communicate.side_effect = [diagnose.subprocess.TimeoutExpired("arecord", 3), (b"", b"")]
    assert "arecord did not close the WAV after SIGINT" in diagnose.finish_audio(recorder)
    recorder.kill.assert_called_once_with()


def test_cli_exit_codes_and_failure_report_without_devices(tmp_path, monkeypatch):
    feedback, commands = capture_rows()
    monkeypatch.setattr(diagnose, "collect", lambda *args: (feedback, commands, {}, []))
    output = tmp_path / "report.json"
    for limit, expected in (("0.01", 0), ("0.001", 1)):
        monkeypatch.setattr(sys, "argv", ["diagnose_hold.py", "--seconds", "1", "--output", str(output),
                                         "--max-speed-rms", limit])
        assert diagnose.main() == expected
        assert json.loads(output.read_text())["passed"] == (expected == 0)
    feedback[3].clear()
    assert diagnose.main() == 1
    assert json.loads(output.read_text())["joints"]["joint4"]["speed_rms_rad_s"] is None
