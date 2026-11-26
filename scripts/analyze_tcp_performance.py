#!/usr/bin/env python3
"""Offline TCP speed, path and endpoint measurements from passive CAN recordings.

Positions in mm are FK estimates from output encoders, not external metrology.
Repeatability requires repeated visits to the same target; a single endpoint
error is not a repeatability measurement. No robot commands are sent.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from piper_control.model import RobotModel
from piper_control.planning import pose_matrix


def analyze(folder):
    folder = Path(folder)
    report = json.loads((folder / 'report.json').read_text())
    if report['failure'] or report.get('result', {}).get('status') != 'succeeded':
        raise ValueError(f'{folder}: incomplete motion; do not score as a completed trial')
    can = np.load(folder / 'can.npz')
    reference = np.load(folder / 'reference.npz')
    accepted = report['accepted_unix']
    duration = report['motion']['duration']
    groups = [can[f'id_{i:03x}'] for i in range(0x2a5, 0x2a8)]
    begin = max(g[0, 0] for g in groups) - accepted
    end = min(g[-1, 0] for g in groups) - accepted
    if begin > -.1 or end < duration + .5:
        raise ValueError('Insufficient pre-motion or final hold coverage')
    gaps = [float(np.max(np.diff(g[:, 0]))) for g in groups]
    if max(gaps) > .1:
        raise ValueError('Output encoder capture contains a gap over 100 ms')
    dt = .005
    t = np.arange(begin, end, dt)
    columns = []
    for g in groups:
        b = g[:, 1:].astype('u1').copy()
        angles = np.frombuffer(b.tobytes(), dtype='>i4').reshape(-1, 2) * np.pi / 180000
        columns.extend(np.interp(t, g[:, 0] - accepted, angles[:, j]) for j in range(2))
    q = np.array(columns).T
    info = report['info']
    model = RobotModel(**{k: info[k] for k in ('tcp_offset', 'tcp_orientation', 'end_effector', 'angle_offsets')})
    transforms = np.array([model.forward(v) for v in q])
    xyz = transforms[:, :3, 3]
    velocity = savgol_filter(xyz, 11, 3, deriv=1, delta=dt, axis=0)
    reference_xyz = np.array([model.forward(v)[:3, 3] for v in reference['q']])
    reference_velocity = np.array([model.jacobian(v)[:3] @ w for v, w in zip(reference['q'], reference['dq'])])
    peak_reference = np.max(np.linalg.norm(reference_velocity, axis=1))
    moving = (t >= 0) & (t <= duration + .3)
    best = None
    for delay in np.arange(-.05, .251, .005):
        expected = np.array([np.interp(t-delay, reference['times'], reference_velocity[:, j], left=0, right=0) for j in range(3)]).T
        rms = float(np.sqrt(np.mean(np.sum((velocity[moving]-expected[moving])**2, axis=1))))
        if best is None or rms < best[0]:
            best = rms, delay, expected
    rms, delay, expected = best
    # Dense chord approximation bounds nearest-path discretization to micrometres.
    dense_t = np.linspace(0, reference['times'][-1], max(2001, len(reference_xyz)))
    dense_xyz = np.array([np.interp(dense_t, reference['times'], reference_xyz[:, j]) for j in range(3)]).T
    path_error, _ = cKDTree(dense_xyz).query(xyz[moving])
    tail = t > t[-1] - .5
    terminal_xyz = np.mean(xyz[tail], axis=0)
    terminal_q = np.mean(q[tail], axis=0)
    target = pose_matrix(report['target_pose'])
    tail_rotation = Rotation.from_matrix(transforms[tail, :3, :3]).mean()
    rotation_error = (Rotation.from_matrix(target[:3, :3]).inv() * tail_rotation).magnitude()
    late = t > duration
    finished_t = report['motion_finished_unix'] - accepted
    post_success = t > finished_t + .1
    raw_velocity_peaks = []
    static_velocity_rms = []
    static_current_ac_rms = []
    for j in range(6):
        g = can[f'id_{0x251+j:03x}']
        b = g[:, 1:].astype('u1')
        values = np.frombuffer(b[:, :4].copy().tobytes(), dtype='>i2').reshape(-1, 2) * .001
        raw_velocity_peaks.append(float(np.max(abs(values[:, 0]))))
        final = g[:, 0] > g[-1, 0] - 2
        static_velocity_rms.append(float(np.sqrt(np.mean(values[final, 0]**2))))
        static_current_ac_rms.append(float(np.std(values[final, 1])))
    speed = np.linalg.norm(velocity, axis=1)
    result = dict(
        measurement='CAN output encoders + configured FK; no external spatial calibration',
        speed_estimator='55 ms, cubic Savitzky-Golay derivative on 200 Hz interpolated output angles',
        requested_speed_m_s=report['requested_speed'], duration_s=duration,
        reference_peak_speed_m_s=float(peak_reference), measured_peak_speed_m_s=float(np.max(speed[moving])),
        fitted_delay_s=float(delay), velocity_vector_nrmse=rms/peak_reference,
        path_error_rms_mm=float(np.sqrt(np.mean(path_error**2))*1000),
        path_error_max_mm=float(np.max(path_error)*1000),
        final_position_mm=(terminal_xyz*1000).tolist(), target_position_mm=(target[:3, 3]*1000).tolist(),
        final_position_error_mm=float(np.linalg.norm(terminal_xyz-target[:3, 3])*1000),
        final_orientation_error_deg=float(np.degrees(rotation_error)),
        final_joint_positions_rad=terminal_q.tolist(),
        final_joint_error_rad=(terminal_q-reference['q'][-1]).tolist(),
        final_position_excursion_mm=float(np.max(np.linalg.norm(xyz[tail]-terminal_xyz, axis=1))*1000),
        post_plan_max_distance_from_final_mm=float(np.max(np.linalg.norm(xyz[late]-terminal_xyz, axis=1))*1000),
        post_success_max_distance_from_final_mm=float(np.max(np.linalg.norm(xyz[post_success]-terminal_xyz, axis=1))*1000),
        completion_delay_s=finished_t-duration,
        raw_joint_velocity_peak_rad_s=raw_velocity_peaks,
        final_2s_velocity_rms_rad_s=static_velocity_rms,
        final_2s_current_ac_rms_a=static_current_ac_rms,
        encoder_max_gap_s=max(gaps),
    )
    (folder/'performance.json').write_text(json.dumps(result, indent=2)+'\n')
    np.savez_compressed(folder/'performance.npz', t=t, q=q, xyz=xyz, velocity=velocity, expected_velocity=expected)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+')
    args = parser.parse_args()
    for folder in args.folders:
        print(folder, json.dumps(analyze(folder)))
