#!/usr/bin/env python3
"""Run a reviewed TCP path suite through the public API and retain all evidence.

This moves hardware. The manifest's starting posture and complete paths must
already have been checked for clearance; joint bounds are not collision checks.
An unsuccessful motion or quality check stops the suite without disabling motors.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from piper_control.client import RobotClient
from piper_control.model import RobotModel
from piper_control.planning import Planner
from analyze_tcp_performance import analyze
from analyze_tcp_repeatability import summarize


def run(args):
    manifest = json.loads(Path(args.manifest).read_text())
    required_limits = ('final_position_error_mm', 'post_success_max_distance_from_final_mm',
                       'hold_velocity_rms_rad_s', 'hold_current_ac_rms_a', 'repeatability_mm')
    if any(not np.isfinite(manifest['limits'][k]) or manifest['limits'][k] <= 0 for k in required_limits):
        raise ValueError('All acceptance limits must be positive and finite')
    motions = manifest['motions']
    if not motions or len({m['name'] for m in motions}) != len(motions):
        raise ValueError('Suite must contain uniquely named motions')
    root = Path(args.out)
    root.mkdir(parents=True, exist_ok=False)
    (root/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    with RobotClient(args.url) as robot:
        info, state = robot.info(), robot.state()
    if info['backend'] != 'hardware' or state['mode'] != 'idle' or not state['command_ready'] or state['errors']:
        raise ValueError('Suite requires an idle, healthy hardware backend')
    for key in ('tcp_offset', 'tcp_orientation', 'end_effector', 'angle_offsets'):
        expected = manifest['model'][key]
        matches = (expected == info[key] if isinstance(expected, str)
                   else np.allclose(expected, info[key], atol=1e-9, rtol=0))
        if not matches:
            raise ValueError(f'Suite model differs from the live service: {key}')
    model = RobotModel(**{k: info[k] for k in ('tcp_offset', 'tcp_orientation', 'end_effector', 'angle_offsets')})
    initial = model.validate_joints(manifest['start_joint_positions'])
    if np.max(np.abs(initial-state['target_positions'])) > .002:
        raise ValueError('Robot nominal target does not match the reviewed starting posture')
    if np.max(np.abs(np.array(state['positions'])-state['target_positions'])) > .02:
        raise ValueError('Initial tracking error exceeds the suite starting envelope')
    planner = Planner(model, max(j['velocity'] for j in info['joint_limits']))
    planner.velocity = np.array([j['velocity'] for j in info['joint_limits']])
    q = np.array(state['target_positions'])
    for motion in manifest['motions']:
        if motion['kind'] not in ('line', 'arc') or not motion['name'].replace('-', '').replace('_', '').isalnum():
            raise ValueError('Invalid motion kind/name')
        if (motion['kind'] == 'arc') != ('via' in motion):
            raise ValueError('A via pose is required exclusively for an arc')
        plan = planner.cartesian(q, motion['target_pose'], motion.get('via'),
                                 speed=motion['speed'], angular_speed=motion.get('angular_speed', 3.))
        samples = [plan.sample(t) for t in np.linspace(0, plan.duration, 501)]
        if np.max(np.abs(np.array([p for p, _ in samples])-q)) >= motion['max_joint_travel']:
            raise ValueError('Planned travel exceeds manifest envelope')
        if np.max(np.abs([v for _, v in samples])) >= motion['max_observed_joint_speed']:
            raise ValueError('Planned velocity exceeds manifest observation envelope')
        q = samples[-1][0]
    if np.max(np.abs(q-initial)) > .002:
        raise ValueError('Repeated suite must return to its starting posture')
    report = dict(info=info, repeats=args.repeats, trials=[], passed=False, failure=None,
                  quality_failures=[], status='running',
                  continue_quality_failures=args.continue_quality_failures)
    folders = []
    script_dir = Path(__file__).resolve().parent
    try:
        for repeat in range(1, args.repeats+1):
            for motion in manifest['motions']:
                folder = root/f"{repeat:02d}-{motion['name']}"
                path_file = root/f"{motion['name']}-path.json"
                path_file.write_text(json.dumps({k: motion[k] for k in ('target_pose', 'via') if k in motion}))
                command = [sys.executable, str(script_dir/'record_tcp_motion.py'), '--url', args.url,
                           '--case', motion['kind']+'_path', '--path-file', str(path_file),
                           '--speed', str(motion['speed']), '--angular-speed', str(motion.get('angular_speed', 3.)),
                           '--max-joint-travel', str(motion['max_joint_travel']),
                           '--max-observed-joint-speed', str(motion['max_observed_joint_speed']),
                           '--camera', args.camera, '--camera-focus', '10', '--fixed-camera-framerate',
                           '--camera-resolution', '1920', '1080', '--out', str(folder)]
                with folder.with_suffix('.log').open('w') as log:
                    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
                trial = dict(folder=str(folder), exit_code=result.returncode)
                report['trials'].append(trial)
                if result.returncode:
                    raise RuntimeError(f'{folder}: recording/motion failed; see its report and log')
                performance = analyze(folder)
                trial['performance'] = performance
                if not performance.get('endpoint_position_estimate_anchored', True):
                    raise RuntimeError(f'{folder}: unanchored zero-band TCP position estimate')
                limits = manifest['limits']
                checks = {
                    'final_position_error_mm': performance['final_position_error_mm'],
                    'post_success_max_distance_from_final_mm': performance['post_success_max_distance_from_final_mm'],
                    'hold_velocity_rms_rad_s': max(performance['final_2s_velocity_rms_rad_s']),
                    'hold_current_ac_rms_a': max(performance['final_2s_current_ac_rms_a']),
                }
                violations = []
                for key, value in checks.items():
                    if not np.isfinite(value):
                        raise RuntimeError(f'{folder}: invalid {key} measurement')
                    if value > limits[key]:
                        reason = f'{folder}: {key}={value:.6g} exceeds {limits[key]}'
                        violations.append(reason)
                        report['quality_failures'].append(reason)
                        if not args.continue_quality_failures or value > 2*limits[key]:
                            raise RuntimeError(reason)
                trial['quality_passed'] = not violations
                folders.append(folder)
                print(json.dumps(dict(folder=str(folder), quality_passed=not violations, **checks)), flush=True)
                (root/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        report['repeatability'] = summarize(folders, manifest['limits']['repeatability_mm'])
        if args.repeats >= 3 and not all(g['pairwise_limit_passed'] for g in report['repeatability']['groups']):
            raise RuntimeError('Repeated endpoint separation exceeds limit')
        if report['quality_failures']:
            raise RuntimeError(f"{len(report['quality_failures'])} quality checks failed; diagnostic sequence retained all samples")
        report['passed'] = True
    except Exception as exc:
        report['failure'] = str(exc)
        raise
    finally:
        report['status'] = 'finished'
        (root/'summary.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--url', default='http://127.0.0.1:8090')
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--continue-quality-failures', action='store_true',
                        help='collect diagnostic repetitions after small quality failures; still fails the suite; '
                             'motion errors, invalid data, and quality values over twice their limit always stop')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('--repeats must be within 1..100')
    run(args)
