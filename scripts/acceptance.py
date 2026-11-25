#!/usr/bin/env python3
"""Small low-speed motions through the public API; save actual measured evidence."""
import argparse
from copy import deepcopy
from datetime import datetime
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np
from piper_control.client import RobotClient
from piper_control.model import RobotModel


def run(url, output):
    robot = RobotClient(url, timeout=60)
    info = robot.info()
    model = RobotModel(tcp_offset=info['tcp_offset'], tcp_orientation=info['tcp_orientation'],
                       end_effector=info['end_effector'], angle_offsets=info['angle_offsets'])
    report = dict(timestamp=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                  url=url, info=info, tests=[], passed=False)
    def save():
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        Path(output).write_text(json.dumps(report, indent=2)+'\n')
    def execute(name, motion, expected=None):
        samples = []
        deadline = time.monotonic()+motion['duration']+10
        while time.monotonic() < deadline:
            state = robot.state()
            if not state['connected'] or state['errors']:
                raise RuntimeError(f"hardware state failure: {state}")
            samples.append({key:state[key] for key in (
                'timestamp','positions','velocities','efforts','tcp_pose',
                'target_positions','feedback_age','mode','errors')})
            result = robot.motion(motion['id'])
            if result['status'] != 'running':
                break
            time.sleep(.02)
        else:
            raise TimeoutError(name)
        if result['status'] != 'succeeded':
            raise RuntimeError(f"{name}: {result}")
        final = robot.state()
        error = float(np.max(np.abs(np.array(final['positions'])-expected))) if expected is not None else None
        record = dict(name=name, result=result, measured_final=final,
                      max_joint_error_rad=error,
                      max_measured_velocity_rad_s=float(np.max(np.abs([s['velocities'] for s in samples]))),
                      samples=samples)
        report['tests'].append(record)
        save()
        if error is not None and error > info['goal_tolerance']:
            raise AssertionError(f"{name} tracking error {error:.4f} rad")
        return final
    try:
        robot.enable()
        time.sleep(.5)
        baseline = robot.state()
        report['initial_state'] = baseline
        q0 = np.array(baseline['positions'])
        q1 = q0.copy(); q1[0] += .02
        moved = execute('movej', robot.movej(q1.tolist(), duration=2), q1)
        actual_change = abs(moved['positions'][0]-q0[0])
        if actual_change < .005:
            raise AssertionError('No measured joint movement despite success status')
        execute('follow_trajectory', robot.trajectory([
            {'positions':q0.tolist(),'time_from_start':2},
            {'positions':q1.tolist(),'time_from_start':4}], info['joint_names']), q1)
        # Small base-axis circle is known reachable, even near a wrist singularity.
        current = np.array(robot.state()['positions'])
        endpoint = current.copy(); endpoint[0] += .012
        execute('movel', robot.movel(model.pose(endpoint), duration=2), endpoint)
        current = np.array(robot.state()['positions'])
        via, end = current.copy(), current.copy()
        via[0] += .015; end[0] += .03
        execute('movec', robot.movec(model.pose(via), model.pose(end), duration=3), end)
        execute('return_before_stream', robot.movej(q0.tolist(),duration=3),q0)
        for kind in ('joints','tcp','twist'):
            start = robot.state()
            target = np.array(start['positions']); target[0] += .01
            pose = model.pose(target)
            for _ in range(50):
                if kind == 'joints': robot.servo_joints(target.tolist())
                elif kind == 'tcp': robot.servo_pose(pose)
                else: robot.servo_twist([0,0,.001],[0,0,0])
                time.sleep(.02)
            time.sleep(info['stream_timeout']+.15)
            state = robot.state()
            if state['mode'] != 'idle' or state['errors']:
                raise AssertionError(f'{kind} stream/watchdog failed: {state}')
            report['tests'].append(dict(name='servo_'+kind+'_watchdog', before=start, after=state, passed=True))
            save()
        current = np.array(robot.state()['positions']); current[0] += .02
        motion = robot.movej(current.tolist(),duration=4)
        time.sleep(.3); robot.stop()
        result = robot.motion(motion['id'])
        if result['status'] != 'cancelled': raise AssertionError('stop did not cancel')
        report['tests'].append(dict(name='stop_cancel',result=result,passed=True))
        execute('return_to_start', robot.movej(q0.tolist()),q0)
        report['final_state'] = robot.state()
        report['passed'] = True
    except Exception as exc:
        report['failure'] = repr(exc)
        try: robot.stop()
        except Exception as stop_error: report['stop_failure'] = repr(stop_error)
        raise
    finally:
        save()
        robot.close()
    print(json.dumps({k:v for k,v in report.items() if k not in ('tests','info')},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8088')
    parser.add_argument('--output', default='docs/acceptance.json')
    args = parser.parse_args()
    run(args.url, args.output)
