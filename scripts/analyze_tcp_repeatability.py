#!/usr/bin/env python3
"""Group repeated TCP stops by target and report encoder/FK repeatability.

Reports centroid radius, maximum pairwise separation, mean bias, and an
ISO-style mean radius + 3 sample standard deviations. This is not an ISO 9283
test or external metrology. Input records must have completed successfully.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.distance import pdist

from analyze_tcp_performance import analyze


def summarize(folders, limit_mm=1.):
    groups = {}
    for folder in folders:
        record = json.loads((Path(folder)/'report.json').read_text())
        result = analyze(folder)
        pose = record['target_pose']
        # Quaternion sign describes the same orientation; normalize that sign.
        rotation = np.asarray(pose['orientation'])
        if rotation[np.argmax(abs(rotation))] < 0:
            rotation = -rotation
        key = tuple(np.round(np.r_[pose['position'], rotation], 6))
        groups.setdefault(key, []).append((str(folder), result))
    output = []
    for rows in groups.values():
        p = np.array([r['final_position_mm'] for _, r in rows])
        target = np.array(rows[0][1]['target_position_mm'])
        center = np.mean(p, axis=0)
        radius = np.linalg.norm(p-center, axis=1)
        span = float(np.max(pdist(p))) if len(p) > 1 else None
        output.append(dict(
            records=[name for name, _ in rows], count=len(rows),
            target_position_mm=target.tolist(), centroid_mm=center.tolist(),
            centroid_error_mm=float(np.linalg.norm(center-target)),
            maximum_radius_mm=float(np.max(radius)), maximum_pairwise_distance_mm=span,
            mean_radius_plus_3sd_mm=float(np.mean(radius)+3*np.std(radius, ddof=1)) if len(p)>1 else None,
            endpoint_errors_mm=[r['final_position_error_mm'] for _, r in rows],
            completion_delays_s=[r['completion_delay_s'] for _, r in rows],
            limit_mm=limit_mm, enough_repetitions=len(p)>=3,
            pairwise_limit_passed=bool(span<=limit_mm) if len(p)>=3 else None,
        ))
    return dict(measurement='Output encoders + configured FK, not external 3D metrology',
                acceptance='Maximum pairwise distance between repeated settled endpoints', groups=output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--limit-mm', type=float, default=1.)
    parser.add_argument('folders', nargs='+')
    args = parser.parse_args()
    if not np.isfinite(args.limit_mm) or args.limit_mm <= 0:
        parser.error('--limit-mm must be positive and finite')
    result = summarize(args.folders, args.limit_mm)
    Path(args.out).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
