#!/usr/bin/env python3
"""Compare settled tool images across repeated visits to one pose, offline.

Removes camera motion using fixed background features. Reports pixels only;
there is no external length calibration or 3D reconstruction here.
Regions use full-frame coordinates normalized to 960 pixels wide.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.distance import pdist


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-region', type=float, nargs=4, required=True)
parser.add_argument('--background-region', type=float, nargs=4, action='append', required=True)
parser.add_argument('--out', required=True)
parser.add_argument('folders', nargs='+')
args = parser.parse_args()
folders = [Path(p) for p in args.folders]
reports = [json.loads((p/'report.json').read_text()) for p in folders]
target = np.array(reports[0]['target_pose']['position'])
if any(np.linalg.norm(np.array(r['target_pose']['position'])-target) > 1e-6 for r in reports):
    raise ValueError('Group only visits to the same endpoint')
rotation = np.array(reports[0]['target_pose']['orientation'])
rotation /= np.linalg.norm(rotation)
for report in reports:
    other = np.array(report['target_pose']['orientation'])
    other /= np.linalg.norm(other)
    if min(np.linalg.norm(rotation-other), np.linalg.norm(rotation+other)) > 1e-6:
        raise ValueError('Group only visits to the same target orientation')
reference = cv2.imread(str(folders[0]/'camera-end.jpg'))
if reference is None:
    raise ValueError('Missing reference end image')
gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
scale = reference.shape[1]/960


def features(regions, count, quality, spacing):
    mask = np.zeros_like(gray)
    for x0, y0, x1, y1 in regions:
        if not (0 <= x0 < x1 <= 960 and 0 <= y0 < y1 <= reference.shape[0]/scale):
            raise ValueError('Region is outside the full frame')
        a, b, c, d = np.rint(np.array([x0, y0, x1, y1])*scale).astype(int)
        mask[b:d, a:c] = 255
    points = cv2.goodFeaturesToTrack(gray, count, quality, spacing*scale, mask=mask,
                                     blockSize=int(5*scale)|1)
    if points is None:
        raise ValueError('No image features in the selected region')
    return points


fg = features([args.tool_region], 80, .03, 5)
bg = features(args.background_region, 160, .03, 6)
preview = reference.copy()
for points, color in ((fg, (0, 255, 0)), (bg, (0, 0, 255))):
    for x, y in points[:, 0]:
        cv2.circle(preview, (int(x), int(y)), 4, color, 1)
output = Path(args.out)
cv2.imwrite(str(output.with_suffix('.features.jpg')), preview)
rows = []


def track(image, points):
    kwargs = dict(winSize=(int(21*scale)|1,)*2, maxLevel=3)
    nxt, status, _ = cv2.calcOpticalFlowPyrLK(gray, image, points, None, **kwargs)
    back, backward, _ = cv2.calcOpticalFlowPyrLK(image, gray, nxt, None, **kwargs)
    good = ((status.ravel() == 1) & (backward.ravel() == 1)
            & (np.linalg.norm(back-points, axis=2).ravel() < .5*scale))
    return points[good], nxt[good]


for folder, report in zip(folders, reports):
    image = cv2.imread(str(folder/'camera-end.jpg'))
    if image is None or image.shape != reference.shape:
        raise ValueError(f'{folder}: missing image or mismatched resolution')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    base_bg, moved_bg = track(image, bg)
    base_fg, moved_fg = track(image, fg)
    if len(base_bg) < 20 or len(base_fg) < 6:
        raise ValueError(f'{folder}: insufficient matched features')
    affine, inliers = cv2.estimateAffinePartial2D(base_bg[:, 0], moved_bg[:, 0], method=cv2.RANSAC,
                                                 ransacReprojThreshold=.5*scale)
    if affine is None or inliers.sum() < 20:
        raise ValueError(f'{folder}: fixed background registration failed')
    corrected = cv2.transform(moved_fg, cv2.invertAffineTransform(affine))
    displacement = np.median((corrected-base_fg)[:, 0], axis=0)/scale
    rows.append(dict(folder=str(folder), displacement_px=displacement.tolist(),
                     tool_features=len(base_fg), background_inliers=int(inliers.sum()),
                     motion_status=report.get('result', {}).get('status'), failure=report['failure']))
points = np.array([r['displacement_px'] for r in rows])
result = dict(measurement='Rigid tool image displacement after fixed background registration',
              units='pixels normalized to full frame width 960; no millimetre calibration',
              tool_region=args.tool_region, background_regions=args.background_region,
              maximum_pairwise_displacement_px=float(max(pdist(points))) if len(points) > 1 else 0.,
              records=rows)
output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
