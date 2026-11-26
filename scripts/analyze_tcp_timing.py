"""Compare camera translation with the commanded quintic timing, for straight tests.

Uses the observed endpoint displacement as scale; this is a smoothness metric,
not a calibration of path accuracy. Only a constant exposure/response delay is fit.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--camera-data',default='camera-analysis.npz');parser.add_argument('--label',default='camera-reference');parser.add_argument('records',nargs='+');args=parser.parse_args()
for name in args.records:
    p=Path(name); r=json.loads((p/'report.json').read_text())
    assert not r['case'].startswith('arc')
    d=np.load(p/args.camera_data); t=d['t']; x=d['position_px']
    duration=r['motion']['duration']
    start=np.mean(x[t<0],axis=0)
    end=np.mean(x[t>duration+.7],axis=0)
    displacement=end-start; length=np.linalg.norm(displacement)
    direction=displacement/length
    velocity=savgol_filter(x,5,2,deriv=1,delta=1/60,axis=0)
    speed=velocity@direction
    selected=(t>0)&(t<duration+.5)
    best=None
    for delay in np.arange(-.05,.301,.005):
        u=np.clip((t-delay)/duration,0,1)
        expected=length/duration*30*u**2*(1-u)**2
        mse=np.mean((speed[selected]-expected[selected])**2)
        if best is None or mse<best[0]:best=(mse,delay,expected)
    mse,delay,expected=best
    peak=1.875*length/duration
    middle=(t>delay+.2*duration)&(t<delay+.8*duration)
    res=dict(translation_px=length,fitted_delay_s=delay,
             velocity_nrmse=float(np.sqrt(mse)/peak),
             peak_ratio=float(np.max(speed[selected])/peak),
             middle_minimum_ratio=float(np.min(speed[middle]/expected[middle])),
             middle_stopped_fraction=float(np.mean(speed[middle]<.15*expected[middle])))
    (p/(args.label+'.json')).write_text(json.dumps(res,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,4));ax.plot(t,speed,label='Camera, along endpoint displacement')
    ax.plot(t,expected,label='Command timing, endpoint scaled');ax.legend();ax.grid(alpha=.3)
    ax.set(xlabel='Time after API response (s)',ylabel='Projected speed (pixel/s)',title=p.name)
    fig.tight_layout();fig.savefig(p/(args.label+'.png'));plt.close(fig)
    print(p.name,json.dumps(res))
