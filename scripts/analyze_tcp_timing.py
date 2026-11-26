"""Compare camera translation with the recorded reference, for straight tests.

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
from piper_control.model import RobotModel

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--camera-data',default='camera-analysis.npz');parser.add_argument('--label',default='camera-reference');parser.add_argument('records',nargs='+');args=parser.parse_args()
for name in args.records:
    p=Path(name); r=json.loads((p/'report.json').read_text())
    assert not r['case'].startswith('arc')
    d=np.load(p/args.camera_data); t=d['t']; x=d['position_px']
    duration=r['motion']['duration']
    start=np.mean(x[t<0],axis=0)
    # The entire period after the planned end can still contain settling motion.
    # Use the final window for scale, and report the preceding drift explicitly.
    assert t[-1]>duration+.5 and np.sum(t<0)>5
    end=np.mean(x[t>t[-1]-.5],axis=0)
    displacement=end-start; length=np.linalg.norm(displacement)
    direction=displacement/length
    reference=np.load(p/'reference.npz');info=r['info']
    model=RobotModel(tcp_offset=info['tcp_offset'],tcp_orientation=info['tcp_orientation'],
                     end_effector=info['end_effector'],angle_offsets=info['angle_offsets'])
    path=model.forward(reference['q'][-1])[:3,3]-model.forward(reference['q'][0])[:3,3]
    distance=np.linalg.norm(path)
    assert distance>1e-6 and length>1e-3, 'Endpoint displacement is too small for a timing comparison'
    rate=np.array([model.jacobian(q)[:3]@dq for q,dq in zip(reference['q'],reference['dq'])])@path/distance**2
    reference_speed=length*rate
    velocity=savgol_filter(x,5,2,deriv=1,delta=1/60,axis=0)
    speed=velocity@direction
    selected=(t>0)&(t<duration+.5)
    best=None
    for delay in np.arange(-.05,.301,.005):
        expected=np.interp(t-delay,reference['times'],reference_speed,left=0,right=0)
        mse=np.mean((speed[selected]-expected[selected])**2)
        if best is None or mse<best[0]:best=(mse,delay,expected)
    mse,delay,expected=best
    peak=np.max(reference_speed)
    middle=(t>delay+.2*duration)&(t<delay+.8*duration)
    res=dict(translation_px=length,fitted_delay_s=delay,
             velocity_nrmse=float(np.sqrt(mse)/peak),
             peak_ratio=float(np.max(speed[selected])/peak),
             middle_minimum_ratio=float(np.min(speed[middle]/expected[middle])),
             middle_stopped_fraction=float(np.mean(speed[middle]<.15*expected[middle])))
    full=t>0;late=t>duration+.5;tail=t>t[-1]-.5
    vector_error=np.sum((velocity-expected[:,None]*direction)**2,axis=1)
    perpendicular=(x-start)-np.outer((x-start)@direction,direction)
    res.update(endpoint_window='final 0.5 s of capture',
               expected_timing='recorded joint reference via TCP Jacobian',
               full_velocity_error=float(np.sqrt(np.trapz(vector_error[full],t[full])/duration)/peak),
               post_plan_excursion_px=float(np.max(np.linalg.norm(x[late]-x[late][0],axis=1))),
               final_window_excursion_px=float(np.max(np.linalg.norm(x[tail]-np.mean(x[tail],axis=0),axis=1))),
               cross_track_rms_px=float(np.sqrt(np.mean(np.sum(perpendicular[selected]**2,axis=1)))),
               completion_delay_s=r.get('motion_finished_unix',r['accepted_unix']+duration)-r['accepted_unix']-duration)
    (p/(args.label+'.json')).write_text(json.dumps(res,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,4));ax.plot(t,speed,label='Camera, along endpoint displacement')
    ax.plot(t,expected,label='Command timing, endpoint scaled');ax.legend();ax.grid(alpha=.3)
    ax.axvline(duration,color='gray',ls=':',label='planned end')
    ax.set(xlabel='Time after API response (s)',ylabel='Projected speed (pixel/s)',title=p.name)
    fig.tight_layout();fig.savefig(p/(args.label+'.png'));plt.close(fig)
    print(p.name,json.dumps(res))
