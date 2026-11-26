"""Local weak-perspective camera projection from independent XYZ line endpoints.
Fits only one time shift for each new arc; no per-arc shape or gain fit.
Pixel error is a motion-quality observation, not calibrated metric accuracy.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from piper_control.model import RobotModel
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--calibration',nargs=3,required=True,metavar=('X_RECORD','Y_RECORD','Z_RECORD'));parser.add_argument('--projection-output',required=True);parser.add_argument('records',nargs='+');args=parser.parse_args();sources=args.calibration;xyz=[];pixels=[]
for name in sources:
 p=Path(name);r=json.loads((p/'report.json').read_text());d=np.load(p/'camera-analysis.npz');t=d['t'];x=d['position_px'];T=r['motion']['duration'];pixels.append(np.mean(x[t>T+.7],axis=0)-np.mean(x[t<0],axis=0));xyz.append(np.array(r['after']['tcp_pose']['position'])-r['before']['tcp_pose']['position'])
projection=np.linalg.solve(np.array(xyz),np.array(pixels)).T
Path(args.projection_output).write_text(json.dumps(dict(sources=sources,pixels_per_m=projection.tolist(),condition=float(np.linalg.cond(xyz)),note=__doc__),indent=2))
m=None
for name in args.records:
 p=Path(name);r=json.loads((p/'report.json').read_text());info=r['info'];m=RobotModel(tcp_offset=info['tcp_offset'],tcp_orientation=info['tcp_orientation'],end_effector=info['end_effector'],angle_offsets=info['angle_offsets']);d=np.load(p/'camera-analysis.npz');t=d['t'];x=d['position_px'];ref=np.load(p/'reference.npz');duration=r['motion']['duration'];v=savgol_filter(x,5,2,deriv=1,delta=1/60,axis=0)
 rv=np.array([m.jacobian(q)[:3]@dq for q,dq in zip(ref['q'],ref['dq'])])@projection.T
 rp=np.array([m.forward(q)[:3,3] for q in ref['q']]);rp=(rp-rp[0])@projection.T
 selected=(t>0)&(t<duration+.5);best=None
 for delay in np.arange(-.05,.301,.005):
  expected=np.column_stack([np.interp(t-delay,ref['times'],rv[:,j],left=0,right=0) for j in range(2)])
  mse=np.mean(np.sum((v[selected]-expected[selected])**2,axis=1))
  if best is None or mse<best[0]:best=mse,delay,expected
 mse,delay,expected=best;peak=np.max(np.linalg.norm(rv,axis=1));middle=(t>delay+.2*duration)&(t<delay+.8*duration);projected=np.sum(v*expected,axis=1)/np.maximum(np.linalg.norm(expected,axis=1),1e-6)
 position=np.column_stack([np.interp(t-delay,ref['times'],rp[:,j]) for j in range(2)])+np.mean(x[t<0],axis=0)
 res=dict(velocity_nrmse=float(np.sqrt(mse)/peak),fitted_delay_s=delay,peak_speed_ratio=float(np.max(np.linalg.norm(v[selected],axis=1))/peak),middle_stopped_fraction=float(np.mean(projected[middle]<.15*np.linalg.norm(expected[middle],axis=1))),position_rms_px=float(np.sqrt(np.mean(np.sum((x[selected]-position[selected])**2,axis=1)))),projection_sources=sources)
 (p/'camera-path-reference.json').write_text(json.dumps(res,indent=2));fig,axs=plt.subplots(2,1,figsize=(10,6));axs[0].plot(x[:,0],x[:,1],label='camera');axs[0].plot(position[:,0],position[:,1],label='projected reference');axs[0].axis('equal');axs[0].set(xlabel='X (pixel)',ylabel='Y (pixel)');axs[1].plot(t,np.linalg.norm(v,axis=1),label='camera');axs[1].plot(t,np.linalg.norm(expected,axis=1),label='projected reference');axs[1].set(xlabel='Time after API response (s)',ylabel='Speed (pixel/s)')
 for ax in axs:ax.legend();ax.grid(alpha=.3)
 fig.suptitle(p.name);fig.tight_layout();fig.savefig(p/'camera-path-reference.png');plt.close(fig);print(p.name,json.dumps(res))
