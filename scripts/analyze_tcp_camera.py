#!/usr/bin/env python3
"""Offline BRIO tool tracking for record_tcp_motion.py output (960x540 video).
The fixed ROI is specific to this commissioning camera; inspect the feature image.
This script never sends robot commands.
"""
import cv2,json,sys,numpy as np
from pathlib import Path
from scipy.signal import savgol_filter
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
for folder in sys.argv[1:]:
 p=Path(folder);r=json.loads((p/'report.json').read_text());ts=np.load(p/'camera-times.npy');c=cv2.VideoCapture(str(p/'camera.avi'));ok,im=c.read();assert ok;prev=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 # On the tool flange and gripping fixture; excludes the table and background.
 mask=np.zeros_like(prev);mask[260:316,254:340]=255
 points=cv2.goodFeaturesToTrack(prev,80,.03,5,mask=mask,blockSize=5);assert len(points)>8
 vis=im.copy()
 for x,y in points[:,0]:cv2.circle(vis,(int(x),int(y)),3,(0,255,0),1)
 cv2.imwrite(str(p/'camera-tracking-features.jpg'),vis)
 displacement=[np.zeros(2)];counts=[len(points)];base=points.copy();last=points.copy()
 while True:
  ok,im=c.read()
  if not ok:break
  curr=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);nxt,status,err=cv2.calcOpticalFlowPyrLK(prev,curr,last,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.005))
  back,st,_=cv2.calcOpticalFlowPyrLK(curr,prev,nxt,None,winSize=(21,21),maxLevel=3)
  valid=(status.ravel()==1)&(st.ravel()==1)&(np.linalg.norm(back-last,axis=2).ravel()<.5)
  if valid.sum()<6:raise RuntimeError('not enough camera features')
  base=base[valid];nxt=nxt[valid];displacement.append(np.median((nxt-base)[:,0],axis=0));counts.append(len(nxt));last=nxt;prev=curr
 c.release();d=np.array(displacement);assert len(d)==len(ts)
 t=ts-r['accepted_unix'];grid=np.arange(t[0],t[-1],1/60);pos=np.array([np.interp(grid,t,d[:,j]) for j in range(2)]).T
 v=savgol_filter(pos,5,2,deriv=1,delta=1/60,axis=0);speed=np.linalg.norm(v,axis=1);duration=r['motion']['duration'];middle=(grid>.2*duration)&(grid<.8*duration)
 metric=dict(retained_features=counts[-1],middle_speed_mean_px_s=float(speed[middle].mean()),middle_speed_std_px_s=float(speed[middle].std()),middle_speed_cv=float(speed[middle].std()/speed[middle].mean()),peak_speed_px_s=float(speed[(grid>0)&(grid<duration)].max()),middle_p10_speed_px_s=float(np.quantile(speed[middle],.1)),middle_p90_speed_px_s=float(np.quantile(speed[middle],.9)))
 (p/'camera-analysis.json').write_text(json.dumps(metric,indent=2));np.savez_compressed(p/'camera-analysis.npz',t=grid,position_px=pos,speed_px_s=speed)
 fig,axs=plt.subplots(2,1,figsize=(10,6),sharex=True);axs[0].plot(grid,pos);axs[1].plot(grid,speed);axs[0].set_ylabel('Tool displacement (pixel)');axs[1].set_ylabel('Tool speed (pixel/s)');axs[1].set_xlabel('Time after API response (s)')
 for ax in axs:ax.grid();ax.axvline(duration,color='gray')
 fig.suptitle(p.name);fig.tight_layout();fig.savefig(p/'camera-analysis.png');plt.close(fig);print(p.name,json.dumps(metric))
