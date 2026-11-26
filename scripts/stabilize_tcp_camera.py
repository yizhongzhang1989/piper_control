#!/usr/bin/env python3
"""Remove camera translation/rotation/focus breathing using static background features.

Offline only. Inspect camera-stabilization-features.jpg: green features must be on
 the tool, red features on the fixed scene. These ROIs are specific to this setup.
The fitted background transform is stored with the measurements for auditing.
"""
import cv2,json,sys,numpy as np
from pathlib import Path
from scipy.signal import savgol_filter
for folder in sys.argv[1:]:
 p=Path(folder);r=json.loads((p/'report.json').read_text());ts=np.load(p/'camera-times.npy');cap=cv2.VideoCapture(str(p/'camera.avi'));ok,im=cap.read();assert ok;prev=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 mask=np.zeros_like(prev);mask[260:316,254:340]=255
 fg=cv2.goodFeaturesToTrack(prev,80,.03,5,mask=mask,blockSize=5);fbase=fg.copy()
 mask[:]=0;mask[350:480,480:610]=255;mask[460:530,180:300]=255;mask[20:180,800:940]=255
 bg=cv2.goodFeaturesToTrack(prev,160,.03,6,mask=mask,blockSize=5);bbase=bg.copy();assert len(bg)>20
 vis=im.copy()
 for x,y in fg[:,0]:cv2.circle(vis,(int(x),int(y)),3,(0,255,0),1)
 for x,y in bg[:,0]:cv2.circle(vis,(int(x),int(y)),2,(0,0,255),1)
 cv2.imwrite(str(p/'camera-stabilization-features.jpg'),vis)
 pos=[np.zeros(2)];transforms=[np.array([[1,0,0],[0,1,0]],float)];quality=[]
 def track(curr,points,base):
  nxt,st,_=cv2.calcOpticalFlowPyrLK(prev,curr,points,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.005))
  back,st2,_=cv2.calcOpticalFlowPyrLK(curr,prev,nxt,None,winSize=(21,21),maxLevel=3)
  good=(st.ravel()==1)&(st2.ravel()==1)&(np.linalg.norm(back-points,axis=2).ravel()<.5)
  return nxt[good],base[good]
 while True:
  ok,im=cap.read()
  if not ok:break
  curr=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);fg,fbase=track(curr,fg,fbase);bg,bbase=track(curr,bg,bbase);assert len(fg)>=6 and len(bg)>=15
  affine,inlier=cv2.estimateAffinePartial2D(bbase[:,0],bg[:,0],method=cv2.RANSAC,ransacReprojThreshold=.5,maxIters=500,confidence=.999);assert affine is not None and inlier.sum()>=15
  corrected=cv2.transform(fg,cv2.invertAffineTransform(affine));pos.append(np.median((corrected-fbase)[:,0],axis=0));transforms.append(affine);quality.append(int(inlier.sum()));prev=curr
 cap.release();assert len(pos)==len(ts);t=ts-r['accepted_unix'];grid=np.arange(t[0],t[-1],1/60);x=np.array([np.interp(grid,t,np.array(pos)[:,j]) for j in range(2)]).T;v=savgol_filter(x,5,2,deriv=1,delta=1/60,axis=0)
 np.savez_compressed(p/'camera-stabilized.npz',t=grid,position_px=x,speed_px_s=np.linalg.norm(v,axis=1),transform=np.array(transforms),frame_t=t)
 tr=np.array(transforms);before=t<0;scale=np.sqrt(tr[:,0,0]**2+tr[:,0,1]**2)
 print(p.name,json.dumps(dict(foreground_features=len(fg),background_features=len(bg),minimum_background_inliers=min(quality),background_translation_ptp_px=np.ptp(tr[:,:,2],axis=0).tolist(),scale_ptp=float(np.ptp(scale)),premotion_translation_ptp_px=np.ptp(tr[before,:,2],axis=0).tolist())))
