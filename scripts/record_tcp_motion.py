#!/usr/bin/env python3
"""Record a small hardware motion through the public API, plus passive CAN and camera evidence.

This command moves the arm. Confirm the chosen local path is clear before running.
Joint travel defaults to a 0.4 rad envelope; that is not a collision check.
"""
import argparse,collections,datetime,json,math,socket,struct,threading,time
from pathlib import Path
from zoneinfo import ZoneInfo
import cv2,numpy as np
from piper_control.client import RobotClient
from piper_control.model import RobotModel
from piper_control.planning import Planner,matrix_pose,pose_matrix
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--url',default='http://127.0.0.1:8090');parser.add_argument('--camera',default='/dev/video0');parser.add_argument('--camera-focus',type=float);parser.add_argument('--camera-resolution',type=int,nargs=2,default=[1280,720],metavar=('WIDTH','HEIGHT'));parser.add_argument('--camera-roi',type=int,nargs=4,metavar=('X','Y','WIDTH','HEIGHT'),help='record this full-resolution region instead of resizing to 960x540');parser.add_argument('--case',choices=['line_x','line_y','line_z','line_xy','line_xz','arc_xz','arc_xy','arc_yz','line_path','arc_path','return',*[f'joint{i}' for i in range(1,7)]],required=True);parser.add_argument('--distance',type=float,default=.03);parser.add_argument('--speed',type=float,default=.02);parser.add_argument('--out',required=True);parser.add_argument('--return-pose')
parser.add_argument('--path-file',help='line_path/arc_path: JSON with target_pose and, for arc_path, via')
parser.add_argument('--angular-speed',type=float,default=.3,help='Cartesian angular speed limit, rad/s')
parser.add_argument('--post-motion-seconds',type=float,default=5.,help='record 2..30 seconds after the terminal motion result')
parser.add_argument('--max-observed-joint-speed',type=float,default=.3,help='stop the trial if any measured joint speed exceeds this rad/s envelope')
parser.add_argument('--max-joint-travel',type=float,default=.4,help='maximum planned joint excursion in rad; measured travel has an additional 0.05 rad margin')
a=parser.parse_args()
if bool(a.path_file) != (a.case in ('line_path','arc_path')):
 parser.error('--path-file is required exclusively for line_path/arc_path')
if not math.isfinite(a.angular_speed) or a.angular_speed <= 0:
 parser.error('--angular-speed must be positive and finite')
if not math.isfinite(a.post_motion_seconds) or not 2 <= a.post_motion_seconds <= 30:
 parser.error('--post-motion-seconds must be within 2..30')
if not math.isfinite(a.max_observed_joint_speed) or a.max_observed_joint_speed <= 0:
 parser.error('--max-observed-joint-speed must be positive and finite')
if not math.isfinite(a.max_joint_travel) or not 0 < a.max_joint_travel <= math.pi:
 parser.error('--max-joint-travel must be within (0, pi]')
out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
r=RobotClient(a.url,timeout=30);before=r.state();info=r.info();assert info['backend']=='hardware';assert before['command_ready'] and before['mode']=='idle' and not before['errors']
m=RobotModel(tcp_offset=info['tcp_offset'],tcp_orientation=info['tcp_orientation'],end_effector=info['end_effector'],angle_offsets=info['angle_offsets']);q=np.array(before['target_positions']);T=m.forward(q);E=T.copy();V=T.copy();dist=a.distance;planr=Planner(m,max(x['velocity'] for x in info['joint_limits']))
planr.velocity=np.array([x['velocity'] for x in info['joint_limits']])
if a.case in ('line_path','arc_path'):
 if not a.path_file:parser.error('line_path/arc_path requires --path-file')
 path=json.loads(Path(a.path_file).read_text());E=pose_matrix(path['target_pose'])
 if a.case=='arc_path':V=pose_matrix(path['via'])
elif a.case=='return':E=m.forward(json.loads(Path(a.return_pose).read_text())['target_positions'])
elif a.case.startswith('joint'):
 end_q=q.copy();end_q[int(a.case[-1])-1]+=dist;E=m.forward(end_q)
elif a.case.startswith('line_'):
 axes=a.case.removeprefix('line_')
 for axis in axes:E['xyz'.index(axis),3]+=dist/math.sqrt(len(axes))
else:
 axes=a.case.removeprefix('arc_');first,second=['xyz'.index(axis) for axis in axes]
 E[first,3]+=dist;V[first,3]+=dist/2;V[second,3]+=dist/2
via=matrix_pose(V) if a.case.startswith('arc_') else None
plan=planr.movej(q,end_q,duration=1.875*abs(dist)/a.speed) if a.case.startswith('joint') else planr.cartesian(q,matrix_pose(E),via,speed=a.speed,angular_speed=a.angular_speed)
# Review larger excursions explicitly; no mode/enable actions.
pts=np.array([plan.sample(t)[0] for t in np.linspace(0,plan.duration,200)]);assert np.max(abs(pts-q))<a.max_joint_travel
plan_ts=np.r_[np.arange(0,plan.duration,.0025),plan.duration];qq=np.array([plan.sample(t)[0] for t in plan_ts]);vv=np.array([plan.sample(t)[1] for t in plan_ts]);np.savez_compressed(out/'reference.npz',times=plan_ts,q=qq,dq=vv)
if np.max(np.abs(vv)) >= a.max_observed_joint_speed:
 raise ValueError('planned speed exceeds the observation envelope; review the path and explicitly set a suitable --max-observed-joint-speed')
receiver=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);receiver.setsockopt(socket.SOL_SOCKET,35,1);receiver.bind((before['extra']['can_port'],));receiver.settimeout(.02);records=collections.defaultdict(list);stop=threading.Event();camera_errors=[]
def can_rx():
 while not stop.is_set():
  try:raw,anc,_,_=receiver.recvmsg(16,socket.CMSG_SPACE(16))
  except TimeoutError:continue
  cid,n,d=struct.unpack('=IB3x8s',raw);cid&=0x1fffffff
  if not (0x251<=cid<=0x256 or 0x261<=cid<=0x266 or 0x2a5<=cid<=0x2a7 or 0x15a<=cid<=0x15f or cid in (0x150,0x151,0x2a1,0x471)):continue
  stamp=time.time()
  for level,kind,data in anc:
   if level==socket.SOL_SOCKET and kind==35:
    sec,ns=struct.unpack('=qq',data[:16]);stamp=sec+ns*1e-9
  records[cid].append((stamp,*d[:n]))
camera_controls={};frame_times=[];camera_ready=threading.Event()
def camera():
 cap=cv2.VideoCapture(a.camera,cv2.CAP_V4L2);writer=None
 try:
  if not cap.isOpened():raise RuntimeError('camera unavailable')
  cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'));cap.set(cv2.CAP_PROP_FRAME_WIDTH,a.camera_resolution[0]);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,a.camera_resolution[1]);cap.set(cv2.CAP_PROP_FPS,60);cap.set(cv2.CAP_PROP_BUFFERSIZE,2)
  if a.camera_focus is not None:
   if not cap.set(cv2.CAP_PROP_AUTOFOCUS,0) or not cap.set(cv2.CAP_PROP_FOCUS,a.camera_focus):raise RuntimeError('could not set fixed camera focus')
  camera_controls.update({key:cap.get(getattr(cv2,key)) for key in ['CAP_PROP_AUTOFOCUS','CAP_PROP_FOCUS','CAP_PROP_AUTO_EXPOSURE','CAP_PROP_EXPOSURE','CAP_PROP_FPS']})
  for _ in range(30):ok,fr=cap.read()
  if not ok:raise RuntimeError('camera read failed')
  camera_controls['capture_resolution']=[fr.shape[1],fr.shape[0]]
  camera_controls['recording_roi']=a.camera_roi
  size=(960,540)
  if a.camera_roi:
   x,y,w,h=a.camera_roi
   if min(x,y)<0 or min(w,h)<=0 or x+w>fr.shape[1] or y+h>fr.shape[0]:raise ValueError('camera ROI is outside the captured frame')
   size=(w,h)
  writer=cv2.VideoWriter(str(out/'camera.avi'),cv2.VideoWriter_fourcc(*'MJPG'),cap.get(cv2.CAP_PROP_FPS),size)
  if not writer.isOpened():raise RuntimeError('camera video writer could not open')
  cv2.imwrite(str(out/'camera-start.jpg'),fr);camera_ready.set()
  while not stop.is_set():
   ok,fr=cap.read()
   if not ok:raise RuntimeError('camera lost frames')
   frame_times.append(time.time());writer.write(fr[y:y+h,x:x+w] if a.camera_roi else cv2.resize(fr,size))
  cv2.imwrite(str(out/'camera-end.jpg'),fr)
 except Exception as exc:camera_errors.append(repr(exc));camera_ready.set()
 finally:
  cap.release()
  if writer:writer.release()
threads=[threading.Thread(target=can_rx),threading.Thread(target=camera)]
for th in threads:th.start()
report={'timestamp':datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'case':a.case,'requested_speed':a.speed,'target_pose':matrix_pose(E),'via':via,'before':before,'info':info,'samples':[],'camera_timing_note':'Host read timestamps; no calibrated exposure/CAN alignment','capture_transmitted_frames':0,'post_motion_seconds':a.post_motion_seconds};failure=None;motion=None
report['max_observed_joint_speed_rad_s']=a.max_observed_joint_speed
report['max_joint_travel_rad']=a.max_joint_travel
report['requested_angular_speed_rad_s']=a.angular_speed
try:
 if not camera_ready.wait(5) or camera_errors:raise RuntimeError(str(camera_errors))
 time.sleep(.5);report['submitted_unix']=time.time()
 motion=r.movej(end_q.tolist(),duration=plan.duration) if a.case.startswith('joint') else (r.movec(via,matrix_pose(E),speed=a.speed,angular_speed=a.angular_speed) if via else r.movel(matrix_pose(E),speed=a.speed,angular_speed=a.angular_speed));report['motion']=motion;report['accepted_unix']=time.time();deadline=time.monotonic()+motion['duration']+info.get('goal_timeout',5.)+2+a.post_motion_seconds;finished=None
 if abs(motion['duration']-plan.duration)>1e-5:raise RuntimeError('server duration differs from the recorded reference')
 while time.monotonic()<deadline:
  st=r.state();report['samples'].append(st)
  if not st['command_ready'] or st['errors']:raise RuntimeError('feedback or readiness failure')
  if max(map(abs,st['velocities']))>a.max_observed_joint_speed:raise RuntimeError('observed joint velocity exceeded trial envelope')
  if np.max(np.abs(np.array(st['positions'])-np.array(before['positions'])))>a.max_joint_travel+.05:raise RuntimeError('observed travel exceeded trial envelope')
  result=r.motion(motion['id'])
  if result['status']!='running':
   if finished is None:finished=time.monotonic();report['motion_finished_unix']=time.time();report['result']=result
   if time.monotonic()-finished>a.post_motion_seconds:break
  time.sleep(.02)
 if finished is None:raise RuntimeError('motion did not finish within timeout')
except Exception as exc:
 failure=repr(exc)
 if motion:
  try:r.stop()
  except Exception as stop_exc:report['stop_error']=repr(stop_exc)
finally:
 report['after']=r.state();report['capture_finished_unix']=time.time();stop.set()
 for th in threads:th.join(timeout=3)
 receiver.close();r.close();report['camera_controls']=camera_controls;report['camera_errors']=camera_errors;report['camera_frames']=len(frame_times)
 if len(frame_times)>1:
  gaps=np.diff(frame_times);report['camera_actual_timing']={'fps':(len(frame_times)-1)/(frame_times[-1]-frame_times[0]),'p99_gap_seconds':float(np.quantile(gaps,.99)),'max_gap_seconds':float(max(gaps))}
 if not failure and (camera_errors or not frame_times):failure='camera acquisition failed'
 if not failure and report.get('result',{}).get('status')!='succeeded':failure='motion did not succeed'
 report['failure']=failure
 np.savez_compressed(out/'can.npz',**{f'id_{cid:03x}':np.asarray(rows) for cid,rows in records.items()});np.save(out/'camera-times.npy',frame_times)
 (out/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
 print(json.dumps({'out':str(out),'failure':failure,'result':report.get('result'),'camera_frames':len(frame_times),'initial_error':(np.array(before['positions'])-q).tolist(),'final_error':(np.array(report['after']['positions'])-report['after']['target_positions']).tolist()},indent=2))
if failure:raise RuntimeError(failure)
