"""Robot-agnostic ROS2 FollowJointTrajectory client (small J1 out-and-back)."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

parser = argparse.ArgumentParser()
parser.add_argument('--namespace', default='/robot')
parser.add_argument('--output')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('generic_trajectory_example')
latest = []
samples = []
node.create_subscription(JointState, args.namespace+'/joint_states',
                         lambda msg: latest.append(msg) if not latest else latest.__setitem__(0,msg),
                         qos_profile_sensor_data)
client = ActionClient(node, FollowJointTrajectory, args.namespace+'/follow_joint_trajectory')
handle = None
try:
    if not client.wait_for_server(timeout_sec=5):
        raise RuntimeError('trajectory action unavailable')
    deadline = time.monotonic()+5
    while not latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if not latest:
        raise RuntimeError('joint feedback unavailable')
    initial = list(latest[0].position)
    target = initial.copy(); target[0] += .01
    goal = FollowJointTrajectory.Goal()
    # Named axes permit controllers to send points in a different joint order.
    goal.trajectory.joint_names = list(reversed(latest[0].name))
    for q,t in ((target,2),(initial,4)):
        point = JointTrajectoryPoint()
        point.positions = list(reversed(q))
        point.time_from_start.sec = t
        goal.trajectory.points.append(point)
    future = client.send_goal_async(goal, feedback_callback=lambda msg: samples.append({
        'actual': list(msg.feedback.actual.positions),
        'desired': list(msg.feedback.desired.positions)}))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done(): raise TimeoutError('goal acceptance timed out')
    handle = future.result()
    if not handle.accepted: raise RuntimeError('goal rejected')
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done(): raise TimeoutError('goal result timed out')
    result = future.result()
    report = dict(status=result.status, error_code=result.result.error_code,
                  error_string=result.result.error_string, initial=initial,
                  measured_final=list(latest[0].position), feedback_samples=samples)
    if args.output: Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    if result.result.error_code != 0: raise RuntimeError(result.result.error_string)
    print(f'ROS trajectory succeeded, {len(samples)} feedback messages')
except Exception:
    if handle is not None:
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    raise
finally:
    node.destroy_node()
    rclpy.shutdown()
