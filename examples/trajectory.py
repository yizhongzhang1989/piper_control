"""The same upper-level controller works with any server implementing this API."""
import argparse
from piper_control.client import RobotClient

parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8088')
args = parser.parse_args()
with RobotClient(args.url) as robot:
    robot.enable()
    state = robot.state()
    start = state['positions']
    goal = start.copy()
    goal[0] += .02
    result = robot.wait(robot.trajectory([
        {'positions':goal, 'time_from_start':2.0},
        {'positions':start, 'time_from_start':4.0},
    ], joint_names=state['joint_names']))
    print(result)
