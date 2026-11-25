"""50 Hz homomorphic joint or SpaceMouse-style TCP velocity input example."""
import argparse
import asyncio
import json
import math
import time
import websockets
from piper_control.client import RobotClient

parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8088')
parser.add_argument('--mode', choices=['joints','twist'], default='joints')
args = parser.parse_args()

async def run():
    with RobotClient(args.url) as robot:
        robot.enable()
        q = robot.state()['positions']
        uri = args.url.replace('http://','ws://').replace('https://','wss://')+'/ws/control'
        async with websockets.connect(uri) as ws:
            start = time.monotonic()
            for index in range(100):
                # Replace this signal with the leader joints or mapped SpaceMouse axes.
                signal = math.sin(index/100*2*math.pi)
                if args.mode == 'joints':
                    target = q.copy()
                    target[0] += .01*signal
                    command = {'type':'joints','positions':target}
                else:
                    command = {'type':'twist','linear':[0,0,.002*signal],
                               'angular':[0,0,0], 'frame':'base'}
                await ws.send(json.dumps(command))
                reply = json.loads(await ws.recv())
                if not reply['accepted']:
                    raise RuntimeError(reply)
                await asyncio.sleep(max(0, start+(index+1)*.02-time.monotonic()))
        # A dropped producer holds after the configured stream_timeout, too.
        robot.stop()

asyncio.run(run())
