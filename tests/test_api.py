import time

from fastapi.testclient import TestClient

from piper_control.model import RobotModel
from piper_control.backends.simulation import SimulationBackend
from piper_control.controller import RobotController
from piper_control.server import create_app


def test_common_http_websocket_contract_and_preflight():
    model = RobotModel()
    robot = RobotController(SimulationBackend(model), model)
    with TestClient(create_app(robot)) as client:
        state = client.get('/api/state').json()
        assert state['backend'] == 'simulation'
        assert len(state['positions']) == 6
        assert client.get('/robot/model.urdf').status_code == 200
        assert client.get('/openapi.json').status_code == 200
        assert client.post('/api/movej', json={'positions':state['positions']}).status_code == 409
        assert client.post('/api/enable').status_code == 200
        for positions in ([0]*5, [0]*7, [99]*6):
            assert client.post('/api/trajectory', json={'points':[{'positions':positions, 'time_from_start':1}]}).status_code == 422
        assert client.post('/api/stream/twist', json={'linear':[0,0,0],'angular':[0,0,0],'frame':'unknown'}).status_code == 422
        with client.websocket_connect('/ws/state') as ws:
            assert ws.receive_json()['enabled']
        with client.websocket_connect('/ws/control') as ws:
            q = state['positions'].copy()
            q[0] += .005
            ws.send_json({'type':'joints','positions':q})
            assert ws.receive_json()['accepted']
            ws.send_json({'type':'tcp','pose':{'position':[99,0,0],'orientation':[0,0,0,1]}})
            assert ws.receive_json()['accepted'] is False
        assert client.post('/api/stop').json()['mode'] == 'idle'
        q = client.get('/api/state').json()['positions']
        q[0] += .01
        motion = client.post('/api/movej', json={'positions':q,'duration':.3}).json()
        for _ in range(100):
            result = client.get('/api/motions/'+motion['id']).json()
            if result['status'] != 'running':
                break
            time.sleep(.01)
        assert result['status'] == 'succeeded'
        assert client.post('/api/disable').json()['enabled'] is False
