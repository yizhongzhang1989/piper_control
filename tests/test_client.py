import httpx
import pytest

from piper_control.client import RobotClient


def test_default_wait_allows_the_server_configured_settling_time(monkeypatch):
    now = [0.]
    monkeypatch.setattr('piper_control.client.time.monotonic', lambda: now[0])
    monkeypatch.setattr('piper_control.client.time.sleep', lambda dt: now.__setitem__(0, now[0]+dt))
    requests = []

    def response(request):
        requests.append(request.url.path)
        body = ({'goal_timeout': 12.} if request.url.path == '/api/info'
                else {'id': 'test', 'duration': .5,
                      'status': 'succeeded' if now[0] >= 11 else 'running'})
        return httpx.Response(200, json=body)

    with RobotClient('http://example.test') as robot:
        robot.http.close()
        robot.http = httpx.Client(base_url='http://example.test', transport=httpx.MockTransport(response))
        assert robot.wait({'id': 'test', 'duration': .5})['status'] == 'succeeded'
        assert requests.count('/api/info') == 1
        now[0] = 0.
        requests.clear()
        with pytest.raises(TimeoutError):
            robot.wait('test', timeout=.1)
        assert '/api/info' not in requests
