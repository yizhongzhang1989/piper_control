"""Robot-agnostic synchronous Python client. It contains no Piper-specific logic."""
import time
import httpx


class RobotClient:
    def __init__(self, url="http://127.0.0.1:8088", timeout=30.):
        self.http = httpx.Client(base_url=url.rstrip("/"), timeout=timeout)

    def _request(self, method, path, **kwargs):
        response = self.http.request(method, "/api/"+path, **kwargs)
        if response.is_error:
            raise RuntimeError(f"{response.status_code}: {response.json().get('detail', response.text)}")
        return response.json()

    def info(self): return self._request("GET", "info")
    def state(self): return self._request("GET", "state")
    def enable(self): return self._request("POST", "enable")
    def disable(self): return self._request("POST", "disable")
    def stop(self): return self._request("POST", "stop")
    def motion(self, motion_id): return self._request("GET", "motions/"+motion_id)
    def movej(self, positions, duration=None):
        return self._request("POST", "movej", json=dict(positions=list(positions), duration=duration))
    def movel(self, pose, **options):
        return self._request("POST", "movel", json=dict(pose=pose, **options))
    def movec(self, via, pose, **options):
        return self._request("POST", "movec", json=dict(via=via, pose=pose, **options))
    def trajectory(self, points, joint_names=None):
        return self._request("POST", "trajectory", json=dict(points=points, joint_names=joint_names))
    def servo_joints(self, positions):
        return self._request("POST", "stream/joints", json=dict(positions=list(positions)))
    def servo_pose(self, pose):
        return self._request("POST", "stream/tcp", json=dict(pose=pose))
    def servo_twist(self, linear, angular=(0,0,0), frame="base"):
        return self._request("POST", "stream/twist", json=dict(linear=list(linear), angular=list(angular), frame=frame))
    def wait(self, motion, timeout=None):
        motion_id = motion if isinstance(motion, str) else motion["id"]
        deadline = time.monotonic()+(timeout if timeout is not None else (motion.get("duration", 30)+10 if isinstance(motion, dict) else 60))
        while time.monotonic() < deadline:
            result = self.motion(motion_id)
            if result["status"] == "succeeded":
                return result
            if result["status"] in ("cancelled", "aborted"):
                raise RuntimeError(f"motion {result['status']}: {result.get('error')}")
            time.sleep(.02)
        raise TimeoutError("motion did not complete before client deadline")
    def close(self): self.http.close()
    def __enter__(self): return self
    def __exit__(self, *exc): self.close()
