"""One controller shared by every transport and backend."""
import copy
from collections import deque
import math
import threading
import time
import uuid
import numpy as np
from .planning import Planner, vector


class RobotController:
    def __init__(self, backend, model, rate_hz=200., max_velocity=.3,
                 stream_timeout=.2, feedback_timeout=.2, goal_tolerance=.01,
                 goal_timeout=5., goal_velocity_tolerance=.02, goal_settle_time=.15,
                 tcp_position_tolerance=None):
        for name, value in dict(rate_hz=rate_hz, stream_timeout=stream_timeout,
                                feedback_timeout=feedback_timeout, goal_tolerance=goal_tolerance,
                                goal_timeout=goal_timeout, goal_velocity_tolerance=goal_velocity_tolerance,
                                goal_settle_time=goal_settle_time).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if goal_settle_time > goal_timeout:
            raise ValueError("goal_settle_time cannot exceed goal_timeout")
        if tcp_position_tolerance is not None and (not math.isfinite(tcp_position_tolerance) or tcp_position_tolerance <= 0):
            raise ValueError("tcp_position_tolerance must be positive and finite")
        self.backend, self.model = backend, model
        self.rate_hz = rate_hz
        self.stream_timeout, self.feedback_timeout = stream_timeout, feedback_timeout
        self.goal_tolerance, self.goal_timeout = goal_tolerance, goal_timeout
        self.goal_velocity_tolerance, self.goal_settle_time = goal_velocity_tolerance, goal_settle_time
        self.tcp_position_tolerance = tcp_position_tolerance
        self._tcp_goal = None
        self.planner = Planner(model, max_velocity)
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._thread = None
        initial_target = getattr(backend, "initial_target", None)
        self._q = None if initial_target is None else np.asarray(initial_target, dtype=float)
        self._plan = None
        self._start = 0.
        self._settled_since = self._settled_last = None
        self._goal_position_reached = False
        self._stream_time = None
        self._stream_goal = None
        self._twist = None
        self._mode = "idle"
        self._motion = None
        self._history = {}
        self._error = None
        self._epoch = 0
        self._cycle_times = deque(maxlen=max(2, int(rate_hz*2)))
        self._tracking(False)

    def _tracking(self, active):
        # Once the endpoint is within position tolerance, freeze corrections
        # while checking its speed dwell. A still-unreached goal may converge.
        update = getattr(self.backend, 'set_tracking_active', None)
        if update is not None:
            update(active)

    def _tracking_requested(self, now):
        return ((self._plan is not None and
                 (now-self._start < self._plan.duration or not self._goal_position_reached)) or
                (self._stream_time is not None and now-self._stream_time <= self.stream_timeout))

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="robot-control",
                                            daemon=not hasattr(self.backend, "hold"))
            self._thread.start()
        return self

    def _record(self, status, error=None):
        if self._motion:
            self._motion.update(status=status, error=error)
            self._history[self._motion["id"]] = copy.deepcopy(self._motion)
            if len(self._history) > 100:
                self._history.pop(next(iter(self._history)))

    def _cancel(self):
        if self._plan is not None:
            self._record("cancelled")
        self._plan = None
        self._tcp_goal = None
        self._settled_since = self._settled_last = None
        self._goal_position_reached = False
        self._stream_time = None
        self._stream_goal = self._twist = None
        self._epoch += 1

    def _ready(self):
        state = self.backend.read()
        if not state.get("connected") or state.get("feedback_age", 0.) > self.feedback_timeout:
            raise RuntimeError("robot feedback is stale or disconnected")
        if not state.get("enabled"):
            raise RuntimeError("robot is disabled; enable it before motion")
        if not state.get("command_ready", True):
            raise RuntimeError("firmware has not confirmed external joint control; inspect extra.firmware")
        if state.get("errors"):
            raise RuntimeError("robot reports: " + str(state["errors"]))
        return state

    def state(self):
        with self._lock:
            data = self.backend.read()
            result = dict(data)
            q = data.get("positions")
            result.update(joint_names=list(self.model.names),
                          timestamp=time.time(), monotonic_time=time.monotonic(),
                          mode=self._mode, backend=getattr(self.backend, "name", "unknown"),
                          tcp_pose=self.model.pose(q) if q is not None and np.all(np.isfinite(q)) else None,
                          motion=copy.deepcopy(self._motion),
                          target_positions=None if self._q is None else self._q.tolist(),
                          control_rate_hz=((len(self._cycle_times)-1)/max(.001, self._cycle_times[-1]-self._cycle_times[0])
                                           if len(self._cycle_times) > 1 else 0.))
            result["errors"] = list(data.get("errors", [])) + ([self._error] if self._error else [])
            return result

    def info(self):
        data = self.model.metadata()
        data.update(api_version="1.0", backend=getattr(self.backend, "name", "unknown"),
                    joint_names=list(self.model.names),
                    joint_limits=[dict(lower=float(l), upper=float(u), velocity=float(v))
                                  for l,u,v in zip(self.model.lower, self.model.upper, self.planner.velocity)],
                    model_url="/robot/model.urdf", base_frame="base_link", tcp_frame="tcp",
                    control_rate_hz=self.rate_hz, stream_timeout=self.stream_timeout,
                    feedback_timeout=self.feedback_timeout, goal_tolerance=self.goal_tolerance,
                    goal_velocity_tolerance=self.goal_velocity_tolerance,
                    goal_settle_time=self.goal_settle_time, goal_timeout=self.goal_timeout,
                    tcp_position_tolerance=self.tcp_position_tolerance,
                    units={"position":"m", "joint":"rad", "orientation":"quaternion_xyzw",
                           "time":"s", "effort":"Nm"},
                    capabilities=["servo_joints", "servo_pose", "servo_twist", "trajectory", "movej", "movel", "movec"])
        return data

    def enable(self):
        with self._lock:
            data = self.backend.read()
            if not data.get("connected") or data.get("feedback_age", 0) > self.feedback_timeout:
                raise RuntimeError("fresh feedback required to enable")
            if data.get("enabled") and data.get("command_ready", True):
                return self.state()
            self._cancel()
            self._q = np.array(data["positions"], dtype=float)
            self.backend.enable(True)
            self._tracking(False)
            self._error = None
            self._mode = "idle"
        return self.state()

    def disable(self):
        with self._lock:
            self._cancel()
            self.backend.enable(False)
            self._tracking(False)
            self._mode = "disabled"
        return self.state()

    def stop(self):
        with self._lock:
            self._cancel()
            self._tracking(False)
            data = self.backend.read()
            if data.get("positions") is not None:
                self._q = self._command_start(data)
            self._mode = "idle" if data.get("enabled") else "disabled"
        return self.state()

    def _command_start(self, data):
        # A following error must not become a position step at a command boundary.
        if (self._q is not None and self._q.shape == (len(self.model.names),)
                and np.all(np.isfinite(self._q))):
            return self._q.copy()
        return np.array(data["positions"], dtype=float)

    def _prepare(self):
        with self._lock:
            data = self._ready()
            if self._plan is not None or self._stream_time is not None:
                raise RuntimeError("robot is executing; stop before submitting a new planned motion")
            return self._command_start(data), self._epoch

    def _submit(self, plan, kind, epoch):
        with self._lock:
            self._ready()
            if epoch != self._epoch:
                raise RuntimeError("robot command changed while planning; submit again")
            self._cancel()
            self._plan, self._start = plan, time.monotonic()
            self._tcp_goal = (self.model.forward(plan.positions[-1])[:3, 3]
                              if kind in ('movel', 'movec') and self.tcp_position_tolerance is not None else None)
            self._motion = dict(id=uuid.uuid4().hex, kind=kind, status="running", progress=0.,
                                duration=plan.duration, error=None)
            self._mode, self._error = kind, None
            return copy.deepcopy(self._motion)

    def movej(self, positions, duration=None):
        start, epoch = self._prepare()
        return self._submit(self.planner.movej(start, positions, duration), "movej", epoch)

    def trajectory(self, points, joint_names=None):
        start, epoch = self._prepare()
        return self._submit(self.planner.trajectory(start, points, joint_names), "trajectory", epoch)

    def movel(self, pose, duration=None, speed=.03, angular_speed=.3):
        start, epoch = self._prepare()
        return self._submit(self.planner.cartesian(start, pose, duration=duration, speed=speed,
                                                  angular_speed=angular_speed), "movel", epoch)

    def movec(self, via, pose, duration=None, speed=.03, angular_speed=.3):
        start, epoch = self._prepare()
        return self._submit(self.planner.cartesian(start, pose, via, duration, speed, angular_speed), "movec", epoch)

    def motion(self, motion_id):
        with self._lock:
            if self._motion and self._motion["id"] == motion_id:
                return copy.deepcopy(self._motion)
            if motion_id not in self._history:
                raise KeyError(motion_id)
            return copy.deepcopy(self._history[motion_id])

    def servo_joints(self, positions):
        q = self.planner.joints(positions)
        with self._lock:
            data = self._ready()
            if self._stream_time is None:
                self._q = self._command_start(data)
            self._cancel()
            self._stream_goal = q
            self._stream_time = time.monotonic()
            self._mode, self._error = "servo_joints", None
        return {"accepted": True}

    def servo_pose(self, pose):
        from .planning import pose_matrix
        with self._lock:
            data = self._ready()
            seed = self._command_start(data)
            epoch = self._epoch
        q = self.model.inverse(pose_matrix(pose), seed)
        with self._lock:
            if epoch != self._epoch:
                raise RuntimeError("robot command changed while solving TCP target; send the latest target again")
            result = self.servo_joints(q)
            self._mode = "servo_pose"
            return result

    def servo_twist(self, linear, angular, frame="base"):
        if frame not in ("base", "tool"):
            raise ValueError("twist frame must be base or tool")
        v, w = vector(linear, 3, "linear"), vector(angular, 3, "angular")
        with self._lock:
            data = self._ready()
            if self._stream_time is None:
                self._q = self._command_start(data)
            self._cancel()
            self._twist = (v, w, frame)
            self._stream_time = time.monotonic()
            self._mode, self._error = "servo_twist", None
        return {"accepted": True}

    def _tick(self, now, dt):
        with self._lock:
            self._cycle_times.append(now)
            self._tracking(self._tracking_requested(now))
            self.backend.step(dt)
            data = self.backend.read()
            fresh = data.get("connected") and data.get("feedback_age", 0) <= self.feedback_timeout
            if not fresh or data.get("errors"):
                self._tracking(False)
                if self._plan is not None:
                    self._record("aborted", "feedback lost or hardware fault")
                self._plan = None
                self._stream_time = None
                # Feedback outages do not invalidate the commanded hold target.
                # The hardware watchdog holds this same target while feedback is
                # unavailable. Replacing it with measured position on recovery
                # would turn the following error into an unrequested motion.
                self._mode = "fault"
                self._epoch += 1
                return
            measured = np.asarray(data["positions"])
            if not data.get("enabled"):
                self._tracking(False)
                if self._plan is not None:
                    self._record("aborted", "robot disabled during motion")
                self._plan = None
                self._stream_time = None
                self._q = measured.copy()
                self._mode = "disabled"
                self._epoch += 1
                return
            if self._q is None:
                self._q = measured.copy()
            if self._mode == "fault" and self._error is None:
                self._mode = "idle"
            dq = np.zeros(len(measured))
            if not data.get("command_ready", True):
                self._tracking(False)
                if self._plan is not None:
                    self._record("aborted", "firmware left external joint control")
                self._plan = None
                self._stream_time = None
                self._epoch += 1
                self._mode = "fault"
                self._error = "firmware has not confirmed external joint control"
                # Maintain only the last fixed hold target; never advance a
                # trajectory on merely fresh telemetry without control authority.
                self.backend.command(self._q.tolist(), dq.tolist())
                return
            if self._error == "firmware has not confirmed external joint control":
                self._error = None
                self._mode = "idle"
            if self._plan is not None:
                elapsed = now-self._start
                self._q, dq = self._plan.sample(elapsed)
                self._motion["progress"] = min(1., elapsed/self._plan.duration)
                if elapsed >= self._plan.duration:
                    error = float(np.max(np.abs(measured-self._q)))
                    tcp_error = (float(np.linalg.norm(self.model.forward(measured)[:3, 3]-self._tcp_goal))
                                 if self._tcp_goal is not None else None)
                    tcp_within = tcp_error is None or tcp_error <= self.tcp_position_tolerance
                    self._goal_position_reached = error <= self.goal_tolerance and tcp_within
                    if tcp_error is not None:
                        self._motion['tcp_position_error'] = tcp_error
                    velocity = np.asarray(data.get("velocities", []), dtype=float)
                    speed = (float(np.max(np.abs(velocity)))
                             if velocity.shape == measured.shape and np.all(np.isfinite(velocity))
                             else math.inf)
                    within = self._goal_position_reached and speed <= self.goal_velocity_tolerance
                    if within:
                        if (self._settled_since is None or
                                now-self._settled_last > self.feedback_timeout):
                            self._settled_since = now
                        self._settled_last = now
                    else:
                        self._settled_since = self._settled_last = None
                    if within and now-self._settled_since >= self.goal_settle_time:
                        self._record("succeeded")
                        self._plan = None
                        self._mode = "idle"
                    elif elapsed > self._plan.duration+self.goal_timeout:
                        if error > self.goal_tolerance:
                            reason = f"goal tracking error {error:.4f} rad exceeds tolerance"
                        elif not tcp_within:
                            reason = f"TCP position error {tcp_error:.6f} m exceeds tolerance"
                        elif not math.isfinite(speed):
                            reason = "goal velocity feedback unavailable or invalid"
                        else:
                            reason = (f"goal did not settle below {self.goal_velocity_tolerance:.4f} rad/s "
                                      f"for {self.goal_settle_time:.3f} s; last speed {speed:.4f} rad/s")
                        self._record("aborted", reason)
                        self._plan = None
                        dq.fill(0.)
                        self._mode = "idle"
            elif self._stream_time is not None:
                if now-self._stream_time > self.stream_timeout:
                    self._stream_time = None
                    self._stream_goal = self._twist = None
                    self._mode = "idle"
                else:
                    goal = self._stream_goal
                    if self._twist is not None:
                        v, w, frame = self._twist
                        T = self.model.forward(self._q)
                        R = T[:3, :3]
                        twist = np.r_[R@v, R@w] if frame == "tool" else np.r_[v, w]
                        jacobian = self.model.jacobian(self._q)
                        velocity = jacobian.T @ np.linalg.solve(jacobian@jacobian.T+1e-6*np.eye(6), twist)
                        goal = self.planner.joints(self._q+velocity*dt)
                    delta = np.asarray(goal)-self._q
                    scale = max(1., float(np.max(np.abs(delta)/(self.planner.velocity*dt))))
                    dq = delta/(scale*dt)
                    self._q = self._q+dq*dt
            self._tracking(self._tracking_requested(now))
            self.backend.command(self._q.tolist(), dq.tolist())

    def _run(self):
        period, previous = 1/self.rate_hz, time.monotonic()
        deadline = previous
        while not self._closed.is_set():
            now = time.monotonic()
            try:
                self._tick(now, min(max(now-previous, .00001), period*4))
            except Exception as exc:
                with self._lock:
                    self._tracking(False)
                    if self._plan is not None:
                        self._record("aborted", str(exc))
                    self._plan = None
                    self._stream_time = None
                    self._error = str(exc)
                    self._mode = "fault"
                    self._epoch += 1
            previous = now
            deadline += period
            if deadline < time.monotonic()-period:
                deadline = time.monotonic()
            self._closed.wait(max(0., deadline-time.monotonic()))

    def close(self):
        if self._closed.is_set():
            return
        if hasattr(self.backend, "hold"):
            # Keep publishing until the persistent owner acknowledges takeover.
            # On failure keep this non-daemon control thread alive; never silently
            # remove the only hold publisher during application shutdown.
            with self._lock:
                self._cancel()
                self._mode = "idle"
            try:
                self.backend.hold()
            except Exception:
                self.start()
                raise
        self._closed.set()
        if self._thread:
            self._thread.join(timeout=3)
        self.backend.close()
