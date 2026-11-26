"""Robot-independent joint and Cartesian path planning. SI units, xyzw quaternions."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline, Slerp


def vector(value, n, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must contain {n} finite numbers")
    return a


def pose_matrix(pose):
    p = vector(pose["position"], 3, "position")
    q = vector(pose["orientation"], 4, "orientation (xyzw)")
    if np.linalg.norm(q) < 1e-8:
        raise ValueError("orientation quaternion must be nonzero")
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = p
    return T


def matrix_pose(T):
    return {"position": T[:3, 3].tolist(),
            "orientation": Rotation.from_matrix(T[:3, :3]).as_quat().tolist()}


@dataclass
class Plan:
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray | None = None

    def __post_init__(self):
        self.duration = float(self.times[-1])
        self._spline = (CubicHermiteSpline(self.times, self.positions, self.velocities)
                        if self.velocities is not None else None)

    def sample(self, t):
        t = float(np.clip(t, 0, self.duration))
        if self._spline is not None:
            return self._spline(t), self._spline(t, 1)
        i = min(len(self.times)-2, max(0, np.searchsorted(self.times, t, side="right")-1))
        dt = self.times[i+1]-self.times[i]
        u = (t-self.times[i])/dt
        s = 10*u**3-15*u**4+6*u**5
        ds = (30*u**2-60*u**3+30*u**4)/dt
        delta = self.positions[i+1]-self.positions[i]
        return self.positions[i]+s*delta, ds*delta

    def scale_time(self, factor):
        return Plan(self.times*factor, self.positions,
                    None if self.velocities is None else self.velocities/factor)


class Planner:
    def __init__(self, model, max_velocity=0.3):
        if not math.isfinite(max_velocity) or max_velocity <= 0:
            raise ValueError("max_velocity must be positive and finite")
        self.model = model
        self.velocity = np.minimum(np.asarray(model.velocity), max_velocity)

    def joints(self, q):
        q = vector(q, len(self.model.names), "positions")
        if np.any(q < np.asarray(self.model.lower)-1e-8) or np.any(q > np.asarray(self.model.upper)+1e-8):
            raise ValueError("joint target exceeds model joint limits")
        return q

    def validate(self, plan, retime=False):
        # Quintic segments are monotone with a known peak speed. For Hermite
        # segments check exact position/velocity extrema, independent of duration.
        ts = np.unique(plan.times)
        if plan._spline is not None:
            for derivative in (1, 2):
                roots = plan._spline.derivative(derivative).roots(extrapolate=False)
                ts = np.unique(np.concatenate([ts, *[r[np.isfinite(r)] for r in roots]]))
        peak = (float(np.max(1.875*np.abs(np.diff(plan.positions, axis=0)) /
                             np.diff(plan.times)[:, None] / self.velocity))
                if plan._spline is None else 0.0)
        for t in ts:
            q, dq = plan.sample(t)
            self.joints(q)
            peak = max(peak, float(np.max(np.abs(dq)/self.velocity)))
        if peak > 1.0001:
            if not retime:
                raise ValueError(f"trajectory exceeds joint velocity limits by {peak:.2f}x; increase duration")
            plan = plan.scale_time(peak*1.01)
        return plan

    def movej(self, start, end, duration=None):
        q = self.joints(end)
        required = max(.1, float(np.max(1.875*np.abs(q-start)/self.velocity)))
        duration = required if duration is None else duration
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive")
        return self.validate(Plan(np.array([0., duration]), np.array([start, q])))

    def trajectory(self, start, points, joint_names=None):
        if not points or len(points) > 10000:
            raise ValueError("trajectory requires 1..10000 points")
        order = list(range(len(self.model.names)))
        if joint_names is not None:
            if len(joint_names) != len(self.model.names) or set(joint_names) != set(self.model.names):
                raise ValueError("joint_names must contain each robot joint exactly once")
            order = [joint_names.index(n) for n in self.model.names]
        times, qs, vs = [], [], []
        has_velocity = any(p.get("velocities") is not None for p in points)
        if has_velocity and not all(p.get("velocities") is not None for p in points):
            raise ValueError("provide velocities at every point or omit them at every point")
        for p in points:
            times.append(float(p["time_from_start"]))
            qs.append(self.joints(vector(p["positions"], len(order), "positions")[order]))
            if has_velocity:
                vs.append(vector(p["velocities"], len(order), "velocities")[order])
        if not np.all(np.isfinite(times)) or times[0] < 0 or np.any(np.diff(times) <= 0):
            raise ValueError("time_from_start must be finite, nonnegative and strictly increasing")
        if times[0] == 0:
            if np.max(np.abs(qs[0]-start)) > 0.01:
                raise ValueError("time-zero point differs from current position; give it a positive arrival time")
            qs[0] = start
        else:
            times.insert(0, 0.)
            qs.insert(0, start)
            if has_velocity:
                vs.insert(0, np.zeros(len(order)))
        if len(times) < 2:
            raise ValueError("trajectory must have a positive duration")
        if has_velocity and np.max(np.abs(vs[-1])) > 1e-6:
            raise ValueError("final waypoint velocity must be zero for terminal hold")
        return self.validate(Plan(np.asarray(times), np.asarray(qs), np.asarray(vs) if has_velocity else None))

    def cartesian(self, start, end_pose, via_pose=None, duration=None, speed=0.03, angular_speed=0.3):
        if not math.isfinite(speed) or not math.isfinite(angular_speed) or speed <= 0 or angular_speed <= 0:
            raise ValueError("speed and angular_speed must be positive")
        begin, end = self.model.forward(start), pose_matrix(end_pose)
        if via_pose is None:
            distance = np.linalg.norm(end[:3, 3]-begin[:3, 3])
            position = lambda u: (1-u)*begin[:3, 3]+u*end[:3, 3]
            knots, transforms = [0., 1.], [begin, end]
        else:
            via = pose_matrix(via_pose)
            a, b = via[:3, 3]-begin[:3, 3], end[:3, 3]-begin[:3, 3]
            normal = np.cross(a, b)
            n2 = np.dot(normal, normal)
            if n2 < 1e-20:
                raise ValueError("movec needs three distinct non-collinear positions")
            center = begin[:3, 3]+(np.dot(a, a)*np.cross(b, normal)+np.dot(b, b)*np.cross(normal, a))/(2*n2)
            e1 = begin[:3, 3]-center
            radius = np.linalg.norm(e1)
            e1 /= radius
            e2 = np.cross(normal/np.sqrt(n2), e1)
            def angle(p):
                v = p-center
                return math.atan2(np.dot(v, e2), np.dot(v, e1)) % (2*math.pi)
            av, ae = angle(via[:3, 3]), angle(end[:3, 3])
            if av > ae:
                av -= 2*math.pi
                ae -= 2*math.pi
            fraction = av/ae
            distance = abs(ae)*radius
            position = lambda u: center+radius*(math.cos(ae*u)*e1+math.sin(ae*u)*e2)
            knots, transforms = [0., fraction, 1.], [begin, via, end]
        rotations = Rotation.from_matrix(np.array([T[:3, :3] for T in transforms]))
        orientation = (Slerp(knots, rotations) if via_pose is None
                       else RotationSpline(knots, rotations))
        angle = sum((rotations[i].inv()*rotations[i+1]).magnitude() for i in range(len(transforms)-1))
        angular_slope = max((rotations[i].inv()*rotations[i+1]).magnitude()/(knots[i+1]-knots[i])
                            for i in range(len(transforms)-1))
        if via_pose is not None:
            # Interpolating each rotation segment independently changes angular
            # velocity abruptly at the via pose. RotationSpline is C2 there.
            rotation_samples = np.concatenate([np.linspace(a, b, 257) for a, b in zip(knots[:-1], knots[1:])])
            angular_slope = float(np.max(np.linalg.norm(orientation(rotation_samples, 1), axis=1))) * 1.01
        auto = duration is None
        duration = max(.5, 1.875*distance/speed, 1.875*angular_slope/angular_speed) if auto else float(duration)
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive")
        count = max(21, int(distance/.001)+1, int(angle/.01)+1, int(duration/.02)+1)
        if count > 20000:
            raise ValueError("path too long; split into shorter paths")
        times = np.linspace(0, duration, count)
        qs, seed = [], np.asarray(start)
        for t in times:
            u = t/duration
            s = 10*u**3-15*u**4+6*u**5
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = orientation(s).as_matrix(), position(s)
            q = self.joints(self.model.inverse(T, seed))
            if np.max(np.abs(q-seed)) > .3:
                raise ValueError("Cartesian path crosses an IK branch discontinuity")
            qs.append(q)
            seed = q
        qs = np.asarray(qs)
        # Consistent derivatives from one clamped C2 spline prevent acceleration
        # jumps at every sampled IK waypoint. The resulting Hermite segments
        # exactly reproduce that spline, including zero endpoint velocity.
        zero = np.zeros(len(start))
        vs = CubicSpline(times, qs, bc_type=((1, zero), (1, zero)))(times, 1)
        return self.validate(Plan(times, qs, vs), retime=auto)
