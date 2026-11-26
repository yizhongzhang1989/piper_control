"""Check physical path derivatives, including near-stop IK and the circle via pose."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from piper_control.model import RobotModel, pose_from_matrix
from piper_control.planning import Planner


@pytest.mark.parametrize("circle", [False, True])
def test_cartesian_path_has_continuous_acceleration_and_no_ik_speed_pulses(circle):
    model = RobotModel()
    planner = Planner(model, np.pi)
    start = np.array([0.3, 0.9, -1.2, 0.2, -0.5, 0.4])
    begin = model.forward(start)
    end, via = begin.copy(), begin.copy()
    end[:3, 3] += [.03, 0, 0]
    via[:3, 3] += [.015, .015, 0]
    plan = planner.cartesian(start, pose_from_matrix(end),
                             pose_from_matrix(via) if circle else None, speed=.02)
    times = np.linspace(0, plan.duration, 1601)
    velocities = np.array([model.jacobian(q)[:3] @ v for q, v in map(plan.sample, times)])
    length = np.pi * .015 if circle else .03
    u = times / plan.duration
    expected_speed = length * 30 * u**2 * (1-u)**2 / plan.duration
    speed = np.linalg.norm(velocities, axis=1)
    # The former coarse IK convergence threshold generated stops and pulses
    # even though the path's geometric error was within ten micrometres.
    assert np.max(np.abs(speed-expected_speed)) < 2e-6
    assert np.max(np.abs(np.gradient(np.gradient(speed, times), times))) < .2
    acceleration = plan._spline.derivative(2)
    knots = plan.times[1:-1]
    assert np.max(np.abs(acceleration(knots-1e-9)-acceleration(knots+1e-9))) < 1e-5
    assert np.max(np.abs(plan.sample(0)[1])) < 1e-10
    assert np.max(np.abs(plan.sample(plan.duration)[1])) < 1e-10


def test_circle_orientation_is_smooth_through_a_noncollinear_via_rotation():
    model = RobotModel()
    planner = Planner(model, np.pi)
    start = np.array([0.3, .9, -1.2, .2, -.5, .4])
    begin = model.forward(start)
    via, end = begin.copy(), begin.copy()
    via[:3, 3] += [.01, .01, 0]
    end[:3, 3] += [.02, 0, 0]
    via[:3, :3] = Rotation.from_rotvec([.08, 0, 0]).as_matrix() @ begin[:3, :3]
    end[:3, :3] = Rotation.from_rotvec([0, .08, 0]).as_matrix() @ begin[:3, :3]
    plan = planner.cartesian(start, pose_from_matrix(end), pose_from_matrix(via), duration=3)
    mid = model.forward(plan.sample(1.5)[0])
    assert np.linalg.norm(mid[:3, 3]-via[:3, 3]) < 2e-7
    assert np.linalg.norm(Rotation.from_matrix(mid[:3, :3] @ via[:3, :3].T).as_rotvec()) < 2e-6
    times = np.linspace(0, 3, 1501)
    omega = np.array([model.jacobian(q)[3:] @ v for q, v in map(plan.sample, times)])
    assert np.max(np.linalg.norm(np.gradient(omega, times, axis=0), axis=1)) < 1.
