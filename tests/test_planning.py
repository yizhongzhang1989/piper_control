import numpy as np
import pytest

from piper_control.model import RobotModel, pose_from_matrix
from piper_control.planning import Planner


def test_cartesian_line_and_circle_follow_task_space_geometry():
    model = RobotModel()
    planner = Planner(model)
    start = np.array([0.3, 0.9, -1.2, 0.2, -0.5, 0.4])
    begin = model.forward(start)
    via, end = begin.copy(), begin.copy()
    via[:3, 3] += [0.002, 0.002, 0]
    end[:3, 3] += [0.004, 0, 0]
    circle = planner.cartesian(start, pose_from_matrix(end), pose_from_matrix(via), duration=2)
    center = begin[:3, 3] + [0.002, 0, 0]
    for t in np.linspace(0, circle.duration, 43):
        transform = model.forward(circle.sample(t)[0])
        assert abs(np.linalg.norm(transform[:3, 3] - center) - 0.002) < 2e-5
        assert abs(transform[2, 3] - begin[2, 3]) < 2e-5
        assert np.allclose(transform[:3, :3], begin[:3, :3], atol=1e-4)
    assert np.linalg.norm(model.forward(circle.sample(1)[0])[:3, 3] - via[:3, 3]) < 2e-5
    line = planner.cartesian(start, pose_from_matrix(end), duration=2)
    for t in np.linspace(0, line.duration, 17):
        point = model.forward(line.sample(t)[0])[:3, 3]
        assert np.linalg.norm(point[1:] - begin[1:3, 3]) < 2e-5
    with pytest.raises(ValueError, match="non-collinear"):
        planner.cartesian(start, pose_from_matrix(end), pose_from_matrix(begin))


def test_named_trajectory_preserves_order_and_checks_interpolation_extrema():
    model = RobotModel()
    planner = Planner(model)
    start = np.array([0, 0.5, -0.5, 0, 0.2, 0])
    end = start + [0.1, 0.02, -0.03, 0.01, 0.02, -0.02]
    reversed_names = model.names[::-1]
    plan = planner.trajectory(start, [{"positions": end[::-1].tolist(), "time_from_start": 2}], reversed_names)
    assert np.allclose(plan.sample(2)[0], end)
    with pytest.raises(ValueError, match="velocity"):
        planner.trajectory(start, [{"positions": end.tolist(), "time_from_start": 0.1}])
    with pytest.raises(ValueError, match="6 finite"):
        planner.trajectory(start, [{"positions": [*end, 0], "time_from_start": 2}])
    # Both waypoints and velocities satisfy limits, but Hermite interpolation
    # overshoots joint1's upper bound between them. Validation must catch it.
    at_limit = start.copy()
    at_limit[0] = model.upper[0] - 0.01
    with pytest.raises(ValueError, match="joint limits"):
        planner.trajectory(at_limit, [
            {"positions": at_limit.tolist(), "velocities": [0.2, 0, 0, 0, 0, 0], "time_from_start": 0},
            {"positions": at_limit.tolist(), "velocities": [0, 0, 0, 0, 0, 0], "time_from_start": 1},
        ])
