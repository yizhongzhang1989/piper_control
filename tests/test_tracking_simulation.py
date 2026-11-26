"""Candidate profile exercises rigid-body dynamics, without ROS or hardware I/O."""
import numpy as np

from piper_control.__main__ import load_control_profile
from piper_control.backends.simulation import SimulationBackend
from piper_control.model import RobotModel
from piper_control.planning import Planner, matrix_pose


def test_tracking_profile_movel_movec_and_stationary_damping():
    profile = load_control_profile('piper_control/assets/tcp_tracking.yaml')
    tracking = profile.pop('tracking')
    model = RobotModel()
    start = np.array([0., 1.53643623, -1.23343026, 1.35488233, 0., 0.])
    sim = SimulationBackend(model, initial_positions=start,
                            arm_config='vendor/Piper_workspace/config/arms/arm_2.yaml',
                            reference_friction_scale=tracking['friction_scale'],
                            reference_friction_factors=tracking.get('friction_factors'),
                            moving_kd=tracking['moving_kd'], gain_ramp_rate=tracking['gain_ramp_rate'],
                            position_integral=tracking.get('position_integral'), **profile)
    try:
        sim.enable()
        planner = Planner(model, .3)
        end = model.forward(start)
        end[2, 3] += .03
        line = planner.cartesian(start, matrix_pose(end), speed=.02)
        peak_damping = 0.
        for t in np.arange(0, line.duration+2, .005):
            q, dq = line.sample(t)
            sim.command(q, dq)
            sim.step(.005)
            peak_damping = max(peak_damping, sim.kd[3])
        assert peak_damping > 1.5
        assert np.max(np.abs(np.array(sim.read()['positions'])-q)) < .01
        assert abs(sim.kd[3]-.53) < 1e-10
        assert np.max(np.abs(sim.read()['simulation']['friction_compensation'])) == 0
        via, end = model.forward(q), model.forward(q)
        via[:2, 3] += [.01, .01]
        end[0, 3] += .02
        arc = planner.cartesian(q, matrix_pose(end), matrix_pose(via), speed=.02)
        for t in np.arange(0, arc.duration+2, .005):
            q, dq = arc.sample(t)
            sim.command(q, dq)
            sim.step(.005)
        assert np.max(np.abs(np.array(sim.read()['positions'])-q)) < .01
        assert np.max(np.abs(sim.read()['velocities'])) < .03
    finally:
        sim.close()
