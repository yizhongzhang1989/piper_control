from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from piper_control.model import RobotModel, matrix_from_pose
from piper_control.backends.simulation import SimulationBackend


def test_fk_matches_reused_driver_and_jacobian(monkeypatch):
    upstream = Path(__file__).parents[1] / "vendor/Piper_workspace/src/piper_x_driver"
    if not upstream.exists():
        upstream = Path.home() / "Documents/Piper_workspace/src/piper_x_driver"
    monkeypatch.syspath_prepend(str(upstream))
    from piper_x_driver.gravity import forward_kinematics

    model = RobotModel()
    model._native = None  # Verify the independent standalone fallback.
    q = np.array([0.3, 0.9, -1.2, 0.2, -0.5, 0.4])
    assert np.allclose(model.forward(q), forward_kinematics(q, model.tcp_offset), atol=1e-12)
    assert np.allclose(model.frames(q)["tcp"], model.forward(q), atol=1e-12)
    numerical = np.zeros((6, 6))
    for index in range(6):
        delta = np.eye(6)[index] * 1e-6
        before, after = model.forward(q - delta), model.forward(q + delta)
        numerical[:3, index] = (after[:3, 3] - before[:3, 3]) / 2e-6
        numerical[3:, index] = Rotation.from_matrix(after[:3, :3] @ before[:3, :3].T).as_rotvec() / 2e-6
    assert np.allclose(model.jacobian(q), numerical, atol=1e-8)


def test_seeded_ik_and_unreachable_target():
    model = RobotModel()
    for q in ([0, 0.5, -0.5, 0, 0.2, 0], [0.6, 1.5, -1.8, 0.3, -0.6, 0.8]):
        q = np.array(q)
        target = model.forward(q)
        result = model.inverse(target, q + [0.03, -0.03, 0.03, -0.03, 0.03, -0.03])
        actual = model.forward(result)
        assert np.linalg.norm(actual[:3, 3] - target[:3, 3]) < 1e-5
        assert np.linalg.norm(Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec()) < 1e-4
        assert np.linalg.norm(result - q) < 0.01
    target[:3, 3] = [10, 10, 10]
    with pytest.raises(ValueError, match="unreachable"):
        model.inverse(target, q)
    with pytest.raises(ValueError, match="nonzero"):
        matrix_from_pose({"position": [0, 0, 0], "orientation": [0, 0, 0, 0]})


def test_physics_feedback_tracking_and_calibrated_frames():
    model = RobotModel(angle_offsets=[0, 0.025, 0.025, 0.025, -0.025, 0])
    sim = SimulationBackend(model)
    try:
        initial = np.asarray(sim.read()["positions"])
        sim.step(0.005)
        assert sim.read()["positions"] == initial.tolist()
        # Compare independently calculated physics world link frames to API FK,
        # including the rotated force-sensor flange and optional angle offsets.
        link = sim._arm[-1]
        state = sim._p.getLinkState(sim._body, link, computeForwardKinematics=True, physicsClientId=sim._client)
        transform = matrix_from_pose({"position": state[4], "orientation": state[5]})
        assert np.allclose(transform @ model.tool_transform, model.forward(initial), atol=1e-7)
        sim.enable(True)
        target = initial.copy()
        target[0] += 0.02
        sim.command(target)
        sim.step(0.005)
        assert 0 < sim.read()["positions"][0] < target[0]  # Integrated, not teleported.
        for _ in range(399):
            sim.step(0.005)
        assert np.max(np.abs(np.asarray(sim.read()["positions"]) - target)) < 1e-3
        assert np.max(np.abs(sim.read()["efforts"])) > 1.0  # Gravity load is modeled.
        sim.enable(False)
        assert np.max(np.abs(sim.read()["velocities"])) == 0
    finally:
        sim.close()


def test_arm_configuration_reuses_calibrated_loads_and_compensation():
    config = Path(__file__).parents[1] / "vendor/Piper_workspace/config/arms/arm_2.yaml"
    kp = [17, 16, 15, 16, 3, 3]
    kd = [5.99, 6.06, 3.16, 1.06, 0.30, 0.08]
    sim = SimulationBackend(arm_config=config, kp=kp, kd=kd)
    try:
        initial = np.asarray(sim.read()["positions"])
        sim.enable(True)
        sim.step(0.001)
        state = sim.read()
        calibration = state["simulation"]
        assert calibration["friction_scale"] == 0.88
        # Source arm_2 gravity fit at this known pose differs materially from
        # nominal URDF gravity; verify it is active in both plant and controller.
        assert calibration["gravity_compensation"][2] == pytest.approx(-8.3754762, abs=1e-6)
        assert np.allclose(state["positions"], initial, atol=1e-10)
        assert np.allclose(calibration["effective_kp"], np.array(kp) * 1.001)
        assert np.allclose(calibration["effective_kd"], np.array(kd) * np.sqrt(1.001))
        for _ in range(400):
            sim.step(0.005)
        assert np.allclose(sim.read()["simulation"]["effective_kp"], np.array(kp) * 3)
        sim.command(initial + [0.02, 0, 0, 0, 0, 0])
        sim.step(0.001)
        assert sim.read()["simulation"]["friction_compensation"][0] > 0.1
        assert np.array_equal(sim.model.angle_offsets, np.zeros(6))
    finally:
        sim.close()
    sim = SimulationBackend(arm_config=config, kp=kp, kd=kd, friction_scale=0)
    try:
        assert sim.config["friction_scale"] == 0.88  # The source profile remains intact.
        sim.enable(True)
        target = np.asarray(sim.read()["positions"]) + [0.02, 0, 0, 0, 0, 0]
        sim.command(target)
        for _ in range(20):
            sim.step(0.001)
        calibration = sim.read()["simulation"]
        assert calibration["friction_scale"] == 0
        assert np.array_equal(calibration["friction_compensation"], np.zeros(6))
        assert calibration["friction_load"][0] > 0.01
        assert abs(calibration["gravity_compensation"][2]) > 8
        assert calibration["effective_kp"][0] > kp[0]
    finally:
        sim.close()
    with pytest.raises(ValueError, match="kd must be positive"):
        SimulationBackend(kd=[5.99, 6.06, 3.16, 1.06, 0.30, 0])


@pytest.mark.parametrize("kp4,kd4", [(64, 2.12), (32, 0.53)])
def test_calibrated_friction_dissipates_small_motion_at_200hz(kp4, kd4):
    # Holding a physical friction load for the entire 5 ms controller tick
    # previously sustained a numerical limit cycle after this small motion.
    config = Path(__file__).parents[1] / "vendor/Piper_workspace/config/arms/arm_2.yaml"
    initial = np.array([-.0776497, .9208008, -.8501848, .7730412, .0713491, 1.1541339])
    sim = SimulationBackend(
        initial_positions=initial, arm_config=config, friction_scale=0,
        kp=[51, 64, 60, kp4, 3, 3], kd=[10.375, 12.12, 6.32, kd4, .3, .08],
    )
    try:
        sim.enable(True)
        target = initial.copy()
        target[3] += .001
        sim.command(target)
        settled = []
        peak_friction_load = 0.
        for index in range(1200):
            sim.step(.005)
            state = sim.read()
            peak_friction_load = max(peak_friction_load, abs(state["simulation"]["friction_load"][3]))
            if index >= 800:
                settled.append([state["positions"][3], state["velocities"][3]])
        settled = np.asarray(settled)
        assert peak_friction_load > .01  # Retain physical friction with compensation disabled.
        assert np.sqrt(np.mean(settled[:, 1] ** 2)) < .002
        assert np.ptp(settled[:, 0]) < 1e-4
        assert abs(target[3] - settled[-1, 0]) < 5e-4
    finally:
        sim.close()
