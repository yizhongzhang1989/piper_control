"""Headless rigid-body simulation using the same URDF as the real arm model."""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from piper_control.model import RobotModel, finite_vector


class SimulationBackend:
    """PyBullet DIRECT backend: actual integration and measured joint feedback.

    Uses upstream link masses, COMs, inertia tensors and gravity, with nominal
    MIT position/velocity gains and inverse-dynamics gravity feedforward. An
    optional arm config reuses upstream fitted gravity/friction as estimated
    loads and compensation. Motor quantization and CAN delays are not reproduced.
    Disabled simulation is paused (virtual brakes), making startup deterministic.
    """

    name = "simulation"

    def __init__(self, model: RobotModel | None = None, initial_positions=None, arm_config=None,
                 kp=None, kd=None):
        import pybullet as bullet

        self._p = bullet
        self.config_path = Path(arm_config).expanduser().resolve() if arm_config else None
        self.config = yaml.safe_load(self.config_path.read_text()) if self.config_path else {}
        self.model = model or RobotModel(end_effector=self.config.get("end_effector", "gripper_with_wrench"))
        if self.config.get("end_effector", self.model.end_effector) != self.model.end_effector:
            raise ValueError("simulation arm config and model end effectors must match")
        self.kp = finite_vector(kp if kp is not None else self.config.get("kp", [17, 16, 15, 4, 3, 3]), 6, "kp")
        self.kd = finite_vector(kd if kd is not None else self.config.get("kd", [5.99, 6.06, 3.16, 0.53, 0.30, 0.08]), 6, "kd")
        if np.any(self.kp < 0) or np.any(self.kd <= 0):
            raise ValueError("kp must be nonnegative and kd must be positive")
        self._gravity_model = self._friction_model = None
        self._gravity_scale = float(self.config.get("gravity_scale", 1))
        self._friction_scale = float(self.config.get("friction_scale", 0))
        self._adaptive_amount = float(self.config.get("s_a", 2))
        self._adaptive_b = float(self.config.get("adaptive_stiffness_b", 0.08))
        self._adaptive_power = float(self.config.get("adaptive_stiffness_power", 3))
        self._offset_ramp_s = float(self.config.get("offset_ramp_s", 2))
        self._adaptive_applied = 0.0 if self._offset_ramp_s > 0 else self._adaptive_amount
        self._error_epsilon = float(self.config.get("friction_error_epsilon", 0.05))
        self._velocity_epsilon = float(self.config.get("friction_velocity_epsilon", 0.02))
        if (not all(math.isfinite(v) for v in [self._gravity_scale, self._friction_scale,
                self._adaptive_amount, self._adaptive_b, self._adaptive_power, self._offset_ramp_s,
                self._error_epsilon, self._velocity_epsilon])
                or not 0 <= self._gravity_scale <= 1 or not 0 <= self._friction_scale <= 1
                or self._adaptive_amount < 0 or self._adaptive_b <= 0 or self._adaptive_power <= 0 or self._offset_ramp_s < 0
                or self._error_epsilon < 0 or self._velocity_epsilon < 0):
            raise ValueError("invalid upstream compensation parameters")
        if self.config.get("gravity_config") or self.config.get("friction_config"):
            self._load_calibration()
        self._static_frictions = [
            self._gravity_model.joint(i).static_friction if self._gravity_model else 0
            for i in range(1, 7)
        ]
        self._gravity_efforts = np.zeros(6)
        self._friction_efforts = np.zeros(6)
        self._effective_kp, self._effective_kd = self.kp.copy(), self.kd.copy()
        self._client = bullet.connect(bullet.DIRECT)
        self._connected = True
        self._enabled = False
        self._last_feedback = time.monotonic()
        self._temporary_urdf = None
        bullet.setGravity(0, 0, -9.81, physicsClientId=self._client)
        bullet.setPhysicsEngineParameter(
            fixedTimeStep=0.001, numSolverIterations=80, physicsClientId=self._client
        )
        urdf_path = self.model.urdf_path
        if np.any(self.model.angle_offsets):
            root = ET.parse(urdf_path).getroot()
            for name, offset in zip(self.model.names, self.model.angle_offsets):
                origin = root.find(f"joint[@name='{name}']/origin")
                rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
                rotation = rotation * Rotation.from_rotvec([0, 0, offset])
                origin.set("rpy", " ".join(map(str, rotation.as_euler("xyz"))))
            for mesh in root.findall(".//mesh"):
                mesh.set("filename", str(self.model.asset_dir / mesh.get("filename")))
            self._temporary_urdf = tempfile.TemporaryDirectory(prefix="piper-simulation-")
            urdf_path = Path(self._temporary_urdf.name) / "model.urdf"
            ET.ElementTree(root).write(urdf_path, encoding="utf-8", xml_declaration=True)
        self._body = bullet.loadURDF(
            str(urdf_path), useFixedBase=True,
            flags=bullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._client,
        )
        self._joint_indices = {}
        self._movable = []
        for index in range(bullet.getNumJoints(self._body, physicsClientId=self._client)):
            info = bullet.getJointInfo(self._body, index, physicsClientId=self._client)
            self._joint_indices[info[1].decode()] = index
            if info[2] != bullet.JOINT_FIXED:
                self._movable.append(index)
            bullet.changeDynamics(
                self._body, index, linearDamping=0, angularDamping=0, jointDamping=0,
                physicsClientId=self._client,
            )
        self._arm = [self._joint_indices[name] for name in self.model.names]
        self._arm_slots = [self._movable.index(index) for index in self._arm]
        self._finger_slots = [slot for slot, index in enumerate(self._movable) if index not in self._arm]
        q = self.model.validate_joints(
            initial_positions if initial_positions is not None else [0, 0.5, -0.5, 0, 0.2, 0]
        )
        for index, value in zip(self._arm, q):
            bullet.resetJointState(self._body, index, float(value), physicsClientId=self._client)
        self._target = q.copy()
        self._target_velocity = np.zeros(6)
        self._efforts = np.zeros(6)
        bullet.setJointMotorControlArray(
            self._body, self._movable, bullet.VELOCITY_CONTROL,
            forces=[0.0] * len(self._movable), physicsClientId=self._client,
        )

    def _load_calibration(self):
        """Import the original pure Python equations, without launching ROS/CAN."""
        try:
            from piper_x_driver.gravity import load_gravity_model
            from piper_x_driver.friction import load_friction_model
        except ImportError:
            candidates = [
                self.config_path.parents[2] / "src" / "piper_x_driver",
                Path(__file__).parents[2] / "vendor/Piper_workspace/src/piper_x_driver",
            ]
            source = next((path for path in candidates if (path / "piper_x_driver").is_dir()), None)
            if source is None:
                raise RuntimeError(
                    "calibrated simulation needs the original piper_x_driver Python package; "
                    "use the repository checkout or source its workspace"
                )
            sys.path.insert(0, str(source))
            from piper_x_driver.gravity import load_gravity_model
            from piper_x_driver.friction import load_friction_model
        if self.config.get("gravity_config"):
            path = self.config_path.parent / self.config["gravity_config"]
            self._gravity_model = load_gravity_model(path).for_effector(self.model.end_effector)
        if self.config.get("friction_config"):
            path = self.config_path.parent / self.config["friction_config"]
            self._friction_model = load_friction_model(path)

    def enable(self, enabled: bool = True):
        if not self._connected:
            raise RuntimeError("simulation is closed")
        if enabled and not self._enabled:
            self._target = np.asarray(self.read()["positions"])
            self._target_velocity = np.zeros(6)
            self._adaptive_applied = 0.0 if self._offset_ramp_s > 0 else self._adaptive_amount
        if not enabled:
            # Disabled simulation is paused; discard stored kinetic energy too.
            states = self._p.getJointStates(self._body, self._movable, physicsClientId=self._client)
            for index, state in zip(self._movable, states):
                self._p.resetJointState(self._body, index, state[0], 0, physicsClientId=self._client)
            self._efforts[:] = 0
        self._enabled = bool(enabled)

    def command(self, positions, velocities=None):
        if not self._enabled:
            raise RuntimeError("robot is disabled")
        self._target = self.model.validate_joints(positions).copy()
        self._target_velocity = finite_vector(
            velocities if velocities is not None else np.zeros(6), 6, "velocities"
        ).copy()

    def step(self, dt: float):
        if not self._connected:
            return
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("simulation step must be positive and finite")
        if self._enabled:
            # The source compensation loop runs at the outer control rate. Hold
            # its sampled gains/feedforward over the faster motor/physics steps.
            count = max(1, math.ceil(dt / 0.001))
            self._p.setTimeStep(dt / count, physicsClientId=self._client)
            states = self._p.getJointStates(self._body, self._movable, physicsClientId=self._client)
            q = np.array([state[0] for state in states])
            dq = np.array([state[1] for state in states])
            zeros = [0.0] * len(q)
            gravity = np.array(self._p.calculateInverseDynamics(
                self._body, q.tolist(), zeros, zeros, physicsClientId=self._client
            ))
            arm_q, arm_dq = q[self._arm_slots], dq[self._arm_slots]
            error = self._target - arm_q
            if self._offset_ramp_s > 0:
                self._adaptive_applied = min(
                    self._adaptive_amount,
                    self._adaptive_applied + dt * max(1.0, self._adaptive_amount) / self._offset_ramp_s,
                )
            multiplier = 1 + self._adaptive_applied / (1 + (np.abs(error) / self._adaptive_b) ** self._adaptive_power)
            active = (self.kp > 0) & (self.kd > 0)
            self._effective_kp = np.where(active, np.minimum(500, self.kp * multiplier), self.kp)
            gain_ratio = np.divide(self._effective_kp, self.kp, out=np.ones(6), where=self.kp > 0)
            self._effective_kd = np.where(active, np.minimum(100, self.kd * np.sqrt(gain_ratio)), self.kd)
            raw_gravity = (np.asarray(self._gravity_model.torques(arm_q))
                           if self._gravity_model else gravity[self._arm_slots])
            gravity_disturbance = raw_gravity - gravity[self._arm_slots]
            self._gravity_efforts = self._gravity_scale * raw_gravity
            plant_friction = np.zeros(6)
            self._friction_efforts = np.zeros(6)
            if self._friction_model is not None:
                common = dict(gravity_torques=raw_gravity, static_frictions=self._static_frictions,
                              error_epsilon=self._error_epsilon, velocity_epsilon=self._velocity_epsilon,
                              active_impedances=active)
                # The fitted moving-friction curve is an estimated plant load.
                # Breakaway terms driven by error/target velocity belong only
                # to compensation. Both sampled estimates update once per tick.
                plant_friction = np.asarray(self._friction_model.torques(arm_q, arm_dq, **common))
                compensation = self._friction_model.torques(
                    arm_q, arm_dq, position_errors=error, target_velocities=self._target_velocity, **common
                )
                self._friction_efforts = self._friction_scale * np.asarray(compensation)
            for index in range(count):
                if index:
                    states = self._p.getJointStates(self._body, self._movable, physicsClientId=self._client)
                    q = np.array([state[0] for state in states])
                    dq = np.array([state[1] for state in states])
                    gravity = np.array(self._p.calculateInverseDynamics(
                        self._body, q.tolist(), zeros, zeros, physicsClientId=self._client
                    ))
                torque = gravity.copy()
                arm_q, arm_dq = q[self._arm_slots], dq[self._arm_slots]
                error = self._target - arm_q
                arm_torque = (self._effective_kp * error
                              + self._effective_kd * (self._target_velocity - arm_dq)
                              + self._gravity_efforts + self._friction_efforts)
                self._efforts = np.clip(arm_torque, -self.model.effort, self.model.effort)
                # Joint-wise calibrated gravity scales cannot be represented by
                # a unique rigid-body mass model. Add their difference as an
                # explicit estimated plant load; retain the URDF mass matrix.
                torque[self._arm_slots] = self._efforts - plant_friction - gravity_disturbance
                for slot in self._finger_slots:
                    torque[slot] += -100 * q[slot] - 2 * dq[slot]
                    torque[slot] = np.clip(torque[slot], -10, 10)
                self._p.setJointMotorControlArray(
                    self._body, self._movable, self._p.TORQUE_CONTROL,
                    forces=torque.tolist(), physicsClientId=self._client,
                )
                self._p.stepSimulation(physicsClientId=self._client)
        self._last_feedback = time.monotonic()

    def read(self) -> dict:
        if not self._connected:
            return {"connected": False, "enabled": False, "errors": ["simulation closed"]}
        states = self._p.getJointStates(self._body, self._arm, physicsClientId=self._client)
        return {
            "positions": [state[0] for state in states],
            "velocities": [state[1] for state in states],
            "efforts": self._efforts.tolist(),
            "enabled": self._enabled,
            "connected": True,
            "feedback_age": time.monotonic() - self._last_feedback,
            "errors": [],
            "simulation": {
                "engine": "PyBullet DIRECT",
                "dynamics": "URDF inertia and gravity with torque-controlled joints",
                "disabled_behavior": "paused with zero velocity",
                "motor_model": "MIT kp/kd with optional upstream fitted compensation; no CAN latency or quantization",
                "arm_config": str(self.config_path) if self.config_path else None,
                "gravity_model": "upstream calibrated joint-load estimate" if self._gravity_model else "URDF",
                "friction_model": "upstream fitted moving-friction load and compensation" if self._friction_model else "none",
                "effective_kp": self._effective_kp.tolist(),
                "effective_kd": self._effective_kd.tolist(),
                "gravity_compensation": self._gravity_efforts.tolist(),
                "friction_compensation": self._friction_efforts.tolist(),
            },
        }

    def close(self):
        if self._connected:
            self._p.disconnect(physicsClientId=self._client)
            self._connected = False
        if self._temporary_urdf is not None:
            self._temporary_urdf.cleanup()
            self._temporary_urdf = None
