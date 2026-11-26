"""Headless rigid-body simulation using the same URDF as the real arm model."""

from __future__ import annotations

import math
from pathlib import Path
import time
import sys

import numpy as np
import yaml

from piper_control.model import RobotModel, finite_vector
from .position_integral import PositionIntegral
from .dynamics import BulletModel, DynamicsFeedforward


class SimulationBackend(BulletModel):
    """PyBullet DIRECT backend: actual integration and measured joint feedback.

    Uses upstream link masses, COMs, inertia tensors and gravity, with nominal
    MIT position/velocity gains and inverse-dynamics gravity feedforward. An
    optional arm config reuses upstream fitted gravity/friction as estimated
    loads and compensation. Motor quantization and CAN delays are not reproduced.
    Disabled simulation is paused (virtual brakes), making startup deterministic.
    """

    name = "simulation"

    def __init__(self, model: RobotModel | None = None, initial_positions=None, arm_config=None,
                 kp=None, kd=None, friction_scale=None, gravity_scale=None, reference_friction_scale=None,
                 moving_kd=None, gain_ramp_rate=2., position_integral=None, reference_friction_factors=None,
                 dynamics=None, reference_torque_limits=None):
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
        self._hold_kd = self.kd.copy()
        self._integral = PositionIntegral(**(position_integral or {}))
        self._tracking_active = True
        self._moving_kd = finite_vector(self.kd if moving_kd is None else moving_kd, 6, 'moving_kd')
        self._gain_rate = float(gain_ramp_rate)
        if (not math.isfinite(self._gain_rate) or not .1 <= self._gain_rate <= 5
                or np.any(self._moving_kd <= 0) or np.any(self._moving_kd > 100)
                or np.max(np.abs(self._moving_kd-self._hold_kd)) > 2*self._gain_rate):
            raise ValueError('Invalid motion/hold damping schedule')
        self._gravity_model = self._friction_model = None
        self._gravity_scale = float(self.config.get("gravity_scale", 1) if gravity_scale is None else gravity_scale)
        self._reference_friction_scale = reference_friction_scale
        self._reference_limits = (finite_vector(reference_torque_limits, 6, 'reference_torque_limits')
                                  if reference_torque_limits is not None else None)
        if self._reference_limits is not None and (np.any(self._reference_limits <= 0) or np.any(self._reference_limits > 4)):
            raise ValueError('reference torque limits must be within 0..4 Nm')
        self._reference_friction_factors = finite_vector(
            [1.]*6 if reference_friction_factors is None else reference_friction_factors, 6, 'reference_friction_factors')
        if np.any(self._reference_friction_factors < 0) or np.any(self._reference_friction_factors > 2):
            raise ValueError('reference friction factors must be within 0..2')
        if reference_friction_scale is not None and not 0 <= reference_friction_scale <= 1:
            raise ValueError('reference_friction_scale must be within 0..1')
        self._friction_scale = float(
            friction_scale if friction_scale is not None else self.config.get("friction_scale", 0)
        )
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
        self._plant_friction = np.zeros(6)
        self._effective_kp, self._effective_kd = self.kp.copy(), self.kd.copy()
        super().__init__(self.model)
        self._enabled = False
        self._last_feedback = time.monotonic()
        try:
            self._dynamics = DynamicsFeedforward(self, **dynamics) if dynamics is not None else None
        except Exception:
            super().close()
            raise
        self._dynamic_efforts = np.zeros(6)
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
            self.kd = self._hold_kd.copy()
            self._integral.bias = (0.,)*6
            if self._dynamics:
                self._dynamics.reset()
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

    def set_tracking_active(self, active):
        self._tracking_active = bool(active)
        if not active:
            self._target_velocity = np.zeros(6)

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
            self._dynamic_efforts = (self._dynamics.evaluate(self._target, self._target_velocity, dt)
                                     if self._dynamics else np.zeros(6))
            wanted_kd = self._hold_kd + (self._moving_kd-self._hold_kd)*np.tanh(np.abs(self._target_velocity)/.003)
            gain_step = self._gain_rate*min(dt, .02)
            self.kd += np.clip(wanted_kd-self.kd, -gain_step, gain_step)
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
            if self._tracking_active:
                self._integral.advance(self._target, arm_q, dt, self._target_velocity, arm_q)
            servo_target = np.asarray(self._integral.apply(self._target))
            error = servo_target - arm_q
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
            self._plant_friction = np.zeros(6)
            self._friction_efforts = np.zeros(6)
            if self._friction_model is not None:
                common = dict(gravity_torques=raw_gravity, static_frictions=self._static_frictions,
                              error_epsilon=self._error_epsilon, velocity_epsilon=self._velocity_epsilon,
                              active_impedances=active)
                # Breakaway terms driven by error/target velocity belong only
                # to the controller compensation, sampled once per outer tick.
                compensation = self._friction_model.torques(
                    arm_q, arm_dq, position_errors=error, target_velocities=self._target_velocity, **common
                )
                self._friction_efforts = self._friction_scale * np.asarray(compensation)
                if self._reference_friction_scale is not None:
                    desired = np.where(np.abs(self._target_velocity) > 1e-6, self._target_velocity, 0.)
                    reference = self._friction_model.torques(
                        arm_q, desired, target_velocities=desired, **common)
                    self._friction_efforts += (self._reference_friction_scale * self._reference_friction_factors
                                               * np.where(desired != 0, reference, 0.))
            reference_effort = self._friction_efforts + self._dynamic_efforts
            if self._reference_limits is not None:
                reference_effort = np.clip(reference_effort, -self._reference_limits, self._reference_limits)
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
                if self._friction_model is not None:
                    # Physical friction responds to current motion at every
                    # physics step; holding it for 5 ms can inject numerical energy.
                    self._plant_friction = np.asarray(self._friction_model.torques(arm_q, arm_dq, **common))
                error = servo_target - arm_q
                arm_torque = (self._effective_kp * error
                              + self._effective_kd * (self._target_velocity - arm_dq)
                              + self._gravity_efforts + reference_effort)
                self._efforts = np.clip(arm_torque, -self.model.effort, self.model.effort)
                # Joint-wise calibrated gravity scales cannot be represented by
                # a unique rigid-body mass model. Add their difference as an
                # explicit estimated plant load; retain the URDF mass matrix.
                torque[self._arm_slots] = self._efforts - self._plant_friction - gravity_disturbance
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
                "friction_scale": self._friction_scale,
                "reference_friction_scale": self._reference_friction_scale,
                "hold_kd": self._hold_kd.tolist(),
                "moving_kd": self._moving_kd.tolist(),
                "command_kd": self.kd.tolist(),
                "position_integral_bias": list(self._integral.bias),
                "position_integral_active": self._tracking_active,
                "position_integral_max_velocity": self._integral.max_velocity,
                "position_integral_motor_lead_limit": self._integral.motor_lead_limit,
                "position_integral_motor_lead_time": self._integral.motor_lead_time,
                "position_integral_motor_lead": self._integral.motor_lead,
                "position_integral_velocity_gated": self._tracking_active and self._integral.velocity_gated,
                "effective_kp": self._effective_kp.tolist(),
                "effective_kd": self._effective_kd.tolist(),
                "gravity_compensation": self._gravity_efforts.tolist(),
                "friction_compensation": self._friction_efforts.tolist(),
                "friction_load": self._plant_friction.tolist(),
                "dynamic_compensation": self._dynamic_efforts.tolist(),
            },
        }

    def close(self):
        if self._dynamics:
            self._dynamics.close()
        super().close()
