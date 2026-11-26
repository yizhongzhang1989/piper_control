"""Shared URDF rigid-body model and bounded reference dynamics compensation."""
from __future__ import annotations

import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from piper_control.model import RobotModel, finite_vector


class BulletModel:
    """Private DIRECT world; inverse dynamics never steps it or accesses hardware."""

    def __init__(self, model: RobotModel):
        import pybullet as bullet

        self.model = model
        self._p = bullet
        self._client = bullet.connect(bullet.DIRECT)
        self._connected = True
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

    def dynamic_efforts(self, positions, velocities, accelerations):
        """M(q) qdd + C(q, qd); gravity is already supplied by the vendor driver."""
        q, dq, ddq = (np.zeros(len(self._movable)) for _ in range(3))
        for full, arm, name in ((q, positions, 'positions'), (dq, velocities, 'velocities'),
                                (ddq, accelerations, 'accelerations')):
            full[self._arm_slots] = finite_vector(arm, 6, name)
        zeros = np.zeros(len(q)).tolist()
        kwargs = dict(physicsClientId=self._client)
        effort = np.asarray(self._p.calculateInverseDynamics(
            self._body, q.tolist(), dq.tolist(), ddq.tolist(), **kwargs))
        gravity = np.asarray(self._p.calculateInverseDynamics(self._body, q.tolist(), zeros, zeros, **kwargs))
        return (effort-gravity)[self._arm_slots]

    def close(self):
        if self._connected:
            self._p.disconnect(physicsClientId=self._client)
            self._connected = False
        if self._temporary_urdf is not None:
            self._temporary_urdf.cleanup()
            self._temporary_urdf = None


class DynamicsFeedforward:
    """Filtered, bounded reference acceleration, with coupling on stationary axes.

    This supplements the existing gravity/friction model. A zero reference
    velocity clears the estimator and produces exactly zero effort, preserving
    the native driver's persistent position hold after an API restart.
    """

    def __init__(self, model, *, scale=.5, acceleration_cutoff_hz=20., max_acceleration=50.):
        self.scale = float(scale)
        self.cutoff = float(acceleration_cutoff_hz)
        self.max_acceleration = float(max_acceleration)
        if (not 0 <= self.scale <= 1 or not 1 <= self.cutoff <= 50
                or not 1 <= self.max_acceleration <= 100):
            raise ValueError('Invalid dynamics feedforward bounds')
        self._owns_model = not isinstance(model, BulletModel)
        self.body = BulletModel(model) if self._owns_model else model
        self.reset()

    def reset(self):
        self._velocity = np.zeros(6)
        self._acceleration = np.zeros(6)
        self.last = dict(torque=[0.]*6, acceleration=[0.]*6, scale=self.scale,
                         acceleration_limited=False)

    def evaluate(self, positions, velocity, dt):
        q = finite_vector(positions, 6, 'reference positions')
        v = finite_vector(velocity, 6, 'reference velocities')
        # Do not differentiate over scheduler stalls or retain a previous
        # trajectory's derivative history through a stop/watchdog boundary.
        if not math.isfinite(dt) or not .0001 <= dt <= .02:
            self.reset()
            self._velocity = v.copy()
            return np.zeros(6)
        if np.max(np.abs(v)) <= 1e-4 or self.scale == 0:
            self.reset()
            return np.zeros(6)
        raw = (v-self._velocity)/dt
        limited = np.clip(raw, -self.max_acceleration, self.max_acceleration)
        self._acceleration += -math.expm1(-2*math.pi*self.cutoff*dt)*(limited-self._acceleration)
        self._velocity = v.copy()
        effort = self.scale*self.body.dynamic_efforts(q, v, self._acceleration)
        if not np.isfinite(effort).all():
            self.reset()
            raise RuntimeError('Nonfinite dynamics feedforward')
        self.last = dict(torque=effort.tolist(), acceleration=self._acceleration.tolist(),
                         scale=self.scale, acceleration_limited=bool(np.any(raw != limited)))
        return effort

    def close(self):
        if self._owns_model:
            self.body.close()
