"""PiPER geometry shared by hardware, simulation, planning and the dashboard.

Angles are radians, translations metres, quaternions xyzw, poses base_link -> TCP.
The upstream driver's original link6 frame remains the tool reference even when
the physical force-sensor assembly has a rotated visual flange frame.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


ASSET_DIR = Path(__file__).parent / "assets" / "model"


def finite_vector(value: Sequence[float], size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def matrix_from_pose(pose: Mapping | Sequence[float] | np.ndarray) -> np.ndarray:
    """Accept a generic pose object or a homogeneous matrix; reject invalid poses."""
    if isinstance(pose, Mapping):
        position = finite_vector(pose["position"], 3, "position")
        quaternion = finite_vector(pose["orientation"], 4, "orientation xyzw")
        if np.linalg.norm(quaternion) < 1e-12:
            raise ValueError("orientation quaternion must have nonzero norm")
        transform = np.eye(4)
        transform[:3, 3] = position
        transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        return transform
    transform = np.asarray(pose, dtype=float)
    if (
        transform.shape != (4, 4)
        or not np.isfinite(transform).all()
        or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
        or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-6)
    ):
        raise ValueError("pose must be a rigid 4x4 transform or position/orientation object")
    return transform.copy()


def pose_from_matrix(transform: np.ndarray) -> dict:
    return {
        "position": transform[:3, 3].tolist(),
        "orientation": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
    }


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        transform[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return transform


class RobotModel:
    """URDF geometry with seeded, bounded six-axis inverse kinematics.

    An installed upstream gravity module supplies its existing fast FK/IK when
    available. The packaged URDF and SciPy fallback also work without ROS or the
    source workspace. Optional angle_offsets are explicit calibration values.
    """

    def __init__(
        self,
        tcp_offset: Sequence[float] = (0.0, 0.0, 0.16),
        end_effector: str = "gripper_with_wrench",
        angle_offsets: Sequence[float] | None = None,
        tcp_orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ):
        self.asset_dir = ASSET_DIR
        self.urdf_path = ASSET_DIR / f"{end_effector}.urdf"
        if not self.urdf_path.is_file():
            raise ValueError(f"unknown end effector: {end_effector}")
        self.end_effector = end_effector
        self.tcp_offset = finite_vector(tcp_offset, 3, "tcp_offset")
        self.tool_transform = matrix_from_pose(
            {"position": self.tcp_offset, "orientation": tcp_orientation}
        )
        self.angle_offsets = finite_vector(
            angle_offsets if angle_offsets is not None else np.zeros(6), 6, "angle_offsets"
        )
        self.names = [f"joint{i}" for i in range(1, 7)]
        self.root = ET.parse(self.urdf_path).getroot()
        self._joints = {joint.get("name"): joint for joint in self.root.findall("joint")}
        self._origins = [_origin(self._joints[name].find("origin")) for name in self.names]
        limits = [self._joints[name].find("limit") for name in self.names]
        self.lower = np.array([float(limit.get("lower")) for limit in limits])
        self.upper = np.array([float(limit.get("upper")) for limit in limits])
        self.velocity = np.array([float(limit.get("velocity")) for limit in limits])
        self.effort = np.array([float(limit.get("effort")) for limit in limits])
        try:
            from piper_x_driver import gravity
        except ImportError:
            gravity = None
        self._native = gravity

    def validate_joints(self, positions: Sequence[float]) -> np.ndarray:
        q = finite_vector(positions, 6, "positions")
        if np.any(q < self.lower - 1e-8) or np.any(q > self.upper + 1e-8):
            raise ValueError("joint positions exceed the upstream URDF joint limits")
        return q

    def _chain(self, positions: Sequence[float]):
        q = finite_vector(positions, 6, "positions") + self.angle_offsets
        transform = np.eye(4)
        origins, axes = [], []
        for origin, angle in zip(self._origins, q):
            transform = transform @ origin
            origins.append(transform[:3, 3].copy())
            axes.append(transform[:3, 2].copy())
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            transform[:3, :3] = transform[:3, :3] @ rotation
        return transform @ self.tool_transform, np.array(origins), np.array(axes)

    def forward(self, positions: Sequence[float]) -> np.ndarray:
        q = finite_vector(positions, 6, "positions")
        if self._native is not None:
            transform = self._native.forward_kinematics(q + self.angle_offsets, self.tcp_offset)
            transform[:3, :3] = transform[:3, :3] @ self.tool_transform[:3, :3]
            return transform
        return self._chain(q)[0]

    def pose(self, positions: Sequence[float]) -> dict:
        return pose_from_matrix(self.forward(positions))

    def jacobian(self, positions: Sequence[float]) -> np.ndarray:
        transform, origins, axes = self._chain(positions)
        return np.vstack((np.cross(axes, transform[:3, 3] - origins).T, axes.T))

    def inverse(self, target, seed: Sequence[float]) -> np.ndarray:
        """Return the nearest seeded solution; never return an unconverged result."""
        target = matrix_from_pose(target)
        seed = np.clip(finite_vector(seed, 6, "seed"), self.lower, self.upper)
        if self._native is not None:
            native_target = target.copy()
            native_target[:3, :3] = target[:3, :3] @ self.tool_transform[:3, :3].T
            result = self._native.inverse_kinematics(
                native_target,
                q_seed=seed + self.angle_offsets,
                tool_offset_m=self.tcp_offset,
                lower_limits=self.lower + self.angle_offsets,
                upper_limits=self.upper + self.angle_offsets,
                pos_tol_m=1e-5,
                rot_tol_rad=1e-4,
                max_iters=80,
                return_result=True,
            )
            if result.converged:
                return np.asarray(result.joints) - self.angle_offsets

        def error(q):
            current = self.forward(q)
            return np.r_[
                target[:3, 3] - current[:3, 3],
                Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec(),
            ]

        # Near-seed damped Newton solves are inexpensive enough for stream targets.
        q = seed.copy()
        best_error = float("inf")
        for _ in range(35):
            residual = error(q)
            if np.linalg.norm(residual[:3]) < 1e-5 and np.linalg.norm(residual[3:]) < 1e-4:
                return q
            magnitude = np.linalg.norm(residual)
            if magnitude >= best_error * 1.001:
                break
            best_error = magnitude
            jacobian = self.jacobian(q)
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-7 * np.eye(6), residual
            )
            delta *= min(1.0, 0.3 / max(np.max(np.abs(delta)), 1e-12))
            updated = np.clip(q + delta, self.lower, self.upper)
            if np.max(np.abs(updated - q)) < 1e-10:
                break
            q = updated

        # Bounded optimization handles limits and more distant Cartesian targets.
        result = least_squares(
            error, q, bounds=(self.lower, self.upper), max_nfev=160,
            ftol=1e-10, xtol=1e-10, gtol=1e-10,
        )
        residual = error(result.x)
        if np.linalg.norm(residual[:3]) < 1e-5 and np.linalg.norm(residual[3:]) < 1e-4:
            return result.x
        raise ValueError(
            "TCP target is unreachable from the supplied joint seed within URDF limits "
            f"(position error {np.linalg.norm(residual[:3]):.6f} m, "
            f"orientation error {np.linalg.norm(residual[3:]):.6f} rad)"
        )

    def frames(self, positions: Sequence[float], gripper_width: float = 0.0) -> dict:
        values = dict(zip(self.names, finite_vector(positions, 6, "positions") + self.angle_offsets))
        values.update(joint7=gripper_width / 2, joint8=-gripper_width / 2)
        frames = {"base_link": np.eye(4)}
        pending = list(self._joints.values())
        while pending:
            remaining = []
            for joint in pending:
                parent = joint.find("parent").get("link")
                if parent not in frames:
                    remaining.append(joint)
                    continue
                transform = frames[parent] @ _origin(joint.find("origin"))
                amount = values.get(joint.get("name"), 0.0)
                axis_element = joint.find("axis")
                axis = np.fromstring(axis_element.get("xyz", "0 0 1"), sep=" ") if axis_element is not None else np.array([0, 0, 1])
                if joint.get("type") == "revolute":
                    transform[:3, :3] = transform[:3, :3] @ Rotation.from_rotvec(axis * amount).as_matrix()
                elif joint.get("type") == "prismatic":
                    transform[:3, 3] += transform[:3, :3] @ (axis * amount)
                frames[joint.find("child").get("link")] = transform
            if len(remaining) == len(pending):
                raise ValueError("URDF contains a disconnected joint tree")
            pending = remaining
        frames["tcp"] = frames["link6"] @ self.tool_transform
        return frames

    def metadata(self) -> dict:
        return {
            "name": "AgileX PiPER X",
            "dof": 6,
            "joint_names": self.names,
            "joint_limits": [
                {"lower": float(lo), "upper": float(hi), "velocity": float(v), "effort": float(e)}
                for lo, hi, v, e in zip(self.lower, self.upper, self.velocity, self.effort)
            ],
            "base_frame": "base_link",
            "tcp_frame": "tcp",
            "tcp_offset": self.tcp_offset.tolist(),
            "tcp_orientation": Rotation.from_matrix(self.tool_transform[:3, :3]).as_quat().tolist(),
            "angle_offsets": self.angle_offsets.tolist(),
            "end_effector": self.end_effector,
            "model_url": "/robot/model.urdf",
            "kinematics": "upstream_piper_x_driver" if self._native is not None else "packaged_urdf",
        }
