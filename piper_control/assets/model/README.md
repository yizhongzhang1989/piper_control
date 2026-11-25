Model assets are derived from `Piper_workspace/src/piper_description` (MIT;
see LICENSE). The driver arm chain, joint limits, meshes, masses, centers of mass
and inertia tensors are reused. The underlying arm parameters originate from
agilexrobotics/agx_arm_urdf commit 680fa925cd00db7e2628563d2edc5540963045de.

Only package mesh URLs were changed to relative paths. For gripper_with_wrench,
an extra fixed `driver_flange_to_visual` joint separates the upstream driver's
original `link6` reference from the force-sensor URDF's rotated flange frame.
This preserves the force-sensor assembly geometry while making FK, tool offsets,
simulation and dashboard use the same flange coordinates. The arm uses the
driver's no-gripper URDF limits, including ±170 degrees for joint6.

`model.urdf` is the default gripper_with_wrench model. The server should serve the
selected RobotModel.urdf_path at `/robot/model.urdf` and the meshes folder beneath
`/robot/meshes`. API TCP defaults to 0.16 m along the driver link6 z-axis; it is a
configurable tool reference, not a measured tool-tip calibration.

Simulation integrates these rigid bodies under gravity. Nominal gains match the
arm_2 MIT configuration. Optional arm configuration reuses fitted gravity and
friction as estimated joint loads and compensation; see docs/simulation.md.
It does not reproduce motor quantization, firmware timing or contact materials. Optional
angle_offsets update both kinematics and simulation joint origins; their default
is zero because gravity-fit offsets are not independent geometric calibration.
