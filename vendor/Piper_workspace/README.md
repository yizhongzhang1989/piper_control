# Reused Piper X packages

Source: <https://github.com/madderscientist/Piper_workspace>, commit
`b206698c25e946efc22a47c8d62edd8f2e01d781` (local snapshot 2026-11-25).

Only the existing driver, its interfaces, robot description, gravity calibration,
and friction calibration packages are included. Cameras, insertion tasks,
teaching controllers, and the original dashboard are not included.
`SHA256SUMS` records the original upstream snapshot; `SHA256SUMS.patched` records
the shipped version. The sole source change is documented below. The original
`~/Documents/Piper_workspace` remains untouched.

`config/arms/arm_2.yaml` and the matching gravity/friction calibration files are
the actual local can1 arm configuration. Calibration is specific to this arm and
end-effector assembly; configure another physical arm with its own parameters.
The upstream calibration commands remain available after building these packages.

Licensing follows each package's `package.xml`: driver Apache-2.0/MIT, interfaces
Apache-2.0, description MIT, calibration packages Apache-2.0. Upstream mesh license
is retained at `src/piper_description/LICENSE`; the dynamics model MIT notice is
retained in `src/piper_x_driver/piper_x_driver/gravity.py`. Model provenance:

* piper_ros humble `017ffefa64511bc6325bd77ddc4e16065c152051` (RosenYin, MIT).
* agx_arm_urdf `680fa925cd00db7e2628563d2edc5540963045de` (aalicecc, MIT).

Build with `../../scripts/build_vendor.sh`. The normal hardware launch can also
use the original `~/Documents/Piper_workspace` installation, preserving its
calibration and compiled 200 Hz driver.

## Optional persistent position hold patch

`patches/position_hold_on_timeout.patch` is the complete four-file diff against
the pinned source. It adds a default-false, read-only ROS parameter
`position_hold_on_timeout`. When enabled, startup latches the measured joint
pose; a stale MIT command retains its last position/gains and zeros velocity and
extra torque while gravity/friction compensation continues. Stale Cartesian MIT
commands latch measured joint position instead. No firmware mode switch occurs.
The flag and active timeout state are published in `control_state` JSON.

The original default-false behavior and shutdown-to-drag semantics remain
unchanged. Do not terminate or hot-swap an active driver to apply this patch.
Legacy running drivers use the API's separate ROS-only guardian instead.

All seven existing/new C++ tests passed against a MockArm, including fixed-pose
timeout retention, continued compensation, no mode writes, and command resumption.
The example arm configurations are included solely to keep the original tests
reproducible. Build and verify with:

```bash
PIPER_BUILD_TESTING=ON ../../scripts/build_vendor.sh --parallel-workers 2
sha256sum --check SHA256SUMS.patched --status
ROS_DOMAIN_ID=128 PIPER_CONFIG_DIR="$PWD/config/examples" build/piper_x_driver/test_cpp_driver
```
