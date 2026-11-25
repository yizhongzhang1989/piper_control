import argparse
import math
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Common robot control service and 3D dashboard")
    parser.add_argument("--backend", choices=("sim", "hardware"), default="sim")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--rate", type=float, default=200.)
    parser.add_argument("--max-velocity", type=float, default=.3, help="configured joint speed cap, rad/s")
    parser.add_argument("--stream-timeout", type=float, default=.2)
    parser.add_argument("--goal-tolerance", type=float, default=.01, help="measured terminal joint error tolerance, rad")
    parser.add_argument("--goal-timeout", type=float, default=5., help="settling time allowed after trajectory end, s")
    parser.add_argument("--control-profile", default=str(Path(__file__).parent/"assets/position_control.yaml"),
                        help="position kp/kd profile YAML; use 'source' for original arm gains")
    parser.add_argument("--workspace", default=os.environ.get("PIPER_WORKSPACE", str(Path.home()/"Documents/Piper_workspace")))
    parser.add_argument("--arm", default="arm_2")
    parser.add_argument("--arm-config", help="simulation: reuse arm YAML gains and fitted gravity/friction models")
    parser.add_argument("--launch-driver", action="store_true", help="explicitly launch existing ROS CAN driver (auto-enables)")
    parser.add_argument("--ros", action="store_true", help="expose standard ROS2 interfaces in /robot")
    parser.add_argument("--tcp-offset", type=float, nargs=3, default=(0.,0.,.16), metavar=("X", "Y", "Z"))
    parser.add_argument("--tcp-orientation", type=float, nargs=4, default=(0.,0.,0.,1.), metavar=("QX", "QY", "QZ", "QW"))
    parser.add_argument("--end-effector", choices=("bare", "gripper", "gripper_with_wrench", "teaching_handle"), default=None)
    args = parser.parse_args()
    if args.backend == "hardware" and args.arm_config:
        parser.error("--arm-config is for simulation; hardware uses --workspace and --arm configuration")
    if not 1 <= args.rate <= 200 or not all(math.isfinite(x) and x > 0 for x in (
            args.max_velocity, args.stream_timeout, args.goal_tolerance, args.goal_timeout)):
        parser.error("rate must be 1..200 Hz; velocity, timeout and tolerance must be positive and finite")
    from .model import RobotModel
    from .controller import RobotController
    from .server import create_app
    if args.ros:
        try:
            from control_msgs.action import FollowJointTrajectory  # noqa: F401
        except ImportError:
            parser.error("ROS action messages missing: run scripts/install_ros_extras.sh and source scripts/env.sh")
    import yaml
    gains = {} if args.control_profile == "source" else yaml.safe_load(Path(args.control_profile).read_text())
    gains = {key: gains[key] for key in ("kp", "kd") if key in gains}
    end_effector = args.end_effector or "gripper_with_wrench"
    if args.arm_config and args.end_effector is None:
        import yaml
        end_effector = yaml.safe_load(Path(args.arm_config).expanduser().read_text()).get("end_effector", end_effector)
    model = RobotModel(tcp_offset=args.tcp_offset, tcp_orientation=args.tcp_orientation, end_effector=end_effector)
    if args.backend == "sim":
        from .backends.simulation import SimulationBackend
        backend = SimulationBackend(model, arm_config=args.arm_config, **gains)
    else:
        from .backends.hardware import HardwareBackend
        backend = HardwareBackend(workspace=args.workspace, arm_id=args.arm, launch_driver=args.launch_driver, **gains)
    controller = RobotController(backend, model, rate_hz=args.rate, max_velocity=args.max_velocity,
                                 stream_timeout=args.stream_timeout, goal_tolerance=args.goal_tolerance,
                                 goal_timeout=args.goal_timeout)
    bridge = None
    try:
        if args.ros:
            from .ros_bridge import RosBridge
            bridge = RosBridge(controller)
        import uvicorn
        uvicorn.run(create_app(controller, bridge), host=args.host, port=args.port)
    finally:
        if bridge:
            bridge.close()
        controller.close()


if __name__ == "__main__":
    main()
