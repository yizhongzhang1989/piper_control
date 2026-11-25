"""Standard ROS action/stream integration against the actual physics backend."""

import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("control_msgs")


def wait_future(future, timeout=8.0):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(timeout), "ROS future timed out"
    return future.result()


def test_standard_action_reordering_cancel_and_tool_twist(monkeypatch):
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import TwistStamped
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from trajectory_msgs.msg import JointTrajectoryPoint

    from piper_control.backends.simulation import SimulationBackend
    from piper_control.controller import RobotController
    from piper_control.model import RobotModel
    from piper_control.ros_bridge import RosBridge

    monkeypatch.setenv("ROS_DOMAIN_ID", "126")
    model = RobotModel()
    controller = RobotController(SimulationBackend(model), model).start()
    controller.enable()
    bridge = RosBridge(controller, namespace="/test_robot")
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("test_client", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(node, FollowJointTrajectory, "/test_robot/follow_joint_trajectory")
    try:
        assert client.wait_for_server(timeout_sec=5)
        start = np.array(controller.state()["positions"])
        target = start.copy()
        target[0] += .06
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = model.names[::-1]
        point = JointTrajectoryPoint()
        point.positions = target[::-1].tolist()
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        handle = wait_future(client.send_goal_async(goal))
        assert handle.accepted
        result = wait_future(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert np.max(np.abs(np.array(controller.state()["positions"]) - target)) < .03

        target[0] += .1
        point.positions = target[::-1].tolist()
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        handle = wait_future(client.send_goal_async(goal))
        assert handle.accepted
        time.sleep(.15)
        assert wait_future(handle.cancel_goal_async()).goals_canceling
        result = wait_future(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert controller.state()["mode"] == "idle"

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher = node.create_publisher(TwistStamped, "/test_robot/servo_twist", qos)
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.02)
        twist = TwistStamped()
        twist.header.frame_id = "tcp"
        twist.twist.linear.x = .001
        publisher.publish(twist)
        time.sleep(.06)
        assert controller.state()["mode"] == "servo_twist"
        assert not controller.state()["errors"]
        time.sleep(.25)
        assert controller.state()["mode"] == "idle"
    finally:
        bridge.close()
        controller.close()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        context.shutdown()
