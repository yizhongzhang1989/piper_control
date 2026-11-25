"""Optional ROS 2 standard messages/actions over the same generic controller.

ROS is imported only when a bridge is instantiated, so simulation and HTTP SDK
use do not require a ROS installation.
"""

from __future__ import annotations

import json
import threading
import time


class RosBridge:
    def __init__(self, controller, namespace="/robot", rate_hz=200.0):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger

        self.controller = controller
        self._closed = False
        self._action_lock = threading.Lock()
        self._action_busy = False
        self._FollowJointTrajectory = FollowJointTrajectory
        self._JointState, self._PoseStamped, self._String = JointState, PoseStamped, String
        self._CancelResponse, self._GoalResponse = CancelResponse, GoalResponse
        self._context = Context()
        rclpy.init(context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("robot_interface", namespace=namespace, context=self._context)
        self._group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_pub = self.node.create_publisher(JointState, "joint_states", qos)
        self._pose_pub = self.node.create_publisher(PoseStamped, "tcp_pose", qos)
        self._state_pub = self.node.create_publisher(String, "state", 1)
        self.node.create_subscription(JointState, "servo_joints", self._servo_joints, qos)
        self.node.create_subscription(PoseStamped, "servo_pose", self._servo_pose, qos)
        self.node.create_subscription(TwistStamped, "servo_twist", self._servo_twist, qos)
        self.node.create_service(Trigger, "stop", self._stop, callback_group=self._group)
        self.node.create_service(SetBool, "enable", self._enable, callback_group=self._group)
        self._server = ActionServer(
            self.node, FollowJointTrajectory, "follow_joint_trajectory",
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self._group,
        )
        self.node.create_timer(1.0 / rate_hz, self._publish)
        self._last_json = 0.0
        self._executor = MultiThreadedExecutor(num_threads=3, context=self._context)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, name="robot-ros", daemon=True)
        self._thread.start()

    def _invoke(self, callback, *args, **kwargs):
        try:
            return callback(*args, **kwargs)
        except (ValueError, RuntimeError) as error:
            self.node.get_logger().warning(str(error))
            return None

    def _servo_joints(self, message):
        names = self.controller.state()["joint_names"]
        if message.name:
            if len(message.name) != len(names) or len(set(message.name)) != len(names) or set(message.name) != set(names):
                self.node.get_logger().warning("servo_joints requires exactly all robot joint names")
                return
            indexes = [message.name.index(name) for name in names]
        else:
            indexes = list(range(len(names)))
        if len(message.position) != len(names):
            self.node.get_logger().warning("servo_joints requires all joint positions")
            return
        self._invoke(self.controller.servo_joints, [message.position[i] for i in indexes])

    def _servo_pose(self, message):
        if message.header.frame_id not in ("", "base", "base_link"):
            self.node.get_logger().warning("servo_pose frame_id must be base_link")
            return
        pose = message.pose
        self._invoke(self.controller.servo_pose, {
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        })

    def _servo_twist(self, message):
        frame = message.header.frame_id or "base_link"
        if frame not in ("base", "base_link", "tcp", "tool"):
            self.node.get_logger().warning("servo_twist frame_id must be base_link or tcp")
            return
        twist = message.twist
        self._invoke(self.controller.servo_twist,
                     [twist.linear.x, twist.linear.y, twist.linear.z],
                     [twist.angular.x, twist.angular.y, twist.angular.z],
                     frame="base" if frame in ("base", "base_link") else "tool")

    def _stop(self, request, response):
        try:
            self.controller.stop()
            response.success, response.message = True, "Holding current position"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def _enable(self, request, response):
        try:
            (self.controller.enable if request.data else self.controller.disable)()
            response.success, response.message = True, "Enabled" if request.data else "Disabled"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def _goal(self, request):
        with self._action_lock:
            if self._action_busy:
                return self._GoalResponse.REJECT
            self._action_busy = True
        return self._GoalResponse.ACCEPT

    @staticmethod
    def _seconds(duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _execute(self, goal):
        action = self._FollowJointTrajectory
        result = action.Result()
        motion_id = None
        try:
            request = goal.request
            trajectory = request.trajectory
            if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
                raise ValueError("Only immediate trajectories (header.stamp=0) are supported")
            if request.path_tolerance or request.goal_tolerance or self._seconds(request.goal_time_tolerance):
                raise ValueError("Custom ROS action tolerances are unsupported; configure controller tolerances")
            if getattr(request, "multi_dof_trajectory", None) and request.multi_dof_trajectory.points:
                raise ValueError("Multi-DOF trajectories are unsupported")
            points = []
            for point in trajectory.points:
                if point.accelerations or point.effort:
                    raise ValueError("Trajectory supports positions and optional velocities; acceleration/effort are unsupported")
                converted = {"positions": list(point.positions), "time_from_start": self._seconds(point.time_from_start)}
                if point.velocities:
                    converted["velocities"] = list(point.velocities)
                points.append(converted)
            motion = self.controller.trajectory(points, joint_names=list(trajectory.joint_names))
            motion_id = motion["id"]
            started = time.monotonic()
            while not self._closed:
                if goal.is_cancel_requested:
                    current = self.controller.motion(motion_id)
                    if current["status"] in ("queued", "running"):
                        self.controller.stop()
                    goal.canceled()
                    result.error_code = action.Result.SUCCESSFUL
                    result.error_string = "Cancelled; holding current position"
                    return result
                motion = self.controller.motion(motion_id)
                status = motion["status"]
                if status == "succeeded":
                    goal.succeed()
                    result.error_code = action.Result.SUCCESSFUL
                    return result
                if status in ("aborted", "cancelled"):
                    goal.abort()
                    result.error_code = action.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = motion.get("error") or motion.get("message") or status
                    return result
                state = self.controller.state()
                feedback = action.Feedback()
                feedback.header.stamp = self.node.get_clock().now().to_msg()
                feedback.joint_names = list(state["joint_names"])
                feedback.actual.positions = list(state["positions"])
                feedback.actual.velocities = list(state.get("velocities", []))
                desired = state.get("target_positions")
                if desired is not None:
                    feedback.desired.positions = list(desired)
                    feedback.error.positions = [d - a for d, a in zip(desired, state["positions"])]
                elapsed = time.monotonic() - started
                feedback.actual.time_from_start.sec = int(elapsed)
                feedback.actual.time_from_start.nanosec = int((elapsed % 1) * 1e9)
                goal.publish_feedback(feedback)
                time.sleep(0.02)
            if motion_id and self.controller.motion(motion_id)["status"] in ("running", "queued"):
                self.controller.stop()
            goal.abort()
            result.error_code = action.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "ROS bridge shutting down"
            return result
        except (ValueError, RuntimeError, KeyError) as error:
            goal.abort()
            result.error_code = action.Result.INVALID_GOAL
            result.error_string = str(error)
            return result
        finally:
            with self._action_lock:
                self._action_busy = False

    def _publish(self):
        try:
            state = self.controller.state()
            stamp = self.node.get_clock().now().to_msg()
            joints = self._JointState()
            joints.header.stamp = stamp
            joints.header.frame_id = "base_link"
            joints.name = list(state["joint_names"])
            joints.position = list(map(float, state["positions"]))
            joints.velocity = list(map(float, state.get("velocities", [])))
            joints.effort = list(map(float, state.get("efforts", [])))
            self._joint_pub.publish(joints)
            tcp = state["tcp_pose"]
            pose = self._PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, "base_link"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, tcp["position"])
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = map(float, tcp["orientation"])
            self._pose_pub.publish(pose)
            if time.monotonic() - self._last_json >= 0.05:
                message = self._String()
                message.data = json.dumps(state, allow_nan=False)
                self._state_pub.publish(message)
                self._last_json = time.monotonic()
        except (ValueError, KeyError, TypeError) as error:
            self.node.get_logger().warning(f"Invalid robot state: {error}")

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._server.destroy()
        self.node.destroy_node()
        if self._context.ok():
            self._context.shutdown()
