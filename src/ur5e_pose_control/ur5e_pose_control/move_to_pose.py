"""Coordinate one UR5e pose command using the existing MoveIt ROS 2 server.

Read ``main()`` first, then ``PoseClient.move()`` to follow the execution flow:

    Terminal input -> current robot state -> inverse kinematics (IK)
                   -> motion planning/execution -> final tool-pose check

This client sends requests; MoveIt supplies the IK solver and motion planner,
and the configured controller executes the trajectory. The supplied demo launch
uses mock hardware. The client itself does not select or start the hardware.

Conventions:
    Position: metres, relative to the robot's ``base_link`` frame.
    Orientation: normalized quaternion in x, y, z, w order.
    Joint angles and limits: radians.
    Controlled tool frame: ``tool0`` (the bare arm, without a gripper offset).

``--current`` only reads feedback. ``--plan-only`` solves IK and plans a path
without executing it. ``--export-plan FILE`` also saves that plan for MuJoCo.
Argument parsing and small mathematical helpers live in
``pose_math.py``. Each command creates one client node and exits when finished.
"""

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetPlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from ur5e_pose_control.pose_math import nearest_joint_angle, parse_args, pose_error

# These names must match the UR model and MoveIt configuration in the launch file.
BASE = "base_link"
TOOL = "tool0"
GROUP = "ur_manipulator"

# Verification thresholds for reported tool feedback after execution.
POSITION_TOLERANCE = 0.005  # Final feedback must be within 5 mm.
ORIENTATION_TOLERANCE = 0.01  # Final angular error, radians (about 0.57 degrees).


def make_ik_request(position, quaternion, robot_state=None):
    """Build a /compute_ik request for both tool position and orientation.

    Args:
        position: Target (x, y, z) in metres, relative to BASE.
        quaternion: Target orientation as normalized (x, y, z, w).
        robot_state: Optional MoveIt RobotState used as the solver's initial guess.
            If omitted, an empty state diff preserves the server's current state.

    Returns:
        A GetPositionIK.Request; this function does not send it.

    IK checks the destination configuration. Planning must still find a path
    from the current configuration to that destination.
    """
    request = GetPositionIK.Request()
    ik = request.ik_request
    ik.group_name = GROUP
    ik.ik_link_name = TOOL
    ik.avoid_collisions = True
    ik.timeout.sec = 2

    # An empty diff keeps MoveIt's current robot state as the IK seed.
    ik.robot_state.is_diff = True
    if robot_state is not None:
        ik.robot_state = robot_state

    # The frame identifies the coordinate system in which the target is expressed.
    ik.pose_stamped.header.frame_id = BASE
    pose = ik.pose_stamped.pose
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation = Quaternion(
        x=quaternion[0],
        y=quaternion[1],
        z=quaternion[2],
        w=quaternion[3],
    )

    return request


def make_goal(joint_state, current_positions, joint_limits, plan_only=False):
    """Build a MoveGroup action goal using the joint angles returned by IK.

    Args:
        joint_state: IK solution containing joint names and positions in radians.
        current_positions: Mapping from joint name to current angle in radians.
        joint_limits: Mapping from joint name to (lower, upper) bounds in radians.
        plan_only: If True, request planning without trajectory execution.

    Returns:
        A MoveGroup.Goal; this function does not send it.

    All joint constraints belong to one goal and must hold together. MoveIt
    checks limits and collisions along the path, then creates a timed trajectory.
    """
    constraints = Constraints()
    for name, position in zip(joint_state.name, joint_state.position):
        # An IK angle plus/minus a full turn gives the same pose. Choose the
        # nearest equivalent to the current joint, respecting the model's limits.
        constraints.joint_constraints.append(
            JointConstraint(
                joint_name=name,
                position=nearest_joint_angle(
                    position, current_positions[name], *joint_limits[name]
                ),
                weight=1.0,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
            )
        )

    goal = MoveGroup.Goal()
    goal.request.group_name = GROUP
    goal.request.pipeline_id = "ompl"
    goal.request.start_state.is_diff = True  # Use MoveIt's current joint state.
    goal.request.goal_constraints = [constraints]
    goal.request.allowed_planning_time = 10.0
    goal.request.num_planning_attempts = 1
    # Scaling factors are fractions of the configured joint speed/acceleration limits.
    goal.request.max_velocity_scaling_factor = 0.1
    goal.request.max_acceleration_scaling_factor = 0.1
    goal.planning_options.plan_only = plan_only
    # Empty diffs preserve the server's planning scene and current robot state.
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal


def error_name(code):
    """Translate a MoveIt error number into its name, keeping unknown codes readable."""
    return next(
        (
            name
            for name in dir(MoveItErrorCodes)
            if name.isupper() and getattr(MoveItErrorCodes, name) == code
        ),
        str(code),
    )


class PoseClient(Node):
    """ROS 2 node that coordinates a pose request and observes the result.

    Services provide short request/response operations: IK, joint-state lookup,
    and robot-model lookup. The MoveGroup action provides motion feedback and
    cancellation. TF supplies the tool pose derived from joint-state feedback.
    """

    def __init__(self):
        """Create the communication clients and TF listener without sending a goal."""
        super().__init__("ur5e_pose_client")

        # The action manages a potentially long-running planning/execution request.
        self.action = ActionClient(self, MoveGroup, "/move_action")

        # Each service client specifies both the message type and the service name.
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.model = self.create_client(GetParameters, "/move_group/get_parameters")

        # The listener receives transforms; the buffer stores them for later lookup.
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        # Keep these objects so a timeout or Ctrl-C can cancel a pending goal.
        # Acceptance, the accepted goal handle, and completion are separate stages.
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.last_feedback = None

    def wait(self, future, seconds, description):
        """Process ROS callbacks until a future completes, then return its result.

        A future represents a response that will arrive later. Spinning lets the
        node receive that response, action feedback, and TF updates while waiting.
        Raise TimeoutError if the future is still pending after ``seconds``.
        """
        rclpy.spin_until_future_complete(self, future, timeout_sec=seconds)
        if not future.done():
            raise TimeoutError(f"Timed out waiting for {description}.")
        return future.result()

    def current_pose(self, after_ns=None, timeout=10.0):
        """Return a fresh tool pose as (position_xyz, quaternion_xyzw).

        Args:
            after_ns: Earliest acceptable transform timestamp in ROS-clock
                nanoseconds. Defaults to the time this method is entered.
            timeout: Wall-clock wait budget in seconds.

        Position is in metres relative to BASE. The transform must also be less
        than two seconds old and must not be dated in the future. This prevents
        a cached pre-motion pose from being used to report execution success.
        Raise TimeoutError when no acceptable transform arrives.
        """
        if after_ns is None:
            after_ns = self.get_clock().now().nanoseconds

        # Use monotonic time for the deadline and ROS time for message timestamps.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                # Time() requests the latest available tool transform in BASE.
                transform = self.tf.lookup_transform(BASE, TOOL, Time())
            except TransformException:
                continue

            stamp = Time.from_msg(transform.header.stamp).nanoseconds
            now = self.get_clock().now().nanoseconds
            if stamp < after_ns or not 0 <= now - stamp < 2_000_000_000:
                continue  # Do not report success using stale feedback.
            p, q = transform.transform.translation, transform.transform.rotation
            return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

        raise TimeoutError(
            "No fresh base_link -> tool0 transform. Check the driver and joint states."
        )

    def feedback(self, message):
        """Log action states such as PLANNING or MONITOR only when they change."""
        state = message.feedback.state
        if state != self.last_feedback:
            self.get_logger().info(f"MoveIt: {state}")
            self.last_feedback = state

    def planning_state(self):
        """Return MoveIt's current RobotState and a dictionary of joint bounds.

        /get_planning_scene supplies joint feedback. The robot_description
        parameter supplies URDF, the XML description of the robot's structure.
        For each rotating joint, intersect its position limits with any narrower
        safety limits. Bounds are returned as {joint_name: (lower, upper)} in
        radians, so equivalent-angle selection uses the configured robot model.
        """
        for client in (self.scene, self.model):
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("MoveIt's robot state/model service is unavailable.")

        # Request just the robot-state part of the planning scene.
        request = GetPlanningScene.Request(
            components=PlanningSceneComponents(
                components=PlanningSceneComponents.ROBOT_STATE
            )
        )
        state = self.wait(
            self.scene.call_async(request), 5.0, "current joint state"
        ).scene.robot_state
        description = (
            self.wait(
                self.model.call_async(
                    GetParameters.Request(names=["robot_description"])
                ),
                5.0,
                "robot model",
            )
            .values[0]
            .string_value
        )

        limits = {}
        for joint in ET.fromstring(description).findall("joint"):
            # UR5e's controlled joints are revolute (rotating) joints.
            if joint.get("type") != "revolute":
                continue
            bound = joint.find("limit")
            lower, upper = float(bound.get("lower")), float(bound.get("upper"))

            safety = joint.find("safety_controller")
            if safety is not None:
                lower = max(lower, float(safety.get("soft_lower_limit", lower)))
                upper = min(upper, float(safety.get("soft_upper_limit", upper)))
            limits[joint.get("name")] = (lower, upper)

        return state, limits

    def export_plan(self, result, position, quaternion, limits, filename):
        """Save a timed plan plus ROS FK samples for the native MuJoCo runner.

        FK means forward kinematics: joint angles -> tool pose. The samples let
        the runner verify its model against MoveIt before simulating any motion.
        This method is only used after a successful planning-only request.
        """
        trajectory = result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("Cannot export an empty trajectory.")
        client = self.create_client(GetPositionFK, "/compute_fk")
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("MoveIt forward-kinematics service is unavailable.")
            samples = []
            indices = {0, len(trajectory.points) // 2, len(trajectory.points) - 1}
            for index in sorted(indices):
                point = trajectory.points[index]
                request = GetPositionFK.Request()
                request.header.frame_id = BASE
                request.fk_link_names = [TOOL]
                request.robot_state.joint_state.name = trajectory.joint_names
                request.robot_state.joint_state.position = point.positions
                response = self.wait(client.call_async(request), 5.0, "model FK sample")
                if response.error_code.val != MoveItErrorCodes.SUCCESS:
                    raise RuntimeError(
                        "Could not calculate the plan's reference tool poses."
                    )
                pose = response.pose_stamped[0].pose
                samples.append(
                    {
                        "positions": list(point.positions),
                        "position": [pose.position.x, pose.position.y, pose.position.z],
                        "quaternion": [
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ],
                    }
                )
        finally:
            self.destroy_client(client)
        payload = {
            "version": 1,
            "robot": "ur5e",
            "base_frame": BASE,
            "tool_frame": TOOL,
            "target": {"position": list(position), "quaternion": list(quaternion)},
            "joint_names": list(trajectory.joint_names),
            "joint_limits": {name: limits[name] for name in trajectory.joint_names},
            "points": [
                {
                    "time": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                    "positions": list(p.positions),
                    "velocities": list(p.velocities),
                    "accelerations": list(p.accelerations),
                }
                for p in trajectory.points
            ],
            "fk_samples": samples,
        }
        Path(filename).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
        print(
            f"EXPORTED: {len(trajectory.points)} trajectory points to {filename}",
            flush=True,
        )

    def move(self, position, quaternion, plan_only=False, export_path=None):
        """Solve IK, request a plan/motion, and verify the reported final pose.

        Args:
            position: Target (x, y, z) in metres relative to BASE.
            quaternion: Target orientation as normalized (x, y, z, w).
            plan_only: If True, stop after successful planning without moving.
            export_path: Optional JSON output file; always implies planning only.

        Print progress and return None on success. Failures raise an exception
        for main() to report and, when needed, cancel the pending action.
        Successful planning alone does not verify an executed final pose.
        """
        plan_only = plan_only or export_path is not None
        # 1. Require the server and live robot feedback before asking for motion.
        if not self.action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "MoveIt action server is unavailable. "
                "Start the demo and check its logs."
            )
        self.current_pose()  # Require live state before submitting a motion request.
        if not self.ik.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MoveIt inverse-kinematics service is unavailable.")

        # 2. Use current joints as the IK seed and solve for the complete tool pose.
        state, limits = self.planning_state()
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        solution = self.wait(
            self.ik.call_async(make_ik_request(position, quaternion, state)),
            5.0,
            "inverse kinematics",
        )
        if solution.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Inverse kinematics failed: {error_name(solution.error_code.val)}."
            )
        print("IK SUCCESS: found joint angles for the requested pose.", flush=True)

        # 3. Send the joint goal. Acceptance only means the server will process it.
        goal = make_goal(solution.solution.joint_state, current, limits, plan_only)
        if export_path is not None:
            # MuJoCo has a ground plane at z=0. Include the same surface in this
            # request's scene diff, without changing the shared RViz demo scene.
            floor = CollisionObject(id="mujoco_floor", operation=CollisionObject.ADD)
            floor.header.frame_id = BASE
            floor.primitives = [
                SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[20.0, 20.0, 0.1])
            ]
            floor_pose = Pose()
            floor_pose.position.z = -0.05
            floor_pose.orientation.w = 1.0
            floor.primitive_poses = [floor_pose]
            goal.planning_options.planning_scene_diff.world.collision_objects = [floor]
        self.goal_future = self.action.send_goal_async(
            goal,
            feedback_callback=self.feedback,
        )
        self.goal_handle = self.wait(self.goal_future, 10.0, "goal acceptance")
        if not self.goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the goal.")

        # 4. Wait for completion and check both ROS action status and MoveIt's result.
        self.result_future = self.goal_handle.get_result_async()
        response = self.wait(self.result_future, 90.0, "planning and execution")
        code = response.result.error_code.val
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or code != MoveItErrorCodes.SUCCESS
        ):
            detail = ""
            if not response.result.planned_trajectory.joint_trajectory.points:
                detail = (
                    " No plan was found for the selected IK solution. "
                    "IK success checks the destination, not the path. "
                    "See ./robot logs for the planner's reason."
                )
            raise RuntimeError(
                f"MoveIt failed: {error_name(code)} "
                f"(action status {response.status}).{detail}"
            )

        # 5. A preview finishes here: there is no executed pose to verify.
        if plan_only:
            if export_path is not None:
                self.export_plan(
                    response.result, position, quaternion, limits, export_path
                )
            print(
                "PLAN SUCCESS: a trajectory was found; no execution was requested.",
                flush=True,
            )
            return

        # 6. Verify feedback published after execution, rather than trusting the goal.
        actual_position, actual_quaternion = self.current_pose(
            after_ns=self.get_clock().now().nanoseconds
        )
        distance, angle = pose_error(
            position, quaternion, actual_position, actual_quaternion
        )
        print(
            f"Final error: {distance * 1000:.2f} mm, {math.degrees(angle):.3f} degrees",
            flush=True,
        )
        # The tiny epsilon prevents floating-point rounding at the tolerance boundary.
        if distance > POSITION_TOLERANCE + 1e-6 or angle > ORIENTATION_TOLERANCE + 1e-6:
            raise RuntimeError(
                "Execution completed, but the reported final pose "
                "is outside tolerance."
            )
        print("SUCCESS: tool0 reached the requested pose within tolerance.", flush=True)

    def cancel(self):
        """Request cancellation of a pending goal and wait for its final result.

        Called after Ctrl-C or an error. A cancellation response alone does not
        establish the final action state, so also wait for the terminal result.
        Log the final action status, or an error if it cannot be confirmed.
        This software cancellation request is not a physical emergency stop.
        """
        if self.goal_future is None or (
            self.result_future and self.result_future.done()
        ):
            return
        try:
            # Ctrl-C may arrive while the original goal is still being acknowledged.
            if self.goal_handle is None:
                self.goal_handle = self.wait(
                    self.goal_future, 3.0, "pending goal acceptance"
                )
            if not self.goal_handle.accepted:
                return

            self.wait(
                self.goal_handle.cancel_goal_async(), 3.0, "cancellation response"
            )
            if self.result_future is None:
                self.result_future = self.goal_handle.get_result_async()
            result = self.wait(self.result_future, 5.0, "the final action state")
            self.get_logger().warning(
                f"Action ended with status {result.status} after cancellation request."
            )
        except Exception as error:
            self.get_logger().error(
                f"Could not confirm the final motion state: {error}"
            )


def main(argv=None):
    """Run one terminal command and clean up this client's ROS resources.

    Args:
        argv: Optional argument list for callers/tests; None uses terminal arguments.

    Returns:
        Process status: 0 for success, 1 for a runtime failure, or 130 for Ctrl-C.
        Invalid arguments exit with status 2 inside parse_args(), before ROS starts.

    The driver, MoveIt, and RViz belong to the separate launch process and keep
    running after this client exits.
    """
    # 1. Parse and validate the terminal input before opening a ROS connection.
    args = parse_args(argv)

    # Keep Python's Ctrl-C handling active so we can cancel before shutting ROS down.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = PoseClient()
    exit_code = 0

    try:
        if args.current:
            position, quaternion = node.current_pose()
            print(f"Current {TOOL} pose in {BASE} (metres; quaternion x y z w):")
            print(
                "--position "
                + " ".join(f"{v:.6f}" for v in position)
                + " --quaternion "
                + " ".join(f"{v:.6f}" for v in quaternion)
            )
        else:
            # 2. Solve IK. 3. Plan/execute. 4. Verify the resulting pose.
            print(
                f"Target {TOOL} in {BASE}: "
                f"position={args.position}, quaternion={args.quaternion}",
                flush=True,
            )
            node.move(args.position, args.quaternion, args.plan_only, args.export_plan)
    except KeyboardInterrupt:
        node.cancel()
        exit_code = 130
    except Exception as error:
        node.get_logger().error(str(error))
        node.cancel()
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
