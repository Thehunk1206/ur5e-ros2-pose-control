"""Exercise the real ROS interfaces against the running mock UR5e.

Run with ./robot test. The supplied demo launch uses mock hardware only.
"""

import os
from pathlib import Path
import subprocess
import sys

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK

from ur5e_pose_control.move_to_pose import BASE, TOOL, PoseClient
from ur5e_pose_control.pose_math import pose_error


def run_cli(arguments, expected=0, env=None):
    command = ["ros2", "run", "ur5e_pose_control", "move_to_pose", *arguments]
    print("\n$ " + " ".join(command), flush=True)
    result = subprocess.run(command, text=True, capture_output=True, timeout=115, env=env)
    print(result.stdout, end="", flush=True)
    print(result.stderr, end="", flush=True)
    if result.returncode != expected:
        raise AssertionError(f"Expected exit {expected}, got {result.returncode}")


def pose_arguments(pose):
    p, q = pose.position, pose.orientation
    return ["--position", *[f"{v:.8f}" for v in (p.x, p.y, p.z)],
            "--quaternion", *[f"{v:.8f}" for v in (q.x, q.y, q.z, q.w)]]


def main():
    root = Path(__file__).resolve().parent
    subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(root), "-v"], check=True)
    rclpy.init()
    node = PoseClient()
    try:
        fk = node.create_client(GetPositionFK, "/compute_fk")
        if not fk.wait_for_service(timeout_sec=30):
            raise RuntimeError("MoveIt /compute_fk is unavailable. Check ./robot logs.")
        request = GetPositionFK.Request()
        request.header.frame_id = BASE
        request.fk_link_names = [TOOL]
        request.robot_state.joint_state.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]
        # UR's supplied test configuration. FK gives poses from the installed model.
        request.robot_state.joint_state.position = [1.54, -1.62, 1.4, -1.2, -1.6, -0.11]
        response = node.wait(fk.call_async(request), 10, "forward kinematics")
        assert response.error_code.val == MoveItErrorCodes.SUCCESS
        first_goal = pose_arguments(response.pose_stamped[0].pose)

        before = node.current_pose()
        # Regression: equivalent full-turn IK angles previously caused timeouts.
        run_cli(["--position", "0.2", "0.2", "0.2", "--rpy-deg", "20", "0", "0", "--plan-only"])
        run_cli([*first_goal, "--plan-only"])
        after = node.current_pose(after_ns=node.get_clock().now().nanoseconds)
        assert pose_error(*before, *after)[0] < 1e-6, "Planning alone changed position"
        assert pose_error(*before, *after)[1] < 1e-6, "Planning alone changed orientation"

        run_cli(first_goal)
        # Changing only the final wrist joint exercises an orientation goal too.
        request.robot_state.joint_state.position[-1] += 0.3
        response = node.wait(fk.call_async(request), 10, "forward kinematics")
        assert response.error_code.val == MoveItErrorCodes.SUCCESS
        second_goal = pose_arguments(response.pose_stamped[0].pose)
        run_cli(second_goal)

        before = node.current_pose()
        run_cli(["--position", "10", "10", "10", "--rpy-deg", "0", "0", "0"], expected=1)
        after = node.current_pose(after_ns=node.get_clock().now().nanoseconds)
        assert pose_error(*before, *after)[0] < 1e-6, "Failed planning changed position"
        assert pose_error(*before, *after)[1] < 1e-6, "Failed planning changed orientation"
        run_cli(["--position", "nan", "0", "0", "--rpy-deg", "0", "0", "0"], expected=2)
        # An isolated ROS domain has no action server: failure must be bounded.
        run_cli(first_goal, expected=1, env={**os.environ, "ROS_DOMAIN_ID": "79"})
        print("\nPASS: planning, two executed poses, final-pose verification, and failure handling.")
        print("\nDemo commands verified on this model:")
        print("./robot move " + " ".join(first_goal))
        print("./robot move " + " ".join(second_goal))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
