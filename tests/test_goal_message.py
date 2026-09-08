import unittest

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    from ur5e_pose_control.move_to_pose import make_goal, make_ik_request
    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Run inside the ROS container to check the actual ROS messages.")
class GoalMessageTests(unittest.TestCase):
    def test_goal_survives_ros_serialization(self):
        ik = make_ik_request([0.4, 0.1, 0.3], [0.0, 0.0, 0.0, 1.0])
        decoded_ik = deserialize_message(serialize_message(ik), GetPositionIK.Request)
        self.assertEqual(decoded_ik.ik_request.pose_stamped.header.frame_id, "base_link")
        self.assertEqual(decoded_ik.ik_request.ik_link_name, "tool0")
        self.assertEqual(decoded_ik.ik_request.pose_stamped.pose.position.x, 0.4)
        self.assertEqual(decoded_ik.ik_request.pose_stamped.pose.orientation.w, 1.0)
        self.assertTrue(decoded_ik.ik_request.avoid_collisions)
        self.assertTrue(decoded_ik.ik_request.robot_state.is_diff)
        # This shoulder angle caused a planning timeout for pose (0.2, 0.2, 0.2).
        # Its nearest legal equivalent must be sent, leaving the IK response intact.
        joints = JointState(name=["wrist_3_joint", "shoulder_lift_joint"],
                            position=[0.25, 3.854335079714831])
        goal = make_goal(joints, {"wrist_3_joint": 0.0, "shoulder_lift_joint": -1.57},
                         {"wrist_3_joint": (-6.13, 6.13), "shoulder_lift_joint": (-6.13, 6.13)})
        decoded = deserialize_message(serialize_message(goal), MoveGroup.Goal)
        self.assertEqual(len(decoded.request.goal_constraints), 1)
        constraint = decoded.request.goal_constraints[0]
        self.assertEqual(len(constraint.joint_constraints), 2)
        self.assertEqual(constraint.joint_constraints[0].joint_name, "wrist_3_joint")
        self.assertEqual(constraint.joint_constraints[0].position, 0.25)
        self.assertAlmostEqual(constraint.joint_constraints[1].position, -2.428850227464755)
        self.assertEqual(joints.position[1], 3.854335079714831)
        self.assertTrue(decoded.request.start_state.is_diff)
        self.assertTrue(decoded.planning_options.planning_scene_diff.is_diff)
        self.assertFalse(decoded.planning_options.plan_only)

    def test_preview_never_requests_execution(self):
        goal = make_goal(JointState(name=["wrist_3_joint"], position=[0.25]),
                         {"wrist_3_joint": 0.0}, {"wrist_3_joint": (-6.13, 6.13)}, plan_only=True)
        self.assertTrue(goal.planning_options.plan_only)


if __name__ == "__main__":
    unittest.main()
