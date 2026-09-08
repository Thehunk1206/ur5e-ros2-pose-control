import contextlib
import io
import math
import unittest

from ur5e_pose_control.pose_math import nearest_joint_angle, normalize_quaternion, parse_args, pose_error, rpy_to_quaternion


class PoseMathTests(unittest.TestCase):
    def test_nearest_equivalent_joint_angle_respects_limits(self):
        cases = [
            (3.854335079714831, -1.57, -6.13, 6.13, -2.428850227464755),
            (1.7, -1.57, -6.13, 6.13, 1.7 - math.tau),  # Nearest is outside [-pi, pi].
            (0.0, 6.1, -6.13, 6.13, 0.0),  # A full turn would exceed the upper limit.
            (-3.13, 3.13, -math.pi, math.pi, -3.13),  # Cannot cross an elbow limit.
            (-5.5, 0.0, -6.13, 6.13, -5.5 + math.tau),
        ]
        for angle, current, lower, upper, expected in cases:
            with self.subTest(angle=angle, current=current):
                actual = nearest_joint_angle(angle, current, lower, upper)
                self.assertAlmostEqual(actual, expected)
                self.assertGreaterEqual(actual, lower)
                self.assertLessEqual(actual, upper)
                self.assertAlmostEqual(math.sin(actual), math.sin(angle))
                self.assertAlmostEqual(math.cos(actual), math.cos(angle))
        with self.assertRaises(ValueError):
            nearest_joint_angle(math.pi, 0.0, -0.1, 0.1)

    def assertQuaternion(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def test_identity_rotation(self):
        self.assertQuaternion(rpy_to_quaternion(0, 0, 0), (0, 0, 0, 1))

    def test_quarter_turn_about_each_axis(self):
        half = math.sqrt(0.5)
        for angles, quaternion in [((90, 0, 0), (half, 0, 0, half)),
                                   ((0, 90, 0), (0, half, 0, half)),
                                   ((0, 0, 90), (0, 0, half, half))]:
            with self.subTest(angles=angles):
                self.assertQuaternion(rpy_to_quaternion(*angles), quaternion)

    def test_rotation_order(self):
        self.assertQuaternion(rpy_to_quaternion(90, 0, 90), (0.5, 0.5, 0.5, 0.5))

    def test_normalization(self):
        self.assertEqual(normalize_quaternion((0, 0, 0, 2)), (0, 0, 0, 1))

    def test_invalid_quaternion(self):
        for q in [(0, 0, 0, 0), (0, 0, float("nan"), 1), (0, 0, 0, float("inf"))]:
            with self.subTest(q=q), self.assertRaises(ValueError):
                normalize_quaternion(q)

    def test_opposite_quaternion_sign_is_same_rotation(self):
        distance, angle = pose_error((0, 0, 0), (0, 0, 0, 1), (0, 0, 0), (0, 0, 0, -1))
        self.assertEqual((distance, angle), (0, 0))

    def test_known_position_and_angle_error(self):
        distance, angle = pose_error((0, 0, 0), (0, 0, 0, 1), (0.003, 0.004, 0), (0, 0, 1, 0))
        self.assertAlmostEqual(distance, 0.005)
        self.assertAlmostEqual(angle, math.pi)

    def test_valid_cli_degrees(self):
        args = parse_args(["--position", "0.4", "-0.1", "0.3", "--rpy-deg", "180", "0", "0"])
        self.assertQuaternion(args.quaternion, (1, 0, 0, 0))

    def test_invalid_cli(self):
        cases = [[], ["--position", "1", "2", "3"],
                 ["--position", "nan", "0", "0", "--rpy-deg", "0", "0", "0"],
                 ["--position", "1", "0", "0", "--quaternion", "0", "0", "0", "0"],
                 ["--current", "--plan-only"]]
        for args in cases:
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                parse_args(args)
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
