"""Small, ROS-independent helpers. Quaternion order is always x, y, z, w."""

import argparse
import math

JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def nearest_joint_angle(angle, current, lower, upper):
    """Choose angle + k*2*pi nearest the current angle, within model limits."""
    first_turn = math.ceil((lower - angle) / math.tau)
    last_turn = math.floor((upper - angle) / math.tau)
    if first_turn > last_turn:
        raise ValueError("IK joint angle has no equivalent within the model's limits.")
    turns = min(last_turn, max(first_turn, round((current - angle) / math.tau)))
    return angle + turns * math.tau


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError(
            "Expected a finite number, not NaN or infinity."
        )
    return number


def normalize_quaternion(values):
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("A quaternion needs four finite numbers: x y z w.")
    length = math.hypot(*values)
    if length < 1e-12 or not math.isfinite(length):
        raise ValueError("Quaternion length must be finite and nonzero.")
    return tuple(v / length for v in values)


def rpy_to_quaternion(roll, pitch, yaw):
    """Degrees; fixed-axis X/Y/Z rotations, equivalent to Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = (math.radians(v) / 2 for v in (roll, pitch, yaw))
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def pose_error(position, quaternion, actual_position, actual_quaternion):
    """Return distance in metres and shortest rotation angle in radians.

    q and -q describe the same rotation, hence the absolute quaternion dot product.
    """
    distance = math.dist(position, actual_position)
    q1, q2 = normalize_quaternion(quaternion), normalize_quaternion(actual_quaternion)
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    angle = 2 * math.acos(min(1.0, dot))
    return distance, angle


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Move UR5e tool0 to a pose in base_link. Position is in metres."
    )
    parser.add_argument(
        "--position", nargs=3, type=finite_number, metavar=("X", "Y", "Z")
    )
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument(
        "--rpy-deg", nargs=3, type=finite_number, metavar=("ROLL", "PITCH", "YAW")
    )
    orientation.add_argument(
        "--quaternion", nargs=4, type=finite_number, metavar=("QX", "QY", "QZ", "QW")
    )
    parser.add_argument(
        "--current", action="store_true", help="Print the current pose and exit."
    )
    parser.add_argument(
        "--plan-only", action="store_true", help="Plan without moving the robot."
    )
    parser.add_argument(
        "--export-plan",
        metavar="FILE",
        help="Save a plan as JSON; implies --plan-only.",
    )
    parser.add_argument(
        "--start-joints",
        nargs=6,
        type=finite_number,
        metavar="RAD",
        help="Simulation start joints in shoulder-to-wrist order; requires --export-plan.",
    )
    args = parser.parse_args(argv)
    if args.start_joints is not None and not args.export_plan:
        parser.error(
            "--start-joints requires --export-plan; it cannot execute a ROS motion."
        )
    if args.export_plan:
        args.plan_only = True
    if args.current:
        if args.position or args.rpy_deg or args.quaternion or args.plan_only:
            parser.error("Use --current on its own.")
        return args
    if args.position is None or (args.rpy_deg is None and args.quaternion is None):
        parser.error("Supply --position and either --rpy-deg or --quaternion.")
    try:
        args.quaternion = (
            rpy_to_quaternion(*args.rpy_deg)
            if args.rpy_deg is not None
            else normalize_quaternion(args.quaternion)
        )
    except ValueError as error:
        parser.error(str(error))
    return args
