"""Plan a terminal pose with ROS/MoveIt, then replay it using MuJoCo physics.

Three steps: plan with MoveIt, simulate the trajectory, check the final tool pose.
Read main(), then simulate(). JSON transfers the plan from Linux to macOS.
Replay has no live feedback connection to ROS.
Use ./robot mujoco --help for commands. On macOS the viewer needs mjpython.
"""

import argparse
from contextlib import nullcontext
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ur5e_pose_control"))
from ur5e_pose_control.pose_math import normalize_quaternion, parse_args, pose_error
from model import JOINTS, load_model, tool_pose

POSITION_TOLERANCE = 0.005  # Same final acceptance thresholds as the ROS client.
ORIENTATION_TOLERANCE = 0.01


def numbers(value, shape, description):
    """Reject malformed or non-finite numeric data before starting physics."""
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{description} must contain finite numbers with shape {shape}.")
    return array


def read_plan(path, model):
    """Validate exported data and map trajectory columns by joint name."""
    plan = json.loads(path.read_text())
    identity = tuple(plan.get(key) for key in (
        "version", "robot", "base_frame", "tool_frame",
    ))
    if identity != (1, "ur5e", "base_link", "tool0"):
        raise ValueError("Expected version 1 UR5e plan for tool0 in base_link.")
    names = plan["joint_names"]
    if len(names) != 6 or set(names) != set(JOINTS):
        raise ValueError("The plan must contain each UR5e joint exactly once.")
    order = [names.index(name) for name in JOINTS]
    points = plan["points"]
    if not points:
        raise ValueError("The trajectory is empty.")
    times = numbers([p["time"] for p in points], (len(points),), "Timestamps")
    if abs(times[0]) > 1e-9 or np.any(np.diff(times) <= 0) or times[-1] > 120:
        raise ValueError("Times must start at zero, increase strictly, and end within 120 s.")
    positions = numbers(
        [p["positions"] for p in points], (len(points), 6), "Positions"
    )[:, order]
    bounds = numbers([plan["joint_limits"][name] for name in JOINTS], (6, 2), "Limits")
    # The runner must respect both MoveIt's soft limits and MuJoCo's joint limits.
    for column, name in enumerate(JOINTS):
        joint = model.joint(name)
        lower = max(bounds[column, 0], joint.range[0])
        upper = min(bounds[column, 1], joint.range[1])
        outside = (positions[:, column] < lower - 1e-6) | (positions[:, column] > upper + 1e-6)
        if lower >= upper or np.any(outside):
            raise ValueError(f"Trajectory violates joint limits: {name}.")
    target = plan["target"]
    position = numbers(target["position"], (3,), "Target position")
    quaternion = normalize_quaternion(target["quaternion"])

    # Use a separate data object for FK checks; this never moves the simulation.
    check = mujoco.MjData(model)
    if not plan.get("fk_samples"):
        raise ValueError("ROS forward-kinematics reference samples are missing.")
    max_distance = max_angle = 0.0
    for sample in plan["fk_samples"]:
        check.qpos[:] = numbers(sample["positions"], (6,), "FK joints")[order]
        mujoco.mj_forward(model, check)
        distance, angle = pose_error(
            numbers(sample["position"], (3,), "FK position"),
            normalize_quaternion(sample["quaternion"]), *tool_pose(check),
        )
        max_distance, max_angle = max(max_distance, distance), max(max_angle, angle)
    if max_distance > 1e-5 or max_angle > 1e-5:
        raise ValueError(
            f"ROS/MuJoCo model mismatch: {max_distance * 1000:.4f} mm, "
            f"{math.degrees(max_angle):.6f} degrees. Check dimensions and frames."
        )
    # A valid file must also end at its advertised target, not just match FK samples.
    check.qpos[:] = positions[-1]
    mujoco.mj_forward(model, check)
    distance, angle = pose_error(position, quaternion, *tool_pose(check))
    if distance > POSITION_TOLERANCE or angle > ORIENTATION_TOLERANCE:
        raise ValueError("The plan's final joint configuration does not reach its target pose.")
    print(
        f"MODEL CHECK PASSED: ROS/MuJoCo FK differ by at most {max_distance * 1000:.6f} mm.",
        flush=True,
    )
    return times, positions, position, quaternion


def sample_trajectory(times, positions, elapsed):
    """Linearly interpolate position and return the segment's target velocity.

    This deliberately simple interpolation preserves every waypoint. The saved
    MoveIt derivatives are retained for inspection but are not used for splines.
    """
    if elapsed >= times[-1] or len(times) == 1:
        return positions[-1], np.zeros(6)
    index = max(0, int(np.searchsorted(times, elapsed, side="right")) - 1)
    duration = times[index + 1] - times[index]
    velocity = (positions[index + 1] - positions[index]) / duration
    return positions[index] + (elapsed - times[index]) * velocity, velocity


def check_contacts(model, data):
    """Stop when physics reports a contact carrying more than 1 N of normal force."""
    force = np.zeros(6)
    for index in range(data.ncon):
        mujoco.mj_contactForce(model, data, index, force)
        if force[0] > 1.0:
            contact = data.contact[index]
            bodies = [model.body(model.geom_bodyid[g]).name
                      for g in (contact.geom1, contact.geom2)]
            raise RuntimeError(
                f"Physical contact at {data.time:.3f} s: {bodies[0]} / {bodies[1]} "
                f"({force[0]:.1f} N). Replay stopped."
            )


def simulate(model, plan, headless=False, close_on_finish=False):
    """Track joint targets with simulated position servos and verify the tool pose.

    qpos is written once to initialize the replay. Every subsequent movement is
    caused by actuator forces and mj_step(). Position targets include velocity
    and gravity/Coriolis feedforward; the model's motor force limits still apply.
    Headless mode uses the same physics, without real-time pacing or a viewer.
    """
    times, positions, target_position, target_quaternion = plan
    data = mujoco.MjData(model)
    gains = model.actuator_gainprm[:, 0]
    damping = -model.actuator_biasprm[:, 2]
    model.site("target").pos = target_position
    model.site("target").quat = np.asarray(target_quaternion)[[3, 0, 1, 2]]
    mujoco.mj_setConst(model, data)  # Refresh cached world-site coordinates.
    data.qpos[:] = positions[0]  # The only direct joint-position assignment.
    data.ctrl[:] = positions[0]
    mujoco.mj_forward(model, data)
    print(f"SIMULATING: {times[-1]:.2f} s motion, then 2 s holding the target.", flush=True)

    context = (nullcontext(None) if headless
               else mujoco.viewer.launch_passive(model, data))
    with context as viewer:
        if viewer is not None:
            viewer.cam.lookat[:] = [0.15, 0, 0.35]
            viewer.cam.distance = 2.2
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        start = time.monotonic()
        next_frame = 0.0
        max_tracking = 0.0
        # Hold the last target for two extra seconds, then measure the result.
        while data.time < times[-1] + 2.0:
            if viewer is not None and not viewer.is_running():
                raise RuntimeError("Viewer closed before simulation completed.")
            lock = viewer.lock() if viewer is not None else nullcontext()
            with lock:
                desired, velocity = sample_trajectory(times, positions, data.time)
                # The upstream actuator is kp*(ctrl-q) - kd*qvel. This offset
                # makes it a PD tracker with feedforward, using simulated forces.
                command = desired + (damping * velocity + data.qfrc_bias) / gains
                data.ctrl[:] = np.clip(
                    command, model.actuator_ctrlrange[:, 0],
                    model.actuator_ctrlrange[:, 1],
                )
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)  # Refresh pose at the integrated state.
                if not np.isfinite(data.qpos).all() or np.any(data.warning.number):
                    raise RuntimeError("MuJoCo reported unstable physics.")
                check_contacts(model, data)
                desired_now, _ = sample_trajectory(times, positions, data.time)
                tracking = float(np.max(np.abs(data.qpos - desired_now)))
                max_tracking = max(max_tracking, tracking)
                if tracking > 0.15:
                    raise RuntimeError(f"Joint tracking error is too large: {tracking:.3f} rad.")
            if viewer is not None and data.time >= next_frame:
                viewer.sync()
                next_frame = data.time + 1 / 60
                time.sleep(max(0, start + data.time - time.monotonic()))
        # Verify the physical state, rather than the commanded joint targets.
        distance, angle = pose_error(target_position, target_quaternion, *tool_pose(data))
        print(
            f"Final simulated error: {distance * 1000:.3f} mm, "
            f"{math.degrees(angle):.4f} degrees", flush=True,
        )
        print(f"Peak joint tracking error: {max_tracking:.6f} rad", flush=True)
        if (distance > POSITION_TOLERANCE or angle > ORIENTATION_TOLERANCE
                or np.max(np.abs(data.qvel)) >= 0.01):
            raise RuntimeError("Simulation did not settle within the tool-pose tolerances.")
        print(
            "MUJOCO SUCCESS: simulated tool0 reached the requested position AND orientation.",
            flush=True,
        )
        if viewer is not None and not close_on_finish:
            print(
                "Final view paused. Close the MuJoCo window to return to the terminal.",
                flush=True,
            )
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.05)
    return {
        "position_error_m": distance,
        "orientation_error_rad": angle,
        "peak_joint_error_rad": max_tracking,
    }


def main(argv=None):
    """Accept the same pose arguments as ./robot move, or replay a saved JSON file."""
    parser = argparse.ArgumentParser(
        description="Plan a UR5e pose using ROS 2, then simulate it in native MuJoCo.",
        epilog="Pose: --position X Y Z plus --rpy-deg R P Y or --quaternion X Y Z W. "
               "Add --plan-only to export without replay. Example: ./robot mujoco "
               "--position 0.4 0.1 0.4 --rpy-deg 180 0 0",
    )
    parser.add_argument("--headless", action="store_true", help="Run physics without a window.")
    parser.add_argument("--close-on-finish", action="store_true", help="Close the viewer after verification.")
    parser.add_argument("--replay", type=Path, help="Replay an exported JSON plan; no ROS connection needed.")
    options, pose_arguments = parser.parse_known_args(argv)
    try:
        if options.replay:
            if pose_arguments:
                parser.error("Use --replay without pose arguments.")
            path = options.replay
        else:
            args = parse_args(pose_arguments)
            if args.current or args.export_plan:
                parser.error("Use ./robot pose for ROS feedback and ./robot export for a custom output file.")
            directory = ROOT / "artifacts" / "mujoco"
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"plan-{time.time_ns()}.json"
            result = subprocess.run([str(ROOT / "robot"), "export", str(path), *pose_arguments])
            if result.returncode:
                return result.returncode
            print(f"Saved plan: {path.relative_to(ROOT)}", flush=True)
            if args.plan_only:
                return 0
        model = load_model()
        plan = read_plan(path, model)
        simulate(model, plan, options.headless, options.close_on_finish)
        return 0
    except KeyboardInterrupt:
        print("MuJoCo demo interrupted.", file=sys.stderr)
        return 130
    except (ValueError, KeyError, TypeError, IndexError, OSError, RuntimeError) as error:
        print(f"MUJOCO ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
