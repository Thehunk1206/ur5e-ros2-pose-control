"""Keep a MuJoCo arm open and plan successive terminal poses with ROS/MoveIt.

Three steps: plan with MoveIt, simulate the trajectory, check the final tool pose.
Read main(), then run_pose() and simulate(). JSON transfers plans to macOS.
Each request supplies MuJoCo's current joints; ROS/RViz state stays separate.
Use ./robot mujoco --help for commands. On macOS the viewer needs mjpython.
"""

import argparse
from contextlib import nullcontext
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import shlex
import signal
import subprocess
import sys
from threading import Thread
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
        raise ValueError(
            f"{description} must contain finite numbers with shape {shape}."
        )
    return array


def read_plan(path, model):
    """Validate exported data and map trajectory columns by joint name."""
    plan = json.loads(path.read_text())
    identity = tuple(
        plan.get(key)
        for key in (
            "version",
            "robot",
            "base_frame",
            "tool_frame",
        )
    )
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
        raise ValueError(
            "Times must start at zero, increase strictly, and end within 120 s."
        )
    positions = numbers(
        [p["positions"] for p in points], (len(points), 6), "Positions"
    )[:, order]
    bounds = numbers([plan["joint_limits"][name] for name in JOINTS], (6, 2), "Limits")
    # The runner must respect both MoveIt's soft limits and MuJoCo's joint limits.
    for column, name in enumerate(JOINTS):
        joint = model.joint(name)
        lower = max(bounds[column, 0], joint.range[0])
        upper = min(bounds[column, 1], joint.range[1])
        outside = (positions[:, column] < lower - 1e-6) | (
            positions[:, column] > upper + 1e-6
        )
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
            normalize_quaternion(sample["quaternion"]),
            *tool_pose(check),
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
        raise ValueError(
            "The plan's final joint configuration does not reach its target pose."
        )
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
            bodies = [
                model.body(model.geom_bodyid[g]).name
                for g in (contact.geom1, contact.geom2)
            ]
            raise RuntimeError(
                f"Physical contact at {data.time:.3f} s: {bodies[0]} / {bodies[1]} "
                f"({force[0]:.1f} N). Replay stopped."
            )


def simulate(model, data, plan, viewer=None):
    """Track joint targets with simulated position servos and verify the tool pose.

    The caller owns the data and viewer across all commands. Movement is
    caused by actuator forces and mj_step(). Position targets include velocity
    and gravity/Coriolis feedforward; the model's motor force limits still apply.
    Headless mode uses the same physics, without real-time pacing or a viewer.
    """
    times, positions, target_position, target_quaternion = plan
    if np.max(np.abs(data.qpos - positions[0])) > 1e-4:
        raise ValueError(
            "Plan start differs from the simulated joints; no motion started."
        )
    gains = model.actuator_gainprm[:, 0]
    damping = -model.actuator_biasprm[:, 2]
    with viewer.lock() if viewer is not None else nullcontext():
        model.site("target").pos = target_position
        model.site("target").quat = np.asarray(target_quaternion)[[3, 0, 1, 2]]
        # Refresh the marker using scratch data so qpos, qvel and time survive.
        mujoco.mj_setConst(model, mujoco.MjData(model))
        mujoco.mj_forward(model, data)
    print(
        f"SIMULATING: {times[-1]:.2f} s motion, then 2 s holding the target.",
        flush=True,
    )

    start = time.monotonic()
    motion_start = data.time
    next_frame = data.time
    max_tracking = 0.0
    # Use time relative to this motion: the simulation clock is never reset.
    while data.time - motion_start < times[-1] + 2.0:
        if viewer is not None and not viewer.is_running():
            raise RuntimeError("Viewer closed before simulation completed.")
        with viewer.lock() if viewer is not None else nullcontext():
            desired, velocity = sample_trajectory(
                times, positions, data.time - motion_start
            )
            # The upstream actuator is kp*(ctrl-q) - kd*qvel. This offset
            # makes it a PD tracker with feedforward, using simulated forces.
            command = desired + (damping * velocity + data.qfrc_bias) / gains
            data.ctrl[:] = np.clip(
                command, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
            )
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or np.any(data.warning.number):
                raise RuntimeError("MuJoCo reported unstable physics.")
            check_contacts(model, data)
            desired_now, _ = sample_trajectory(
                times, positions, data.time - motion_start
            )
            tracking = float(np.max(np.abs(data.qpos - desired_now)))
            max_tracking = max(max_tracking, tracking)
            if tracking > 0.15:
                raise RuntimeError(
                    f"Joint tracking error is too large: {tracking:.3f} rad."
                )
        if viewer is not None and data.time >= next_frame:
            viewer.sync()
            next_frame = data.time + 1 / 60
            time.sleep(max(0, start + data.time - motion_start - time.monotonic()))
    # Verify the physical state, rather than the commanded joint targets.
    distance, angle = pose_error(target_position, target_quaternion, *tool_pose(data))
    print(
        f"Final simulated error: {distance * 1000:.3f} mm, "
        f"{math.degrees(angle):.4f} degrees",
        flush=True,
    )
    print(f"Peak joint tracking error: {max_tracking:.6f} rad", flush=True)
    if (
        distance > POSITION_TOLERANCE
        or angle > ORIENTATION_TOLERANCE
        or np.max(np.abs(data.qvel)) >= 0.01
    ):
        raise RuntimeError("Simulation did not settle within the tool-pose tolerances.")
    print(
        "MUJOCO SUCCESS: simulated tool0 reached the requested position AND orientation.",
        flush=True,
    )
    return {
        "position_error_m": distance,
        "orientation_error_rad": angle,
        "peak_joint_error_rad": max_tracking,
    }


PROMPT_HELP = """Enter a pose here; keep the MuJoCo window open:
  move --position 0.4 0.1 0.4 --rpy-deg 180 0 0
  move --position 0.35 -0.1 0.45 --rpy-deg 180 0 30
Add --plan-only to check a goal without moving. --quaternion X Y Z W also works.
  pose   Show the current simulated tool pose
  help   Show these commands
  quit   Close the session
Physics pauses while waiting for a command or a plan. RViz state is separate."""


def read_command(viewer):
    """Wait for terminal input while the main thread keeps the viewer responsive."""
    commands = Queue()

    def read_line():
        try:
            commands.put(input("mujoco> "))
        except EOFError:
            commands.put(None)

    Thread(target=read_line, daemon=True).start()
    while viewer is None or viewer.is_running():
        try:
            return commands.get(timeout=0.05)
        except Empty:
            if viewer is not None:
                viewer.sync()
    return None


def export_pose(arguments, joints, viewer):
    """Run the existing ROS exporter, with this simulation's joints as its start."""
    directory = ROOT / "artifacts" / "mujoco"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"plan-{time.time_ns()}.json"
    command = [
        str(ROOT / "robot"),
        "export",
        str(path),
        *arguments,
        "--start-joints",
        *(str(float(value)) for value in joints),
    ]
    # ROS owns its own stdin (none), leaving our terminal prompt as the only reader.
    with subprocess.Popen(
        command, stdin=subprocess.DEVNULL, start_new_session=True
    ) as process:
        try:
            while process.poll() is None:
                if viewer is not None:
                    if not viewer.is_running():
                        raise KeyboardInterrupt
                    viewer.sync()
                time.sleep(0.05)
        except KeyboardInterrupt:
            # Stop the local exporter process group. Remote work is plan-only
            # and bounded by the ROS client's timeouts; no robot is executing.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass  # The exporter finished just as the window closed.
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            raise
        if process.returncode:
            raise ValueError("Planning/export failed. The simulated arm has not moved.")
    print(f"Saved plan: {path.relative_to(ROOT)}", flush=True)
    return path


def run_pose(arguments, model, data, viewer):
    """Validate and plan first; change the physical state only after a valid plan."""
    args = parse_args(arguments)
    if args.current or args.export_plan or args.start_joints is not None:
        raise ValueError(
            "Use 'pose' for simulation feedback; enter only target pose arguments."
        )
    path = export_pose(arguments, data.qpos.copy(), viewer)
    plan = read_plan(path, model)
    if not args.plan_only:
        simulate(model, data, plan, viewer)


def main(argv=None):
    """Own one model, data object and viewer for the whole terminal session."""
    parser = argparse.ArgumentParser(
        description="Plan a UR5e pose using ROS 2, then simulate it in native MuJoCo.",
        epilog="With no arguments, open a window and a mujoco> command prompt. "
        "Pose: --position X Y Z plus --rpy-deg R P Y or --quaternion X Y Z W. "
        "Add --plan-only to export without replay. Example: ./robot mujoco "
        "--position 0.4 0.1 0.4 --rpy-deg 180 0 0",
    )
    parser.add_argument(
        "--headless", action="store_true", help="Run physics without a window."
    )
    parser.add_argument(
        "--close-on-finish",
        action="store_true",
        help="Run one pose/replay and close the viewer after verification.",
    )
    parser.add_argument(
        "--replay",
        type=Path,
        help="Replay an exported JSON plan; no ROS connection needed.",
    )
    options, pose_arguments = parser.parse_known_args(argv)
    if options.replay and pose_arguments:
        parser.error("Use --replay without pose arguments.")
    if options.close_on_finish and not (options.replay or pose_arguments):
        parser.error("--close-on-finish needs a pose or --replay.")
    # A pose with --headless or --plan-only keeps the original one-shot CLI.
    one_shot = (
        options.close_on_finish
        or bool(
            pose_arguments and (options.headless or "--plan-only" in pose_arguments)
        )
        or bool(options.replay and options.headless)
    )
    try:
        model = load_model()
        data = mujoco.MjData(model)
        plan = read_plan(options.replay, model) if options.replay else None
        if plan is not None:
            data.qpos[:] = plan[1][0]  # Initialize saved replay once, before opening.
            data.ctrl[:] = data.qpos
        else:
            mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        mujoco.mj_forward(model, data)
        no_window = options.headless or "--plan-only" in pose_arguments
        context = (
            nullcontext(None)
            if no_window
            else mujoco.viewer.launch_passive(model, data)
        )
        with context as viewer:
            if viewer is not None:
                viewer.cam.lookat[:] = [0.15, 0, 0.35]
                viewer.cam.distance = 2.2
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25
                viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
                viewer.sync()
            if plan is not None:
                simulate(model, data, plan, viewer)
            if one_shot:
                if pose_arguments:
                    run_pose(pose_arguments, model, data, viewer)
                return 0
            print(PROMPT_HELP, flush=True)
            pending = pose_arguments
            while viewer is None or viewer.is_running():
                try:
                    if pending:
                        arguments, pending = pending, []
                    else:
                        line = read_command(viewer)
                        if line is None:
                            break
                        arguments = shlex.split(line)
                    if not arguments:
                        continue
                    if arguments == ["quit"]:
                        break
                    if arguments == ["help"]:
                        print(PROMPT_HELP, flush=True)
                    elif arguments == ["pose"]:
                        position, quaternion = tool_pose(data)
                        print(
                            "Simulated tool0 in base_link (metres; quaternion x y z w):"
                        )
                        print(
                            "--position "
                            + " ".join(f"{v:.6f}" for v in position)
                            + " --quaternion "
                            + " ".join(f"{v:.6f}" for v in quaternion),
                            flush=True,
                        )
                    else:
                        if arguments[0] == "move":
                            arguments = arguments[1:]
                        run_pose(arguments, model, data, viewer)
                except SystemExit:
                    pass  # argparse has explained invalid input; keep the session.
                except (ValueError, KeyError, TypeError, IndexError, OSError) as error:
                    print(f"MUJOCO ERROR: {error}", file=sys.stderr, flush=True)
                # Physics errors are fatal: do not continue from an unstable state.
        return 0
    except KeyboardInterrupt:
        print("MuJoCo demo interrupted.", file=sys.stderr)
        return 130
    except (
        ValueError,
        KeyError,
        TypeError,
        IndexError,
        OSError,
        RuntimeError,
    ) as error:
        print(f"MUJOCO ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
