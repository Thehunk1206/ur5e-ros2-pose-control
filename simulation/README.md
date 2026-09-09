# Native MuJoCo UR5e demo

Enter a tool position and orientation, let the existing ROS 2/MoveIt node plan,
then watch a native macOS MuJoCo window simulate that plan. The runner measures
the resulting tool pose and checks both position and orientation.

## Start and demonstrate

Tested here: Apple M1 Pro, macOS 26.2, native arm64 Python 3.12, MuJoCo 3.12.0.
Use a native Python installation, not a Python running under Rosetta. The setup
command prefers `python3.12`, falling back to `python3`.

Terminal 1, from the project directory:

First follow the [host installation and clone instructions](../README.md#installation-and-setup).
The paths below assume the example clone location `~/ur5e-ros2-pose-control`.

```bash
cd ~/ur5e-ros2-pose-control
./robot mujoco-setup
colima start --cpus 4 --memory 6 --disk 40 --vm-type vz
docker context use colima
./robot start
open 'http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
```

Wait for the robot to appear in RViz. Terminal 2:

```bash
cd ~/ur5e-ros2-pose-control
./robot mujoco
```

The window opens at the model's home configuration. Enter commands at the
`mujoco>` prompt in that same terminal, one at a time:

```text
move --position 0.4 0.1 0.4 --rpy-deg 180 0 0
move --position 0.35 -0.1 0.45 --rpy-deg 180 0 30
pose
move --position 0.2 0.2 0.2 --rpy-deg 20 0 0 --plan-only
help
quit
```

Keep the window open between moves. Each plan uses the current **MuJoCo joints**
as its IK seed and planning start. The same simulated robot continues from its
last pose, without resetting. Orange marks the requested target; green and the
tool axes show the actual end effector. Physics pauses during input and planning.
`pose` prints simulated feedback. `quit`, Ctrl-C or closing the window ends the
session. Invalid input or a planning failure returns to the prompt without motion.

Position is in metres, relative to `base_link`. RPY is in degrees. The tool is
`tool0`, with no added gripper. Quaternions use ROS order **x y z w**:

```bash
./robot mujoco --position 0.4 0.1 0.4 --quaternion 1 0 0 0
```

Passing a pose on startup executes it first and then displays the same prompt.
Add `--close-on-finish` to exit after that one motion. `--headless` with a pose
also exits after one motion; without a pose it accepts commands without a window.
The prompt is local to this process, so enter subsequent commands there instead
of starting another `./robot mujoco` process.

MuJoCo does not update the ROS mock robot. `./robot pose` reports ROS feedback;
`pose` at `mujoco>` reports MuJoCo feedback. RViz motion still uses `./robot move`.

## Other useful commands

```bash
# Plan and save JSON, without moving either robot.
./robot mujoco --position 0.35 -0.1 0.45 --rpy-deg 180 0 30 --plan-only

# Run the same physics and final verification without a window.
./robot mujoco --position 0.35 -0.1 0.45 --rpy-deg 180 0 30 --headless

# Export to a chosen local file, then replay without contacting ROS.
mkdir -p artifacts/mujoco
./robot export artifacts/mujoco/demo.json --position 0.35 -0.1 0.45 --rpy-deg 180 0 30
./robot mujoco --replay artifacts/mujoco/demo.json
```

Fresh commands save unique plans under ignored `artifacts/mujoco/`.
The Mac wrapper uses `mjpython`, required by the macOS passive viewer. Only the
planner uses Colima; MuJoCo runs on the Mac CPU with a native OpenGL viewer.

## How the code works

1. `mujoco_demo.py` validates the terminal pose with our existing parser and calls
   `./robot export`. That runs `move_to_pose.py` inside the ROS container.
2. The runner passes its six current joint angles through `--start-joints`.
   The ROS node uses them for both IK and the start of a timed MoveIt plan.
   This override is allowed only with `--export-plan`; it cannot execute a ROS motion.
   Export always implies planning only. A floor at z=0 is added to that request's
   planning scene so the planner knows about the MuJoCo ground surface.
3. The node writes joint names, timestamps, positions, derivatives, limits, target
   pose, and three reference FK samples to JSON. The Docker CLI copies it to macOS.
4. `model.py` loads the licensed Menagerie UR5e and corrects its base frame and
   nominal dimensions to match the driver's model. Before replay, ROS/MuJoCo FK
   must agree within 0.01 mm and 0.00001 rad at the sampled configurations.
5. The runner creates one model, state and viewer for the session. Fresh sessions
   start at the model's home configuration; saved replay starts at its first
   waypoint. Before each motion, the plan's first joints must match the simulated
   joints within 0.0001 rad. Subsequent commands never reset the state or clock.
   It interpolates the timed joint targets and commands the model's position
   servos. Each call to
   `mj_step()` advances forces, accelerations, velocities, and joint positions.
6. It checks actual simulated `tool0` feedback. Success requires error below
   **5 mm and 0.01 rad**, with all joint speeds below 0.01 rad/s. It holds the last
   joint target for two seconds after the planned motion, then makes this check.

The actuator controller is a standard position/velocity tracker with feedforward:

```text
motor force = kp * (desired position - actual position)
            + kd * (desired velocity - actual velocity)
            + gravity/Coriolis compensation
```

MuJoCo's configured force limits still apply. This compensates gravity through
motor commands; it does not disable gravity or teleport the joints. Waypoint
interpolation is linear; exported derivatives are retained for inspection but
are not used to reconstruct MoveIt's exact interpolation.

## Limitations and failures

- This is trajectory replay with local actuator feedback, not a live ROS hardware
  bridge. RViz and MuJoCo are separate demonstrations, not synchronized views.
- Menagerie uses simplified collision shapes and inertias. FK agreement does
  not imply identical collision or dynamic models. Only the ground is added to
  exported MoveIt requests; custom MoveIt obstacles are not imported into MuJoCo.
- Contact above 1 N, tracking error above 0.15 rad, unstable physics, a model
  mismatch, or failure to settle causes an error exit. Contact detection is a
  simulation check, not a physical robot safety system.
- A current ROS configuration can intersect the MuJoCo ground because the
  ordinary RViz launch has no floor collision object. This matters when replaying
  a file saved by `./robot export` from ROS's current state. Fresh MuJoCo sessions
  instead start at the model's home pose. If a path collides, choose a different
  pose/start; the demo does not automatically retry IK branches.
- Invalid input, an unreachable goal or planning failure leaves the interactive
  session ready for another command. Physics failures end the session. Close the
  viewer or press Ctrl-C to stop.
- The bare-arm model has no payload, gripper, measured calibration, network delay,
  or actual hardware feedback. A physical UR5e would require the real driver
  configuration, calibration, payload/TCP setup, and hardware validation.

References: [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html),
[model source and adaptations](models/SOURCE.md).
