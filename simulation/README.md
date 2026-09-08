# Native MuJoCo UR5e demo

Enter a tool position and orientation, let the existing ROS 2/MoveIt node plan,
then watch a native macOS MuJoCo window simulate that plan. The runner measures
the resulting tool pose and checks both position and orientation.

## Alignment with the assignment

The source requirements are in [UR Simulation and Motion Planning.pdf](../UR%20Simulation%20and%20Motion%20Planning.pdf).

| Requirement | Implementation |
| --- | --- |
| Correct ROS 2 UR5e driver | Existing official `ur_robot_driver`, with mock hardware in the supplied launch. |
| Python terminal input for position and orientation | Existing `move_to_pose.py` and its pose parser; MuJoCo accepts the same pose arguments. |
| Commands through ROS to the UR5e | `./robot move` uses MoveIt and the driver's trajectory controller. |
| Mandatory RViz motion | Existing `./robot start` and `./robot move`; see the main README. |
| Physics simulation on this Mac | Additional `./robot mujoco` command: ROS planning, exported trajectory, native actuator-driven physics. |
| Bonus Isaac Sim | Not implemented. MuJoCo is an alternative demonstration, not the named bonus. |

MuJoCo replay does not send execution commands through the UR driver. The
existing ROS/RViz workflow demonstrates that requirement. Preserve both parts
when presenting this project.

## Start and demonstrate

Tested here: Apple M1 Pro, macOS 26.2, native arm64 Python 3.12, MuJoCo 3.12.0.
Use a native Python installation, not a Python running under Rosetta. The setup
command prefers `python3.12`, falling back to `python3`.

Terminal 1, from the project directory:

```bash
cd /Users/tauhidkhan/Desktop/projects/mowito-robotics
./robot mujoco-setup
colima start --cpus 4 --memory 6 --disk 40 --vm-type vz
docker context use colima
./robot start
open 'http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
```

Wait for the robot to appear in RViz. Terminal 2:

```bash
cd /Users/tauhidkhan/Desktop/projects/mowito-robotics
./robot pose

# Demonstrate the mandatory ROS driver + RViz motion first.
# This also establishes a clear starting pose above the floor for replay.
./robot move --position 0.4 0.1 0.4 --rpy-deg 180 0 0

# Plan from that ROS state, then open the native MuJoCo window and simulate.
./robot mujoco --position 0.35 -0.1 0.45 --rpy-deg 180 0 30
```

The MuJoCo window opens after planning succeeds. Orange marks the target; the
green tool marker and coordinate axes show the actual end effector. At success,
the final view is paused for inspection. Close the window to return to the
terminal. `--close-on-finish` closes it automatically instead.

Then try the earlier target:

```bash
./robot mujoco --position 0.2 0.2 0.2 --rpy-deg 20 0 0
```

Position is in metres, relative to `base_link`. RPY is in degrees. The tool is
`tool0`, with no added gripper. Quaternions use ROS order **x y z w**:

```bash
./robot mujoco --position 0.4 0.1 0.4 --quaternion 1 0 0 0
```

Each replay starts at the current **ROS mock robot** configuration captured by
MoveIt. MuJoCo does not update that ROS state after replay. Therefore a second
MuJoCo command starts from ROS's configuration again, not from the last MuJoCo
result. `./robot pose` also reports ROS feedback, not MuJoCo feedback.

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
2. The ROS node reads current joints, solves IK, and requests a timed MoveIt plan.
   Export always implies planning only. A floor at z=0 is added to that request's
   planning scene so the planner knows about the MuJoCo ground surface.
3. The node writes joint names, timestamps, positions, derivatives, limits, target
   pose, and three reference FK samples to JSON. The Docker CLI copies it to macOS.
4. `model.py` loads the licensed Menagerie UR5e and corrects its base frame and
   nominal dimensions to match the driver's model. Before replay, ROS/MuJoCo FK
   must agree within 0.01 mm and 0.00001 rad at the sampled configurations.
5. The runner initializes joints to the trajectory start **once**. It interpolates
   the timed joint targets and commands the model's position servos. Each call to
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

## Validation

Verified on this Mac on 8 September 2026:

| Requested pose (metres; RPY degrees) | Final position error | Final orientation error |
| --- | --- | --- |
| `0.4 0.1 0.4`; `180 0 0` | 0.035 mm | 0.0061 degrees |
| `0.35 -0.1 0.45`; `180 0 30` | 0.034 mm | 0.0023 degrees |
| `0.2 0.2 0.2`; `20 0 0` | 0.023 mm | 0.0122 degrees |

Invalid numeric input and an unreachable goal were checked through
`./robot mujoco`; neither started
replay. Saved-plan replay passed with an unavailable Docker endpoint.

These are results of particular plans and starting states, not universal accuracy
claims. Peak joint tracking error was below 0.0006 rad for these runs. The native
viewer was opened and visually inspected, and ROS poses before/after export and
replay were unchanged.

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
  ordinary RViz launch has no floor collision object. Start with the documented
  above-floor RViz pose. If another path collides, choose a different pose/start;
  the demo does not automatically retry IK branches or replan from physics.
- Invalid input exits before planning. An unreachable goal or planning failure
  never launches replay. Close the viewer early or press Ctrl-C to stop.
- The bare-arm model has no payload, gripper, measured calibration, network delay,
  or actual hardware feedback. A physical UR5e would require the real driver
  configuration, calibration, payload/TCP setup, and hardware validation.

References: [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html),
[model source and adaptations](models/SOURCE.md),
[Isaac Sim feasibility and future live bridge](../docs/physics-simulation-plan.md).
