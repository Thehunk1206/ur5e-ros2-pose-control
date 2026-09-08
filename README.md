# UR5e pose control

Enter a tool position and orientation in the terminal. A small Python ROS 2 node
asks MoveIt 2 to plan and execute the motion. RViz shows the UR5e moving, and the
node checks the final reported pose.

This implements **Stage 1** of the assignment: ROS 2, the official UR5e driver,
Python terminal input, and RViz. The supplied launch always uses **mock hardware**.
Isaac Sim is the optional Stage 2 and is not included.

## Requirements and the role of Colima

This setup targets **Apple Silicon macOS** with an ARM64 Linux container. Docker
CLI, Docker Compose, Colima, and Python 3 are already installed on this Mac. The
VM configuration below uses 4 CPUs, 6 GiB of memory, and a 40 GiB data disk.

| Component | What it does |
| --- | --- |
| Colima | Starts a Linux VM with Docker Engine inside it. |
| Docker CLI and Compose | Build and manage the project's container from the Mac terminal. |
| ROS 2 and MoveIt | Run the driver, solve inverse kinematics, and plan and execute motion. |
| RViz | Displays the robot using its reported joint states. |
| noVNC | Streams the Linux RViz window to your Mac browser. |

**Docker Desktop's engine and GUI are not needed for this setup.** Colima provides
the engine that the Docker CLI connects to. Colima does not perform the robot
simulation or visualization; those programs run inside the container.

The commands below use this Mac's project path. On another machine, change the
`cd` command to the directory containing this README and install the prerequisites
first. You do not need to install ROS or RViz directly on macOS.

## Start the demo: Terminal 1

Open Terminal on your Mac. Run these commands one at a time:

```bash
cd /Users/tauhidkhan/Desktop/projects/mowito-robotics

colima start --cpus 4 --memory 6 --disk 40 --vm-type vz
docker context use colima
docker info

./robot start
```

If Colima reports that it is already running, continue. `docker info` should show
both client and server information. `./robot start` builds the image and starts
the robot container in the background. The first build downloads ROS packages
and takes several minutes; later starts reuse the image cache.

Open RViz in your browser:

```bash
open 'http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
```

You can also paste [the RViz URL](http://localhost:6080/vnc.html?autoconnect=true&resize=scale)
into your browser. Wait until the robot appears. Its red, green, and blue tool
axes represent X, Y, and Z. If the page opens before the display is ready, reload it.

Show the startup and controller logs in Terminal 1:

```bash
./robot logs
```

This command keeps following the logs. Use a second terminal for motion commands.
Pressing Ctrl-C **while following logs** only stops the log viewer; the container
continues running.

## Move the robot: Terminal 2

Open another Terminal window and keep RViz visible beside it. Run each command
separately, waiting for it to finish before starting the next one.

```bash
cd /Users/tauhidkhan/Desktop/projects/mowito-robotics
```

### 1. Read the current tool pose

```bash
./robot pose
```

This prints the current `tool0` pose relative to `base_link`, including position
and quaternion arguments that you can copy into a move command. If it reports
that no fresh transform is available during startup, check Terminal 1's logs and
try again once the driver has started.

### 2. Preview a plan without moving

```bash
./robot move --position 0.4 0.1 0.4 --rpy-deg 180 0 0 --plan-only
```

Expected result: `IK SUCCESS`, then `PLAN SUCCESS`. The robot stays still.
Plan-only does not save a trajectory; a later move command computes a new plan.

### 3. Execute the first pose

```bash
./robot move --position 0.4 0.1 0.4 --rpy-deg 180 0 0
```

Watch the robot move in RViz. The terminal reports the final position and angular
errors, followed by `SUCCESS` when both are within tolerance. Exact errors and
the sampled path can vary between runs.

### 4. Change both position and orientation

```bash
./robot move --position 0.35 -0.1 0.45 --rpy-deg 180 0 30
```

Wait for `SUCCESS`, then show the resulting pose:

```bash
./robot pose
```

### 5. Demonstrate an unreachable input

```bash
./robot move --position 10 10 10 --rpy-deg 0 0 0
```

This deliberately unreachable pose should report `NO_IK_SOLUTION` and exit with
code 1 without moving the robot. It demonstrates error handling.

Pressing Ctrl-C **during a move command** requests cancellation of that motion
and waits for the action's final state. The client exits with code 130.

## Input reference

| Argument | Meaning |
| --- | --- |
| `--position X Y Z` | Position in **metres**, relative to `base_link`. |
| `--rpy-deg ROLL PITCH YAW` | Orientation in **degrees**, using fixed-axis X/Y/Z rotations. |
| `--quaternion QX QY QZ QW` | Alternative orientation input; order is **x, y, z, w**. |
| `--plan-only` | Calculate a plan without requesting execution. |

Supply a position and exactly one orientation format. For roll/pitch/yaw, the
rotation matrix is `Rz(yaw) Ry(pitch) Rx(roll)`. Quaternion input is normalized;
zero-length quaternions, NaN, and infinity are rejected. The target is the bare
arm's `tool0` frame; no gripper offset is modelled.

The first demo pose can also be expressed as:

```bash
./robot move --position 0.4 0.1 0.4 --quaternion 1 0 0 0
```

## Command reference

Run these commands from the project directory:

| Command | Purpose |
| --- | --- |
| `./robot start` | Build the image if necessary and start the container. |
| `./robot pose` | Print the current tool pose. |
| `./robot move ...` | Plan and execute a target pose. |
| `./robot logs` | Follow startup, planning, and controller logs. |
| `./robot test` | Run unit and integration tests; the integration test moves the simulated robot. |
| `./robot shell` | Open a container shell with ROS and this package already sourced. |
| `./robot stop` | Stop and remove this project's container; keep the image and source files. |
| `./robot help` | Print the available commands. |
| `colima status` | Check the Linux VM and its Docker runtime. |
| `docker ps` | List running containers in the selected Docker context. |

Inside `./robot shell`, the underlying ROS command is:

```bash
ros2 run ur5e_pose_control move_to_pose --position 0.4 0.1 0.4 --rpy-deg 180 0 0
```

Use `exit` to leave that shell. The `./robot` wrapper is a convenience for running
the same ROS commands from the Mac.

## Stop or restart

After the demonstration, run in the Mac terminal:

```bash
./robot stop
colima stop
```

`colima stop` also stops the VM used by any other containers on this Colima
instance. You can leave Colima running if you still need those containers.

To reset the robot to its startup pose while leaving Colima running:

```bash
./robot stop
./robot start
```

Reload the RViz browser tab after a restart. Run the full Terminal 1 startup
sequence again if Colima was stopped.

Python source is mounted from this directory, so normal Python edits take effect
on the next command. The package is built each time the container starts. After
changing launch files, configuration, package metadata, or dependencies, stop and
start the container with the two commands above. Running `./robot start` alone
does not restart an already-running, unchanged container.

The wrapper uses an ignored `.docker-cli/` directory for public image downloads
and Compose plugin discovery. It leaves the Mac's normal Docker credentials and
credential-helper settings untouched. The source mount is read-only inside the
container; generated ROS build files live inside the container.

## Read the code in this order

| File | Responsibility |
| --- | --- |
| [move_to_pose.py](src/ur5e_pose_control/ur5e_pose_control/move_to_pose.py) | The main assignment: solve IK, create a motion goal, wait, and check the result. |
| [pose_math.py](src/ur5e_pose_control/ur5e_pose_control/pose_math.py) | Validate inputs, convert degrees to a quaternion, choose equivalent joint angles, and measure pose error. |
| [demo.launch.py](src/ur5e_pose_control/launch/demo.launch.py) | Start UR's driver, MoveIt, and RViz using the supplied UR model/configuration. |
| [robot](robot), [Dockerfile](Dockerfile), [docker/](docker/) | Mac/Linux setup and convenience commands; these contain no motion-planning logic. |
| [tests/](tests/) | Input/math checks, ROS-message checks, and the simulation integration test. |

The robot-specific names are constants: `base_link`, `tool0`, `ur_manipulator`.
One process handles one command. There are no custom messages, custom planners,
or third-party Python wrappers around MoveIt.

## What happens when you give a goal

1. **Validate.** Read position and orientation. Reject missing or non-finite
   values. Convert roll/pitch/yaw degrees into a normalized quaternion.
2. **Solve inverse kinematics (IK).** Send the complete pose to MoveIt's
   `/compute_ik` service. Its KDL solver uses the current robot state as a starting
   guess and finds joint angles matching both position and orientation. The
   destination must also pass the planning scene's collision check.
3. **Plan and execute.** `make_goal()` chooses the equivalent target angle nearest
   each current joint, within the robot model's position and safety limits. For
   example, 221 degrees and -139 degrees give the same tool pose; one can require
   much less motion from the current state. `planning_state()` reads the current
   joints and URDF limits from MoveIt, so those limits are not hardcoded. The node
   puts the chosen angles into a `MoveGroup` goal and sends it to `/move_action`.
   MoveIt's OMPL pipeline finds a path from
   the current state to that joint configuration, checking collisions along the
   path. It creates a timed joint trajectory
   and sends it to `joint_trajectory_controller`.
4. **Observe and verify.** Mock hardware mirrors controller commands into joint
   feedback. `robot_state_publisher` computes link transforms from that feedback;
   RViz displays them. After execution completes, our node reads a fresh
   `base_link -> tool0` transform and checks the final pose.

```text
Python pose -> MoveIt IK -> MoveIt planner -> controller -> mock hardware
                    ^                                         |
                    +------------ joint feedback -------------+
                                          |
                               robot_state_publisher -> RViz
                                          |
                                  final pose check
```

`rclpy.spin_until_future_complete()` processes incoming ROS callbacks while a
request is pending. We use a ROS action because motion takes time, has feedback,
and can be cancelled. Sending the request is not treated as successful execution.

Position error is Euclidean distance. Orientation error is the shortest rotation
angle between the goal and feedback quaternions. `q` and `-q` mean the same
orientation. The final feedback must be within **5 mm and 0.01 radians** (about
0.57 degrees) of the requested pose. These are verification thresholds. IK targets
the exact pose, and the planner's joint goal tolerance is 0.0001 radians per joint.

Planning has a 10-second budget; the client waits at most 90 seconds for the
combined planning/execution result. Velocity and acceleration scaling are both
0.1. The mock controller runs at 100 Hz to suit a laptop VM. Ctrl-C or a client
timeout requests cancellation; the client reports if it
cannot confirm the action's final state. Cancellation is not a physical emergency stop.
The IK request has a two-second search budget. IK is a service call because it
returns a short calculation result; motion uses an action because it runs over time.

Exit codes: `0` success, `1` ROS/planning/execution/verification failure, `2` invalid
arguments, `130` interrupted by Ctrl-C.

## Checks

With the container running and no other motion command in progress:

```bash
./robot test
```

This runs 12 unit tests followed by the integration test. It moves the simulated
robot and deliberately exercises failure cases. Expected failure messages appear
during the run; look for the final `PASS: planning, two executed poses, final-pose
verification, and failure handling.` message to confirm the full test passed.

Pure input/math tests also run directly on macOS, without ROS:

```bash
PYTHONPATH=src/ur5e_pose_control python3 -m unittest discover -s tests -v
```

Two ROS-message tests are skipped on the Mac and run inside the container.
They also check that a full-turn IK angle is converted to its nearest legal
equivalent without modifying the original IK response. The math tests cover both
positive and negative full turns and cases where a closer angle would exceed a
joint limit. `./robot test` checks actual
message serialization, plan-only behaviour (including the regression pose
`--position 0.2 0.2 0.2 --rpy-deg 20 0 0`), two
executed goal poses (including an orientation change), final pose errors, an
unreachable goal, bad input, and a missing action server. The test computes its
goals using forward kinematics of the installed UR5e model and prints reusable
demo commands when all checks pass.

Initial validation on this Apple Silicon Mac on 2026-09-07: all 11 original container unit tests
and the integration test passed. The two integration moves reported 0.04/0.05 mm
position error and 0.010/0.011 degrees orientation error. Both short RPY commands
above also executed successfully. An interactive Ctrl-C test returned cancellation
status 5 (`CANCELED`), exit code 130, and unchanged feedback over the following
one-second observation. These measurements come from ideal mock hardware.

Tested packages: ROS 2 Jazzy, UR driver/config 3.8.0, MoveIt core 2.12.4,
RViz 14.1.22, on a Colima VM with 4 CPUs and 6 GB RAM.

After the joint-angle selection fix, all 12 unit tests passed. The regression pose
and both README demo poses each passed three plan-only requests from the startup
configuration (nine successful plans). Fresh feedback confirmed that neither
position nor orientation changed during those requests. This regression check
did not execute motion.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| Docker cannot connect to its engine | Start Colima, run `docker context use colima`, then check `docker info`. |
| `service "robot" is not running` | Run `./robot start`; inspect `./robot logs` if it exits. |
| Browser says connection refused or disconnected | Check `docker ps` and `./robot logs`, wait for startup, then reload the RViz URL. |
| MoveIt is unavailable or no fresh transform arrives | Startup may still be in progress. Check the driver/controller logs, then retry `./robot pose`. |
| `NO_IK_SOLUTION` or a planning failure | Check metres versus degrees, the reference frame, and the orientation. Try one of the verified demo poses. |
| `IK SUCCESS` followed by a planning failure | IK found a destination configuration, but the planner could not connect it to the current state. Read `./robot logs` for the detailed reason, such as a timeout. |
| Successful command but no visible motion | The robot may already be at that pose. Use the other demo pose and check the final error output. |
| Port 6080 is already in use | Use `lsof -nP -iTCP:6080 -sTCP:LISTEN` to identify the listener and resolve the port conflict. |
| RViz disconnects after restarting the container | Reload the browser tab to reconnect to the new noVNC session. |

For diagnostics, run from the project directory:

```bash
colima status
docker context show
docker info
docker ps -a
./robot logs
```

IK success alone does not guarantee that a path will be found. During testing,
one shoulder target of about 221 degrees timed out; its equivalent -139 degree
target planned successfully. The script now chooses the nearest equivalent angle
within the model's limits before planning. A different IK branch, joint limits, or collisions can still
prevent planning. A timeout does not prove that the requested pose is unreachable.

## A short explanation you can give

"My Python node takes a position and orientation from the terminal. MoveIt first
finds joint angles for that pose using inverse kinematics. It then plans a
collision-checked joint trajectory and sends it to the ROS controller. The UR
mock hardware publishes joint feedback, which RViz uses to display the motion.
I wait for execution to finish and check the final tool pose against the input.
I reuse the standard driver, IK solver, and planner; my code handles input,
coordination, and verification."

## Limitations to explain in the interview

- A pose can be unreachable even when its position looks nearby: orientation,
  joint limits, self-collision, and the starting configuration also matter.
- A planning timeout means no solution was found in that budget. It does not
  prove no valid path exists.
- A pose can have multiple joint solutions. This minimal implementation selects
  one IK solution and plans to it. If that path fails, it does not search all other
  IK branches. The sampled solution and path can vary between runs.
- Choosing the nearest legal equivalent angle reduces each joint's requested
  rotation. It does not guarantee the shortest collision-free trajectory or
  choose the best of all possible IK branches.
- Run one goal command at a time. Concurrent requests can preempt one another.
- The pose applies at the destination. It does not enforce a straight tool
  path or a constant tool orientation throughout the motion.
- Only the arm is modelled. RViz's floor grid is not a collision object. A table,
  gripper, workpiece, and other obstacles must be added to the planning scene.
- Mock hardware is an ideal position simulation. It does not model gravity,
  payload, contacts, tracking error, or protective stops. Reported success is a
  simulation result, not proof of real-world accuracy or safety.
- A real arm needs the correct calibration, TCP/tool geometry, payload, controller
  configuration, External Control program, networking, workspace model, and
  validated speed/stop behaviour. The provided launch does not connect to one.

## Recording and submission

Build the image before recording so the screencast does not include package
downloads. Then use `./robot stop` to prepare for a fresh launch. Keep the browser
and terminal visible together and follow this sequence:

1. Run `./robot start`, open the RViz URL, and show the startup logs in Terminal 1.
2. In Terminal 2, run `./robot pose` and explain the position and orientation fields.
3. Run the plan-only command and show that it succeeds without moving the robot.
4. Execute the first demo pose; show the RViz motion and final error output.
5. Execute the second demo pose; point out the change in position and tool axes.
6. Enter the unreachable pose and show the reported failure without motion.
7. Explain the four steps in the code: validate, solve IK, plan/execute, verify.
8. State that this uses mock hardware and describe the real-robot limitations.

For the assignment submission, keep the repository private, give `puru07` access,
and share its link by email. Upload the screencast to Google Drive with public
link access, as requested. Update the placeholder maintainer metadata in
[package.xml](src/ur5e_pose_control/package.xml) before submitting.

## References

- [Official UR ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt architecture](https://moveit.picknik.ai/main/doc/concepts/move_group.html)
- [MoveGroup action definition](https://github.com/moveit/moveit_msgs/blob/ros2/action/MoveGroup.action)
- [Colima](https://github.com/abiosoft/colima)
