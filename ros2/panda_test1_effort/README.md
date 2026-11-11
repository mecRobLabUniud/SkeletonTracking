# panda_test1_effort

Sends SkeletonTracking's `src/trajectories/test1` joint trajectory to a Franka
Emika Panda through an **effort** joint-trajectory controller.

The package contains:

| File | Purpose |
|---|---|
| `src/test1_trajectory_sender.cpp` | C++ ROS 2 node that loads `q.csv` (and optional `qd.csv`/`qdd.csv`) and sends a `FollowJointTrajectory` goal |
| `config/panda_effort_jtc.yaml` | `joint_trajectory_controller/JointTrajectoryController` configured with `command_interfaces: [effort]` and PID gains |
| `launch/panda_test1_effort.launch.py` | Brings up `franka.launch.py`, spawns `panda_effort_jtc`, starts the sender |
| `launch/panda_mujoco_sim.launch.py` | Runs the trajectory in MuJoCo, optionally together with the real Panda (`real_robot:=true`) |

## How it works

`joint_trajectory_controller/JointTrajectoryController` (Humble) supports an
`effort` command interface. In that mode it runs one `control_toolbox::Pid`
per joint and writes the resulting torque to the hardware interface. The
multipanda `franka_hardware` plugin switches the robot into libfranka joint
torque mode when an effort controller is active, so the commanded effort is
executed as a joint torque. The Franka control interface compensates gravity;
the PID only has to track the reference and reject disturbances.

The sender:

1. Reads `q.csv` (the only required file; 10 000 samples at 1 kHz for
   `test1`). If `qd.csv` / `qdd.csv` are present they are used directly,
   otherwise velocities and accelerations are derived from `q.csv` by finite
   differences.
   For `test1` the first and last samples are the Panda "ready" pose
   `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`.
2. Waits for a complete `/joint_states` message and for the action server.
3. Prepends a smooth quintic move from the current joint position to the first
   trajectory sample (`approach_duration`, default 3 s), so the effort
   controller does not see a position step. If the robot is farther than
   `max_approach_distance` (default 0.5 rad) from the first sample, the sender
   refuses to move and asks you to reposition the robot.
4. Sends the whole trajectory (approach + 10 s of dense samples) in one
   `control_msgs/action/FollowJointTrajectory` goal.
5. Logs tracking error at 1 Hz and the final JTC result code.

Time can be slowed down with `time_scale` (e.g. `2.0` = half speed). The
velocities and accelerations are scaled accordingly.

## Build

Inside the `R3D_project` container:

```bash
source /opt/ros/humble/setup.bash
source /home/lab/multipanda_ws/install/setup.bash
cd /home/lab/multipanda_ws
colcon build --packages-select panda_test1_effort --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The Docker image also builds this package automatically (see
`docker/Dockerfile`).

## Run on a real Panda

Put the robot in a safe state, make sure the FCI is enabled and the robot is
near the trajectory start pose, then:

```bash
source /opt/ros/humble/setup.bash
source /home/lab/multipanda_ws/install/setup.bash

ros2 launch panda_test1_effort panda_test1_effort.launch.py robot_ip:=172.16.0.2
```

Useful launch arguments:

```bash
# Only build and print the trajectory, do not send anything
robot_ip:=172.16.0.2 dry_run:=true

# Half speed
robot_ip:=172.16.0.2 time_scale:=2.0

# Do not start automatically; send on demand
robot_ip:=172.16.0.2 auto_start:=false

# No approach move: require the robot to already be at the start pose
robot_ip:=172.16.0.2 approach_duration:=0.0

# Execute a different joint-position CSV (qd.csv / qdd.csv are optional)
robot_ip:=172.16.0.2 trajectory_file:=my_trajectory.csv
```

If `auto_start:=false`, trigger and cancel with:

```bash
ros2 service call /test1_trajectory_sender/start std_srvs/srv/Trigger {}
ros2 service call /test1_trajectory_sender/cancel std_srvs/srv/Trigger {}
```

Run the sender standalone against an already-running robot/controller:

```bash
ros2 run panda_test1_effort test1_trajectory_sender --ros-args \
  -p trajectory_dir:=/home/lab/SkeletonTracking/src/trajectories/test1 \
  -p controller_action:=/panda_effort_jtc/follow_joint_trajectory \
  -p auto_start:=true
```

## Simulation only, or simulation + real robot

`panda_mujoco_sim.launch.py` always runs the trajectory in MuJoCo. Add
`real_robot:=true` to bring up the real Panda at the same time and run the same
trajectory on it:

```bash
# Simulation only (default)
ros2 launch panda_test1_effort panda_mujoco_sim.launch.py

# Simulation + real robot, safe dry run (connects, builds/logs, sends nothing)
ros2 launch panda_test1_effort panda_mujoco_sim.launch.py \
  real_robot:=true robot_ip:=172.16.0.2 real_dry_run:=true

# Simulation + real robot, half speed (this actually moves the robot)
ros2 launch panda_test1_effort panda_mujoco_sim.launch.py \
  real_robot:=true robot_ip:=172.16.0.2 real_dry_run:=false real_time_scale:=2.0
```

The real stack lives in the `real` namespace so it cannot collide with the
simulation: `/real/controller_manager`, `/real/joint_states`,
`/real/panda_effort_jtc/...`, while the simulation keeps `/controller_manager`,
`/joint_states` and `/panda_effort_jtc/...`. `real_dry_run` defaults to `true`
and `real_time_scale` defaults to `2.0` on purpose; real motion requires an
explicit `real_dry_run:=false`.

### libfranka version must match the robot system

This lab's Panda runs system **4.1.2**, which requires libfranka **0.8.0**
(server version 4). A workspace built against libfranka 0.9.2 fails at connect
time with:

```text
libfranka: Incompatible library version (server version: 4, library version: 5)
```

Rebuild the Franka packages against the side-by-side 0.8.0 install:

```bash
source /opt/ros/humble/setup.bash
cd /home/lab/multipanda_ws
colcon build --cmake-force-configure \
  --packages-select franka_semantic_components franka_hardware franka_gripper \
                   franka_example_controllers franka_robot_state_broadcaster \
                   multi_mode_controller_impl multi_mode_controller franka_control2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DFranka_DIR=/opt/libfranka-0.8.0/lib/cmake/Franka
```

`/opt/libfranka-0.8.0` is kept next to `/opt/libfranka` (0.9.2). Its SONAME is
also exposed in `/opt/libfranka/lib`, so the rebuilt libraries load with the
image's existing `LD_LIBRARY_PATH`. Robots with system >= 4.2.1 need
libfranka 0.9.1-0.9.2 (`/opt/libfranka`) instead.

### Network: FCI needs a direct layer-2 interface

FCI uses a TCP control connection plus a UDP state stream that the robot sends
back to the workstation address it observed. With Docker's default bridge the
robot sees a NAT address and the UDP stream never reaches the container; the
connection then fails with `libfranka: UDP receive: Timeout` right after a
successful TCP version handshake. Give the container an address on the robot
subnet:

```bash
# Once, on the host (enp3s0f1 must be the NIC connected to the robot)
docker network create -d macvlan \
  --subnet=172.16.0.0/24 --gateway=172.16.0.1 \
  -o parent=enp3s0f1 franka_macvlan

# Attach the running container; this survives container restarts
docker network connect --ip 172.16.0.100 franka_macvlan R3D_project
```

Check reachability from inside the container before launching:

```bash
timeout 3 bash -c 'cat < /dev/null > /dev/tcp/172.16.0.2/443' && echo reachable
```

Running the container with `network_mode: host` also avoids the UDP problem.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `trajectory_dir` | `/home/lab/SkeletonTracking/src/trajectories/test1` | Directory with the trajectory CSVs |
| `trajectory_file` | `q.csv` | Joint-position CSV to execute, relative to `trajectory_dir` or absolute |
| `joint_names` | `panda_joint1` … `panda_joint7` | Joint order used in the message |
| `sample_period` | `0.001` | Time between dense samples (s) |
| `time_scale` | `1.0` | Slow-motion factor (`>1` is slower) |
| `approach_duration` | `3.0` | Length of the move to the first sample (s); `0` disables it |
| `approach_dt` | `0.01` | Sample period of the approach segment (s) |
| `max_approach_distance` | `0.5` | Refuse to run if the robot is farther than this from the first sample (rad) |
| `start_tolerance` | `0.15` | Maximum allowed start-pose error when `approach_duration == 0` (rad) |
| `start_delay` | `1.0` | Wait after the first joint state before sending (s) |
| `auto_start` | `true` | Send automatically when ready |
| `dry_run` | `false` | Build/log the trajectory but do not send |
| `controller_action` | `/panda_effort_jtc/follow_joint_trajectory` | JTC action name |
| `joint_state_topic` | `/joint_states` | Joint state topic |

## Controller configuration

`config/panda_effort_jtc.yaml` uses:

- `command_interfaces: [effort]`
- `state_interfaces: [position, velocity]`
- `interpolation_method: splines`
- per-joint path tolerance 0.15 rad, goal tolerance 0.05 rad
- PID gains chosen so that the torque stays inside the Panda joint limits even
  at the path-tolerance abort threshold:
  - joints 1–4: `p = 400`, `d = 20`
  - joint 5: `p = 80`, `d = 8`
  - joint 6: `p = 60`, `d = 6`
  - joint 7: `p = 40`, `d = 4`
  - `ff_velocity_scale = 0`

If tracking is too soft for your robot, increase the gains carefully while
respecting the per-joint torque limits (87 Nm for joints 1–4, 12 Nm for
joints 5–7).

Humble's `JointTrajectoryController` prints
`'ff_velocity_scale' parameters is not defined under 'gains.<joint_name>.'`
once per joint when the value is `0.0`, because it cannot distinguish an unset
parameter from an explicit zero. The warning is benign: the controller falls
back to the deprecated `ff_velocity_scale/<joint>` parameter, which also
defaults to `0.0`, so no feed-forward is applied.

## Safety notes

- The trajectory is executed in **joint torque mode**. Keep the enabling device
  within reach and keep the workspace clear.
- The approach segment is a straight line in joint space from the current pose
  to the first sample. The sender refuses to run if the distance exceeds
  `max_approach_distance` (0.5 rad by default). For `test1`, the first sample
  is the Panda "ready" pose, so put the robot in or near that pose before
  starting. Only increase `max_approach_distance` if you have checked that the
  straight-line joint-space path is collision-free.
- If the sender process is killed while a trajectory is executing, the
  controller keeps executing the goal. Use
  `ros2 service call /test1_trajectory_sender/cancel std_srvs/srv/Trigger {}`
  or the robot's safety controls.
- Always test with `dry_run:=true` first and start with `time_scale:=2.0` on a
  new setup.
