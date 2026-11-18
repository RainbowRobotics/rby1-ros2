# Examples (`rby1_examples`)

[Overview](01_overview.md) · [Driver](02_driver.md) · [Examples](03_examples.md) · [Description](04_description.md) · [Hardware Interface](05_hardware_interface.md) · [MoveIt](06_moveit.md) · [Tools](07_tools.md) · [UPC Topic Bridge](08_upc_topic_bridge.md)

Python scripts, one per feature of the [driver](02_driver.md). Each runs in its own terminal while the driver is up;
**try them in the simulator first**, the control examples move the robot.

```bash
source install/setup.bash
ros2 run rby1_examples 06_zero_pose
```

The values an example uses — postures, times, speeds, priorities, how long it waits before a cancel — are constants in
one block at the top of its file, under the imports (`# Values to change: edit these to adjust the example.`).
Examples 15 and 16 take theirs as ROS parameters; the block holds their defaults. The examples that run until Ctrl+C
(02, 03, 04, 15, 16) end cleanly on it.

| Example | What it does | Uses |
|---------|--------------|------|
| `01_power_control` | Power on, servo on, servo off, power off | `robot_power`, `robot_servo` |
| `02_robot_status_monitor` | Prints the robot state until Ctrl+C: control manager, brakes, battery, tool flanges, emergency stop | `robot_state`, `battery_state` |
| `03_tool_flange_monitoring` | Prints the tool flanges: status from `robot_state`, IMU and wrench topics | `publish_tool_flange_state` |
| `04_joint_state_monitoring` | Prints the joint positions per part | `joint_states` (`PartJointStates`) |
| `05_gravity_compensation` | Gravity compensation on, a pause for direct teaching, off | `gravity_compensation` |
| `06_zero_pose` | Every joint to 0 rad in one goal | `robot_joint` |
| `07_joint_command` | Zero pose, then a target pose with one control mode per part: joint impedance for the right arm, joint position for the left arm and the head, joint group position for three torso joints (`use_group_joint`, only the joints named in `joint_names`) | `robot_joint` |
| `08_cartesian_command` | The ready pose, then a Cartesian target per arm: position (right), impedance (left) | `robot_cartesian` |
| `09_multi_controls` | Joint and Cartesian control of different parts at once | both |
| `10_trajectory_joint_command` | A pre-computed whole-body trajectory, in position control and again in joint impedance | `follow_joint_trajectory`, `stream_control`, `set_trajectory_impedance` |
| `11_cancel_control` | An action cancel, then the `cancel_control` service | [below](#two-ways-to-stop-a-command) |
| `12_mobile_base_control` | Drives the base: opens the `mobile` channel, publishes `cmd_vel`, closes it | `cmd_vel`, `stream_control` |
| `13_stream_command` | Zero and ready poses over the arm channels with varying waits between commands | `stream_joint`, `stream_control` |
| `14_collision_safety_control` | Watches `robot_state.collision` and retreats to a safe pose when it is set | `robot_state`, `robot_joint` |
| `15_cartesian_target_move` | The simplest target executor: hand targets from a topic, straight to each with the driver alone | [below](#15_cartesian_target_move) |
| `16_target_shuttle_publisher` | Publishes two hand poses in turn for example 15 or a planner to carry out | [below](#16_target_shuttle_publisher) |

In the simulator battery, force/torque and IMU read 0. Example 10's first trajectory ends with `error_code -1`: it is
replaced on purpose by the second one on the same parts ([Driver — Command streams](02_driver.md#command-streams)).

## Two ways to stop a command

Example 11 shows both.

1. **Action cancel** stops the goal it was sent for. Stream channels stay open, so the next command goes at once; a
   trajectory's cancel stops the channels of the parts it names and no other.
2. **`cancel_control`** stops every command at once **and closes every stream channel**. After it, open the
   channels again before the next `cmd_vel` or stream command.

Sequential Python clients should spin while they wait (`rclpy.spin_once` in a loop, as `13_stream_command` does)
instead of `time.sleep()`, or DDS answers pile up and the client logs `Ignoring unexpected goal/result response`.

## Target examples (15, 16)

Two examples work with **targets**, the same ones the MoveIt planner takes ([MoveIt — target
interface](06_moveit.md#target-interface)): a target is one `std_msgs/Float64MultiArray` of 16 values on
`/rby1/<arm>/target_pose`, the row-major 4×4 pose of the arm's tool frame in `base`, answered on
`/rby1/<arm>/target_status`.

Run them together, one terminal each, with `source install/setup.bash` in every terminal. Before terminal 2 the robot
must be powered, its servos on and the right arm bent (see the notes under example 15). Stop with Ctrl+C in terminal 3,
then terminal 2.

### Terminal 1 — the driver

```bash
ros2 launch rby1_driver rby1_ros2_driver.launch.py
```

### Terminal 2 — example 15, which moves the hand

```bash
ros2 run rby1_examples 15_cartesian_target_move                              # READY, then serves targets until Ctrl+C
ros2 run rby1_examples 15_cartesian_target_move --ros-args -p mode:=stream   # or: a stream of small steps, up to 10 cm/s
```

### Terminal 3 — example 16, which sends the targets

```bash
ros2 run rby1_examples 16_target_shuttle_publisher                           # two points near the ready pose, in turn
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p period:=8.0 # or: more time per target (stream mode)
```

### Terminal 4 — the answers (optional)

```bash
ros2 topic echo /rby1/right_arm/target_status std_msgs/msg/String            # EXECUTING …, DONE
```

### `15_cartesian_target_move`

The simplest target executor: the driver alone, no planner, **nothing is checked for collisions**. Until Ctrl+C it
takes the hand straight to each target it receives on `target_topic` and answers on `status_topic`. To go around
obstacles use the [planner](06_moveit.md#executor-rby1_moveit_executor) instead.

| Status | When |
|--------|------|
| `READY` | Once, when it is ready for targets |
| `EXECUTING mode=command distance=…m duration=…s`, `EXECUTING mode=stream distance=…m max_speed=…m/s` | A target is started |
| `DONE` | The hand ended within `tolerance` of the target (its log adds `hand is … mm from the target`) |
| `FAILED: <reason and what to do>` | The driver refused or cut the move short, the move took longer than `timeout`, or the hand ended further than `tolerance` away |
| `FAILED: rejected target: …` | Not a pose: not 16 finite values, bottom row not 0 0 0 1, or not a rotation |

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mode` | `command` | `command`: one `robot_cartesian` goal per target, as in `08_cartesian_command`, taking `duration` s. `stream`: `stream_cartesian` steps at `rate` Hz on that arm's channel, which the example opens for a move and closes after it, also on Ctrl+C |
| `target_topic`, `status_topic` | `/rby1/right_arm/target_pose`, `/rby1/right_arm/target_status` | `/rby1/left_arm/…` for the left arm |
| `target_link` | from `target_topic` | `ee_right` for a `…/right_arm/…` topic, `ee_left` for `…/left_arm/…` |
| `ref_link` | `base` | Frame of the targets |
| `arm_base_link` | `link_torso_5` | The commands are sent in this frame, so only arm joints turn |
| `duration` | `3.0` | `command`: seconds a move takes |
| `rate`, `max_speed`, `max_turn_speed`, `acceleration` | `20.0`, `0.10`, `0.5`, `0.2` | `stream`: steps per second, speed of the hand (m/s) and of its orientation (rad/s), ramp (m/s²) |
| `lead` | `0.03` | `stream`: how far a step may be ahead of the measured hand (m) |
| `tolerance`, `timeout` | `0.005`, `30.0` | How far from the target the hand may end (m); time for a move (s) |

- A target that arrives while the hand moves waits for the move to end; only the latest is kept.
- Only one executor may serve an arm: at start the example gives discovery a second and refuses to run when
  `target_topic` already has a subscriber (the planner, the cuMotion executor, a `ros2 topic echo`).
- The robot must be powered, its servos on and the arm bent. A planner's launch leaves it so; **stop that launch
  before starting the example** (both would move the arm).
- In `stream` mode a step the driver refuses (a joint over its limits) holds the arm where it is, and the target is
  answered `FAILED:` with the driver's reason ([Driver — Command streams](02_driver.md#command-streams)).
- Simulator: with example 16's defaults, `command` answered `DONE` for every target, 0.0 mm off; `stream` within
  1.4–2.3 mm with no step refused. Not run on a real robot.
- The hand's pose comes from `get_cartesian_pose` (the SDK's model). From v1.1 on it matches the URDF the planner
  uses; on v1.0 the `ee_*` frames differ by 46 mm between the two.

### `16_target_shuttle_publisher`

Publishes two hand poses in turn on `target_topic`, `point_a` then `point_b`, one every `period` seconds, until
Ctrl+C. It only publishes: it does not wait for `DONE`, does not ask where the hand is, does not use TF. What moves
the arm is whatever listens: example 15, the planner, or the cuMotion executor of the RB-Y1 Isaac ROS repository.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `target_topic`, `status_topic` | `/rby1/right_arm/target_pose`, `/rby1/right_arm/target_status` | `/rby1/left_arm/…` for the left arm, with points on the left side |
| `point_a`, `point_b` | `[0.463, -0.367, 1.084, -1.571, -1.071, 1.571]`, `[0.413, -0.367, 1.034, -1.571, -1.071, 1.571]` | x, y, z (m, `base`), roll, pitch, yaw (rad; Rz(yaw) Ry(pitch) Rx(roll)) of the tool frame. Six values, each a float (`1.0`, not `1`). The defaults are 5 cm forward and up of the right hand at the ready pose (which is at (0.413, −0.367, 1.034)) and back: within reach of example 15's IK and of the planner alike |
| `period` | `5.0` | Seconds from one target to the next; longer than a move takes (`8.0` around an obstacle) |

Other targets, in terminal 3 in place of the default command:

```bash
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p period:=8.0 \
    -p "point_a:=[0.413,-0.367,1.034,-1.571,-1.071,1.571]" -p "point_b:=[0.413,-0.167,1.134,-1.571,-1.071,1.571]"
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args \
    -p target_topic:=/rby1/left_arm/target_pose -p status_topic:=/rby1/left_arm/target_status \
    -p "point_a:=[0.463,0.367,1.084,1.571,-1.071,-1.571]" -p "point_b:=[0.413,0.367,1.034,1.571,-1.071,-1.571]"
```

- Before the first target it waits for a listener on `target_topic`; after 15 s without one it ends and names what to
  start.
- It logs every line it hears on `status_topic` under the target it sent, and sends the next target either way.
- A target that arrives while the arm still moves is the listener's to handle: the planner and example 15 run it when
  the current move ends.

### Following a marker

Not an example of this repository. The RB-Y1 Isaac ROS repository's `rby1_apriltag` has the marker detector and,
next to it, `marker_target.launch.py`, which turns each arm's marker into targets on `/rby1/<arm>/target_pose` (a
target executor moves the hand), and `follow_head:=true`, which turns the head with head-only `stream_joint`
commands on the `head` channel. The camera and its mount come from [Tools — Camera](07_tools.md#camera).
