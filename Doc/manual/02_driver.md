# Driver (`rby1_driver`)

[Overview](01_overview.md) · [Driver](02_driver.md) · [Examples](03_examples.md) · [Description](04_description.md) · [Hardware Interface](05_hardware_interface.md) · [MoveIt](06_moveit.md) · [Tools](07_tools.md) · [UPC Topic Bridge](08_upc_topic_bridge.md)

The driver node `rby1_ros2_driver` connects to the robot or the simulator over gRPC, publishes its state every 10 ms
(100 Hz, the rate the robot gives it; 97–100 Hz measured, depending on the PC) and takes commands as services and
actions. Everything is under its namespace, `/rby1/` by default.

```bash
ros2 launch rby1_driver rby1_ros2_driver.launch.py
ros2 launch rby1_driver rby1_ros2_driver.launch.py robot_ip:=192.168.30.1:50051 model:=a namespace:=rby1 params_file:=/path/to/my.yaml
```

| Launch argument | Default | Meaning |
|---|---|---|
| `params_file` | `config/driver_parameters.yaml` | The parameter file below |
| `namespace` | `rby1` | Namespace of the node and of everything that talks to it |
| `robot_ip`, `model` | the file's | Given here they override the file, for a robot or model other than the file's |

![state_rate](../img/topic_hz.png)

**What it does**
- **Joint control**: `robot_joint` moves torso, right arm, left arm and head to joint angles, each part in position or
  impedance control, all in one goal. **Cartesian control**: `robot_cartesian` moves a tool frame to a 4×4 pose, in
  position or impedance control. **Gravity compensation** per part for direct teaching. **Trajectories**: the
  standard `follow_joint_trajectory` action (what MoveIt and the planner send). **Command streams**: continuous
  control over five channels, one per part, for `stream_joint`, `stream_cartesian`, `cmd_vel` and the trajectories
  ([Command streams](#command-streams)).
- **Base and body at once**: while `cmd_vel` drives the base, `robot_joint` moves the arms and head.
- **TF**: `odom → base_footprint` from the robot's odometry every cycle, and once at start the static
  `base_footprint → base` of the model (M: the same frame; A: `base` 0.228 m behind the wheel axle), so the odometry
  and the robot model form one tree ([Description](04_description.md)).
- **Safety**: motion commands are refused while the control manager is not enabled; a minor fault during a command is
  reset and control resumed; a major fault needs `control_manager_command 3`. The driver stops when the robot's state
  cannot be read for 1 s. At shutdown, on `cancel_control` and on servo-off it cancels the running control, closes
  its streams, waits for the robot to go idle and disables the control manager.
- **Collision checking**, in two places, both only warn — nothing stops: before a `robot_joint` or `robot_cartesian`
  command the target posture is checked with the SDK's model (for a Cartesian target through the driver's own IK;
  when that does not converge the check is skipped with a warning), and in every state cycle the measured link
  distances are read. Below `collision_threshold` the first logs `[PREDICTIVE COLLISION WARNING] …` and the second sets
  `collision` in `robot_state` and logs `Collision detected! Distance: … (Links: … <-> …)` once. Example 14 acts on it.
- **Control right**: `hardware_control` hands the robot to the `ros2_control` plugin; while it is claimed every other
  command source is refused ([Hardware Interface](05_hardware_interface.md)).

Source layout: `src/rby1_ros2_driver.cpp` (the node) includes `power.cpp`, `state_manager.cpp`, `control.cpp`
(single commands, `cancel_control`), `control_stream.cpp` (channels, trajectories, streams, `cmd_vel`),
`command_builder.cpp` and `kinematics.cpp`; `main.cpp` runs it. Values fixed in code are gathered in
`include/default_parameters.hpp`, each with its unit and reason.

## Parameters

`rby1_driver/config/driver_parameters.yaml`. With `--symlink-install` an edit needs no rebuild. For the simulator keep
`robot_ip: "127.0.0.1:50051"` (or `localhost:50051`): the driver treats those two addresses as the simulator (power
device names, no tool flange hardware).

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `robot_ip` | `"127.0.0.1:50051"` | - | Robot address and gRPC port |
| `model` | `"m"` | - | `"a"` (RB-Y1 A) or `"m"` (RB-Y1 M) |
| `stream_command_hz` | `30.0` | Hz | How often `cmd_vel` is expected: each `cmd_vel` is carried out for 1.1 of that period, so the base keeps moving between messages and stops soon after they end |
| `joint_position_command_cutoff_frequency` | `15.0` | Hz | Written to the robot at connect (robot parameter `joint_position_command.cutoff_frequency`): the joint position command filter. The robot's own default; the SDK's examples use 5 |
| `cartesian_command_cutoff_frequency` | `15.0` | Hz | Written to the robot at connect (`cartesian_command.cutoff_frequency`): the Cartesian command filter. The robot's default |
| `default_acceleration_limit_scaling` | `1.0` | - | Written to the robot at connect (`default.acceleration_limit_scaling`): scales the acceleration limits of commands that give none, 0 to 1 |
| `default_velocity_limit_scaling` | `1.0` | - | Written to the robot at connect (`default.velocity_limit_scaling`): likewise for velocity limits |
| `default_linear_acceleration_limit` | `20.0` | m/s² | Written to the robot at connect (`default.linear_acceleration_limit`): Cartesian commands' linear acceleration limit |
| `default_angular_acceleration_limit` | `10.0` | rad/s² | Written to the robot at connect (`default.angular_acceleration_limit`): Cartesian commands' angular acceleration limit |
| `se2_minimum_time` | `1.0` | s | Interpolation ramp of a `cmd_vel` (SE2 velocity) command |
| `se2_linear_acceleration_limit` | `0.5` | m/s² | Linear acceleration limit of `cmd_vel` commands |
| `se2_angular_acceleration_limit` | `0.5` | rad/s² | Angular acceleration limit of `cmd_vel` commands |
| `collision_threshold` | `0.02` | m | Link distance below which a posture counts as a collision |
| `control_manager_unlimited` | `false` | - | Enable the control manager in unlimited mode: joint limits are not checked at enable, so a joint that stopped just outside its limit can be moved back. Applies to `CMD_ENABLE` too |
| `publish_battery_state` | `true` | - | Publish `battery_state` |
| `publish_tool_flange_state` | `true` | - | Publish the tool flanges' IMU and wrench topics. The flanges' status (connected, voltage, switch) is always in `robot_state` |

The six "written to the robot at connect" values are stored by the robot itself (SDK `SetParameter`) and kept after
the driver exits. They are the robot's own defaults as shipped; drivers before 2026-10-26 wrote other values here
(cutoff 4.71 and 1.5 Hz, linear acceleration 1.0 m/s²), and the first connection of this driver replaces them.

**Fixed in the driver**, not parameters (`include/default_parameters.hpp`, with the reason for each value): the state
loop period (10 ms) and the state loss timeout (1 s); the defaults of a command that leaves a field at 0 —
`minimum_time` 2 s, Cartesian velocity limits 1.5 m/s and 4.712388 rad/s, acceleration scaling 1.0, the Cartesian
weights (translation 1000, rotation 100), joint impedance 100 N·m/rad, damping ratio 1.0, torque limit 10 N·m; the
streams' control hold time (1e6 s: a streamed command holds its part until the channel is closed), the idle close
(60 s), the reopen gap (0.3 s), the trajectory start check (a first waypoint may need at most the joints' full velocity
limit); the local IK's step, iterations and tolerances; the power and control-manager waits. Change one there and
rebuild.

## Messages

Package `rby1_msgs`. The standard types used: `sensor_msgs/JointState`, `sensor_msgs/BatteryState`,
`sensor_msgs/Imu`, `geometry_msgs/WrenchStamped`, `geometry_msgs/Twist`, `nav_msgs/Odometry`,
`control_msgs/action/FollowJointTrajectory`, `std_srvs/Trigger`.

### `JointCommand` (inside `Rby1JointCommand` goals and `StreamJointCommand`)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `joint_names` | `string[]` | — | Optional joint names |
| `position` | `float64[]` | — | Target joint positions (rad). Empty: the part is not commanded |
| `use_impedance` | `bool` | `false` | Joint impedance instead of position control |
| `use_group_joint` | `bool` | `false` | Group joint movement |
| `minimum_time` | `float64` | `2.0` | Minimum execution time (s); 0 gets the driver's default 2.0 |
| `control_hold_time` | `float64` | `0.1` | Hold after reaching the target (s); under a stream it is the stream's (1e6) |
| `velocity_limit` | `float64` | `4.7` | Joint velocity limit (rad/s) |
| `acceleration_limit` | `float64` | `1.0` | Acceleration scaling |
| `stiffness` | `float64[]` | — | Impedance stiffness per joint (N·m/rad); empty: 100 |
| `damping_ratio` | `float64` | `1.0` | Impedance damping ratio |
| `torque_limit` | `float64` | `10.0` | Impedance torque limit (N·m) |

### `CartesianCommand` (inside `Rby1CartesianCommand` goals and `StreamCartesianCommand`)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `transform` | `geometry_msgs/Transform` | — | Target pose of `target_link` in `ref_link` |
| `ref_link` | `string` | — | Reference link. Both links empty: the part is not commanded |
| `target_link` | `string` | — | The link moved: `ee_right`, `ee_left`, `link_torso_5`, … |
| `use_impedance` | `bool` | `false` | Cartesian impedance instead of position control |
| `minimum_time` | `float64` | `2.0` | Minimum execution time (s) |
| `control_hold_time` | `float64` | `0.1` | Hold after reaching the target (s) |
| `linear_velocity_limit` | `float64` | `1.5` | m/s; 0 gets 1.5 |
| `angular_velocity_limit` | `float64` | `4.7` | rad/s; 0 gets 4.712388 |
| `acceleration_limit_scaling` | `float64` | `1.0` | 0 to 1; 0 gets 1.0 |
| `translation_weight` | `float64[]` | — | Weights `[x, y, z]` of the position in the robot's QP; empty: 1000 each |
| `rotation_weight` | `float64[]` | — | Weights of the orientation; empty: 100 each |
| `add_joint_position_target` | `bool` | `false` | Also move one joint to a value |
| `add_joint_name` | `string` | — | That joint |
| `add_joint_value` | `float64` | — | Its target (rad) |

### Action goals

| Action | Goal | Result | Feedback |
|--------|------|--------|----------|
| `Rby1JointCommand` | `JointCommand torso`, `right_arm`, `left_arm`, `head`; `int32 priority` (10; 1 to run beside `cmd_vel`) | `bool success`, `string finish_code` (`kOk`, `kCanceled`, `kUnknown`, …) | `string current_state` |
| `Rby1CartesianCommand` | `CartesianCommand torso`, `right_arm`, `left_arm`; `stop_position_tracking_error` (1e-5), `stop_orientation_tracking_error` (1e-5); `priority` | the same | the same |
| `StreamJoint` | `StreamJointCommand command` (`JointCommand torso`, `right_arm`, `left_arm`, `head`) | `bool success` | — |
| `StreamCartesian` | `StreamCartesianCommand command` (`CartesianCommand torso`, `right_arm`, `left_arm`) | `bool success` | — |

`StreamPosition.action` is defined in the package but served by nothing.

### Services

| Service type | Request | Response |
|------|---------|----------|
| `StateOnOff` | `bool state`, `string parameters`, `float64 value` | `bool success`, `string message` |
| `ControlManagerCommand` | `int8 command`: `CMD_NONE` 0, `CMD_ENABLE` 1, `CMD_DISABLE` 2, `CMD_RESET` 3, `CMD_UNLIMIT` 4 | `success`, `message` |
| `GravityCompensation` | `string part_name`, `bool state` | `bool response` |
| `GetCartesianPose` | `string ref_link`, `string target_link` | `geometry_msgs/Transform transform` |
| `SetTrajectoryImpedance` | `bool[] state` (torso, right arm, left arm), `float64[] torso_stiffness`, `right_arm_stiffness`, `left_arm_stiffness`, `damping_ratio`, `torque_limit` | `success`, `message` |

### State

| Message | Fields |
|---------|--------|
| `RobotState` | `control_manager_state` (`int32`, [states](#control-manager-states)), `brake_state` (`BrakeState`: per joint of each part), `tool_flange_left`, `tool_flange_right` (`ToolFlange`), `emo_state` (emergency stop pressed), `center_of_mass` (`float64[3]`, m), `robot_stream_state` (`true` while any channel is open), `collision` |
| `ToolFlange` | `connected` (the robot gets updates from it), `output_voltage` (V: 0, 12, 24), `switch_a` |
| `PartJointStates` | `header`, and a `sensor_msgs/JointState` each for `torso`, `right_arm`, `left_arm`, `head` |

## Topics

| Topic | Type | When | Description |
|-------|------|:---:|-------------|
| `joint_overall_states` | `sensor_msgs/JointState` | always | Every joint of the robot, wheels included — the standard topic TF, MoveIt, RViz and relays read |
| `joint_states` | `rby1_msgs/PartJointStates` | always | The same split by part for monitoring and logging; no wheels (the base's motion is on `odom`) |
| `robot_state` | `rby1_msgs/RobotState` | always | Control manager state, brakes, tool flanges, emergency stop, centre of mass, stream open, collision |
| `odom` | `nav_msgs/Odometry` | always | Odometry, and TF `odom → base_footprint` |
| `battery_state` | `sensor_msgs/BatteryState` | `publish_battery_state` | Voltage, current, percentage |
| `tool_flange/<side>/imu` | `sensor_msgs/Imu` | `publish_tool_flange_state` | The flange's IMU (`left`, `right`), frame `ee_<side>` |
| `tool_flange/<side>/wrench` | `geometry_msgs/WrenchStamped` | `publish_tool_flange_state` | The flange's force/torque sensor |

In the simulator battery, force/torque and IMU read 0.

| Subscribed | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Base velocity (linear x, y and angular z). Needs the `mobile` stream channel open (`stream_control`); each message is carried out for 1.1 × 1/`stream_command_hz` (about 37 ms at 30 Hz), so the base stops by itself soon after the messages end, with the channel still open |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `robot_power` | `StateOnOff` | Power on/off. `parameters`: `all` (`.*`), or a device such as `48v` (on the simulator every name is treated as `all`) |
| `robot_servo` | `StateOnOff` | Servo on/off. `parameters`: `all`, or joint/part names. Off also cancels control and disables the control manager |
| `tool_flange_power` | `StateOnOff` | Tool flange output: `parameters` `12`/`12v` or `24`/`24v` with `state: true`; `state: false` switches it off |
| `control_manager_command` | `ControlManagerCommand` | `CMD_ENABLE` (position hold; unlimited when `control_manager_unlimited` is set), `CMD_DISABLE` (cancels control first), `CMD_RESET` (clears a fault), `CMD_UNLIMIT` (enable ignoring the range of motion) |
| `gravity_compensation` | `GravityCompensation` | On/off per part (`torso`, `right_arm`, `left_arm`) |
| `cancel_control` | `std_srvs/Trigger` | Stops every command at once **and closes every stream channel**. To stop one motion and keep the streams, cancel the action instead ([Examples — stopping](03_examples.md#two-ways-to-stop-a-command)) |
| `get_cartesian_pose` | `GetCartesianPose` | The transform of `target_link` in `ref_link`, from the SDK's model |
| `stream_control` | `StateOnOff` | Open (`state: true`) or close (`state: false`) the channels named in `parameters` — `right_arm`, `left_arm`, `torso`, `mobile`, `head`, comma-separated; empty or `all` is every channel ([Command streams](#command-streams)) |
| `set_trajectory_impedance` | `SetTrajectoryImpedance` | Joint impedance on or off for the torso and each arm, for `follow_joint_trajectory` and `stream_joint`: stiffness per joint, damping ratio and torque limit (100 N·m/rad, 1.0, 10 N·m when left out). Refused only while a trajectory runs; with a stream open the next command goes out in the new mode |
| `hardware_control` | `StateOnOff` | Claim (`true`) or release (`false`) the control right for a `ros2_control` controller manager |

### Control manager states

`RobotState.control_manager_state`, also as constants `RobotState.STATE_*`:

| Value | Constant | Meaning |
|-------|----------|---------|
| `0` | `STATE_NONE` | Not initialized or disconnected |
| `1` | `STATE_IDLE` | Disabled |
| `2` | `STATE_ENABLE` | Enabled, holding position |
| `3` | `STATE_EXECUTING` | A command is running |
| `4` | `STATE_MAJOR_FAULT` | Needs `CMD_RESET` |
| `5` | `STATE_MINOR_FAULT` | The driver resets it itself |

## Actions

| Action | Type | Description |
|--------|------|-------------|
| `robot_joint` | `Rby1JointCommand` | Whole-body joint command: each part in one goal, position or impedance per part |
| `robot_cartesian` | `Rby1CartesianCommand` | Whole-body Cartesian command: a pose per arm and for the torso |
| `follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | A joint trajectory of the torso, the arms and the head, in any combination (MoveIt, the planner, example 10) |
| `stream_joint` | `StreamJoint` | One joint command on the parts' stream channels, for a client that sends them continuously |
| `stream_cartesian` | `StreamCartesian` | One Cartesian target per arm on its channel; the driver solves the joints itself |

With no channel open, `robot_joint` and `robot_cartesian` are single commands: the robot reports their end, the result
carries its `finish_code`. **With any channel open** they go out on the parts' channels instead: the goal ends after
the command's `minimum_time` (the longest of the parts'), without the robot's word on arrival — check arrival on
`joint_overall_states` — and a part whose channel is closed is refused (`Stream: stream channel 'right_arm' is not
open …`).

## Command streams

Continuous control — `stream_joint`, `stream_cartesian`, `cmd_vel`, `follow_joint_trajectory` — goes over the
driver's command streams: **five channels**, one per part, each its own SDK command stream, so one keeps going while
another is commanded (head commands during an arm trajectory, one arm while the other moves). `stream_control` opens
and closes them by name; name the ones you mean (`right_arm, left_arm` for both arms), nothing or `all` means every
channel, when opening and when closing. Open the channels of the parts you will command before you send.

```bash
ros2 service call /rby1/stream_control rby1_msgs/srv/StateOnOff "{state: true, parameters: 'right_arm, left_arm, head'}"
ros2 service call /rby1/stream_control rby1_msgs/srv/StateOnOff "{state: true}"                          # every channel
ros2 service call /rby1/stream_control rby1_msgs/srv/StateOnOff "{state: false, parameters: 'head'}"     # the arm channels stay open
ros2 service call /rby1/stream_control rby1_msgs/srv/StateOnOff "{state: false}"                         # every channel
```

- **Opening holds the part.** The first command on a channel just opened is the driver's own: the part held where it
  is measured, with the stream's control hold time (1e6 s; in joint impedance when `set_trajectory_impedance` switched
  the part to it). The `mobile` channel gets none. The answer lists what happened: `Stream channels opened: right_arm,
  left_arm; already open: head.`, `Stream channels closed: head. Still open: right_arm, left_arm.` A channel whose
  servo is off cannot be opened; the call then fails and opens nothing.
- **A command needs its channel, and the driver does not open one for it.** With any channel open, a command for a
  part whose channel is closed is refused and the answer names the channel to open — for a trajectory, `robot_joint`
  / `robot_cartesian`, `stream_joint` / `stream_cartesian` and `cmd_vel` alike. A trajectory uses the channels of the
  parts it names: a right-arm trajectory needs `right_arm` only. With no channel open at all the stream commands are
  refused (`Stream is not active`).
- **Closes by itself only before its first command.** A channel that got no command within 60 s of being opened is
  closed again, alone, with `[STREAM TIMEOUT]`. A channel that was commanded once stays open until `stream_control`
  closes it (or `cancel_control`, a fault, power or servo off): its last command holds its part, so a client that
  pauses between commands keeps its stream.
- **One cycle, one round trip.** The body channels' commands of a cycle are sent together, so a cycle costs the
  longest of them (simulator: 3.9 ms for one channel and for both arms, median).
- **Opening right after closing.** A stream opened within about 0.1 s of closing one takes commands the robot ignores,
  so `stream_control` waits until 0.3 s have passed since a stream was last closed. A stream that was cancelled or
  found expired is replaced the same way: the next command on that channel opens the new stream 0.3 s after, waiting
  for it, so a trajectory sent right after a cancel is delayed at most 0.3 s and not cut.
- **Trajectories (`follow_joint_trajectory`)** run per part: trajectories on different parts run at the same time, one
  that names a part already running **replaces** that part's trajectory (`Preempted by a new trajectory on the same
  part`), and a cancel stops the channels of the parts it names and no other (a head tracker on `stream_joint` keeps
  running while an arm trajectory is cancelled). A trajectory is **refused, and every running trajectory stopped**,
  when it is invalid: no points, a part whose channel is closed, or a first waypoint a joint cannot reach in its time
  within its velocity limit — an invalid command means the sender's idea of the robot has gone wrong, so when in
  doubt the robot stops (the stopped parts hold their last waypoint; the result says `Stopped: a trajectory sent to
  the driver was rejected (…)`). If three waypoints in a row cannot be sent — also when the channel was closed under
  it — the goal is aborted, not reported done. A segment given 0 time gets 0.01 s.
- **Long commands under the stream.** A `robot_joint` / `robot_cartesian` command waits for its own `minimum_time`;
  nothing closes its channel meanwhile. A command that needs the stream replaced (impedance switched) waits for it up
  to 0.5 s; a `stream_joint` / `stream_cartesian` command for a part still moving under a trajectory is refused.
  Cancelling a `robot_joint` / `robot_cartesian` goal ends every stream; the head's is opened again at its next command.
- **The robot can end a Cartesian command early.** Under the stream the driver does not get the robot's own end of
  the command, and the robot sometimes ends a Cartesian command well before `minimum_time` (simulator: 4 of 895 torso
  commands; why is not known). A check for it is in the source but switched off (`[early-end 1/4]` … `[4/4]`).
- **Rate.** `stream_joint` and `stream_cartesian` keep finished goals for 2 s, so a 50 Hz client stays at 50 Hz; the
  arm follows about 0.1 s behind.
- **`stream_cartesian`.** The driver solves the joints for each target itself (the SDK's QP IK, staying near the arm's
  joints) and streams them. A step that needs more than a joint's acceleration limit within `minimum_time` is refused
  (`[STREAM REJECTED] … required acceleration … exceeds limit`) **and stops the arm** where it is measured; the
  channel stays open. Stream small steps from where the arm is. Next to the straight-wrist singularity of the ready
  pose a small hand step can still ask for a large wrist turn.
