# Hardware Interface (`rby1_hardware`)

[Overview](01_overview.md) · [Driver](02_driver.md) · [Examples](03_examples.md) · [Description](04_description.md) · [Hardware Interface](05_hardware_interface.md) · [MoveIt](06_moveit.md) · [Tools](07_tools.md) · [UPC Topic Bridge](08_upc_topic_bridge.md)

`rby1_hardware/RBY1SystemHardware` is a `ros2_control` `SystemInterface` plugin: a controller manager
(`ros2_control_node`) loads it from the `<ros2_control>` block of the robot's URDF and runs the standard controllers
on it — `joint_trajectory_controller`s per part and a `joint_state_broadcaster`. MoveIt uses it through the
`rby1_moveit_*` `demo.launch.py` ([MoveIt](06_moveit.md#demolaunchpy)); it also works without MoveIt
([below](#using-it-without-moveit)).

## How it is built

- **Two connections to the robot.** The plugin opens its own gRPC connection to the robot for the command streams,
  and talks to the running **driver** for the rest: on activation it calls `/rby1/robot_power` and `/rby1/robot_servo`
  (`power_on`, `servo_on` parameters; default `all`) and claims the **control right** with `/rby1/hardware_control
  state: true`. While it holds the right the driver refuses its own command sources (actions, streams, the examples)
  and keeps publishing the state; on deactivation the plugin cancels its control, closes its streams, waits for the
  robot to go idle, disables the control manager, disconnects and releases the right (`state: false`). The claim is
  asked up to three times (an answer was once lost while the client was still being matched).
- **Streams.** Two SDK command streams: one for the upper body (torso, arms, head, sent as one component-based
  command) and one for the mobile base. An I/O thread sends the latest commanded positions of every joint it maps,
  at the controller manager's rate (100 Hz in the shipped `ros2_controllers.yaml`), with the velocity and acceleration
  limits of the `<ros2_control>` parameters when given.
- **Joints.** Each URDF joint is mapped to an SDK joint by name. A joint the SDK does not have — the finger joints of
  a v1.3 model — is left unmapped: not sent, and its state reads back whatever was commanded (the gripper controller
  is what moves those; [Tools — Gripper](07_tools.md#gripper)). One position command interface and position and
  velocity state interfaces per joint. `read()` sets every command to the measured position each cycle, so a
  controller that is not active leaves its joints where they are.
- **State.** The plugin reads positions and velocities from the robot itself; it also subscribes to the driver's
  `/rby1/joint_overall_states`.
- **The controllers.** The `ros2_controllers.yaml` of each MoveIt package: `right_arm_controller`,
  `left_arm_controller`, `torso_controller`, `head_controller`, `both_arms_controller`, `body_controller` (v1.3 also
  `gripper_r_controller`, `gripper_l_controller`), all `JointTrajectoryController`s with a position command interface,
  and `joint_state_broadcaster`. Two active controllers cannot share a joint, so `both_arms_controller` and
  `right_arm_controller` do not run together; `ros2 control switch_controllers` changes the set at run time.
  Controllers on different parts run at the same time: a trajectory to the right arm while the left arm's runs.

`<ros2_control>` parameters of the plugin, set by the xacro (`RBY1_<M|A>_v<version>.ros2_control.xacro`) from the
launch arguments:

| Parameter | Default | Meaning |
|---|---|---|
| `robot_ip` | `127.0.0.1:50051` | The robot, the same one the driver is connected to |
| `model` | `m` | `a` or `m` |
| `driver_namespace` | `rby1` | Namespace of the running driver (its services and `joint_overall_states`) |
| `power_on`, `servo_on` | `all` | What the plugin asks the driver to switch on at activation |
| `collision_check_enable`, `collision_threshold` | `false`, `0.01` | Read by the plugin; the driver's own collision check is the one that warns |

> [!WARNING]
> The plugin cannot verify the robot's version. A `rby1_moveit_*` package of another version than the robot gives
> MoveIt and the robot different kinematics; **match the package to the robot** before running on real hardware.

Build: `RBY1_SDK_PATH` must point at the built SDK ([Overview — Install](01_overview.md#install-and-build)); the
CMake stops with a message otherwise.

## Using it without MoveIt

`rby1_hardware.launch.py` starts the controller manager with the robot's URDF, `robot_state_publisher`, the
`joint_state_broadcaster` and the trajectory controllers — nothing of MoveIt. The driver must be running.

```bash
# terminal 1
ros2 launch rby1_driver rby1_ros2_driver.launch.py
# terminal 2
ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2
ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2 robot_ip:=192.168.30.1:50051   # a robot
ros2 launch rby1_hardware rby1_hardware.launch.py model:=m version:=1.2 use_fake_hardware:=true        # no robot, no driver (mock hardware)
```

**Which robot** it is for is named on the command line, `model:=m version:=1.2` (both required): they pick
`rby1_moveit_<model>_<version>`, whose URDF (with the `<ros2_control>` block) and `config/ros2_controllers.yaml` are
used — only its files. The plugin compares them with the connected robot when it activates and refuses if they differ
(it then returns the control right to the driver). Which controllers start is set at the top of the launch file:

| Line | Meaning |
|---|---|
| `CONTROLLERS` | the controllers started active (default: `right_arm_controller`, `left_arm_controller`, `torso_controller`, `head_controller`) |

| Argument | Default | Meaning |
|---|---|---|
| `robot_ip` | `127.0.0.1:50051` | Robot address and gRPC port |
| `driver_namespace` | `rby1` | Namespace of the running driver |
| `use_fake_hardware` | `false` | `true`: `mock_components/GenericSystem` instead of the robot |
| `model`, `version` | required | Robot kind (`a`, `m`) and version (`1.2`) |

From then on it is plain `ros2_control`: joint states on `/joint_states`, a trajectory to
`/<controller>/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) or
`/<controller>/joint_trajectory`, `ros2 control list_controllers`,
`ros2 control switch_controllers --deactivate right_arm_controller --activate both_arms_controller`. Ctrl+C on the
launch deactivates the plugin and returns the control right to the driver. Not together with a `demo.launch.py` (one
controller manager per robot). In `rby1_bringup` this is the `ROS2_CONTROL` flag.

```bash
ros2 action send_goal /right_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [right_arm_0, right_arm_1, right_arm_2, right_arm_3, right_arm_4, right_arm_5, right_arm_6],
     points: [{positions: [0.0, -0.1, 0.0, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

Simulator (2026-10-30): the launch started and stopped three times in a row, each time with the control right
claimed and released; trajectories to each arm, the head and the torso through their controllers all reached their
targets. Not run on a real robot.

## Real-time scheduling

`ros2_control_node` sets `SCHED_FIFO` priority 50 for itself at start when the user may (`realtime_tools`), and
otherwise warns and runs with normal scheduling. To allow it, give the user real-time rights and log in again:

```bash
sudo bash -c 'echo "$USER - rtprio 99" > /etc/security/limits.d/99-realtime.conf; echo "$USER - memlock unlimited" >> /etc/security/limits.d/99-realtime.conf'
```

The log then says `Successful set up FIFO RT scheduling policy with priority 50`. No `chrt` prefix is needed — the
launch files had one until 2026-10-30, which made the node fail to start without rights instead of warning.
