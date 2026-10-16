# rby1-sdk coverage

Checked against rby1-sdk v0.10.0 (`robot_command_builder.h`, `protos/rb/api/*`, `rb::Robot`).

## Motion controllers

| SDK controller | Components | Driver | Controller | Status |
|---|---|---|---|---|
| JointPositionCommand | torso, arms, head, body | command mode `joint_position` | JTC, `JointStreamController` | done |
| JointGroupPositionCommand | torso | `joint_group_position` | JTC on a subset of torso joints | done |
| JointImpedanceControlCommand | torso, arms, body | `joint_impedance` | `JointStreamController` (impedance) | done |
| GravityCompensationCommand | torso, arms, body | `gravity_compensation` | `GravityCompensationController` | done |
| JointVelocityCommand | mobility | `joint_velocity` | `JointGroupVelocityController` | done |
| SE2VelocityCommand | mobility | `se2_velocity` | `BaseVelocityController` | done |
| CartesianCommand | torso, arms, body | `cartesian` | `TaskSpaceController` | done |
| CartesianImpedanceControlCommand | torso, arms, body | `cartesian_impedance` | `TaskSpaceController` | done |
| ImpedanceControlCommand | torso, arms | `impedance` | `TaskSpaceController` | done |
| OptimalControlCommand | torso, body | `optimal_control` | `TaskSpaceController` | done |
| StopCommand | whole body | robot manager `~/stop` | — | done |
| JogCommand | one joint | robot manager `~/jog` | — | done |
| RealTimeControlCommand (`Robot::Control`, UDP 2 ms) | torso, arms, head (exclusive) | mode `realtime` (real-time loop instead of the stream) | `JointStreamController` (realtime), `realtime:=true` | done, validated in both simulators; **robot validation open** |

All modes run against the fake backend in the unit tests and were validated in the MuJoCo and Isaac Sim
simulator cores ([VALIDATION.md](VALIDATION.md)); behaviour on the real robot is still to be validated.

## Gripper

| Device | SDK | Driver | Status |
|---|---|---|---|
| RB gripper (2 Dynamixel, UPC serial) | `rb::DynamixelBus`, `rb::upc` | `rby1_gripper_hardware` backend `rb_dynamixel` | done (fake tested; hardware to validate) |
| Robotiq on the tool flange | proto `GripperCommandService` only | backend `robotiq_flange` | **open** (see below) |

## Management API (robot manager)

Done: power, servo, brake engage/release, control manager enable (with unlimited mode) / disable /
reset fault, cancel control, time scale, tool flange voltage and digital outputs, LED, robot
parameters (get/set, `write_db` explicit), odometry reset, fault log list and last logs, gravity
compensation (one-shot). Maintenance services (`enable_maintenance_services:=true`): position PID
gains, home offset reset, preset position.

Not exposed (robot administration, not driver functions): Wi-Fi and network settings, system time,
battery configuration, robot model import, file download, gamepad upload, serial streams, factory
reset of parameters. The leader arm is a teleoperation device, not a robot controller; it can feed
`JointStreamController` from its own node.

## Open items

1. **RealTimeControl** (`Robot::Control`, UDP at 2 ms). Implemented as the `realtime` mode: when a
   controller claims `position`, `effort` and `feedback_gain` of every torso and arm joint, the
   hardware closes the command stream and runs the SDK real-time loop on a backend thread; on
   deactivation the loop finishes (the robot holds) and the stream resumes. The loop enforces the
   robot's fault rules itself: at most `realtime.max_target_step` (0.004 rad) per 2 ms tick, targets
   within `realtime.max_tracking_error` (0.15 rad) of the position, feedback gain 0-10, feedforward
   torque within the URDF effort, wheels at zero velocity, finish after `realtime.stale_ticks` (25)
   ticks without new state. Tested against the fake backend only. Still open: the loop's timing from
   the UPC across the robot's internal network (2 ms budget; measure with SCHED_FIFO and CPU isolation)
   and behaviour in the simulator core and on the robot. Feedback gain 0 is pure torque on the robot.
2. **Robotiq tool-flange gripper.** `GripperCommandService` (GripperInitialization, GripperMove with
   position, velocity, force) exists in the robot core and the protos but not in `rb::Robot`. The
   driver would need its own gRPC stubs built from the SDK protos and linked against the same gRPC
   version the SDK uses privately, and the RPC has no position feedback for `GripperCommand` results.
3. **Mode switches.** The robot fixes each component's controller type per stream, so the driver
   restarts the stream on a change; measured in both simulators the joints move less than 0.5 mrad.
