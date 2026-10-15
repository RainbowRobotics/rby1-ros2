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
| RealTimeControlCommand (`Robot::Control`, UDP 2 ms) | whole body | planned `backend: realtime` | forward command controllers | **open** (see below) |

All modes are exercised against the fake backend. Behaviour on the robot and in the simulator core
(trajectories, gains, switching transients) still has to be validated per mode.

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

1. **RealTimeControl** (`Robot::Control`, UDP at 2 ms). It is a blocking, whole-body, exclusive loop:
   it cannot share the command stream with the other modes, so it needs a second backend selected
   with `backend: realtime`, and the driver has to enforce the robot's fault rules itself
   (|target - position| > 0.2 rad faults the control manager; single-tick jumps of about 1 degree are
   refused by the joint drivers; feedback gain 0 is pure torque). From the UPC the loop crosses the
   robot's internal network, so 2 ms jitter has to be measured with SCHED_FIFO and CPU isolation first.
2. **Robotiq tool-flange gripper.** `GripperCommandService` (GripperInitialization, GripperMove with
   position, velocity, force) exists in the robot core and the protos but not in `rb::Robot`. The
   driver would need its own gRPC stubs built from the SDK protos and linked against the same gRPC
   version the SDK uses privately, and the RPC has no position feedback for `GripperCommand` results.
3. **Mode-switch transients.** Switching SDK controllers on one stream is supported by the protocol,
   but the core's behaviour during `kSwitching` (jumps, delay) still has to be measured in the
   simulator for every pair of modes.
