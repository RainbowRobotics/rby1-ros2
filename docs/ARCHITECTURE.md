# Architecture

```
 controllers (JTC, JointStream, TaskSpace, ...)          rby1_robot_manager (services)
        │ command/state interfaces                               │ own gRPC client
        ▼                                                        ▼
 Rby1SystemInterface ── read(): copy latest state ◄── state stream thread (StartStateUpdate)
        │ write(): CommandMode::condition() per part (RT, no allocation)
        ▼
   command slot (mutex, latest wins)
        │
 stream thread @ command_rate: CommandMode::build() ─► RobotCommandRequest (SDK-free)
        │
 RobotBackend::send() ─► SdkRobotBackend: rby1-sdk builders ─► CommandStream::SendCommand
                     └─► FakeRobotBackend (tests, backend:=fake)
```

## Components and modes

The rby1-sdk command tree is `ComponentBased{Mobility, Body{Torso, RightArm, LeftArm} | Body, Head}`.
The hardware keeps one active `CommandMode` per component (`torso`, `right_arm`, `left_arm`, `head`,
`mobility`, `body`). On every controller switch it groups the claimed command interfaces by
component and selects the single mode whose interface signature matches:

| Claimed interfaces of one component | Mode |
|---|---|
| `<joint>/position` of every joint | joint_position |
| `<joint>/position` of some torso joints | joint_group_position |
| `<joint>/{position,stiffness,damping_ratio}` (+`torque_limit`) of every joint | joint_impedance |
| `<wheel>/velocity` of every wheel | joint_velocity |
| `<part>_gravity_compensation/enable` | gravity_compensation |
| `base_se2/{linear_x,linear_y,angular_z}` | se2_velocity |
| `<part>_cartesian/t<i>.{x,y,z,qx,qy,qz,qw}` (+ limits) | cartesian |
| `<part>_cartesian_impedance/...` | cartesian_impedance |
| `<part>_impedance/t0.*` | impedance |
| `<part>_optimal_control/...` (+ `com.*`) | optimal_control |

A claim that matches no mode, or a partial claim of an arm, is rejected (the controller does not
activate). A `body` mode excludes torso and arm modes.

GPIO blocks carry `kind` and `component` parameters, plus the static part of the SDK command
(reference and target links, gains, limits); the per-cycle targets are command interfaces.

## Streaming

- The stream thread starts sending when the first command-claiming controller activates and keeps
  sending until the hardware is deactivated.
- Torso, arms and head without an active mode get a joint position hold at the position they had
  when their last mode ended (or at activation). The mobile base is never held.
- Every command carries `control_hold_time` (default 1 s): if the driver stops sending, the robot
  stops executing within that time.
- A finished or expired stream is re-opened on the next cycle; repeated failures, a control
  manager fault, an emergency stop or stale state (`state_timeout`) stop streaming and make
  `read()`/`write()` return ERROR, which the controller manager turns into deactivating the hardware.

## Parameters (`<hardware>` in the URDF)

| Parameter | Default | Meaning |
|---|---|---|
| `model` | `a` | a, m or ub |
| `robot_ip` | `192.168.30.1:50051` | gRPC address |
| `backend` | `sdk` | sdk or fake |
| `auto_enable` | `false` | power on, servo on, enable the control manager on activation |
| `state_rate` / `command_rate` | 200 / 100 Hz | SDK state stream / command stream rates |
| `state_timeout` | 0.1 s | stale-state limit while streaming |
| `stream_priority` | 1 | command stream priority |
| `minimum_time`, `control_hold_time`, `acceleration_limit`, `velocity_limit_scale`, `velocity_limit` | 0.1 s, 1 s, 2 rad/s², 0.5 x URDF, — | joint command timing and limits; prefix with a mode name (`joint_impedance.minimum_time`) to override per mode |
| `max_tracking_error`, `max_step` | 0.15 rad, 0.05 rad | guards (see SAFETY.md) |
| `joint_impedance.<part>.stiffness`, `.torque_limit`, `damping_ratio` | — | defaults when a controller leaves them NaN |
| `robot_parameter.<name>` | — | robot parameter set on activation (never persisted) |
