# Safety

Motion on the real robot must be started by a person at the emergency stop. The driver does not
power, servo or enable the robot unless `auto_enable:=true`.

## Driver guards (per command cycle, in the real-time loop)

| Guard | Default | Why |
|---|---|---|
| Joint limits from the URDF | on | |
| Target within `max_tracking_error` of the measured position | 0.15 rad | the control manager faults (MajorFault, power off) when a target is more than 0.2 rad from the position |
| Target change per cycle `max_step` | 0.05 rad | joint drivers refuse large single-cycle reference jumps ("Invalid reference value") |
| Non-finite targets | hold previous | |
| Cartesian targets: normalized quaternion, `max_linear_step` | 0.05 m | |
| `control_hold_time` on every command | 1 s | the robot stops executing when the driver stops sending |
| Stale state (`state_timeout`), control manager fault, emergency stop | 0.1 s | stop streaming, hardware error |

The robot manager refuses power or servo off while a command is executing (unless
`allow_power_off_while_executing`), and jog while a command executes.

## Facts measured on an RB-Y1 Model A (2026-10)

- A 0.074 rad single-tick target jump made the joint driver raise an input error and the control
  manager a major fault; a 1 degree single-tick jump was refused, 0.5 degree accepted.
- In joint impedance mode the robot realizes 1.3-1.45x the commanded stiffness (1.06x wrist roll).
- The upper body has structural resonances (torso 2.5-2.75 Hz, upper body 9-11 Hz): avoid
  oscillating targets near these frequencies. A torso impedance *step* oscillated at about 16 Hz and
  faulted; use smooth references for the torso.
- Joint torque limits seen as driver saturation: shoulder 136.5 N·m, elbow 78 N·m, wrist roll >= 44 N·m
  (the URDF effort limits are lower and are not the saturation).
