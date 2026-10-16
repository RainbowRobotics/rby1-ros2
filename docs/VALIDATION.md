# Simulator validation (2026-10-17)

Every controller was run against both rby1 simulator cores with `tools/sim_validation`, on Humble and
on Jazzy (same results within noise). Simulators:

- **MuJoCo**: `rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2` (core 0.10.6)
- **Isaac Sim**: `rainbowroboticsofficial/rby1-sim-isaac:0.10.7-a_v1.2-isaac6.1.0` with the upstream
  `rby1-sim-isaac` sources (commit 2417a2b), headless, real-time factor about 0.75

| Test (model A) | MuJoCo | Isaac Sim |
|---|---|---|
| `/joint_states` | 100 Hz, delay 1-7 ms | 100 Hz, delay 2-8 ms |
| JTC arms / torso / head (2 s goals) | error ≤ 2e-4 rad (head settles ~1 s later) | error ≤ 1.4e-3 rad |
| Position stream (`JointStreamController` position, 0.3 rad step) | error 9e-6 rad, 90 % in 0.71 s | error 8e-5 rad, 90 % in 1.0 s |
| Joint impedance, right arm (0.3 rad step) | error 1e-4 rad, overshoot 9-10 % | error 1e-4 rad, no overshoot |
| Joint impedance, torso (0.05 rad step) | error 4e-4 rad | error 1.8e-3 rad |
| Gravity compensation (3 s) | drift 5 mrad | drift 0.1 mrad |
| SE2 base velocity (0.1 m/s, 0.2 rad/s, 3 s) | 0.19 m | 0.24 m |
| Wheel velocity | wheels turn | wheels turn |
| Joint group position (torso_4/5) | error 8.5e-5 rad | error 5e-4 rad |
| Cartesian (+5 cm) | error 0.14 mm | error 0.25-0.32 mm |
| Cartesian impedance (+5 cm) | error 0.5-0.8 mm | error 14-16 mm (compliant, default joint stiffness 80) |
| Impedance (single link, +5 cm) | error 5.5 mm | error 14-16 mm (compliant) |
| Optimal control (hold torso + both hands + CoM) | within 0.16 mm | within 0.2 mm |
| Real-time control (UDP 2 ms, 0.3 rad step) | error 9e-5 rad, 90 % in 0.18 s | error 4.6e-4 rad, 90 % in 0.23 s |
| Mode switches on a still arm (position/impedance/stream/gravity/hold) | ≤ 0.5 mrad (gravity: 3 mrad drift) | ≤ 0.04 mrad |
| Robot manager stop while streaming | driver stops, hardware unconfigured | same |

## Driver defects found and fixed during validation

1. The robot fixes each component's controller type when a stream's control starts; later commands
   of another type were ignored (impedance, gravity compensation, base after a position command).
   The driver now restarts the stream when the command structure changes.
2. A robot manager stop was undone within one cycle because the driver re-opened its stream. A
   stream ended by another command now stops the driver (`on_stream_preempted`).
3. `auto_enable` sent servo-on before power had come up (Isaac Sim), and powered up with the
   control manager enabled, which faults it. The driver now disables the control manager first and
   waits for power and servo (`enable_timeout`).
4. The Isaac Sim core pauses its state stream for about 0.1 s when it starts a Cartesian impedance
   control; the default `state_timeout` is now 0.25 s.

## Simulator behaviour to know

- From the zero pose the arms are singular: Cartesian commands make the control manager raise a
  MinorFault ("Singularity"). Start task-space control from a bent posture (`validate.py ready`).
- The images set `RBY1_SKIP_CONTROL_TIMEOUT=1`: a dead client's control is not ended by its hold time,
  and the next stream with the same priority is rejected. `tools/sim_validation/reset_sim.py` cancels it.
- On Humble a hardware error leaves the controllers listed as active although the hardware is
  unconfigured; reactivate the hardware with `ros2 control set_hardware_component_state`.

## Not covered by the simulators

The RB gripper (Dynamixel on the UPC serial port) and the Robotiq flange gripper; timing on the UPC
(Jetson Orin) with SCHED_FIFO; behaviour of the real joint drivers.
