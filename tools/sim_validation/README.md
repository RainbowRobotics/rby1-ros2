# Simulator validation

Exercises every controller of the driver against an rby1 simulator core (the MuJoCo image
`rainbowroboticsofficial/rby1-sim:<ver>-a_v1.2` or `rby1-sim-isaac`) and prints one `RESULT` JSON
line per test.

```bash
# simulator core on 127.0.0.1:50051, e.g. MuJoCo (needs an X display for its viewer)
docker run -d --rm --name rby1-sim-mujoco --network host --gpus all -e DISPLAY=:1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2

python3 reset_sim.py                         # cancel leftover controls, clear faults (rby1-sdk Python)
ros2 launch rby1_bringup rby1_sim.launch.py model:=a        # add realtime:=true for the realtime test
python3 validate.py ready jtc stream impedance gravity base group \
                    cartesian cartesian_impedance impedance_pose optimal switch stop
```

`run.sh` and `start_driver.sh` wrap these steps for the development containers (`rby1-dev-<distro>`).
Run `ready` before the task-space tests: from the zero pose the arms are singular and the
simulator refuses Cartesian commands (control manager MinorFault).

The simulator images set `RBY1_SKIP_CONTROL_TIMEOUT=1`: a control whose client died is not ended by
its hold time there, so run `reset_sim.py` after killing a driver.
