# RB-Y1 ROS 2 Control Driver

`ros2_control` driver for Rainbow Robotics RB-Y1 (models A, M and UB) on ROS 2 Humble and Jazzy.

- Every rby1-sdk motion controller is available as a `ros2_control` command mode: joint position,
  joint group position, joint impedance, gravity compensation, joint and SE2 velocity, Cartesian,
  Cartesian impedance, impedance, optimal control and real-time control (`realtime:=true`)
  ([docs/SDK_COVERAGE.md](docs/SDK_COVERAGE.md)).
- The hardware picks the SDK controller of each body part (torso, right arm, left arm, head,
  mobile base, whole body) from the interfaces the active controller claims, so standard controllers
  (`joint_trajectory_controller`, `forward_command_controller`, `GripperActionController`) and the
  RB-Y1 controllers in `rby1_controllers` work side by side and can be switched at runtime.
- Compatible with the Isaac ROS gear-assembly inference nodes
  (`isaac_ros_manipulation_dnn_policy`): `/joint_states`, a joint impedance controller taking
  `sensor_msgs/JointState` targets, `switch_controller` and `GripperCommand`
  ([docs/ISAAC_ROS.md](docs/ISAAC_ROS.md)).
- RB gripper (Dynamixel on the UPC serial port) as a second hardware component.
- Runs natively or in Docker on the UPC (Jetson Orin, Ubuntu 22.04) or on any PC on the robot network.

## Packages

| Package | Contents |
|---|---|
| `rby1_hardware_interface` | `Rby1SystemInterface` plugin, command modes, SDK and fake backends, `rby1_probe` |
| `rby1_controllers` | `JointStreamController`, `GravityCompensationController`, `BaseVelocityController`, `TaskSpaceController` |
| `rby1_gripper_hardware` | `Rby1GripperSystem` plugin (RB gripper, fake) |
| `rby1_robot_manager` | Non-real-time services: power, servo, brakes, control manager, stop, jog, time scale, LED, tool flange, parameters, logs |
| `rby1_description` | URDF/xacro for A (v1.0-1.2), M (v1.0-1.3) and UB with the `ros2_control` blocks |
| `rby1_bringup` | Launch files and controller configurations |
| `rby1_msgs` | Status messages and manager services |

## Build

The hardware interface needs the rby1-sdk C++ library (the `rby1-sdk` pip wheel has no C++ headers).

### Native (UPC or development PC)

```bash
mkdir -p ~/rby1_ws/src && cd ~/rby1_ws/src
git clone -b feature/ros2_control_commands https://github.com/RainbowRobotics/rby1-ros2.git
./rby1-ros2/scripts/install_rby1_sdk.sh            # /opt/rby1-sdk, or ~/.local/rby1-sdk without sudo
cd ~/rby1_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

`CMAKE_PREFIX_PATH` or `-DRBY1_SDK_ROOT=<prefix>` select another SDK install.

### Docker

```bash
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t rby1-ros2:humble .
docker/run_upc.sh -- model:=a gripper:=true          # on the UPC
```

See [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) for the UPC, real-time settings and the gripper udev rule.

## Run

```bash
# Without a robot: the real hardware interface against an in-memory robot
ros2 launch rby1_bringup rby1_mock.launch.py model:=a gripper:=true

# rby1 simulator core (rby1-sim-isaac or the MuJoCo simulator image) on 127.0.0.1:50051
ros2 launch rby1_bringup rby1_sim.launch.py model:=a

# Robot (model and version detected with rby1_probe)
ros2 launch rby1_bringup rby1_control.launch.py robot_ip:=192.168.30.1:50051 gripper:=true
```

Only `joint_state_broadcaster` is active after start-up; every other controller is loaded inactive.
Nothing is commanded until a controller that claims command interfaces is activated. Activate
controllers at start-up with `activate_controllers:="right_arm_controller left_arm_controller"` or
switch them at runtime:

```bash
ros2 control switch_controllers --activate right_arm_impedance_controller
ros2 topic pub /right_arm_impedance_controller/joint_commands sensor_msgs/msg/JointState \
  "{name: [right_arm_0, right_arm_1, right_arm_2, right_arm_3, right_arm_4, right_arm_5, right_arm_6],
    position: [0.0, -0.2, 0.0, -0.5, 0.0, 0.3, 0.0]}"
```

Before activation the robot must be powered, servoed and its control manager enabled (robot manager
services, or `auto_enable:=true`). See [docs/SAFETY.md](docs/SAFETY.md) before moving the robot.

## Controllers

| Controller (per part) | Type | SDK controller |
|---|---|---|
| `<part>_controller` | `joint_trajectory_controller/JointTrajectoryController` | JointPositionCommand |
| `<arm>_position_stream_controller` | `rby1_controllers/JointStreamController` (position) | JointPositionCommand |
| `<arm>_impedance_controller`, `torso_impedance_controller` | `rby1_controllers/JointStreamController` (impedance) | JointImpedanceControlCommand |
| `<part>_gravity_compensation_controller` | `rby1_controllers/GravityCompensationController` | GravityCompensationCommand |
| `base_velocity_controller` | `rby1_controllers/BaseVelocityController` | SE2VelocityCommand |
| `wheel_velocity_controller` | `velocity_controllers/JointGroupVelocityController` | JointVelocityCommand |
| `<part>_cartesian_controller` | `rby1_controllers/TaskSpaceController` | CartesianCommand |
| `<part>_cartesian_impedance_controller` | `rby1_controllers/TaskSpaceController` | CartesianImpedanceControlCommand |
| `<part>_impedance_pose_controller` | `rby1_controllers/TaskSpaceController` | ImpedanceControlCommand |
| `<part>_optimal_control_controller` | `rby1_controllers/TaskSpaceController` | OptimalControlCommand |
| `body_realtime_controller` (`realtime:=true`) | `rby1_controllers/JointStreamController` (realtime) | RealTimeControl (`Robot::Control`) |
| `<side>_gripper_controller` | `position_controllers/GripperActionController` | RB gripper (Dynamixel) |

Stop and jog are whole-robot commands and live in the robot manager (`/rby1/stop`, `/rby1/jog`).

## Documentation

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): how the hardware interface works
- [docs/SDK_COVERAGE.md](docs/SDK_COVERAGE.md): every rby1-sdk controller and API, and where it is exposed
- [docs/ISAAC_ROS.md](docs/ISAAC_ROS.md): running the Isaac ROS gear-assembly policy on RB-Y1
- [docs/SAFETY.md](docs/SAFETY.md): guards, limits and the real-robot facts behind them
- [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md): UPC native and Docker deployment
- [docs/ADDING_A_MODE.md](docs/ADDING_A_MODE.md): adding a new SDK controller

## Tests

```bash
colcon test --packages-select rby1_hardware_interface rby1_controllers rby1_gripper_hardware rby1_bringup
colcon test-result --verbose
```

The tests run without a robot: command-mode selection and guards, the system interface against the
fake backend, the gripper with a fake bus, plugin loading, and a launch test of the Isaac ROS contract.
