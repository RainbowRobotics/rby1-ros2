# RBY1 ROS 2 Driver Package

> [!CAUTION]
> ## The current driver is in beta. For safe use, please test the features in a simulation first.

Please click the link below for more details.

[RBY1 ROS 2 Driver Documentation](https://rainbowrobotics.github.io/rby1-dev/ros2/ros2_driver.html)

## Overview

`rby1_ros2` is a unified ROS 2 driver package for controlling the Rainbow Robotics RBY1 robot.  
It wraps the RBY1 C++ SDK into a ROS 2 node, providing state monitoring and multiple control modes (Joint Position, Cartesian Position, Impedance, Gravity Compensation, and Trajectory Streaming) through a clean action/service/topic interface.

- **ROS 2 version**: Humble
- **OS**: Ubuntu 22.04
- **SDK compatibility**: rby1-sdk `0.10.x` and later

### System Architecture Overview

![System Architecture](Doc/img/system_architecture.png)

The system operates using two primary pipelines to interface with the robot:
1. **Direct Control Mode (`rby1_ros2_driver`)**: A standalone C++ ROS 2 node that communicates directly with the RBY1 robot or simulator via gRPC. User applications/scripts send standard ROS 2 topics, services, and actions (e.g. `robot_joint`, `robot_cartesian`) to command movements and monitor status.
2. **MoveIt 2 Integration (`rby1_hardware`)**: Bridges MoveIt 2 and `ros2_control` to the robot via a custom `RBY1SystemHardware` interface plugin. The hardware plugin streams command inputs to the robot via a direct gRPC connection, while querying status and coordinating power, servo states, and control rights with the `rby1_ros2_driver` node via internal ROS 2 service calls (like `/hardware_control`).

---

## 1. Quick Start

- **If you install in an environment such as conda or miniforge, issues may arise due to Python and CMake path conflicts, so please install it in a local environment.**

### 1-1. Install ROS 2 Humble

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

### 1-2. Install RB-Y1 SDK

<https://github.com/RainbowRobotics/rby1-sdk>

### 1-3. Install RBY1 Simulator (Docker)

<https://hub.docker.com/r/rainbowroboticsofficial/rby1-sim>

### 1-4. Install MoveIt 2(optional)

> [!CAUTION]
> if you'll not use MoveIt, pleasse delete moveit package(rby1_moveit folder) in workspace.

#### Option A(binary install. recommended in robot UPC)

- it can use robot's UPC(jetson).

```bash
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-visual-tools ros-humble-interactive-markers

# install check
source /opt/ros/humble/setup.bash
ros2 pkg list | grep moveit
```
#### Option B(install package)
- Please proceed up to  `~ Optional: add the previous command to your .bashrc`

<https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html>

#### Install additional tool

```bash
sudo apt install ros-humble-gripper-controllers
sudo apt install ros-humble-joint-trajectory-controller
```

### 1-5. Environment Setup

Add the following lines to `~/.bashrc`:

```bash
sudo nano ~/.bashrc

# Add at the bottom:
export PATH=/opt/cmake/bin:$PATH
source /opt/ros/humble/setup.bash

# Apply changes
source ~/.bashrc
```

### 1-5. Build

```bash
mkdir -p rby1_ros2_ws/src
cd rby1_ros2_ws/src
git clone https://github.com/RainbowRobotics/rby1-ros2.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 1-6. Configure `driver_parameters.yaml`

Located at `rby1_driver/config/driver_parameters.yaml`.  
Edit this file to match your robot before launching the driver.  
Because the workspace was built with `--symlink-install`, **no rebuild is needed** after editing.

> [!IMPORTANT]
> If you use simulation for testing, keep `robot_ip: "127.0.0.1:50051"`.  
> Some state values (battery, tool flange FT/IMU) will show zeros in simulation because no physical sensors are attached.

- Main Parameters(for the detail of driver_parameters.yaml, see config/driver_parametersyaml)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `robot_ip` | `"127.0.0.1:50051"` | - | Robot IP address and gRPC port |
| `model` | `"m"` | - | Robot model — `"a"` (RBY1-A) or `"m"` (RBY1-M) |
| `get_state_period` | `0.01` | s | State publish interval — default 0.01(100 Hz) |
| `publish_battery_state` | `true` | - | Enable battery state topic |
| `publish_tool_flange_state` | `true` | - | Enable tool flange state topics (left + right) |

---

> [!NOTE]
> **`get_state_period` and communication frequency:**  
> `get_state_period` sets the interval (in seconds) at which the driver reads the robot state via `GetState()` and publishes all state topics.  
> Actual throughput may be slightly lower (97–100 Hz) depending on PC environment and CPU load.

![get_state_period_1](Doc/img/topic_hz.png)

### 1-7. Run Simulator (optional)

If you do not have a physical robot, run the Docker simulator.  
The robot IP in this case is `"127.0.0.1:50051"` or `"localhost:50051"`.  
Change the tag at the end to select a model/version (e.g. `a_v1.2`, `m_v1.3`).

```bash
# Example: Model A,  v1.2
sudo docker run --rm \
  -e DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 50051:50051 \
  rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2
```

> [!IMPORTANT]
> ## Model `a` only supports  up to v1.2. Model `m` supports v1.0–v1.3.

---

### 1-8. Launch the Driver

```bash
# In your workspace root
source install/setup.bash

ros2 launch rby1_driver rby1_ros2_driver.launch.py

```

## Launch or Run

### Examples
Each example can be run in a **separate terminal** while the driver is active:
```bash
source install/setup.bash
ros2 run rby1_examples <example_name>
# ex ) ros2 run rby1_examples 01_power_control
```

### Visualization & Robot Description
You can use the robot's basic TF structure and state publisher through the commands below. When implementing features related to rby1, please use the model files from the corresponding package.

```bash
source install/setup.bash
ros2 launch rby1_description rby1_state_publisher.launch.py model:=a version:=1_1
```

### MoveIt 2

```bash
# open another terminal
source install/setup.bash

# Real hardware (default: use_fake_hardware:=false)
ros2 launch rby1_moveit_m_1_2 demo.launch.py

# With a custom robot IP
ros2 launch rby1_moveit_m_1_2 demo.launch.py robot_ip:=192.168.30.1:50051

# Fake hardware / simulation (no real robot required)
ros2 launch rby1_moveit_m_1_2 demo.launch.py use_fake_hardware:=true
```
