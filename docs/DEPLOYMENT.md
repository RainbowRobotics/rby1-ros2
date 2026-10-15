# Deployment on the UPC

The RB gripper is on the UPC's serial port, so the controller manager runs on the UPC (Jetson Orin,
Ubuntu 22.04, ROS 2 Humble). It reaches the robot's control PC at `192.168.30.1:50051`.

## Native

```bash
sudo apt install ros-humble-ros-base python3-rosdep python3-colcon-common-extensions
mkdir -p ~/rby1_ws/src && cd ~/rby1_ws/src
git clone -b feature/ros2_control_commands https://github.com/RainbowRobotics/rby1-ros2.git
./rby1-ros2/scripts/install_rby1_sdk.sh
cd ~/rby1_ws && source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Docker

```bash
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t rby1-ros2:humble .
docker/run_upc.sh -- model:=a gripper:=true
```

`run_upc.sh` uses host networking, passes `/dev/rby1_gripper` and grants `SYS_NICE` and `rtprio` so the
controller manager can use SCHED_FIFO.

## Gripper serial latency

The SDK sets the USB serial latency timer to 1 ms through sysfs, which needs root and is not possible
inside Docker. Install the udev rule once instead:

```bash
sudo cp config/udev/99-rby1-gripper.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

## Real-time

For the controller manager's SCHED_FIFO on a native install add the user to a group with
`rtprio 99` in `/etc/security/limits.conf`. Put the Orin in its maximum power mode (`nvpmodel`,
`jetson_clocks`) when using high update rates.

## Clocks

Isaac ROS checks the age of `/joint_states` stamps. If the inference runs on another computer,
synchronize the clocks (chrony) or raise `joint_state_age_threshold_ms`.
