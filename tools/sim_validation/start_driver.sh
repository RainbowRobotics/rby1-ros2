#!/bin/bash
# Start the driver against the simulator core in the background; log to /tmp/driver.log.
source /opt/ros/$ROS_DISTRO/setup.bash; source /ws/install_$ROS_DISTRO/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-90}
pkill -f ros2_control_node; pkill -f "ros2 launch"; sleep 1
nohup ros2 launch rby1_bringup rby1_sim.launch.py model:=a version:=1.2 "$@" > /tmp/driver.log 2>&1 &
echo $! > /tmp/driver.pid
for i in $(seq 1 60); do
  grep -q "RB-Y1 hardware active" /tmp/driver.log && break
  grep -qE "process has died|Failed to connect|not ready" /tmp/driver.log && break
  sleep 1
done
sleep 8
grep -E "rby1_system_interface|ERROR|Detected" /tmp/driver.log | head -20
