#!/usr/bin/env bash
# Run the RB-Y1 driver container on the UPC (Jetson Orin) or any host on the robot network.
#
#   docker/run_upc.sh [--image rby1-ros2:humble] [--no-gripper] -- <launch arguments>
#   docker/run_upc.sh -- model:=a gripper:=true activate_controllers:="right_arm_controller"
#
# Host networking (DDS + gRPC to the control PC), the gripper serial device, and the
# capabilities the controller manager needs for SCHED_FIFO.
set -euo pipefail

IMAGE="rby1-ros2:humble"
GRIPPER_DEVICE="/dev/rby1_gripper"
USE_GRIPPER=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) IMAGE="$2"; shift 2 ;;
    --gripper-device) GRIPPER_DEVICE="$2"; shift 2 ;;
    --no-gripper) USE_GRIPPER=0; shift ;;
    --) shift; break ;;
    *) break ;;
  esac
done

DEVICE_ARGS=()
if [[ ${USE_GRIPPER} -eq 1 ]]; then
  if [[ ! -e "${GRIPPER_DEVICE}" ]]; then
    echo "gripper device ${GRIPPER_DEVICE} not found (use --no-gripper)" >&2
    exit 1
  fi
  DEVICE_ARGS+=(--device "$(readlink -f "${GRIPPER_DEVICE}"):/dev/rby1_gripper")
fi

exec docker run --rm -it \
  --name rby1-ros2 \
  --network host --ipc host \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  "${DEVICE_ARGS[@]}" \
  "${IMAGE}" \
  ros2 launch rby1_bringup rby1_control.launch.py "$@"
