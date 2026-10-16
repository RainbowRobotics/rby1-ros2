#!/bin/bash
# run.sh <driver launch args...> -- <tests...>   (run on the VM host)
HERE=$(cd "$(dirname "$0")" && pwd)
ARGS=(); while [[ $# -gt 0 && "$1" != "--" ]]; do ARGS+=("$1"); shift; done; shift
sg docker -c "docker exec rby1-dev-humble bash -c 'pkill -f \"ros2 launch\"; pkill -f ros2_control_node; pkill -f rby1_robot_manager'" ; sleep 2
/home/kjs-vm/rb5090/rby1_lab/deploy/.venv/bin/python -I "$HERE/reset_sim.py"
sg docker -c "docker exec -e ROS_DISTRO=${DISTRO:-humble} rby1-dev-${DISTRO:-humble} bash /ws/src/rby1-ros2/tools/sim_validation/start_driver.sh ${ARGS[*]}" | grep -c "hardware active"
sg docker -c "docker exec -e ROS_DOMAIN_ID=90 rby1-dev-${DISTRO:-humble} bash -c 'source /opt/ros/${DISTRO:-humble}/setup.bash; source /ws/install_${DISTRO:-humble}/setup.bash; cd /ws/src/rby1-ros2/tools/sim_validation && python3 validate.py $* 2>&1 | grep RESULT'"
