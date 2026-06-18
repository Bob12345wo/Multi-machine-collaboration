#!/bin/bash
set -e

cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh

if [ "$ROBOT_NS" != "robot1" ]; then
  echo "ERROR: web control should run on car1 / ROS master."
  echo "Current ROBOT_NS=$ROBOT_NS ROS_IP=$ROS_IP"
  exit 1
fi

if rosnode list 2>/dev/null | grep -q "^/robot1/teleop_keyboard$"; then
  echo "ERROR: /robot1/teleop_keyboard is running."
  echo "Stop it before using web-only control:"
  echo "  rosnode kill /robot1/teleop_keyboard"
  exit 1
fi

roslaunch cluster_bringup web_visualizer.launch "$@"
