#!/bin/bash
# ROS network environment for the two-car LIMO cluster.
#
# Usage in any terminal:
#   source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
#
# It detects the current car by hostname or local IP and sets:
#   ROS_MASTER_URI=http://192.168.137.248:11311
#   ROS_IP=<this car ip>
#   ROBOT_NS=robot1 or robot2

LEADER_IP="192.168.137.248"
FOLLOWER_IP="192.168.137.133"
HOST="$(hostname)"
LOCAL_IPS="$(hostname -I 2>/dev/null)"

export ROS_MASTER_URI="http://${LEADER_IP}:11311"
unset ROS_HOSTNAME

if echo "$LOCAL_IPS" | grep -qw "$LEADER_IP" || [ "$HOST" = "car1" ]; then
  export ROS_IP="$LEADER_IP"
  export ROBOT_ROLE="leader"
  export ROBOT_NS="robot1"
elif echo "$LOCAL_IPS" | grep -qw "$FOLLOWER_IP" || [ "$HOST" = "car2" ]; then
  export ROS_IP="$FOLLOWER_IP"
  export ROBOT_ROLE="follower"
  export ROBOT_NS="robot2"
else
  echo "ERROR: cannot detect car role."
  echo "Hostname: $HOST"
  echo "Local IPs: $LOCAL_IPS"
  echo "Expected car1=$LEADER_IP or car2=$FOLLOWER_IP"
  return 1 2>/dev/null || exit 1
fi

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "ROBOT_NS=$ROBOT_NS"
