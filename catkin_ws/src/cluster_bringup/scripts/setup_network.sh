#!/bin/bash
# ROS network environment for the LIMO cluster.
#
# Usage in any terminal:
#   source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
#
# It detects the current car by hostname or local IP and sets:
#   ROS_MASTER_URI=http://192.168.137.248:11311
#   ROS_IP=<this car ip>
#   ROBOT_NS=robot1, robot2 or robot3

LEADER_IP="192.168.137.248"
FOLLOWER_IP="192.168.137.133"
CAR3_IP="192.168.137.99"
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
elif echo "$LOCAL_IPS" | grep -qw "$CAR3_IP" || [ "$HOST" = "car3" ]; then
  export ROS_IP="$CAR3_IP"
  export ROBOT_ROLE="follower"
  export ROBOT_NS="robot3"
else
  echo "ERROR: cannot detect car role."
  echo "Hostname: $HOST"
  echo "Local IPs: $LOCAL_IPS"
  echo "Expected car1=$LEADER_IP, car2=$FOLLOWER_IP or car3=$CAR3_IP"
  return 1 2>/dev/null || exit 1
fi

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "ROBOT_NS=$ROBOT_NS"
