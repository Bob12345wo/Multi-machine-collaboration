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

if echo "$LOCAL_IPS" | grep -qw "$LEADER_IP"; then
  EXPECTED_IP="$LEADER_IP"
  export ROBOT_ROLE="leader"
  export ROBOT_NS="robot1"
elif echo "$LOCAL_IPS" | grep -qw "$FOLLOWER_IP"; then
  EXPECTED_IP="$FOLLOWER_IP"
  export ROBOT_ROLE="follower"
  export ROBOT_NS="robot2"
elif echo "$LOCAL_IPS" | grep -qw "$CAR3_IP"; then
  EXPECTED_IP="$CAR3_IP"
  export ROBOT_ROLE="follower"
  export ROBOT_NS="robot3"
else
  case "$HOST" in
    car1)
      EXPECTED_IP="$LEADER_IP"
      export ROBOT_ROLE="leader"
      export ROBOT_NS="robot1"
      ;;
    car2)
      EXPECTED_IP="$FOLLOWER_IP"
      export ROBOT_ROLE="follower"
      export ROBOT_NS="robot2"
      ;;
    car3)
      EXPECTED_IP="$CAR3_IP"
      export ROBOT_ROLE="follower"
      export ROBOT_NS="robot3"
      ;;
    *)
      echo "ERROR: cannot detect car role."
      echo "Hostname: $HOST"
      echo "Local IPs: $LOCAL_IPS"
      return 1 2>/dev/null || exit 1
      ;;
  esac
fi

# Never advertise an address that is not assigned to this machine. ROS nodes
# can still register with the master in that state, but subscribers cannot
# open TCPROS connections and every topic appears to have no messages.
if ! echo "$LOCAL_IPS" | grep -qw "$EXPECTED_IP"; then
  echo "ERROR: $HOST must own $EXPECTED_IP before ROS is started."
  echo "Hostname: $HOST"
  echo "Local IPs: $LOCAL_IPS"
  echo "Expected car1=$LEADER_IP, car2=$FOLLOWER_IP, car3=$CAR3_IP"
  echo "Fix the wlan0 static address or reconnect the hotspot, then retry."
  return 1 2>/dev/null || exit 1
fi

export ROS_IP="$EXPECTED_IP"

echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "ROBOT_NS=$ROBOT_NS"
