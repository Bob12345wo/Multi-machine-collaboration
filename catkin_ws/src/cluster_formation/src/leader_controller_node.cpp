echo $ROS_PACKAGE_PATH#include <ros/ros.h>
#include "cluster_formation/leader_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "leader_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cluster_formation::LeaderController controller(nh, pnh);

  controller.spin();

  return 0;
}
