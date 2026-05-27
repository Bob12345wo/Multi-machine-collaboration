#include <ros/ros.h>
#include "cluster_following/follower_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cluster_following::FollowerController controller(nh, pnh);

  controller.spin();

  return 0;
}
