#ifndef CLUSTER_COMMON_POSE_UTILS_H
#define CLUSTER_COMMON_POSE_UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace cluster_common {

struct Pose2D {
  double x;
  double y;
  double theta;
};

double normalizeAngle(double angle);

double degToRad(double deg);

double radToDeg(double rad);

Pose2D odomToPose2D(const nav_msgs::Odometry& odom);

geometry_msgs::Pose pose2DToGeometryMsg(const Pose2D& pose);

double quatToYaw(const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion yawToQuat(double yaw);

double clamp(double value, double min_val, double max_val);

}  // namespace cluster_common

#endif  // CLUSTER_COMMON_POSE_UTILS_H
