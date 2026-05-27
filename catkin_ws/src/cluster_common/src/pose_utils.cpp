#include "cluster_common/pose_utils.h"
#include <cmath>

namespace cluster_common {

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double degToRad(double deg) {
  return deg * M_PI / 180.0;
}

double radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

Pose2D odomToPose2D(const nav_msgs::Odometry& odom) {
  Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  pose.theta = quatToYaw(odom.pose.pose.orientation);
  return pose;
}

geometry_msgs::Pose pose2DToGeometryMsg(const Pose2D& pose) {
  geometry_msgs::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = 0.0;
  msg.orientation = yawToQuat(pose.theta);
  return msg;
}

double quatToYaw(const geometry_msgs::Quaternion& q) {
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

geometry_msgs::Quaternion yawToQuat(double yaw) {
  geometry_msgs::Quaternion q;
  double half_yaw = yaw * 0.5;
  q.w = std::cos(half_yaw);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  return q;
}

double clamp(double value, double min_val, double max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

}  // namespace cluster_common
