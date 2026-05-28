#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "cluster_common/pose_utils.h"

class CmdSafetyFilter {
public:
  CmdSafetyFilter() : pnh_("~") {
    pnh_.param<std::string>("input_cmd_topic", input_cmd_topic_, "/robot2/cmd_vel_raw");
    pnh_.param<std::string>("output_cmd_topic", output_cmd_topic_, "/robot2/cmd_vel");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/robot2/scan");
    pnh_.param("loop_rate", loop_rate_, 20.0);
    pnh_.param("cmd_timeout", cmd_timeout_, 0.5);
    pnh_.param("scan_timeout", scan_timeout_, 0.7);
    pnh_.param("safe_distance", safe_distance_, 0.45);
    pnh_.param("danger_distance", danger_distance_, 0.25);
    pnh_.param("obstacle_min_valid_range", obstacle_min_valid_range_, 0.18);
    pnh_.param("front_overhang_distance", front_overhang_distance_, 0.30);
    pnh_.param("front_angle_center", front_angle_center_, 0.0);
    pnh_.param("front_sector", front_sector_, 1.05);
    pnh_.param("max_linear_speed", max_linear_speed_, 0.6);
    pnh_.param("max_angular_speed", max_angular_speed_, 0.8);
    pnh_.param("min_linear_speed", min_linear_speed_, 0.05);
    pnh_.param("min_angular_speed", min_angular_speed_, 0.05);
    pnh_.param("min_avoid_linear_speed", min_avoid_linear_speed_, 0.08);
    pnh_.param("slowdown_gain", slowdown_gain_, 1.0);
    pnh_.param("turn_gain", turn_gain_, 0.35);
    pnh_.param("danger_reverse_speed", danger_reverse_speed_, -0.06);

    cmd_sub_ = nh_.subscribe(input_cmd_topic_, 10, &CmdSafetyFilter::cmdCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &CmdSafetyFilter::scanCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_cmd_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
                             &CmdSafetyFilter::controlLoop, this);

    ROS_INFO("CmdSafetyFilter: %s + %s -> %s",
             input_cmd_topic_.c_str(), scan_topic_.c_str(), output_cmd_topic_.c_str());
  }

private:
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    latest_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    cmd_received_ = true;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan_ = *msg;
    last_scan_time_ = ros::Time::now();
    scan_received_ = true;
  }

  static double angleDiff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  bool closestObstacle(double& min_range, double& min_angle) const {
    if (!scan_received_) return false;

    min_range = std::numeric_limits<double>::infinity();
    min_angle = 0.0;
    const double half_sector = front_sector_ * 0.5;

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      const float r = latest_scan_.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < latest_scan_.range_min || r > latest_scan_.range_max) continue;
      if (r < obstacle_min_valid_range_) continue;

      const double angle = latest_scan_.angle_min +
          static_cast<double>(i) * latest_scan_.angle_increment;
      if (std::fabs(angleDiff(angle, front_angle_center_)) > half_sector) continue;

      if (r < min_range) {
        min_range = r;
        min_angle = angleDiff(angle, front_angle_center_);
      }
    }

    return std::isfinite(min_range);
  }

  void controlLoop(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;
    const ros::Time now = ros::Time::now();

    if (!cmd_received_ || (now - last_cmd_time_).toSec() > cmd_timeout_) {
      cmd_pub_.publish(cmd);
      return;
    }

    cmd = latest_cmd_;

    const bool scan_fresh = scan_received_ &&
        (now - last_scan_time_).toSec() <= scan_timeout_;
    double obstacle_dist = 0.0;
    double obstacle_angle = 0.0;
    if (scan_fresh && cmd.linear.x > min_avoid_linear_speed_ &&
        closestObstacle(obstacle_dist, obstacle_angle)) {
      const double clearance = obstacle_dist - front_overhang_distance_;
      applyAvoidance(clearance, obstacle_angle, cmd);
    }

    cmd.linear.x = cluster_common::clamp(cmd.linear.x,
                                         -max_linear_speed_, max_linear_speed_);
    cmd.angular.z = cluster_common::clamp(cmd.angular.z,
                                          -max_angular_speed_, max_angular_speed_);
    if (std::fabs(cmd.linear.x) < min_linear_speed_) cmd.linear.x = 0.0;
    if (std::fabs(cmd.angular.z) < min_angular_speed_) cmd.angular.z = 0.0;
    cmd_pub_.publish(cmd);
  }

  void applyAvoidance(double clearance, double angle, geometry_msgs::Twist& cmd) const {
    if (clearance >= safe_distance_) return;

    const double span = std::max(safe_distance_ - danger_distance_, 0.01);
    const double risk = cluster_common::clamp((safe_distance_ - clearance) / span, 0.0, 1.0);
    const double turn_dir = (angle >= 0.0) ? -1.0 : 1.0;

    if (clearance <= danger_distance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z += turn_dir * turn_gain_;
      ROS_WARN_THROTTLE(0.5, "Danger obstacle clearance %.2fm angle %.2f",
                        clearance, angle);
      return;
    }

    if (cmd.linear.x > 0.0) {
      cmd.linear.x *= std::max(0.0, 1.0 - slowdown_gain_ * risk);
    }
    cmd.angular.z += turn_dir * turn_gain_ * risk;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;

  geometry_msgs::Twist latest_cmd_;
  sensor_msgs::LaserScan latest_scan_;
  ros::Time last_cmd_time_;
  ros::Time last_scan_time_;
  bool cmd_received_{false};
  bool scan_received_{false};

  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  std::string scan_topic_;
  double loop_rate_;
  double cmd_timeout_;
  double scan_timeout_;
  double safe_distance_;
  double danger_distance_;
  double obstacle_min_valid_range_;
  double front_overhang_distance_;
  double front_angle_center_;
  double front_sector_;
  double max_linear_speed_;
  double max_angular_speed_;
  double min_linear_speed_;
  double min_angular_speed_;
  double min_avoid_linear_speed_;
  double slowdown_gain_;
  double turn_gain_;
  double danger_reverse_speed_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_safety_filter");
  CmdSafetyFilter filter;
  ros::spin();
  return 0;
}
