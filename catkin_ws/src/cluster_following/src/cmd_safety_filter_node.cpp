#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cluster_common/pose_utils.h"

class CmdSafetyFilter {
public:
  CmdSafetyFilter() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("input_cmd_topic", input_cmd_topic_, "/robot2/cmd_vel_raw");
    pnh_.param<std::string>("output_cmd_topic", output_cmd_topic_, "/robot2/cmd_vel");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/robot2/scan");
    pnh_.param<std::string>("enable_topic", enable_topic_, "/robot2/avoidance_enabled");
    pnh_.param<std::string>("leader_frame", leader_frame_, "robot1/base_link");
    pnh_.param<std::string>("follower_frame", follower_frame_, "robot2/base_link");
    pnh_.param<std::string>("peer_frame", peer_frame_, "");
    pnh_.param("enabled", enabled_, true);
    pnh_.param("robot_keepout_enabled", robot_keepout_enabled_, true);
    pnh_.param("peer_keepout_enabled", peer_keepout_enabled_, true);
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
    pnh_.param("robot_safe_distance", robot_safe_distance_, 0.55);
    pnh_.param("robot_danger_distance", robot_danger_distance_, 0.50);
    pnh_.param("robot_slowdown_gain", robot_slowdown_gain_, 1.0);
    pnh_.param("robot_turn_gain", robot_turn_gain_, 0.0);
    pnh_.param("peer_safe_distance", peer_safe_distance_, 0.75);
    pnh_.param("peer_danger_distance", peer_danger_distance_, 0.60);
    pnh_.param("peer_slowdown_gain", peer_slowdown_gain_, 1.0);
    pnh_.param("peer_turn_gain", peer_turn_gain_, 0.0);
    pnh_.param("peer_danger_wz_scale", peer_danger_wz_scale_, 0.3);
    pnh_.param("debug_enabled", debug_enabled_, false);
    pnh_.param("debug_period", debug_period_, 0.5);

    cmd_sub_ = nh_.subscribe(input_cmd_topic_, 10, &CmdSafetyFilter::cmdCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &CmdSafetyFilter::scanCallback, this);
    enable_sub_ = nh_.subscribe(enable_topic_, 5, &CmdSafetyFilter::enableCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_cmd_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
                             &CmdSafetyFilter::controlLoop, this);

    ROS_INFO("CmdSafetyFilter: %s + %s -> %s",
             input_cmd_topic_.c_str(), scan_topic_.c_str(), output_cmd_topic_.c_str());
  }

private:
  struct KeepoutResult {
    bool tf_ok{false};
    bool moving_toward{false};
    double distance{0.0};
    double forward{0.0};
    double lateral{0.0};
  };

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

  void enableCallback(const std_msgs::Bool::ConstPtr& msg) {
    enabled_ = msg->data;
    ROS_INFO("CmdSafetyFilter avoidance %s", enabled_ ? "enabled" : "disabled");
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
    const double raw_vx = cmd.linear.x;
    const double raw_wz = cmd.angular.z;

    const bool scan_fresh = scan_received_ &&
        (now - last_scan_time_).toSec() <= scan_timeout_;
    double obstacle_dist = 0.0;
    double obstacle_angle = 0.0;
    double obstacle_clearance = 0.0;
    bool obstacle_found = false;
    bool lidar_applied = false;
    if (enabled_ && scan_fresh && cmd.linear.x > min_avoid_linear_speed_ &&
        closestObstacle(obstacle_dist, obstacle_angle)) {
      obstacle_found = true;
      obstacle_clearance = obstacle_dist - front_overhang_distance_;
      lidar_applied = applyAvoidance(obstacle_clearance, obstacle_angle, cmd);
    }
    const double after_lidar_vx = cmd.linear.x;
    const double after_lidar_wz = cmd.angular.z;

    KeepoutResult leader_keepout;
    KeepoutResult peer_keepout;
    bool robot_keepout_applied = false;
    bool peer_keepout_applied = false;
    if (enabled_ && robot_keepout_enabled_) {
      robot_keepout_applied = applyFrameKeepout(
          leader_frame_, "leader", robot_safe_distance_, robot_danger_distance_,
          robot_slowdown_gain_, robot_turn_gain_, 1.0, cmd, leader_keepout);
    }
    if (enabled_ && peer_keepout_enabled_ && !peer_frame_.empty()) {
      peer_keepout_applied = applyFrameKeepout(
          peer_frame_, "peer", peer_safe_distance_, peer_danger_distance_,
          peer_slowdown_gain_, peer_turn_gain_, peer_danger_wz_scale_,
          cmd, peer_keepout);
    }

    cmd.linear.x = cluster_common::clamp(cmd.linear.x,
                                         -max_linear_speed_, max_linear_speed_);
    cmd.angular.z = cluster_common::clamp(cmd.angular.z,
                                          -max_angular_speed_, max_angular_speed_);
    if (std::fabs(cmd.linear.x) < min_linear_speed_) cmd.linear.x = 0.0;
    if (std::fabs(cmd.angular.z) < min_angular_speed_) cmd.angular.z = 0.0;
    if (debug_enabled_) {
      ROS_INFO_THROTTLE(debug_period_,
          "[SAFETY_DBG] en=%d raw(vx=%.3f,wz=%.3f) lidar(fresh=%d,found=%d,applied=%d,dist=%.3f,clear=%.3f,angle=%.3f,after_vx=%.3f,after_wz=%.3f) leader(tf=%d,dist=%.3f,front=%.3f,lat=%.3f,toward=%d,applied=%d,safe=%.2f,danger=%.2f) peer(frame=%s,tf=%d,dist=%.3f,front=%.3f,lat=%.3f,toward=%d,applied=%d,safe=%.2f,danger=%.2f) final(vx=%.3f,wz=%.3f)",
          enabled_ ? 1 : 0, raw_vx, raw_wz,
          scan_fresh ? 1 : 0, obstacle_found ? 1 : 0, lidar_applied ? 1 : 0,
          obstacle_dist, obstacle_clearance, obstacle_angle,
          after_lidar_vx, after_lidar_wz,
          leader_keepout.tf_ok ? 1 : 0, leader_keepout.distance,
          leader_keepout.forward, leader_keepout.lateral,
          leader_keepout.moving_toward ? 1 : 0, robot_keepout_applied ? 1 : 0,
          robot_safe_distance_, robot_danger_distance_,
          peer_frame_.c_str(), peer_keepout.tf_ok ? 1 : 0,
          peer_keepout.distance, peer_keepout.forward, peer_keepout.lateral,
          peer_keepout.moving_toward ? 1 : 0, peer_keepout_applied ? 1 : 0,
          peer_safe_distance_, peer_danger_distance_,
          cmd.linear.x, cmd.angular.z);
    }
    cmd_pub_.publish(cmd);
  }

  bool applyAvoidance(double clearance, double angle, geometry_msgs::Twist& cmd) const {
    if (clearance >= safe_distance_) return false;

    const double span = std::max(safe_distance_ - danger_distance_, 0.01);
    const double risk = cluster_common::clamp((safe_distance_ - clearance) / span, 0.0, 1.0);
    const double turn_dir = (angle >= 0.0) ? -1.0 : 1.0;

    if (clearance <= danger_distance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z += turn_dir * turn_gain_;
      ROS_WARN_THROTTLE(0.5, "Danger obstacle clearance %.2fm angle %.2f",
                        clearance, angle);
      return true;
    }

    if (cmd.linear.x > 0.0) {
      cmd.linear.x *= std::max(0.0, 1.0 - slowdown_gain_ * risk);
    }
    cmd.angular.z += turn_dir * turn_gain_ * risk;
    return true;
  }

  bool applyFrameKeepout(const std::string& target_frame, const std::string& label,
                         double safe_distance, double danger_distance,
                         double slowdown_gain, double turn_gain,
                         double danger_wz_scale, geometry_msgs::Twist& cmd,
                         KeepoutResult& result) {
    geometry_msgs::TransformStamped target_in_follower;
    try {
      target_in_follower = tf_buffer_.lookupTransform(
          follower_frame_, target_frame, ros::Time(0), ros::Duration(0.02));
      result.tf_ok = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "%s keepout TF unavailable (%s -> %s): %s",
                        label.c_str(), follower_frame_.c_str(),
                        target_frame.c_str(), ex.what());
      result.tf_ok = false;
      return false;
    }

    result.forward = target_in_follower.transform.translation.x;
    result.lateral = target_in_follower.transform.translation.y;
    result.distance = std::hypot(result.forward, result.lateral);
    if (result.distance >= safe_distance) return false;

    result.moving_toward =
        (cmd.linear.x > min_linear_speed_ && result.forward > 0.0) ||
        (cmd.linear.x < -min_linear_speed_ && result.forward < 0.0);
    if (!result.moving_toward) {
      if (result.distance <= danger_distance &&
          std::fabs(cmd.linear.x) <= min_linear_speed_ &&
          std::fabs(cmd.angular.z) > min_angular_speed_) {
        cmd.angular.z *= cluster_common::clamp(danger_wz_scale, 0.0, 1.0);
        ROS_WARN_THROTTLE(0.5, "%s keepout close %.2fm: limiting turn only",
                          label.c_str(), result.distance);
        return true;
      }
      return false;
    }

    const double span = std::max(safe_distance - danger_distance, 0.01);
    const double risk = cluster_common::clamp(
        (safe_distance - result.distance) / span, 0.0, 1.0);
    if (result.distance <= danger_distance) {
      cmd.linear.x = 0.0;
      cmd.angular.z *= cluster_common::clamp(danger_wz_scale, 0.0, 1.0);
      ROS_WARN_THROTTLE(0.5, "%s keepout danger %.2fm, forward %.2f lateral %.2f",
                        label.c_str(), result.distance,
                        result.forward, result.lateral);
      return true;
    }

    cmd.linear.x *= std::max(0.0, 1.0 - slowdown_gain * risk);
    if (turn_gain > 0.0) {
      const double turn_dir = (result.lateral >= 0.0) ? -1.0 : 1.0;
      cmd.angular.z += turn_dir * turn_gain * risk;
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber enable_sub_;
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
  std::string enable_topic_;
  std::string leader_frame_;
  std::string follower_frame_;
  std::string peer_frame_;
  bool enabled_;
  bool robot_keepout_enabled_;
  bool peer_keepout_enabled_;
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
  double robot_safe_distance_;
  double robot_danger_distance_;
  double robot_slowdown_gain_;
  double robot_turn_gain_;
  double peer_safe_distance_;
  double peer_danger_distance_;
  double peer_slowdown_gain_;
  double peer_turn_gain_;
  double peer_danger_wz_scale_;
  bool debug_enabled_;
  double debug_period_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_safety_filter");
  CmdSafetyFilter filter;
  ros::spin();
  return 0;
}
