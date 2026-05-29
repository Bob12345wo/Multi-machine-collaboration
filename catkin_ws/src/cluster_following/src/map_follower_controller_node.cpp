#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cluster_common/pose_utils.h"
#include "cluster_msgs/FollowerStatus.h"
#include "cluster_msgs/LeaderCmd.h"

class MapFollowerController {
public:
  MapFollowerController() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("leader_frame", leader_frame_, "robot1/base_link");
    pnh_.param<std::string>("follower_frame", follower_frame_, "robot2/base_link");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/robot2/cmd_vel");
    pnh_.param<std::string>("control_mode_topic", control_mode_topic_,
                            "/robot2/follower_control_mode");
    pnh_.param<std::string>("control_mode", control_mode_, "body_orbit");
    pnh_.param("use_leader_offsets", use_leader_offsets_, true);
    pnh_.param("offset_x", offset_x_, -0.8);
    pnh_.param("offset_y", offset_y_, 0.0);
    pnh_.param("loop_rate", loop_rate_, 20.0);
    pnh_.param("max_linear_speed", max_linear_speed_, 0.6);
    pnh_.param("max_angular_speed", max_angular_speed_, 0.8);
    pnh_.param("min_linear_speed", min_linear_speed_, 0.05);
    pnh_.param("min_angular_speed", min_angular_speed_, 0.05);
    pnh_.param("pos_deadband", pos_deadband_, 0.05);
    pnh_.param("yaw_deadband", yaw_deadband_, 0.08);
    pnh_.param("k_v", k_v_, 0.9);
    pnh_.param("k_l", k_l_, 0.8);
    pnh_.param("k_a", k_a_, 0.7);
    pnh_.param("k_heading", k_heading_, 1.0);
    pnh_.param("k_approach_heading", k_approach_heading_, 1.4);
    pnh_.param("leader_vx_gain", leader_vx_gain_, 1.0);
    pnh_.param("leader_wz_gain", leader_wz_gain_, 0.8);
    pnh_.param("orbit_v_gain", orbit_v_gain_, 0.8);
    pnh_.param("target_filter_alpha", target_filter_alpha_, 0.35);
    pnh_.param("cmd_filter_alpha", cmd_filter_alpha_, 0.45);
    pnh_.param("heading_lookahead", heading_lookahead_, 0.45);
    pnh_.param("max_target_jump", max_target_jump_, 0.8);
    pnh_.param("final_align_distance", final_align_distance_, 0.35);
    pnh_.param("static_position_tolerance", static_position_tolerance_, 0.12);
    pnh_.param("static_leader_v_threshold", static_leader_v_threshold_, 0.04);
    pnh_.param("static_leader_w_threshold", static_leader_w_threshold_, 0.04);
    pnh_.param("approach_heading_limit", approach_heading_limit_, 0.9);
    pnh_.param("yaw_priority_threshold", yaw_priority_threshold_, 0.26);
    pnh_.param("turn_in_place_wz_threshold", turn_in_place_wz_threshold_, 0.18);
    pnh_.param("max_turn_linear_speed", max_turn_linear_speed_, 0.08);
    pnh_.param("yaw_priority_vx_scale", yaw_priority_vx_scale_, 0.25);
    pnh_.param("turn_radius_compensation", turn_radius_compensation_, true);
    pnh_.param("allow_reverse_while_leader_forward", allow_reverse_while_leader_forward_, false);
    pnh_.param("allow_orbit_reverse", allow_orbit_reverse_, true);
    pnh_.param("debug_enabled", debug_enabled_, false);
    pnh_.param("debug_period", debug_period_, 0.5);
    pnh_.param("path_keepout_enabled", path_keepout_enabled_, true);
    pnh_.param("path_keepout_radius", path_keepout_radius_, 0.70);
    pnh_.param("path_detour_radius", path_detour_radius_, 0.85);
    pnh_.param("path_detour_step", path_detour_step_, 0.55);
    pnh_.param("path_detour_goal_tolerance", path_detour_goal_tolerance_, 0.18);

    leader_cmd_sub_ = nh_.subscribe("/robot1/leader_cmd", 10,
        &MapFollowerController::leaderCmdCallback, this);
    control_mode_sub_ = nh_.subscribe(control_mode_topic_, 5,
        &MapFollowerController::controlModeCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    status_pub_ = nh_.advertise<cluster_msgs::FollowerStatus>(
        "/robot2/follower_status", 10);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
        &MapFollowerController::controlLoop, this);
  }

private:
  struct Pose2D {
    double x;
    double y;
    double yaw;
  };

  void leaderCmdCallback(const cluster_msgs::LeaderCmd::ConstPtr& msg) {
    latest_leader_cmd_ = *msg;
    leader_cmd_received_ = true;
    last_leader_cmd_time_ = ros::Time::now();
  }

  void controlModeCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data != "body_orbit" && msg->data != "wheeltec_global") {
      ROS_WARN("Ignoring unsupported follower control mode: %s", msg->data.c_str());
      return;
    }
    if (control_mode_ != msg->data) {
      ROS_INFO("Follower control mode: %s -> %s",
               control_mode_.c_str(), msg->data.c_str());
      control_mode_ = msg->data;
      control_mode_changed_ = true;
      target_initialized_ = false;
    }
  }

  bool lookupPose(const std::string& frame, Pose2D& pose) {
    try {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(map_frame_, frame, ros::Time(0),
                                     ros::Duration(0.05));
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "TF lookup failed %s -> %s: %s",
                        map_frame_.c_str(), frame.c_str(), ex.what());
      return false;
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!leader_cmd_received_ ||
        (ros::Time::now() - last_leader_cmd_time_).toSec() > 0.5) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_LOST,
                    0.0, 0.0, 0.0, 0.0, false);
      return;
    }
    if (latest_leader_cmd_.mode != cluster_msgs::LeaderCmd::MODE_FORMATION) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_IDLE,
                    0.0, 0.0, 0.0, 0.0, true);
      return;
    }

    Pose2D leader;
    Pose2D follower;
    if (!lookupPose(leader_frame_, leader) || !lookupPose(follower_frame_, follower)) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_LOST,
                    0.0, 0.0, 0.0, 0.0, false);
      return;
    }

    double active_offset_x = offset_x_;
    double active_offset_y = offset_y_;
    if (use_leader_offsets_) {
      active_offset_x = latest_leader_cmd_.offset_x;
      active_offset_y = latest_leader_cmd_.offset_y;
    }

    if (control_mode_changed_) {
      if (control_mode_ == "wheeltec_global") {
        formation_anchor_yaw_ = leader.yaw;
      }
      control_mode_changed_ = false;
      target_initialized_ = false;
    }

    if (latest_leader_cmd_.formation != last_formation_) {
      if (control_mode_ == "wheeltec_global") {
        formation_anchor_yaw_ = leader.yaw;
      }
      last_formation_ = latest_leader_cmd_.formation;
      target_initialized_ = false;
    }

    const double cos_l = std::cos(leader.yaw);
    const double sin_l = std::sin(leader.yaw);
    double target_x = leader.x + active_offset_x * cos_l - active_offset_y * sin_l;
    double target_y = leader.y + active_offset_x * sin_l + active_offset_y * cos_l;
    double target_yaw = leader.yaw;

    const double leader_vx = latest_leader_cmd_.leader_vx;
    const double leader_wz = latest_leader_cmd_.leader_vyaw;

    if (control_mode_ == "wheeltec_global") {
      const double cos_a = std::cos(formation_anchor_yaw_);
      const double sin_a = std::sin(formation_anchor_yaw_);
      target_x = leader.x + active_offset_x * cos_a - active_offset_y * sin_a;
      target_y = leader.y + active_offset_x * sin_a + active_offset_y * cos_a;
    } else if (control_mode_ != "body_orbit" &&
        turn_radius_compensation_ && std::fabs(leader_wz) > min_angular_speed_ &&
        std::fabs(leader_vx) > min_linear_speed_) {
      const double radius = leader_vx / leader_wz;
      target_yaw = cluster_common::normalizeAngle(
          leader.yaw + std::atan2(active_offset_y, active_offset_x + radius));
    }

    Pose2D final_target{target_x, target_y, target_yaw};
    const bool leader_static_for_path =
        std::fabs(leader_vx) < static_leader_v_threshold_ &&
        std::fabs(leader_wz) < static_leader_w_threshold_;
    Pose2D raw_target = applyPathKeepout(leader, follower, final_target,
                                         leader_static_for_path);
    Pose2D target = filterTarget(raw_target);

    const double dx = target.x - follower.x;
    const double dy = target.y - follower.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    const double cos_f = std::cos(follower.yaw);
    const double sin_f = std::sin(follower.yaw);
    const double forward_err = dx * cos_f + dy * sin_f;
    const double lateral_err = -dx * sin_f + dy * cos_f;
    const double yaw_err = cluster_common::normalizeAngle(target.yaw - follower.yaw);
    const double heading_err = std::atan2(lateral_err,
        std::max(std::fabs(forward_err), heading_lookahead_));
    const double heading_weight = cluster_common::clamp(
        (distance - pos_deadband_) /
        std::max(final_align_distance_ - pos_deadband_, 0.01),
        0.0, 1.0);

    if (distance < pos_deadband_ && std::fabs(yaw_err) < yaw_deadband_ &&
        std::fabs(leader_vx) < min_linear_speed_ &&
        std::fabs(leader_wz) < min_angular_speed_) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_IDLE,
                    lateral_err, forward_err, yaw_err, distance, true);
      return;
    }

    double vx = 0.0;
    double wz = 0.0;
    if (control_mode_ == "wheeltec_global") {
      vx = leader_vx_gain_ * leader_vx + k_v_ * forward_err;
      wz = leader_wz_gain_ * leader_wz +
          heading_weight * 0.5 * k_l_ * lateral_err +
          k_a_ * std::sin(yaw_err);
    } else {
      const bool leader_static =
          std::fabs(leader_vx) < static_leader_v_threshold_ &&
          std::fabs(leader_wz) < static_leader_w_threshold_;

      if (control_mode_ == "body_orbit" && leader_static) {
        if (distance <= static_position_tolerance_) {
          vx = 0.0;
          wz = k_a_ * std::sin(yaw_err);
        } else {
        const double target_heading = std::atan2(dy, dx);
        const double approach_err =
            cluster_common::normalizeAngle(target_heading - follower.yaw);
        const double approach_speed_scale =
            cluster_common::clamp(std::cos(approach_err), 0.0, 1.0);

        vx = k_v_ * distance * approach_speed_scale;
        if (std::fabs(approach_err) > approach_heading_limit_) {
          vx = 0.0;
        }
        wz = k_approach_heading_ * approach_err;
        }
      } else {
        double vx_ff = leader_vx_gain_ * leader_vx;
        if (control_mode_ == "body_orbit") {
        const double offset_map_x = active_offset_x * cos_l - active_offset_y * sin_l;
        const double offset_map_y = active_offset_x * sin_l + active_offset_y * cos_l;
        const double target_vel_x = leader_vx * cos_l - leader_wz * offset_map_y;
        const double target_vel_y = leader_vx * sin_l + leader_wz * offset_map_x;
        vx_ff = orbit_v_gain_ * (target_vel_x * cos_f + target_vel_y * sin_f);
        }

        vx = vx_ff + k_v_ * forward_err;
        wz = leader_wz_gain_ * leader_wz +
            heading_weight * k_l_ * lateral_err +
            k_heading_ * heading_weight * heading_err +
            k_a_ * std::sin(yaw_err);

        const bool leader_turning = std::fabs(leader_wz) > turn_in_place_wz_threshold_;
        const bool yaw_priority = std::fabs(yaw_err) > yaw_priority_threshold_;
        if (leader_turning || yaw_priority) {
          vx *= yaw_priority_vx_scale_;
          vx = cluster_common::clamp(vx, -max_turn_linear_speed_, max_turn_linear_speed_);
        }
      }
    }

    const bool orbit_reverse_allowed = control_mode_ == "body_orbit" &&
        allow_orbit_reverse_ && std::fabs(leader_wz) > min_angular_speed_;
    if (!allow_reverse_while_leader_forward_ && !orbit_reverse_allowed &&
        (leader_vx > min_linear_speed_ || std::fabs(leader_wz) > min_angular_speed_) &&
        vx < 0.0) {
      vx = 0.0;
    }

    vx = cluster_common::clamp(vx, -max_linear_speed_, max_linear_speed_);
    wz = cluster_common::clamp(wz, -max_angular_speed_, max_angular_speed_);

    if (std::fabs(vx) < min_linear_speed_) vx = 0.0;
    if (std::fabs(wz) < min_angular_speed_) wz = 0.0;

    geometry_msgs::Twist cmd = filterCommand(vx, wz);
    cmd_vel_pub_.publish(cmd);
    if (debug_enabled_) {
      ROS_INFO_THROTTLE(debug_period_,
          "[FOLLOW_DBG] mode=%s form=%u offset(x=%.3f,y=%.3f) leader(x=%.3f,y=%.3f,yaw=%.3f,vx=%.3f,wz=%.3f) follower(x=%.3f,y=%.3f,yaw=%.3f) target(final_x=%.3f,final_y=%.3f,raw_x=%.3f,raw_y=%.3f,raw_yaw=%.3f,x=%.3f,y=%.3f,yaw=%.3f,detour=%d) err(fwd=%.3f,lat=%.3f,yaw=%.3f,dist=%.3f,heading=%.3f,weight=%.3f) cmd_raw(vx=%.3f,wz=%.3f) cmd_out(vx=%.3f,wz=%.3f)",
          control_mode_.c_str(), latest_leader_cmd_.formation,
          active_offset_x, active_offset_y,
          leader.x, leader.y, leader.yaw, leader_vx, leader_wz,
          follower.x, follower.y, follower.yaw,
          final_target.x, final_target.y,
          raw_target.x, raw_target.y, raw_target.yaw,
          target.x, target.y, target.yaw,
          path_detour_active_ ? 1 : 0,
          forward_err, lateral_err, yaw_err, distance, heading_err, heading_weight,
          vx, wz, cmd.linear.x, cmd.angular.z);
    }
    publishStatus(cluster_msgs::FollowerStatus::STATE_TRACKING,
                  lateral_err, forward_err, yaw_err, distance, true);
  }

  void stop() {
    geometry_msgs::Twist cmd;
    cmd_vel_pub_.publish(cmd);
    last_cmd_ = cmd;
  }

  void publishStatus(uint8_t state, double error_x, double error_y,
                     double error_yaw, double error_dist,
                     bool leader_visible) {
    cluster_msgs::FollowerStatus status;
    status.header.stamp = ros::Time::now();
    status.state = state;
    status.error_x = error_x;
    status.error_y = error_y;
    status.error_yaw = error_yaw;
    status.error_dist = error_dist;
    status.leader_visible = leader_visible;
    status_pub_.publish(status);
  }

  Pose2D filterTarget(const Pose2D& raw_target) {
    const double alpha = cluster_common::clamp(target_filter_alpha_, 0.0, 1.0);

    if (!target_initialized_) {
      filtered_target_ = raw_target;
      target_initialized_ = true;
      return filtered_target_;
    }

    const double dx = raw_target.x - filtered_target_.x;
    const double dy = raw_target.y - filtered_target_.y;
    const double jump = std::sqrt(dx * dx + dy * dy);
    if (jump > max_target_jump_) {
      ROS_WARN_THROTTLE(1.0, "Target pose jumped %.2f m, holding filtered target", jump);
      return filtered_target_;
    }

    filtered_target_.x = alpha * raw_target.x + (1.0 - alpha) * filtered_target_.x;
    filtered_target_.y = alpha * raw_target.y + (1.0 - alpha) * filtered_target_.y;
    filtered_target_.yaw = cluster_common::normalizeAngle(
        filtered_target_.yaw +
        alpha * cluster_common::normalizeAngle(raw_target.yaw - filtered_target_.yaw));

    return filtered_target_;
  }

  Pose2D applyPathKeepout(const Pose2D& leader, const Pose2D& follower,
                          const Pose2D& final_target, bool leader_static) {
    if (!path_keepout_enabled_ || control_mode_ != "body_orbit" ||
        !leader_static) {
      path_detour_active_ = false;
      return final_target;
    }

    const double target_dist = std::hypot(final_target.x - follower.x,
                                          final_target.y - follower.y);
    if (target_dist <= path_detour_goal_tolerance_) {
      path_detour_active_ = false;
      return final_target;
    }

    const double line_clearance = distancePointToSegment(
        leader.x, leader.y, follower.x, follower.y,
        final_target.x, final_target.y);
    if (line_clearance >= path_keepout_radius_) {
      path_detour_active_ = false;
      return final_target;
    }

    const double from_angle = std::atan2(follower.y - leader.y,
                                         follower.x - leader.x);
    const double target_angle = std::atan2(final_target.y - leader.y,
                                           final_target.x - leader.x);
    const double delta = cluster_common::normalizeAngle(target_angle - from_angle);
    const double step = cluster_common::clamp(delta, -path_detour_step_,
                                             path_detour_step_);
    const double detour_radius = std::max(path_detour_radius_, path_keepout_radius_);
    Pose2D detour = final_target;
    detour.x = leader.x + detour_radius * std::cos(from_angle + step);
    detour.y = leader.y + detour_radius * std::sin(from_angle + step);

    if (!path_detour_active_) {
      ROS_WARN("Path keepout detour active: line_clearance=%.2f, target_dist=%.2f",
               line_clearance, target_dist);
    }
    path_detour_active_ = true;
    return detour;
  }

  static double distancePointToSegment(double px, double py,
                                       double ax, double ay,
                                       double bx, double by) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;
    const double ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-9) {
      return std::hypot(px - ax, py - ay);
    }
    const double t = cluster_common::clamp((apx * abx + apy * aby) / ab2,
                                           0.0, 1.0);
    const double cx = ax + t * abx;
    const double cy = ay + t * aby;
    return std::hypot(px - cx, py - cy);
  }

  geometry_msgs::Twist filterCommand(double vx, double wz) {
    const double alpha = cluster_common::clamp(cmd_filter_alpha_, 0.0, 1.0);
    geometry_msgs::Twist cmd;
    cmd.linear.x = alpha * vx + (1.0 - alpha) * last_cmd_.linear.x;
    cmd.angular.z = alpha * wz + (1.0 - alpha) * last_cmd_.angular.z;
    if (std::fabs(cmd.linear.x) < min_linear_speed_) cmd.linear.x = 0.0;
    if (std::fabs(cmd.angular.z) < min_angular_speed_) cmd.angular.z = 0.0;
    last_cmd_ = cmd;
    return cmd;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber leader_cmd_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher status_pub_;
  ros::Timer control_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cluster_msgs::LeaderCmd latest_leader_cmd_;
  bool leader_cmd_received_{false};
  ros::Time last_leader_cmd_time_;

  std::string map_frame_;
  std::string leader_frame_;
  std::string follower_frame_;
  std::string cmd_vel_topic_;
  std::string control_mode_topic_;
  std::string control_mode_;

  double offset_x_;
  double offset_y_;
  double loop_rate_;
  double max_linear_speed_;
  double max_angular_speed_;
  double min_linear_speed_;
  double min_angular_speed_;
  double pos_deadband_;
  double yaw_deadband_;
  double k_v_;
  double k_l_;
  double k_a_;
  double k_heading_;
  double k_approach_heading_;
  double leader_vx_gain_;
  double leader_wz_gain_;
  double orbit_v_gain_;
  double target_filter_alpha_;
  double cmd_filter_alpha_;
  double heading_lookahead_;
  double max_target_jump_;
  double final_align_distance_;
  double static_position_tolerance_;
  double static_leader_v_threshold_;
  double static_leader_w_threshold_;
  double approach_heading_limit_;
  double yaw_priority_threshold_;
  double turn_in_place_wz_threshold_;
  double max_turn_linear_speed_;
  double yaw_priority_vx_scale_;
  bool turn_radius_compensation_;
  bool allow_reverse_while_leader_forward_;
  bool allow_orbit_reverse_;
  bool debug_enabled_;
  double debug_period_;
  bool path_keepout_enabled_;
  double path_keepout_radius_;
  double path_detour_radius_;
  double path_detour_step_;
  double path_detour_goal_tolerance_;
  bool use_leader_offsets_;
  bool control_mode_changed_{false};
  bool target_initialized_{false};
  bool path_detour_active_{false};
  uint8_t last_formation_{255};
  double formation_anchor_yaw_{0.0};
  Pose2D filtered_target_{0.0, 0.0, 0.0};
  geometry_msgs::Twist last_cmd_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_follower_controller");
  MapFollowerController controller;
  ros::spin();
  return 0;
}
