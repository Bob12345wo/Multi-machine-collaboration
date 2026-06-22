#include "cluster_formation/leader_controller.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

namespace cluster_formation {

LeaderController::LeaderController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  , self_odom_received_(false)
  , follower_odom_received_(false)
  , follower_status_received_(false)
  , follower3_status_received_(false)
  , teleop_cmd_received_(false)
  , home_pose_received_(false)
  , return_home_active_(false)
  , current_mode_(cluster_msgs::LeaderCmd::MODE_IDLE)
  , current_formation_(cluster_msgs::LeaderCmd::FORMATION_COLUMN)
  , last_non_circle_formation_(cluster_msgs::LeaderCmd::FORMATION_COLUMN)
  , use_custom_offsets_(false)
  , circle_show_was_active_(false)
  , error_exceeded_(false) {

  // Load params
  pnh_.param("loop_rate", loop_rate_, 20.0);
  pnh_.param("follower_watchdog_enabled", follower_watchdog_enabled_, false);
  pnh_.param("formation_error_watchdog_enabled", formation_error_watchdog_enabled_, false);
  pnh_.param("follower_lost_timeout", follower_lost_timeout_, 3.0);
  pnh_.param("max_formation_error", max_formation_error_, 2.0);
  pnh_.param("max_error_duration", max_error_duration_, 5.0);
  pnh_.param("max_linear_speed", max_linear_speed_, 0.8);
  pnh_.param("max_angular_speed", max_angular_speed_, 1.0);
  pnh_.param("speed_limit", speed_limit_, 0.6);
  pnh_.param("return_home_pos_tolerance", return_home_pos_tolerance_, 0.08);
  pnh_.param("return_home_yaw_tolerance", return_home_yaw_tolerance_, 0.12);
  pnh_.param("return_home_k_v", return_home_k_v_, 0.7);
  pnh_.param("return_home_k_w", return_home_k_w_, 1.0);
  pnh_.param("return_home_max_linear_speed", return_home_max_linear_speed_, 0.35);
  pnh_.param("return_home_max_angular_speed", return_home_max_angular_speed_, 0.55);
  pnh_.param("return_home_use_map", return_home_use_map_, true);
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("self_frame", self_frame_, "robot1/base_link");
  pnh_.param("circle_show_radius", circle_show_radius_, 0.5);
  pnh_.param("circle_show_angular_speed", circle_show_angular_speed_, 0.32);
  pnh_.param("circle_start_settle_error", circle_start_settle_error_, 0.16);
  pnh_.param("circle_start_settle_dwell", circle_start_settle_dwell_, 0.6);
  pnh_.param("circle_pause_error", circle_pause_error_, 0.30);
  pnh_.param("circle_exit_settle_error", circle_exit_settle_error_, 0.12);
  pnh_.param("circle_exit_settle_dwell", circle_exit_settle_dwell_, 1.0);
  pnh_.param("cmd_filter_alpha", cmd_filter_alpha_, 0.75);
  pnh_.param("cmd_slew_linear", cmd_slew_linear_, 0.8);
  pnh_.param("cmd_slew_angular", cmd_slew_angular_, 1.6);
  pnh_.param("adaptive_formation_speed_enabled",
             adaptive_formation_speed_enabled_, true);
  pnh_.param("formation_full_speed_error", formation_full_speed_error_, 0.15);
  pnh_.param("formation_min_speed_error", formation_min_speed_error_, 0.55);
  pnh_.param("formation_min_speed_scale", formation_min_speed_scale_, 0.35);
  pnh_.param("formation_status_timeout", formation_status_timeout_, 0.6);
  pnh_.param("formation_angular_scale_floor",
             formation_angular_scale_floor_, 0.65);
  pnh_.param("formation_speed_attack_alpha",
             formation_speed_attack_alpha_, 0.35);
  pnh_.param("formation_speed_release_alpha",
             formation_speed_release_alpha_, 0.08);
  adaptive_formation_speed_scale_ = 1.0;
  latest_formation_max_error_ = 0.0;
  formation_status_fresh_ = false;
  controller_start_time_ = ros::Time::now();

  std::string initial_mode;
  pnh_.param<std::string>("initial_mode", initial_mode, "idle");
  if (initial_mode == "teleop" || initial_mode == "1") {
    current_mode_ = cluster_msgs::LeaderCmd::MODE_TELEOP;
  } else if (initial_mode == "formation" || initial_mode == "2") {
    current_mode_ = cluster_msgs::LeaderCmd::MODE_FORMATION;
  } else if (initial_mode == "follow" || initial_mode == "3") {
    current_mode_ = cluster_msgs::LeaderCmd::MODE_FOLLOW;
  } else {
    current_mode_ = cluster_msgs::LeaderCmd::MODE_IDLE;
  }

  // Subscribers
  self_odom_sub_ = nh_.subscribe("/robot1/odom", 1,
      &LeaderController::selfOdomCallback, this);
  follower_odom_sub_ = nh_.subscribe("/robot2/odom", 1,
      &LeaderController::followerOdomCallback, this);
  follower_status_sub_ = nh_.subscribe("/robot2/follower_status", 1,
      &LeaderController::followerStatusCallback, this);
  follower3_status_sub_ = nh_.subscribe("/robot3/follower_status", 1,
      &LeaderController::follower3StatusCallback, this);
  teleop_vel_sub_ = nh_.subscribe("/robot1/teleop_vel", 1,
      &LeaderController::teleopVelCallback, this);
  nav_vel_sub_ = nh_.subscribe("/robot1/nav_vel", 1,
      &LeaderController::navVelCallback, this);
  return_home_sub_ = nh_.subscribe("/robot1/return_home", 1,
      &LeaderController::returnHomeCallback, this);
  circle_exit_sub_ = nh_.subscribe("/robot1/circle_exit", 1,
      &LeaderController::circleExitCallback, this);

  // Publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1);
  leader_cmd_pub_ = nh_.advertise<cluster_msgs::LeaderCmd>("/robot1/leader_cmd", 1);

  // Services
  set_mode_srv_ = nh_.advertiseService("/robot1/set_mode",
      &LeaderController::setModeCallback, this);
  set_formation_srv_ = nh_.advertiseService("/robot1/set_formation",
      &LeaderController::setFormationCallback, this);

  // Timer
  double dt = 1.0 / loop_rate_;
  control_timer_ = nh_.createTimer(ros::Duration(dt),
      &LeaderController::controlLoop, this);

  // Initialize cached leader cmd
  cached_leader_cmd_.mode = current_mode_;
  cached_leader_cmd_.formation = current_formation_;
  cached_leader_cmd_.speed_limit = speed_limit_;

  ROS_INFO("LeaderController initialized. Mode: %d, Formation: COLUMN",
           current_mode_);
}

void LeaderController::publishCmdVel(const geometry_msgs::Twist& cmd,
                                     bool reset_filter,
                                     bool smooth) {
  geometry_msgs::Twist target = cmd;
  target.linear.x = cluster_common::clamp(target.linear.x,
                                          -max_linear_speed_, max_linear_speed_);
  target.angular.z = cluster_common::clamp(target.angular.z,
                                           -max_angular_speed_, max_angular_speed_);

  ros::Time now = ros::Time::now();
  geometry_msgs::Twist out = target;

  if (smooth && !reset_filter && !last_cmd_vel_time_.isZero()) {
    double dt = (now - last_cmd_vel_time_).toSec();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / std::max(loop_rate_, 1.0);
    }

    const double alpha = cluster_common::clamp(cmd_filter_alpha_, 0.0, 1.0);
    const double filtered_vx =
        alpha * target.linear.x + (1.0 - alpha) * last_cmd_vel_.linear.x;
    const double filtered_wz =
        alpha * target.angular.z + (1.0 - alpha) * last_cmd_vel_.angular.z;

    const double max_dv = std::max(0.0, cmd_slew_linear_) * dt;
    const double max_dw = std::max(0.0, cmd_slew_angular_) * dt;
    out.linear.x = last_cmd_vel_.linear.x +
        cluster_common::clamp(filtered_vx - last_cmd_vel_.linear.x,
                              -max_dv, max_dv);
    out.angular.z = last_cmd_vel_.angular.z +
        cluster_common::clamp(filtered_wz - last_cmd_vel_.angular.z,
                              -max_dw, max_dw);
  }

  if (std::fabs(out.linear.x) < 0.01) out.linear.x = 0.0;
  if (std::fabs(out.angular.z) < 0.01) out.angular.z = 0.0;

  cmd_vel_pub_.publish(out);
  last_cmd_vel_ = out;
  last_cmd_vel_time_ = now;
}

// ---------- Callbacks ----------

void LeaderController::selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_self_odom_ = *msg;
  self_odom_received_ = true;
  if (!home_pose_received_) {
    ensureHomePose();
  }
}

void LeaderController::followerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_follower_odom_ = *msg;
  follower_odom_received_ = true;
}

void LeaderController::followerStatusCallback(
    const cluster_msgs::FollowerStatus::ConstPtr& msg) {
  latest_follower_status_ = *msg;
  follower_status_received_ = true;
  last_follower_status_time_ = ros::Time::now();
}

void LeaderController::follower3StatusCallback(
    const cluster_msgs::FollowerStatus::ConstPtr& msg) {
  latest_follower3_status_ = *msg;
  follower3_status_received_ = true;
  last_follower3_status_time_ = ros::Time::now();
}

void LeaderController::updateAdaptiveFormationSpeedScale() {
  if (!adaptive_formation_speed_enabled_ ||
      current_mode_ != cluster_msgs::LeaderCmd::MODE_FORMATION) {
    adaptive_formation_speed_scale_ = 1.0;
    return;
  }

  const ros::Time now = ros::Time::now();
  double max_error = 0.0;
  bool has_fresh_status = false;
  if (follower_status_received_ &&
      (now - last_follower_status_time_).toSec() <= formation_status_timeout_ &&
      std::isfinite(latest_follower_status_.error_dist)) {
    max_error = std::max(max_error, latest_follower_status_.error_dist);
    has_fresh_status = true;
  }
  if (follower3_status_received_ &&
      (now - last_follower3_status_time_).toSec() <= formation_status_timeout_ &&
      std::isfinite(latest_follower3_status_.error_dist)) {
    max_error = std::max(max_error, latest_follower3_status_.error_dist);
    has_fresh_status = true;
  }

  latest_formation_max_error_ = max_error;
  formation_status_fresh_ = has_fresh_status;

  double target_scale = 1.0;
  if (has_fresh_status && max_error > formation_full_speed_error_) {
    const double span = std::max(
        formation_min_speed_error_ - formation_full_speed_error_, 0.01);
    const double ratio = cluster_common::clamp(
        (max_error - formation_full_speed_error_) / span, 0.0, 1.0);
    target_scale = 1.0 - ratio * (1.0 - formation_min_speed_scale_);
  }

  const double alpha = target_scale < adaptive_formation_speed_scale_
      ? formation_speed_attack_alpha_
      : formation_speed_release_alpha_;
  adaptive_formation_speed_scale_ +=
      cluster_common::clamp(alpha, 0.0, 1.0) *
      (target_scale - adaptive_formation_speed_scale_);
  adaptive_formation_speed_scale_ = cluster_common::clamp(
      adaptive_formation_speed_scale_, formation_min_speed_scale_, 1.0);

  if (adaptive_formation_speed_scale_ < 0.95) {
    ROS_INFO_THROTTLE(2.0,
        "Adaptive formation speed: scale=%.2f max_follower_error=%.2fm",
        adaptive_formation_speed_scale_, max_error);
  }
}

geometry_msgs::Twist LeaderController::applyAdaptiveFormationSpeed(
    const geometry_msgs::Twist& cmd) const {
  if (!adaptive_formation_speed_enabled_ ||
      current_mode_ != cluster_msgs::LeaderCmd::MODE_FORMATION) {
    return cmd;
  }

  geometry_msgs::Twist scaled = cmd;
  scaled.linear.x *= adaptive_formation_speed_scale_;
  const double angular_scale = formation_angular_scale_floor_ +
      (1.0 - formation_angular_scale_floor_) * adaptive_formation_speed_scale_;
  scaled.angular.z *= cluster_common::clamp(angular_scale, 0.0, 1.0);
  return scaled;
}

void LeaderController::teleopVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  latest_teleop_cmd_ = *msg;
  teleop_cmd_received_ = true;
  last_teleop_cmd_time_ = ros::Time::now();

  if (circle_recovery_.phase() != CircleShowPhase::NORMAL) {
    return;
  }

  if (!return_home_active_ && current_mode_ != cluster_msgs::LeaderCmd::MODE_IDLE) {
    publishCmdVel(applyAdaptiveFormationSpeed(*msg), false, false);
  }
}

void LeaderController::navVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  latest_teleop_cmd_ = *msg;
  teleop_cmd_received_ = true;
  last_teleop_cmd_time_ = ros::Time::now();

  if (circle_recovery_.phase() != CircleShowPhase::NORMAL) {
    return;
  }

  if (!return_home_active_ &&
      current_mode_ != cluster_msgs::LeaderCmd::MODE_IDLE) {
    // DWA closes its control loop using the velocity it requested. Scaling
    // translation and rotation independently here changes the trajectory and
    // can trigger false oscillation detection. Car1 navigation is already
    // speed-limited by the DWA configuration.
    publishCmdVel(*msg, false, false);
  }
}

void LeaderController::returnHomeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) {
    return_home_active_ = false;
    geometry_msgs::Twist zero_cmd;
    publishCmdVel(zero_cmd, true, false);
    return;
  }
  if (!ensureHomePose()) {
    ROS_WARN("Return home requested before home pose is available");
    return;
  }
  if (!self_odom_received_) {
    ROS_WARN("Return home requested but /robot1/odom is not available");
    return;
  }
  cluster_common::Pose2D self;
  if (return_home_use_map_) {
    if (!lookupSelfMapPose(self)) {
      ROS_WARN("Return home requested but %s -> %s TF is not available",
               map_frame_.c_str(), self_frame_.c_str());
      return;
    }
  } else {
    self = cluster_common::odomToPose2D(latest_self_odom_);
  }
  const double dx = home_pose_.x - self.x;
  const double dy = home_pose_.y - self.y;
  ROS_INFO("Return home requested: current=(%.2f, %.2f, %.2f), home=(%.2f, %.2f, %.2f), dist=%.2f",
           self.x, self.y, self.theta,
           home_pose_.x, home_pose_.y, home_pose_.theta,
           std::sqrt(dx * dx + dy * dy));
  return_home_active_ = true;
  current_mode_ = cluster_msgs::LeaderCmd::MODE_TELEOP;
  circle_recovery_.abort();
}

void LeaderController::circleExitCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;
  if (circle_recovery_.requestExit()) {
    ROS_INFO("CIRCLE_SHOW exit requested; recovering formation %u",
             circle_recovery_.recoveryFormation());
  }
}

bool LeaderController::followerSettled(
    const cluster_msgs::FollowerStatus& status,
    const ros::Time& receipt_time, const ros::Time& now,
    double settle_error) const {
  if (receipt_time.isZero() ||
      (now - receipt_time).toSec() > formation_status_timeout_ ||
      !std::isfinite(status.error_dist)) {
    return false;
  }
  if (status.state == cluster_msgs::FollowerStatus::STATE_LOST ||
      status.state == cluster_msgs::FollowerStatus::STATE_EMERGENCY) {
    return false;
  }
  return std::fabs(status.error_dist) <= settle_error;
}

// ---------- Service Callbacks ----------

bool LeaderController::setModeCallback(
    cluster_msgs::SetMode::Request& req,
    cluster_msgs::SetMode::Response& res) {

  if (req.mode > cluster_msgs::LeaderCmd::MODE_FOLLOW) {
    res.success = false;
    res.message = "Invalid mode. Valid: 0=IDLE, 1=TELEOP, 2=FORMATION, 3=FOLLOW";
    return true;
  }

  ROS_INFO("SetMode: %d -> %d", current_mode_, req.mode);
  if (req.mode != cluster_msgs::LeaderCmd::MODE_FORMATION) {
    circle_recovery_.abort();
  }

  if (req.mode == cluster_msgs::LeaderCmd::MODE_FORMATION &&
      circle_recovery_.phase() != CircleShowPhase::NORMAL) {
    res.success = false;
    res.message = "CIRCLE_SHOW is active or recovering; use circle_exit first";
    return true;
  }
  current_mode_ = req.mode;

  if (req.mode == cluster_msgs::LeaderCmd::MODE_FORMATION) {
    if (req.formation <= cluster_msgs::LeaderCmd::FORMATION_TRIANGLE) {
      current_formation_ = req.formation;
      if (current_formation_ != cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW) {
        last_non_circle_formation_ = current_formation_;
      }
    }
    // Check for custom offsets
    if (req.offset_x != 0.0 || req.offset_y != 0.0 || req.offset_yaw != 0.0) {
      use_custom_offsets_ = true;
      custom_offset_.x = req.offset_x;
      custom_offset_.y = req.offset_y;
      custom_offset_.yaw = req.offset_yaw;
    } else {
      use_custom_offsets_ = false;
    }
    if (current_formation_ == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW &&
        !use_custom_offsets_) {
      circle_recovery_.enter(last_non_circle_formation_);
    }
  }

  res.success = true;
  res.message = "Mode set successfully";
  return true;
}

bool LeaderController::setFormationCallback(
    cluster_msgs::SetFormation::Request& req,
    cluster_msgs::SetFormation::Response& res) {

  if (req.formation > cluster_msgs::LeaderCmd::FORMATION_TRIANGLE) {
    res.success = false;
    res.message = "Invalid formation. Valid: 0=COLUMN, 1=LINE, 2=CIRCLE_SHOW, 3=TRIANGLE";
    return true;
  }

  if (circle_recovery_.phase() != CircleShowPhase::NORMAL) {
    res.success = false;
    res.message = "CIRCLE_SHOW is active or recovering; use circle_exit first";
    return true;
  }

  current_formation_ = req.formation;
  if (current_formation_ != cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW) {
    last_non_circle_formation_ = current_formation_;
  }
  current_mode_ = cluster_msgs::LeaderCmd::MODE_FORMATION;
  use_custom_offsets_ = false;
  if (current_formation_ == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW) {
    circle_recovery_.enter(last_non_circle_formation_);
  }
  ROS_INFO("SetFormation: %d, forcing FORMATION mode", current_formation_);

  res.success = true;
  res.message = "Formation set successfully";
  return true;
}

// ---------- Control Loop ----------

void LeaderController::controlLoop(const ros::TimerEvent& event) {
  checkSafety();
  updateAdaptiveFormationSpeedScale();

  if (current_mode_ != cluster_msgs::LeaderCmd::MODE_FORMATION) {
    circle_recovery_.abort();
  }

  if (return_home_active_) {
    computeReturnHome();
    return;
  }

  if (current_mode_ != cluster_msgs::LeaderCmd::MODE_FORMATION &&
      circle_show_was_active_) {
    geometry_msgs::Twist zero_cmd;
    publishCmdVel(zero_cmd, true, false);
    circle_show_was_active_ = false;
  }

  // Compute target and publish LeaderCmd
  switch (current_mode_) {
    case cluster_msgs::LeaderCmd::MODE_FORMATION:
      computeFormationTarget();
      break;
    case cluster_msgs::LeaderCmd::MODE_FOLLOW:
      computeFollowTarget();
      break;
    case cluster_msgs::LeaderCmd::MODE_IDLE:
    case cluster_msgs::LeaderCmd::MODE_TELEOP:
    default:
      // Publish idle/teleop leader cmd (no target)
      cached_leader_cmd_.header.stamp = ros::Time::now();
      cached_leader_cmd_.mode = current_mode_;
      cached_leader_cmd_.formation = current_formation_;
      cached_leader_cmd_.target_pose = geometry_msgs::Pose();
      leader_cmd_pub_.publish(cached_leader_cmd_);
      break;
  }
}

void LeaderController::computeReturnHome() {
  if (!self_odom_received_ || !ensureHomePose()) {
    ROS_WARN_THROTTLE(2.0, "Return home skipped: odom/home pose not ready");
    return_home_active_ = false;
    return;
  }

  cluster_common::Pose2D self;
  if (return_home_use_map_) {
    if (!lookupSelfMapPose(self)) {
      ROS_WARN_THROTTLE(2.0, "Return home skipped: %s -> %s TF not ready",
                        map_frame_.c_str(), self_frame_.c_str());
      geometry_msgs::Twist zero_cmd;
      publishCmdVel(zero_cmd, true, false);
      return;
    }
  } else {
    self = cluster_common::odomToPose2D(latest_self_odom_);
  }
  const double dx = home_pose_.x - self.x;
  const double dy = home_pose_.y - self.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double target_heading = std::atan2(dy, dx);
  const double heading_err = cluster_common::normalizeAngle(target_heading - self.theta);
  const double yaw_err = cluster_common::normalizeAngle(home_pose_.theta - self.theta);

  geometry_msgs::Twist cmd;
  const double linear_limit =
      std::min(max_linear_speed_, return_home_max_linear_speed_);
  const double angular_limit =
      std::min(max_angular_speed_, return_home_max_angular_speed_);
  if (distance > return_home_pos_tolerance_) {
    const double speed_scale = std::max(0.0, std::cos(heading_err));
    cmd.linear.x = cluster_common::clamp(return_home_k_v_ * distance * speed_scale,
                                         0.0, linear_limit);
    cmd.angular.z = cluster_common::clamp(return_home_k_w_ * heading_err,
                                          -angular_limit, angular_limit);
  } else if (std::fabs(yaw_err) > return_home_yaw_tolerance_) {
    cmd.angular.z = cluster_common::clamp(return_home_k_w_ * yaw_err,
                                          -angular_limit, angular_limit);
  } else {
    return_home_active_ = false;
    publishCmdVel(cmd, true, false);
    ROS_INFO("Return home complete");
    return;
  }

  publishCmdVel(cmd);

  cached_leader_cmd_.header.stamp = ros::Time::now();
  cached_leader_cmd_.mode = cluster_msgs::LeaderCmd::MODE_TELEOP;
  cached_leader_cmd_.formation = current_formation_;
  cached_leader_cmd_.leader_vx = cmd.linear.x;
  cached_leader_cmd_.leader_vyaw = cmd.angular.z;
  cached_leader_cmd_.speed_limit = speed_limit_;
  leader_cmd_pub_.publish(cached_leader_cmd_);
}

bool LeaderController::lookupSelfMapPose(cluster_common::Pose2D& pose) {
  try {
    geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
        map_frame_, self_frame_, ros::Time(0), ros::Duration(0.05));
    pose.x = tf_msg.transform.translation.x;
    pose.y = tf_msg.transform.translation.y;
    pose.theta = tf2::getYaw(tf_msg.transform.rotation);
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.theta);
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(3.0, "Cannot lookup home/map pose %s -> %s: %s",
                      map_frame_.c_str(), self_frame_.c_str(), ex.what());
    return false;
  }
}

bool LeaderController::ensureHomePose() {
  if (home_pose_received_) {
    return true;
  }

  if (return_home_use_map_) {
    cluster_common::Pose2D map_pose;
    if (!lookupSelfMapPose(map_pose)) {
      return false;
    }
    home_pose_ = map_pose;
    home_pose_received_ = true;
    ROS_INFO("Recorded robot1 map home pose: x=%.2f y=%.2f yaw=%.2f",
             home_pose_.x, home_pose_.y, home_pose_.theta);
    return true;
  }

  if (!self_odom_received_) {
    return false;
  }
  home_pose_ = cluster_common::odomToPose2D(latest_self_odom_);
  home_pose_received_ = true;
  ROS_INFO("Recorded robot1 odom home pose: x=%.2f y=%.2f yaw=%.2f",
           home_pose_.x, home_pose_.y, home_pose_.theta);
  return true;
}

void LeaderController::computeFormationTarget() {
  if (!self_odom_received_) {
    ROS_WARN_THROTTLE(2.0, "Formation skipped: /robot1/odom not received");
    return;
  }

  const bool recovering =
      circle_recovery_.phase() == CircleShowPhase::RECOVERING;
  if (recovering) {
    current_formation_ = circle_recovery_.recoveryFormation();
    use_custom_offsets_ = false;
    geometry_msgs::Twist stop;
    publishCmdVel(stop, true, false);
    circle_show_was_active_ = false;
    const ros::Time now = ros::Time::now();
    const bool robot2_settled = followerSettled(
        latest_follower_status_, last_follower_status_time_, now,
        circle_exit_settle_error_);
    const bool robot3_settled = followerSettled(
        latest_follower3_status_, last_follower3_status_time_, now,
        circle_exit_settle_error_);
    if (circle_recovery_.updateRecovery(robot2_settled, robot3_settled,
                                        now.toSec(), circle_exit_settle_dwell_)) {
      ROS_INFO("CIRCLE_SHOW recovery complete in formation %u",
               current_formation_);
    }
  }

  const bool circle_preparing =
      current_formation_ == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW &&
      !use_custom_offsets_ &&
      circle_recovery_.phase() == CircleShowPhase::PREPARING;
  if (circle_preparing) {
    geometry_msgs::Twist stop;
    publishCmdVel(stop, true, false);
    circle_show_was_active_ = false;
    const ros::Time now = ros::Time::now();
    const bool robot2_settled = followerSettled(
        latest_follower_status_, last_follower_status_time_, now,
        circle_start_settle_error_);
    const bool robot3_settled = followerSettled(
        latest_follower3_status_, last_follower3_status_time_, now,
        circle_start_settle_error_);
    if (circle_recovery_.updateStart(robot2_settled, robot3_settled,
                                     now.toSec(), circle_start_settle_dwell_)) {
      ROS_INFO("CIRCLE_SHOW synchronized start");
    }
  }

  auto leader_pose = cluster_common::odomToPose2D(latest_self_odom_);
  const bool circle_show =
      current_formation_ == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW &&
      !use_custom_offsets_ &&
      circle_recovery_.phase() == CircleShowPhase::ACTIVE;
  geometry_msgs::Twist show_cmd;
  if (circle_show) {
    show_cmd.linear.x = cluster_common::clamp(
        circle_show_radius_ * circle_show_angular_speed_,
        0.0, max_linear_speed_);
    show_cmd.angular.z = cluster_common::clamp(
        circle_show_angular_speed_,
        -max_angular_speed_, max_angular_speed_);
    double circle_scale = adaptive_formation_speed_enabled_
        ? adaptive_formation_speed_scale_ : 1.0;
    if (formation_status_fresh_ &&
        latest_formation_max_error_ > circle_pause_error_) {
      circle_scale = 0.0;
      ROS_WARN_THROTTLE(1.0,
          "CIRCLE_SHOW paused: follower error %.2fm exceeds %.2fm",
          latest_formation_max_error_, circle_pause_error_);
    }
    const CircleCommand scaled = CircleShowRecovery::scaleCircleCommand(
        {show_cmd.linear.x, show_cmd.angular.z}, circle_scale);
    show_cmd.linear.x = scaled.linear_x;
    show_cmd.angular.z = scaled.angular_z;
    publishCmdVel(show_cmd);
    circle_show_was_active_ = true;
  } else if (circle_show_was_active_) {
    publishCmdVel(show_cmd, true, false);
    circle_show_was_active_ = false;
  }

  // Get formation offset
  FormationOffset offset = use_custom_offsets_
      ? custom_offset_
      : getFormationOffset(current_formation_);

  // Compute target pose in leader body → world frame
  geometry_msgs::Pose target = computeTargetPose(leader_pose, offset);

  // Populate LeaderCmd
  cached_leader_cmd_.header.stamp = ros::Time::now();
  cached_leader_cmd_.mode = cluster_msgs::LeaderCmd::MODE_FORMATION;
  cached_leader_cmd_.formation = current_formation_;
  cached_leader_cmd_.offset_x = offset.x;
  cached_leader_cmd_.offset_y = offset.y;
  cached_leader_cmd_.offset_yaw = offset.yaw;
  bool teleop_fresh = teleop_cmd_received_ &&
      (ros::Time::now() - last_teleop_cmd_time_).toSec() < 0.3;
  cached_leader_cmd_.leader_vx = circle_show
      ? show_cmd.linear.x
      : circle_preparing
      ? 0.0
      : teleop_fresh
      ? last_cmd_vel_.linear.x
      : latest_self_odom_.twist.twist.linear.x;
  cached_leader_cmd_.leader_vyaw = circle_show
      ? show_cmd.angular.z
      : circle_preparing
      ? 0.0
      : teleop_fresh
      ? last_cmd_vel_.angular.z
      : latest_self_odom_.twist.twist.angular.z;
  cached_leader_cmd_.target_pose = target;
  cached_leader_cmd_.speed_limit =
      speed_limit_ * adaptive_formation_speed_scale_;

  leader_cmd_pub_.publish(cached_leader_cmd_);
}

void LeaderController::computeFollowTarget() {
  if (!self_odom_received_) return;

  auto leader_pose = cluster_common::odomToPose2D(latest_self_odom_);

  // In follow mode, we don't pre-compute a target position.
  // The follower maintains its own trajectory buffer.
  // We just publish leader state so follower can record trajectory.
  cached_leader_cmd_.header.stamp = ros::Time::now();
  cached_leader_cmd_.mode = cluster_msgs::LeaderCmd::MODE_FOLLOW;
  cached_leader_cmd_.formation = 0;
  bool teleop_fresh = teleop_cmd_received_ &&
      (ros::Time::now() - last_teleop_cmd_time_).toSec() < 0.3;
  cached_leader_cmd_.leader_vx = teleop_fresh
      ? latest_teleop_cmd_.linear.x
      : latest_self_odom_.twist.twist.linear.x;
  cached_leader_cmd_.leader_vyaw = teleop_fresh
      ? latest_teleop_cmd_.angular.z
      : latest_self_odom_.twist.twist.angular.z;
  // target_pose is not used in follow mode (follower tracks trajectory buffer)
  cached_leader_cmd_.target_pose = geometry_msgs::Pose();
  cached_leader_cmd_.speed_limit = speed_limit_;

  leader_cmd_pub_.publish(cached_leader_cmd_);
}

void LeaderController::checkSafety() {
  if (!follower_watchdog_enabled_ && !formation_error_watchdog_enabled_) {
    return;
  }

  ros::Time now = ros::Time::now();

  // Check follower connection
  bool follower_lost = false;
  if (follower_status_received_) {
    double dt = (now - last_follower_status_time_).toSec();
    if (dt > follower_lost_timeout_) {
      follower_lost = true;
    }
  } else {
    // No status received at all yet — not an error if we just started
    if (self_odom_received_) {
      double elapsed = (now - ros::Time(
          latest_self_odom_.header.stamp)).toSec();
      // If we've been getting odom for > 5s but no follower status
      if (elapsed > 5.0 && !follower_status_received_) {
        follower_lost = true;
      }
    }
  }
  if (!follower_status_received_ && self_odom_received_ &&
      (now - controller_start_time_).toSec() > 5.0) {
    follower_lost = true;
  }
  if (follower3_status_received_) {
    double dt = (now - last_follower3_status_time_).toSec();
    if (dt > follower_lost_timeout_) {
      follower_lost = true;
    }
  } else if (self_odom_received_) {
    double elapsed = (now - ros::Time(
        latest_self_odom_.header.stamp)).toSec();
    if (elapsed > 5.0 && !follower3_status_received_) {
      follower_lost = true;
    }
  }

  if (!follower3_status_received_ && self_odom_received_ &&
      (now - controller_start_time_).toSec() > 5.0) {
    follower_lost = true;
  }

  // Check formation error
  if (formation_error_watchdog_enabled_ &&
      current_mode_ == cluster_msgs::LeaderCmd::MODE_FORMATION) {
    double error = 0.0;
    bool has_error = false;
    if (follower_status_received_) {
      error = std::max(error, latest_follower_status_.error_dist);
      has_error = true;
    }
    if (follower3_status_received_) {
      error = std::max(error, latest_follower3_status_.error_dist);
      has_error = true;
    }
    if (has_error && error > max_formation_error_) {
      if (!error_exceeded_) {
        error_exceeded_ = true;
        error_start_time_ = now;
      } else if ((now - error_start_time_).toSec() > max_error_duration_) {
        ROS_WARN("Formation error %.2f exceeded max %.2f for %.1fs. Stopping.",
                 error, max_formation_error_, max_error_duration_);
        current_mode_ = cluster_msgs::LeaderCmd::MODE_IDLE;
      }
    } else if (has_error) {
      error_exceeded_ = false;
    }
  }

  if (follower_watchdog_enabled_ &&
      follower_lost && current_mode_ != cluster_msgs::LeaderCmd::MODE_IDLE &&
      current_mode_ != cluster_msgs::LeaderCmd::MODE_TELEOP) {
    ROS_WARN_THROTTLE(2.0, "Follower connection lost! Forcing IDLE.");
    current_mode_ = cluster_msgs::LeaderCmd::MODE_IDLE;
    geometry_msgs::Twist zero_cmd;
    publishCmdVel(zero_cmd, true, false);
  }
}

// ---------- Pose Computation ----------

geometry_msgs::Pose LeaderController::computeTargetPose(
    const cluster_common::Pose2D& leader_pose,
    const FormationOffset& offset) {

  double cos_theta = std::cos(leader_pose.theta);
  double sin_theta = std::sin(leader_pose.theta);

  cluster_common::Pose2D target;
  target.x = leader_pose.x + offset.x * cos_theta - offset.y * sin_theta;
  target.y = leader_pose.y + offset.x * sin_theta + offset.y * cos_theta;
  target.theta = cluster_common::normalizeAngle(leader_pose.theta + offset.yaw);

  return cluster_common::pose2DToGeometryMsg(target);
}

FormationOffset LeaderController::getFormationOffset(uint8_t formation_type) {
  FormationOffset offset;
  // Offsets in leader body frame: X=forward, Y=left
  switch (formation_type) {
    case cluster_msgs::LeaderCmd::FORMATION_COLUMN:
      offset = {-0.8, 0.0, 0.0};
      break;
    case cluster_msgs::LeaderCmd::FORMATION_LINE:
      offset = {0.0, -0.8, 0.0};
      break;
    case cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW: {
      const double phase = 2.0943951023931953;  // 120 degrees behind car1.
      const double r = circle_show_radius_;
      offset = {-r * std::sin(phase), r * (1.0 - std::cos(phase)), -phase};
      break;
    }
    case cluster_msgs::LeaderCmd::FORMATION_TRIANGLE:
      offset = {-0.8, -0.8, 0.0};
      break;
    default:
      offset = {-0.8, 0.0, 0.0};
      break;
  }
  return offset;
}

void LeaderController::spin() {
  ros::spin();
}

}  // namespace cluster_formation
