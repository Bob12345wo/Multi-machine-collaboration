#include "cluster_following/follower_controller.h"
#include <cmath>
#include <algorithm>

namespace cluster_following {

FollowerController::FollowerController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  , self_odom_received_(false)
  , leader_odom_received_(false)
  , leader_cmd_received_(false)
  , current_mode_(cluster_msgs::LeaderCmd::MODE_IDLE)
  , calibrated_(false)
  , frame_shift_x_(0.0)
  , frame_shift_y_(0.0)
  , frame_shift_theta_(0.0)
  , prev_vz_(0.0)
  , last_error_x_(0.0)
  , last_error_y_(0.0)
  , last_error_yaw_(0.0)
  , last_error_dist_(0.0) {

  pnh_.param("loop_rate", loop_rate_, 20.0);
  pnh_.param("max_linear_speed", max_linear_speed_, 0.8);
  pnh_.param("max_angular_speed", max_angular_speed_, 1.0);
  pnh_.param("leader_lost_timeout", leader_lost_timeout_, 2.0);
  pnh_.param("pos_deadband", pos_deadband_, 0.05);
  pnh_.param("yaw_deadband", yaw_deadband_, 0.05);
  pnh_.param("min_linear_speed", min_linear_speed_, 0.08);
  pnh_.param("reverse_entry_distance", reverse_entry_distance_, 0.10);
  pnh_.param("leader_vx_feedforward_gain", leader_vx_feedforward_gain_, 0.85);
  pnh_.param("leader_wz_feedforward_gain", leader_wz_feedforward_gain_, 1.0);
  pnh_.param("yaw_error_gain", yaw_error_gain_, 0.9);
  pnh_.param("lateral_error_gain", lateral_error_gain_, 0.25);
  pnh_.param("turn_yaw_correction_scale", turn_yaw_correction_scale_, 0.15);
  pnh_.param("turn_lateral_correction_scale", turn_lateral_correction_scale_, 0.5);
  pnh_.param("turn_in_place_threshold", turn_in_place_threshold_, 0.05);
  pnh_.param("allow_reverse_while_leader_forward",
             allow_reverse_while_leader_forward_, false);

  pnh_.param("follow_delay_seconds", follow_delay_, 2.0);
  pnh_.param("max_buffer_size", max_buffer_size_, 500);

  double lin_kp, lin_ki, lin_kd, lin_i_max, lin_out_max;
  double ang_kp, ang_ki, ang_kd, ang_i_max, ang_out_max;

  pnh_.param("linear_pid/kp", lin_kp, 1.0);
  pnh_.param("linear_pid/ki", lin_ki, 0.01);
  pnh_.param("linear_pid/kd", lin_kd, 0.1);
  pnh_.param("linear_pid/i_max", lin_i_max, 0.1);   // reduced from 0.3
  pnh_.param("linear_pid/output_max", lin_out_max, 0.8);

  pnh_.param("angular_pid/kp", ang_kp, 0.8);        // reduced from 1.2
  pnh_.param("angular_pid/ki", ang_ki, 0.0);
  pnh_.param("angular_pid/kd", ang_kd, 0.4);        // increased from 0.3
  pnh_.param("angular_pid/i_max", ang_i_max, 0.05); // reduced from 0.2
  pnh_.param("angular_pid/output_max", ang_out_max, 1.0);

  lin_pid_ = cluster_common::PID(lin_kp, lin_ki, lin_kd, lin_i_max, lin_out_max);
  ang_pid_ = cluster_common::PID(ang_kp, ang_ki, ang_kd, ang_i_max, ang_out_max);

  leader_odom_sub_ = nh_.subscribe("/robot1/odom", 10,
      &FollowerController::leaderOdomCallback, this);
  self_odom_sub_ = nh_.subscribe("/robot2/odom", 10,
      &FollowerController::selfOdomCallback, this);
  leader_cmd_sub_ = nh_.subscribe("/robot1/leader_cmd", 10,
      &FollowerController::leaderCmdCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 10);
  status_pub_ = nh_.advertise<cluster_msgs::FollowerStatus>("/robot2/follower_status", 10);

  double dt = 1.0 / loop_rate_;
  control_timer_ = nh_.createTimer(ros::Duration(dt),
      &FollowerController::controlLoop, this);
  status_timer_ = nh_.createTimer(ros::Duration(0.1),
      &FollowerController::statusLoop, this);

  ROS_INFO("FollowerController initialized. Loop rate: %.1f Hz", loop_rate_);
}

// ---------- Callbacks ----------

void FollowerController::leaderOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_leader_odom_ = *msg;
  leader_odom_received_ = true;
  last_leader_odom_time_ = msg->header.stamp;

  if (current_mode_ == cluster_msgs::LeaderCmd::MODE_FOLLOW) {
    TrajectoryPoint pt;
    pt.stamp = msg->header.stamp;
    auto pose = cluster_common::odomToPose2D(*msg);
    pt.x = pose.x;
    pt.y = pose.y;
    pt.theta = pose.theta;
    trajectory_buffer_.push_back(pt);
    ros::Time cutoff = pt.stamp - ros::Duration(follow_delay_);
    while (!trajectory_buffer_.empty() && trajectory_buffer_.front().stamp < cutoff)
      trajectory_buffer_.pop_front();
    while (trajectory_buffer_.size() > static_cast<size_t>(max_buffer_size_))
      trajectory_buffer_.pop_front();
  }
}

void FollowerController::selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_self_odom_ = *msg;
  self_odom_received_ = true;
}

void FollowerController::leaderCmdCallback(const cluster_msgs::LeaderCmd::ConstPtr& msg) {
  latest_leader_cmd_ = *msg;
  leader_cmd_received_ = true;
  last_leader_cmd_time_ = msg->header.stamp;

  if (msg->mode != current_mode_) {
    ROS_INFO("Follower mode transition: %d -> %d", current_mode_, msg->mode);
    current_mode_ = msg->mode;

    if (current_mode_ == cluster_msgs::LeaderCmd::MODE_FOLLOW)
      trajectory_buffer_.clear();

    // Reset PID and recalibrate when entering formation
    lin_pid_.reset();
    ang_pid_.reset();
    prev_vz_ = 0.0;
    if (current_mode_ == cluster_msgs::LeaderCmd::MODE_FORMATION)
      calibrated_ = false;  // re-calibrate on every formation entry
  }
}

// ---------- Timers ----------

void FollowerController::controlLoop(const ros::TimerEvent& event) {
  ros::Time now = ros::Time::now();
  double leader_age = (now - last_leader_odom_time_).toSec();
  bool leader_visible = (leader_age < leader_lost_timeout_);

  if (current_mode_ == cluster_msgs::LeaderCmd::MODE_IDLE ||
      current_mode_ == cluster_msgs::LeaderCmd::MODE_TELEOP) {
    stop();
    return;
  }

  if (!self_odom_received_ || !leader_odom_received_) { stop(); return; }

  if (!leader_visible) {
    ROS_WARN_THROTTLE(2.0, "Leader lost! Stopping.");
    stop();
    return;
  }

  // Only calibrate in formation mode, while stationary
  if (!calibrated_ && current_mode_ == cluster_msgs::LeaderCmd::MODE_FORMATION) {
    tryCalibrate();
  }

  if (!calibrated_ && current_mode_ == cluster_msgs::LeaderCmd::MODE_FOLLOW) {
    // FOLLOW mode tracks leader odometry history directly. If the operator did
    // not enter FORMATION first, use the shared odom frame without an offset.
    calibrated_ = true;
    frame_shift_x_ = 0.0;
    frame_shift_y_ = 0.0;
    frame_shift_theta_ = 0.0;
  }
  if (!calibrated_) return;

  switch (current_mode_) {
    case cluster_msgs::LeaderCmd::MODE_FORMATION:
      trackTargetPose();
      break;
    case cluster_msgs::LeaderCmd::MODE_FOLLOW:
      followLeaderPath();
      break;
    default:
      stop();
      break;
  }
}

void FollowerController::statusLoop(const ros::TimerEvent& event) {
  cluster_msgs::FollowerStatus status;
  status.header.stamp = ros::Time::now();
  ros::Time now = ros::Time::now();
  bool leader_visible = (now - last_leader_odom_time_).toSec() < leader_lost_timeout_;

  if (!leader_visible)
    status.state = cluster_msgs::FollowerStatus::STATE_LOST;
  else if (current_mode_ == cluster_msgs::LeaderCmd::MODE_IDLE)
    status.state = cluster_msgs::FollowerStatus::STATE_IDLE;
  else
    status.state = cluster_msgs::FollowerStatus::STATE_TRACKING;

  status.error_x = last_error_x_;
  status.error_y = last_error_y_;
  status.error_yaw = last_error_yaw_;
  status.error_dist = last_error_dist_;
  status.leader_visible = leader_visible;
  status_pub_.publish(status);
}

// ---------- Formation Tracking ----------

void FollowerController::trackTargetPose() {
  if (!leader_cmd_received_) { stop(); return; }

  auto leader = cluster_common::odomToPose2D(latest_leader_odom_);
  auto self = cluster_common::odomToPose2D(latest_self_odom_);

  double ox = latest_leader_cmd_.offset_x;
  double oy = latest_leader_cmd_.offset_y;
  double oyaw = latest_leader_cmd_.offset_yaw;

  double cos_l = std::cos(leader.theta);
  double sin_l = std::sin(leader.theta);

  double tx = leader.x + ox * cos_l - oy * sin_l;
  double ty = leader.y + ox * sin_l + oy * cos_l;
  double ttheta = cluster_common::normalizeAngle(leader.theta + oyaw);

  // SE(2) transform: rotate then translate
  double cos_fs = std::cos(frame_shift_theta_);
  double sin_fs = std::sin(frame_shift_theta_);
  double tx_rot = tx * cos_fs - ty * sin_fs;
  double ty_rot = tx * sin_fs + ty * cos_fs;
  tx = tx_rot + frame_shift_x_;
  ty = ty_rot + frame_shift_y_;
  ttheta = cluster_common::normalizeAngle(ttheta + frame_shift_theta_);

  // --- Error in follower BODY frame ---
  double dx = tx - self.x;
  double dy = ty - self.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Project error onto follower's heading
  double cos_s = std::cos(self.theta);
  double sin_s = std::sin(self.theta);
  double forward_err = dx * cos_s + dy * sin_s;    // + ahead, - behind
  double lateral_err = -dx * sin_s + dy * cos_s;   // + left, - right

  double leader_vx = latest_leader_cmd_.leader_vx;
  double leader_wz = latest_leader_cmd_.leader_vyaw;

  // Heading error: direction to target from follower
  double heading_to_target = std::atan2(dy, dx);
  double yaw_err = cluster_common::normalizeAngle(ttheta - self.theta);
  bool reverse_tracking =
      (forward_err < -reverse_entry_distance_ || leader_vx < -0.03) &&
      std::fabs(yaw_err) < M_PI * 0.5;
  double tracking_heading = reverse_tracking
      ? cluster_common::normalizeAngle(heading_to_target + M_PI)
      : heading_to_target;
  double heading_err = cluster_common::normalizeAngle(tracking_heading - self.theta);
  double motion_sign = reverse_tracking ? -1.0 : 1.0;

  updateTrackingError(lateral_err, forward_err, yaw_err, distance);

  // --- Deadband check ---
  if (distance < pos_deadband_ && std::fabs(yaw_err) < yaw_deadband_) {
    publishCmdVel(0.0, 0.0);
    lin_pid_.reset();
    ang_pid_.reset();
    prev_vz_ = 0.0;
    return;
  }

  // --- Linear velocity: leader feed-forward plus formation error feedback ---
  double dt = 1.0 / loop_rate_;
  double vx = leader_vx_feedforward_gain_ * leader_vx + lin_pid_.update(forward_err, dt);

  // --- Angular velocity: keep the same turn direction as leader first, then
  // trim yaw and lateral error. This avoids the follower turning opposite when
  // the leader rotates in place and the target point moves sideways.
  bool leader_turning = std::fabs(leader_wz) > turn_in_place_threshold_;
  double yaw_scale = leader_turning ? turn_yaw_correction_scale_ : 1.0;
  double lateral_scale = leader_turning ? turn_lateral_correction_scale_ : 1.0;
  double yaw_vz = yaw_scale * yaw_error_gain_ * yaw_err;
  double lateral_vz = motion_sign * lateral_scale * lateral_error_gain_ * lateral_err;
  double heading_weight = leader_turning
      ? 0.0
      : cluster_common::clamp((distance - 0.35) / 0.65, 0.0, 1.0);
  double heading_vz = heading_weight * ang_pid_.update(heading_err, dt);
  double vz = leader_wz_feedforward_gain_ * leader_wz + yaw_vz + lateral_vz + heading_vz;

  // --- Rate limiting on angular velocity ---
  double max_vz_step = 0.5;  // rad/s per control cycle (~10 rad/s^2)
  double vz_step = cluster_common::clamp(vz - prev_vz_, -max_vz_step, max_vz_step);
  vz = prev_vz_ + vz_step;
  prev_vz_ = vz;

  // --- Minimum speed to overcome friction ---
  if (std::fabs(forward_err) > 0.1) {
    if (vx > 0 && vx < min_linear_speed_) vx = min_linear_speed_;
    if (vx < 0 && vx > -min_linear_speed_) vx = -min_linear_speed_;
  }
  // Near target: slow down proportionally
  if (std::fabs(forward_err) < 0.15) {
    vx = vx * std::fabs(forward_err) / 0.15;
  }

  // When the leader is moving forward or turning in place, do not let a
  // transient pose error command the follower to back up. That behavior makes
  // the formation unwind after turns on real LIMO bases with odom drift.
  bool leader_forward_or_turning =
      leader_vx > 0.03 || (std::fabs(leader_wz) > turn_in_place_threshold_ &&
                           leader_vx > -0.03);
  if (!allow_reverse_while_leader_forward_ && leader_forward_or_turning &&
      vx < 0.0) {
    vx = 0.0;
    lin_pid_.reset();
  }

  // --- Speed limiting ---
  double linear_limit = max_linear_speed_;
  if (latest_leader_cmd_.speed_limit > 0.0) {
    linear_limit = std::min(linear_limit, latest_leader_cmd_.speed_limit);
  }
  vx = cluster_common::clamp(vx, -linear_limit, linear_limit);
  vz = cluster_common::clamp(vz, -max_angular_speed_, max_angular_speed_);

  // --- Anti-windup on saturation ---
  if (std::fabs(vx) >= linear_limit * 0.95) {
    lin_pid_.reset();
  }

  publishCmdVel(vx, vz);
}

// ---------- Frame Calibration ----------

void FollowerController::tryCalibrate() {
  if (!self_odom_received_ || !leader_odom_received_ || !leader_cmd_received_)
    return;

  // Only calibrate in FORMATION mode
  if (current_mode_ != cluster_msgs::LeaderCmd::MODE_FORMATION)
    return;

  auto self = cluster_common::odomToPose2D(latest_self_odom_);
  auto leader = cluster_common::odomToPose2D(latest_leader_odom_);

  // Both cars must be nearly stationary
  double ls = std::fabs(latest_leader_odom_.twist.twist.linear.x);
  double ss = std::fabs(latest_self_odom_.twist.twist.linear.x);
  if (ls > 0.05 || ss > 0.05) return;

  double ox = latest_leader_cmd_.offset_x;
  double oy = latest_leader_cmd_.offset_y;
  double oyaw = latest_leader_cmd_.offset_yaw;

  double cos_l = std::cos(leader.theta);
  double sin_l = std::sin(leader.theta);

  double tx = leader.x + ox * cos_l - oy * sin_l;
  double ty = leader.y + ox * sin_l + oy * cos_l;
  double ttheta = cluster_common::normalizeAngle(leader.theta + oyaw);

  frame_shift_theta_ = cluster_common::normalizeAngle(self.theta - ttheta);

  // Rotate target, then compute translation shift
  double cos_fs = std::cos(frame_shift_theta_);
  double sin_fs = std::sin(frame_shift_theta_);
  double tx_rot = tx * cos_fs - ty * sin_fs;
  double ty_rot = tx * sin_fs + ty * cos_fs;
  frame_shift_x_ = self.x - tx_rot;
  frame_shift_y_ = self.y - ty_rot;

  calibrated_ = true;
  ROS_INFO("Frame calibrated. shift=(%.3f, %.3f, %.3f rad)",
           frame_shift_x_, frame_shift_y_, frame_shift_theta_);
  ROS_INFO("  Leader pose: (%.3f, %.3f, %.3f)  Self pose: (%.3f, %.3f, %.3f)",
           leader.x, leader.y, leader.theta, self.x, self.y, self.theta);
}

// ---------- Following Mode (Path Tracking) ----------

void FollowerController::followLeaderPath() {
  if (trajectory_buffer_.empty()) { stop(); return; }

  auto current = cluster_common::odomToPose2D(latest_self_odom_);
  TrajectoryPoint target = trajectory_buffer_.front();

  // Frame-shift the trajectory point
  double cos_fs = std::cos(frame_shift_theta_);
  double sin_fs = std::sin(frame_shift_theta_);
  double tx = target.x * cos_fs - target.y * sin_fs + frame_shift_x_;
  double ty = target.x * sin_fs + target.y * cos_fs + frame_shift_y_;

  double dx = tx - current.x;
  double dy = ty - current.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  double current_speed = latest_self_odom_.twist.twist.linear.x;
  double lookahead = computeLookaheadDistance(current_speed);

  double heading_to_target = std::atan2(dy, dx);
  bool reverse_tracking = latest_leader_cmd_.leader_vx < -0.03;
  double tracking_heading = reverse_tracking
      ? cluster_common::normalizeAngle(heading_to_target + M_PI)
      : heading_to_target;
  double heading_error = cluster_common::normalizeAngle(
      tracking_heading - current.theta);

  double curvature = 2.0 * std::sin(heading_error) / std::max(lookahead, 0.1);
  double speed_sign = reverse_tracking ? -1.0 : 1.0;
  double linear_limit = max_linear_speed_;
  if (latest_leader_cmd_.speed_limit > 0.0) {
    linear_limit = std::min(linear_limit, latest_leader_cmd_.speed_limit);
  }
  double speed_mag = linear_limit * (1.0 - std::fabs(heading_error) / M_PI);

  speed_mag = cluster_common::clamp(speed_mag, 0.0,
      linear_limit * std::min(distance / lookahead, 1.0));
  double vx = speed_sign * speed_mag;
  double vz = curvature * linear_limit;
  vz = cluster_common::clamp(vz, -max_angular_speed_, max_angular_speed_);

  if (distance < pos_deadband_) { vx = 0.0; vz = 0.0; }

  double cos_s = std::cos(current.theta);
  double sin_s = std::sin(current.theta);
  double forward_err = dx * cos_s + dy * sin_s;
  double lateral_err = -dx * sin_s + dy * cos_s;
  double target_theta = cluster_common::normalizeAngle(target.theta + frame_shift_theta_);
  double yaw_err = cluster_common::normalizeAngle(target_theta - current.theta);
  updateTrackingError(lateral_err, forward_err, yaw_err, distance);

  publishCmdVel(vx, vz);

  while (trajectory_buffer_.size() > 1 &&
         trajectory_buffer_.front().stamp <
         trajectory_buffer_.back().stamp - ros::Duration(follow_delay_)) {
    trajectory_buffer_.pop_front();
  }
}

double FollowerController::computeLookaheadDistance(double current_speed) {
  double min_lookahead = 0.3;
  double max_lookahead = 1.5;
  double speed_factor = std::fabs(current_speed) / max_linear_speed_;
  return min_lookahead + speed_factor * (max_lookahead - min_lookahead);
}

// ---------- Helpers ----------

void FollowerController::stop() {
  publishCmdVel(0.0, 0.0);
  prev_vz_ = 0.0;
}

void FollowerController::publishCmdVel(double vx, double vz) {
  geometry_msgs::Twist cmd;
  cmd.linear.x = vx;
  cmd.angular.z = vz;
  cmd_vel_pub_.publish(cmd);
}

void FollowerController::updateTrackingError(double error_x, double error_y,
                                             double error_yaw,
                                             double error_dist) {
  last_error_x_ = error_x;
  last_error_y_ = error_y;
  last_error_yaw_ = error_yaw;
  last_error_dist_ = error_dist;
}

void FollowerController::spin() {
  ros::spin();
}

}  // namespace cluster_following
