#include "cluster_formation/leader_controller.h"
#include <cmath>
#include <string>

namespace cluster_formation {

LeaderController::LeaderController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  , self_odom_received_(false)
  , follower_odom_received_(false)
  , follower_status_received_(false)
  , teleop_cmd_received_(false)
  , home_pose_received_(false)
  , return_home_active_(false)
  , current_mode_(cluster_msgs::LeaderCmd::MODE_IDLE)
  , current_formation_(cluster_msgs::LeaderCmd::FORMATION_COLUMN)
  , use_custom_offsets_(false)
  , error_exceeded_(false) {

  // Load params
  pnh_.param("loop_rate", loop_rate_, 20.0);
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
  self_odom_sub_ = nh_.subscribe("/robot1/odom", 10,
      &LeaderController::selfOdomCallback, this);
  follower_odom_sub_ = nh_.subscribe("/robot2/odom", 10,
      &LeaderController::followerOdomCallback, this);
  follower_status_sub_ = nh_.subscribe("/robot2/follower_status", 10,
      &LeaderController::followerStatusCallback, this);
  teleop_vel_sub_ = nh_.subscribe("/robot1/teleop_vel", 10,
      &LeaderController::teleopVelCallback, this);
  return_home_sub_ = nh_.subscribe("/robot1/return_home", 5,
      &LeaderController::returnHomeCallback, this);

  // Publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 10);
  leader_cmd_pub_ = nh_.advertise<cluster_msgs::LeaderCmd>("/robot1/leader_cmd", 10);

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

// ---------- Callbacks ----------

void LeaderController::selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_self_odom_ = *msg;
  self_odom_received_ = true;
  if (!home_pose_received_) {
    home_pose_ = cluster_common::odomToPose2D(*msg);
    home_pose_received_ = true;
    ROS_INFO("Recorded robot1 home pose: x=%.2f y=%.2f yaw=%.2f",
             home_pose_.x, home_pose_.y, home_pose_.theta);
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
  last_follower_status_time_ = msg->header.stamp;
}

void LeaderController::teleopVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  latest_teleop_cmd_ = *msg;
  teleop_cmd_received_ = true;
  last_teleop_cmd_time_ = ros::Time::now();

  // Relay teleop commands to cmd_vel in all modes
  // (in IDLE, we still allow teleop for manual driving)
  if (current_mode_ != cluster_msgs::LeaderCmd::MODE_IDLE) {
    cmd_vel_pub_.publish(*msg);
  }
}

void LeaderController::returnHomeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) {
    return_home_active_ = false;
    return;
  }
  if (!home_pose_received_) {
    ROS_WARN("Return home requested before home pose is available");
    return;
  }
  return_home_active_ = true;
  current_mode_ = cluster_msgs::LeaderCmd::MODE_TELEOP;
  ROS_INFO("Return home requested");
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
  current_mode_ = req.mode;

  if (req.mode == cluster_msgs::LeaderCmd::MODE_FORMATION) {
    if (req.formation <= cluster_msgs::LeaderCmd::FORMATION_TRIANGLE_RIGHT) {
      current_formation_ = req.formation;
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
  }

  res.success = true;
  res.message = "Mode set successfully";
  return true;
}

bool LeaderController::setFormationCallback(
    cluster_msgs::SetFormation::Request& req,
    cluster_msgs::SetFormation::Response& res) {

  if (req.formation > cluster_msgs::LeaderCmd::FORMATION_TRIANGLE_RIGHT) {
    res.success = false;
    res.message = "Invalid formation. Valid: 0=COLUMN, 1=LINE, 2=TRIANGLE_LEFT, 3=TRIANGLE_RIGHT";
    return true;
  }

  current_formation_ = req.formation;
  use_custom_offsets_ = false;
  ROS_INFO("SetFormation: %d", current_formation_);

  res.success = true;
  res.message = "Formation set successfully";
  return true;
}

// ---------- Control Loop ----------

void LeaderController::controlLoop(const ros::TimerEvent& event) {
  checkSafety();

  if (return_home_active_) {
    computeReturnHome();
    return;
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
  if (!self_odom_received_ || !home_pose_received_) {
    return_home_active_ = false;
    return;
  }

  cluster_common::Pose2D self = cluster_common::odomToPose2D(latest_self_odom_);
  const double dx = home_pose_.x - self.x;
  const double dy = home_pose_.y - self.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double target_heading = std::atan2(dy, dx);
  const double heading_err = cluster_common::normalizeAngle(target_heading - self.theta);
  const double yaw_err = cluster_common::normalizeAngle(home_pose_.theta - self.theta);

  geometry_msgs::Twist cmd;
  if (distance > return_home_pos_tolerance_) {
    const double speed_scale = std::max(0.0, std::cos(heading_err));
    cmd.linear.x = cluster_common::clamp(return_home_k_v_ * distance * speed_scale,
                                         0.0, max_linear_speed_);
    cmd.angular.z = cluster_common::clamp(return_home_k_w_ * heading_err,
                                          -max_angular_speed_, max_angular_speed_);
  } else if (std::fabs(yaw_err) > return_home_yaw_tolerance_) {
    cmd.angular.z = cluster_common::clamp(return_home_k_w_ * yaw_err,
                                          -max_angular_speed_, max_angular_speed_);
  } else {
    return_home_active_ = false;
    cmd_vel_pub_.publish(cmd);
    ROS_INFO("Return home complete");
  }

  cmd_vel_pub_.publish(cmd);

  cached_leader_cmd_.header.stamp = ros::Time::now();
  cached_leader_cmd_.mode = cluster_msgs::LeaderCmd::MODE_TELEOP;
  cached_leader_cmd_.formation = current_formation_;
  cached_leader_cmd_.leader_vx = cmd.linear.x;
  cached_leader_cmd_.leader_vyaw = cmd.angular.z;
  cached_leader_cmd_.speed_limit = speed_limit_;
  leader_cmd_pub_.publish(cached_leader_cmd_);
}

void LeaderController::computeFormationTarget() {
  if (!self_odom_received_) return;

  auto leader_pose = cluster_common::odomToPose2D(latest_self_odom_);

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
  cached_leader_cmd_.leader_vx = teleop_fresh
      ? latest_teleop_cmd_.linear.x
      : latest_self_odom_.twist.twist.linear.x;
  cached_leader_cmd_.leader_vyaw = teleop_fresh
      ? latest_teleop_cmd_.angular.z
      : latest_self_odom_.twist.twist.angular.z;
  cached_leader_cmd_.target_pose = target;
  cached_leader_cmd_.speed_limit = speed_limit_;

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

  // Check formation error
  if (current_mode_ == cluster_msgs::LeaderCmd::MODE_FORMATION &&
      follower_status_received_) {
    double error = latest_follower_status_.error_dist;
    if (error > max_formation_error_) {
      if (!error_exceeded_) {
        error_exceeded_ = true;
        error_start_time_ = now;
      } else if ((now - error_start_time_).toSec() > max_error_duration_) {
        ROS_WARN("Formation error %.2f exceeded max %.2f for %.1fs. Stopping.",
                 error, max_formation_error_, max_error_duration_);
        current_mode_ = cluster_msgs::LeaderCmd::MODE_IDLE;
      }
    } else {
      error_exceeded_ = false;
    }
  }

  if (follower_lost && current_mode_ != cluster_msgs::LeaderCmd::MODE_IDLE &&
      current_mode_ != cluster_msgs::LeaderCmd::MODE_TELEOP) {
    ROS_WARN_THROTTLE(2.0, "Follower connection lost! Forcing IDLE.");
    current_mode_ = cluster_msgs::LeaderCmd::MODE_IDLE;
    geometry_msgs::Twist zero_cmd;
    cmd_vel_pub_.publish(zero_cmd);
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
    case cluster_msgs::LeaderCmd::FORMATION_TRIANGLE_LEFT:
      offset = {-0.8, 0.8, 0.0};
      break;
    case cluster_msgs::LeaderCmd::FORMATION_TRIANGLE_RIGHT:
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
