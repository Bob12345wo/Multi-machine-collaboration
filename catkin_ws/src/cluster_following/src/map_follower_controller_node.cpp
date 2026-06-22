#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
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
    pnh_.param<std::string>("leader_odom_frame", leader_odom_frame_, "robot1/odom");
    pnh_.param<std::string>("follower_odom_frame", follower_odom_frame_, "robot2/odom");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/robot2/cmd_vel");
    pnh_.param<std::string>("status_topic", status_topic_, "/robot2/follower_status");
    pnh_.param<std::string>("assigned_goal_topic", assigned_goal_topic_, "");
    pnh_.param<std::string>("leader_imu_topic", leader_imu_topic_,
                            "/robot1/imu_fixed");
    pnh_.param<std::string>("follower_imu_topic", follower_imu_topic_,
                            "/robot2/imu_fixed");
    pnh_.param<std::string>("control_mode_topic", control_mode_topic_,
                            "/robot2/follower_control_mode");
    pnh_.param<std::string>("control_mode", control_mode_, "body_orbit");
    pnh_.param("use_leader_offsets", use_leader_offsets_, true);
    pnh_.param("follower_index", follower_index_, 2);
    pnh_.param("formation_spacing", formation_spacing_, 0.8);
    pnh_.param("circle_show_radius", circle_show_radius_, 0.5);
    pnh_.param("use_assigned_goal", use_assigned_goal_, true);
    pnh_.param("assigned_goal_timeout", assigned_goal_timeout_, 0.7);
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
    pnh_.param("yaw_error_linear_limit", yaw_error_linear_limit_, 1.2);
    pnh_.param("k_heading", k_heading_, 1.0);
    pnh_.param("k_approach_heading", k_approach_heading_, 1.4);
    pnh_.param("leader_vx_gain", leader_vx_gain_, 1.0);
    pnh_.param("leader_wz_gain", leader_wz_gain_, 0.8);
    pnh_.param("orbit_v_gain", orbit_v_gain_, 0.8);
    pnh_.param("target_filter_alpha", target_filter_alpha_, 0.35);
    pnh_.param("cmd_filter_alpha", cmd_filter_alpha_, 0.45);
    pnh_.param("near_target_speed_gain", near_target_speed_gain_, 0.80);
    pnh_.param("near_target_max_speed", near_target_max_speed_, 0.22);
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
    pnh_.param("use_imu_yaw_assist", use_imu_yaw_assist_, false);
    pnh_.param("imu_max_age", imu_max_age_, 0.4);
    pnh_.param("imu_offset_alpha", imu_offset_alpha_, 0.0);
    pnh_.param("tf_max_age", tf_max_age_, 1.0);
    pnh_.param("map_tf_future_tolerance", map_tf_future_tolerance_, 1.2);
    pnh_.param("enable_odom_prediction", enable_odom_prediction_, true);
    pnh_.param("prediction_max_age", prediction_max_age_, 6.0);
    pnh_.param("prediction_odom_max_age", prediction_odom_max_age_, 1.0);
    pnh_.param("final_yaw_distance", final_yaw_distance_, 0.22);
    pnh_.param("final_yaw_gain_scale", final_yaw_gain_scale_, 0.25);
    pnh_.param("disable_final_spin", disable_final_spin_, false);
    pnh_.param("settle_pos_tolerance", settle_pos_tolerance_, 0.10);
    pnh_.param("settle_release_distance", settle_release_distance_, 0.18);
    pnh_.param("settle_yaw_tolerance", settle_yaw_tolerance_, 0.18);
    pnh_.param("settle_yaw_release", settle_yaw_release_, 0.35);
    pnh_.param("final_spin_max_angular_speed", final_spin_max_angular_speed_, 0.28);
    pnh_.param("final_yaw_align_max_error", final_yaw_align_max_error_, 0.9);
    pnh_.param("final_yaw_align_timeout", final_yaw_align_timeout_, 5.0);
    pnh_.param("static_yaw_blend_distance", static_yaw_blend_distance_, 0.10);
    pnh_.param("path_keepout_enabled", path_keepout_enabled_, true);
    pnh_.param("path_keepout_radius", path_keepout_radius_, 0.70);
    pnh_.param("path_detour_radius", path_detour_radius_, 0.85);
    pnh_.param("path_detour_step", path_detour_step_, 0.55);
    pnh_.param("path_detour_goal_tolerance", path_detour_goal_tolerance_, 0.18);

    leader_cmd_sub_ = nh_.subscribe("/robot1/leader_cmd", 1,
        &MapFollowerController::leaderCmdCallback, this);
    if (use_assigned_goal_ && !assigned_goal_topic_.empty()) {
      assigned_goal_sub_ = nh_.subscribe(assigned_goal_topic_, 1,
          &MapFollowerController::assignedGoalCallback, this);
    }
    control_mode_sub_ = nh_.subscribe(control_mode_topic_, 1,
        &MapFollowerController::controlModeCallback, this);
    if (use_imu_yaw_assist_) {
      leader_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(leader_imu_topic_, 1,
          &MapFollowerController::leaderImuCallback, this,
          ros::TransportHints().tcpNoDelay());
      follower_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(follower_imu_topic_, 1,
          &MapFollowerController::followerImuCallback, this,
          ros::TransportHints().tcpNoDelay());
      ROS_INFO("IMU yaw assist enabled: leader=%s follower=%s",
               leader_imu_topic_.c_str(), follower_imu_topic_.c_str());
    }
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    status_pub_ = nh_.advertise<cluster_msgs::FollowerStatus>(status_topic_, 1);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
        &MapFollowerController::controlLoop, this);
  }

private:
  struct Pose2D {
    double x;
    double y;
    double yaw;
  };

  struct FormationOffset {
    double x;
    double y;
    double yaw;
  };

  struct PoseCache {
    bool valid{false};
    Pose2D map_pose{0.0, 0.0, 0.0};
    Pose2D odom_pose{0.0, 0.0, 0.0};
    ros::Time stamp;
  };

  struct ImuYawCache {
    bool received{false};
    bool aligned{false};
    double imu_yaw{0.0};
    double map_offset{0.0};
    ros::Time stamp;
  };

  void leaderCmdCallback(const cluster_msgs::LeaderCmd::ConstPtr& msg) {
    latest_leader_cmd_ = *msg;
    leader_cmd_received_ = true;
    last_leader_cmd_time_ = ros::Time::now();
  }

  void assignedGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_assigned_goal_ = *msg;
    assigned_goal_received_ = true;
    last_assigned_goal_time_ = ros::Time::now();
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
      final_yaw_active_ = false;
      final_yaw_timed_out_ = false;
    }
  }

  void leaderImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    updateImuYaw(*msg, leader_imu_);
  }

  void followerImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    updateImuYaw(*msg, follower_imu_);
  }

  void updateImuYaw(const sensor_msgs::Imu& msg, ImuYawCache& cache) {
    const geometry_msgs::Quaternion& q = msg.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || norm < 1e-9) {
      return;
    }
    cache.imu_yaw = tf2::getYaw(q);
    // Use local receive time for freshness. In the multi-car setup IMU header
    // stamps are produced on different computers and can drift enough to trip
    // the controller even while messages are arriving normally.
    cache.stamp = ros::Time::now();
    cache.received = true;
  }

  void applyImuYawAssist(const std::string& label,
                         ImuYawCache& imu,
                         Pose2D& pose) {
    if (!use_imu_yaw_assist_ || !imuYawFresh(label, imu)) {
      return;
    }

    const double measured_offset =
        cluster_common::normalizeAngle(pose.yaw - imu.imu_yaw);
    if (!imu.aligned) {
      imu.map_offset = measured_offset;
      imu.aligned = true;
      ROS_INFO("%s IMU yaw assist aligned for %s: map_offset=%.3f rad",
               ros::this_node::getName().c_str(), label.c_str(), imu.map_offset);
    } else if (imu_offset_alpha_ > 0.0) {
      const double offset_err =
          cluster_common::normalizeAngle(measured_offset - imu.map_offset);
      if (std::fabs(offset_err) < 0.25) {
        const double alpha = cluster_common::clamp(imu_offset_alpha_, 0.0, 1.0);
        imu.map_offset =
            cluster_common::normalizeAngle(imu.map_offset + alpha * offset_err);
      }
    }

    pose.yaw = cluster_common::normalizeAngle(imu.imu_yaw + imu.map_offset);
  }

  bool imuYawFresh(const std::string& label, const ImuYawCache& imu) const {
    if (!use_imu_yaw_assist_ || !imu.received) {
      return false;
    }
    const double age = (ros::Time::now() - imu.stamp).toSec();
    if (age > imu_max_age_ || age < -0.2) {
      ROS_WARN_THROTTLE(3.0,
          "%s IMU yaw assist stale for %s: age %.2fs, using TF yaw",
          ros::this_node::getName().c_str(), label.c_str(), age);
      return false;
    }
    return true;
  }

  void applyPairedImuYawAssist(Pose2D& leader, Pose2D& follower) {
    if (!use_imu_yaw_assist_) {
      return;
    }
    const bool leader_fresh = imuYawFresh("leader", leader_imu_);
    const bool follower_fresh = imuYawFresh("follower", follower_imu_);
    if (!leader_fresh || !follower_fresh) {
      ROS_WARN_THROTTLE(3.0,
          "%s IMU yaw assist skipped this cycle: leader_fresh=%d follower_fresh=%d",
          ros::this_node::getName().c_str(),
          leader_fresh ? 1 : 0, follower_fresh ? 1 : 0);
      return;
    }
    applyImuYawAssist("leader", leader_imu_, leader);
    applyImuYawAssist("follower", follower_imu_, follower);
  }

  bool lookupOdomPose(const std::string& odom_frame,
                      const std::string& base_frame,
                      Pose2D& pose,
                      ros::Time& stamp) {
    try {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(odom_frame, base_frame, ros::Time(0),
                                     ros::Duration(0.02));
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = tf2::getYaw(tf.transform.rotation);
      stamp = tf.header.stamp;
      if (prediction_odom_max_age_ > 0.0 && !stamp.isZero()) {
        const double age = (ros::Time::now() - stamp).toSec();
        if (age > prediction_odom_max_age_ || age < -0.2) {
          return false;
        }
      }
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  bool predictPoseFromOdom(const std::string& frame,
                           const std::string& odom_frame,
                           const PoseCache& cache,
                           Pose2D& pose) {
    if (!enable_odom_prediction_ || !cache.valid) {
      return false;
    }
    const double cache_age = (ros::Time::now() - cache.stamp).toSec();
    if (cache_age > prediction_max_age_ || cache_age < -0.2) {
      return false;
    }

    Pose2D current_odom;
    ros::Time odom_stamp;
    if (!lookupOdomPose(odom_frame, frame, current_odom, odom_stamp)) {
      return false;
    }

    const double dx_odom = current_odom.x - cache.odom_pose.x;
    const double dy_odom = current_odom.y - cache.odom_pose.y;
    const double yaw_delta =
        cluster_common::normalizeAngle(current_odom.yaw - cache.odom_pose.yaw);
    const double map_from_odom_yaw =
        cluster_common::normalizeAngle(cache.map_pose.yaw - cache.odom_pose.yaw);
    const double c = std::cos(map_from_odom_yaw);
    const double s = std::sin(map_from_odom_yaw);

    pose.x = cache.map_pose.x + dx_odom * c - dy_odom * s;
    pose.y = cache.map_pose.y + dx_odom * s + dy_odom * c;
    pose.yaw = cluster_common::normalizeAngle(cache.map_pose.yaw + yaw_delta);
    ROS_DEBUG_THROTTLE(5.0,
        "Using short odom prediction for %s: cached map TF age %.2fs",
        frame.c_str(), cache_age);
    return true;
  }

  bool lookupPose(const std::string& frame, const std::string& odom_frame,
                  PoseCache& cache, Pose2D& pose) {
    try {
      const geometry_msgs::TransformStamped map_to_odom =
          tf_buffer_.lookupTransform(map_frame_, odom_frame, ros::Time(0),
                                     ros::Duration(0.05));
      const geometry_msgs::TransformStamped odom_to_base =
          tf_buffer_.lookupTransform(odom_frame, frame, ros::Time(0),
                                     ros::Duration(0.05));
      const double map_yaw = tf2::getYaw(map_to_odom.transform.rotation);
      const double odom_yaw = tf2::getYaw(odom_to_base.transform.rotation);
      const double c = std::cos(map_yaw);
      const double s = std::sin(map_yaw);
      const double ox = odom_to_base.transform.translation.x;
      const double oy = odom_to_base.transform.translation.y;
      pose.x = map_to_odom.transform.translation.x + ox * c - oy * s;
      pose.y = map_to_odom.transform.translation.y + ox * s + oy * c;
      pose.yaw = cluster_common::normalizeAngle(map_yaw + odom_yaw);

      const bool map_tf_fresh = transformFresh(map_to_odom, odom_frame, true);
      const bool odom_tf_fresh = transformFresh(odom_to_base, frame);
      if (!map_tf_fresh || !odom_tf_fresh) {
        return predictPoseFromOdom(frame, odom_frame, cache, pose);
      }

      Pose2D odom_pose;
      ros::Time odom_stamp;
      if (lookupOdomPose(odom_frame, frame, odom_pose, odom_stamp)) {
        cache.valid = true;
        cache.map_pose = pose;
        cache.odom_pose = odom_pose;
        cache.stamp = ros::Time::now();
      }
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "TF lookup failed %s -> %s: %s",
                        map_frame_.c_str(), frame.c_str(), ex.what());
      return predictPoseFromOdom(frame, odom_frame, cache, pose);
    }
  }

  bool transformFresh(const geometry_msgs::TransformStamped& tf,
                      const std::string& frame,
                      bool allow_amcl_future = false) const {
    if (tf_max_age_ <= 0.0 || tf.header.stamp.isZero()) {
      return true;
    }

    const double age = (ros::Time::now() - tf.header.stamp).toSec();
    if (age > tf_max_age_) {
      ROS_WARN_THROTTLE(5.0,
          "TF stale for %s: age %.2fs exceeds %.2fs, using prediction if available",
          frame.c_str(), age, tf_max_age_);
      return false;
    }
    const double future_tolerance =
        allow_amcl_future ? map_tf_future_tolerance_ : 0.2;
    if (age < -future_tolerance) {
      ROS_WARN_THROTTLE(1.0,
          "TF from future for %s: age %.2fs exceeds %.2fs, check clock sync",
          frame.c_str(), age, future_tolerance);
      return false;
    }
    return true;
  }

  FormationOffset getFollowerOffset(uint8_t formation) const {
    if (follower_index_ == 2 && use_leader_offsets_) {
      return {latest_leader_cmd_.offset_x, latest_leader_cmd_.offset_y,
              latest_leader_cmd_.offset_yaw};
    }

    const double d = formation_spacing_;
    if (follower_index_ == 3) {
      switch (formation) {
        case cluster_msgs::LeaderCmd::FORMATION_COLUMN:
          return {-2.0 * d, 0.0, 0.0};
        case cluster_msgs::LeaderCmd::FORMATION_LINE:
          return {0.0, d, 0.0};
        case cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW: {
          const double phase = 4.1887902047863905;  // 240 degrees behind car1.
          const double r = circle_show_radius_;
          return {-r * std::sin(phase), r * (1.0 - std::cos(phase)), -phase};
        }
        case cluster_msgs::LeaderCmd::FORMATION_TRIANGLE:
          return {-d, d, 0.0};
        default:
          return {-2.0 * d, 0.0, 0.0};
      }
    }

    if (use_leader_offsets_) {
      return {latest_leader_cmd_.offset_x, latest_leader_cmd_.offset_y,
              latest_leader_cmd_.offset_yaw};
    }
    return {offset_x_, offset_y_, 0.0};
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
    if (!lookupPose(leader_frame_, leader_odom_frame_, leader_cache_, leader) ||
        !lookupPose(follower_frame_, follower_odom_frame_, follower_cache_, follower)) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_LOST,
                    0.0, 0.0, 0.0, 0.0, false);
      return;
    }
    applyPairedImuYawAssist(leader, follower);

    FormationOffset active_offset = getFollowerOffset(latest_leader_cmd_.formation);
    double active_offset_x = active_offset.x;
    double active_offset_y = active_offset.y;
    double active_offset_yaw = active_offset.yaw;

    if (control_mode_changed_) {
      if (control_mode_ == "wheeltec_global") {
        formation_anchor_yaw_ = leader.yaw;
      }
      control_mode_changed_ = false;
      target_initialized_ = false;
      settled_ = false;
    }

    if (latest_leader_cmd_.formation != last_formation_) {
      if (control_mode_ == "wheeltec_global") {
        formation_anchor_yaw_ = leader.yaw;
      }
      last_formation_ = latest_leader_cmd_.formation;
      target_initialized_ = false;
      settled_ = false;
      final_yaw_active_ = false;
      final_yaw_timed_out_ = false;
    }

    const double cos_l = std::cos(leader.yaw);
    const double sin_l = std::sin(leader.yaw);
    double target_x = leader.x + active_offset_x * cos_l - active_offset_y * sin_l;
    double target_y = leader.y + active_offset_x * sin_l + active_offset_y * cos_l;
    double target_yaw = cluster_common::normalizeAngle(leader.yaw + active_offset_yaw);

    const bool assigned_goal_fresh = use_assigned_goal_ && assigned_goal_received_ &&
        (ros::Time::now() - last_assigned_goal_time_).toSec() <= assigned_goal_timeout_ &&
        latest_assigned_goal_.header.frame_id == map_frame_;
    if (assigned_goal_fresh) {
      target_x = latest_assigned_goal_.pose.position.x;
      target_y = latest_assigned_goal_.pose.position.y;
      target_yaw = tf2::getYaw(latest_assigned_goal_.pose.orientation);

      const double map_dx = target_x - leader.x;
      const double map_dy = target_y - leader.y;
      active_offset_x = map_dx * cos_l + map_dy * sin_l;
      active_offset_y = -map_dx * sin_l + map_dy * cos_l;
      active_offset_yaw = cluster_common::normalizeAngle(target_yaw - leader.yaw);
    }
    const bool circle_assigned_goal = assigned_goal_fresh &&
        latest_leader_cmd_.formation == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW;

    const double leader_vx = latest_leader_cmd_.leader_vx;
    const double leader_wz = latest_leader_cmd_.leader_vyaw;

    if (control_mode_ == "wheeltec_global" && !circle_assigned_goal) {
      const double cos_a = std::cos(formation_anchor_yaw_);
      const double sin_a = std::sin(formation_anchor_yaw_);
      target_x = leader.x + active_offset_x * cos_a - active_offset_y * sin_a;
      target_y = leader.y + active_offset_x * sin_a + active_offset_y * cos_a;
    } else if (control_mode_ != "body_orbit" &&
        turn_radius_compensation_ && std::fabs(leader_wz) > min_angular_speed_ &&
        std::fabs(leader_vx) > min_linear_speed_) {
      const double radius = leader_vx / leader_wz;
      target_yaw = cluster_common::normalizeAngle(
          leader.yaw + active_offset_yaw +
          std::atan2(active_offset_y, active_offset_x + radius));
    }

    Pose2D final_target{target_x, target_y, target_yaw};
    const bool leader_static_for_path =
        std::fabs(leader_vx) < static_leader_v_threshold_ &&
        std::fabs(leader_wz) < static_leader_w_threshold_;
    Pose2D raw_target = final_target;
    if (!assigned_goal_fresh) {
      raw_target = applyPathKeepout(leader, follower, final_target,
                                    leader_static_for_path);
    } else {
      path_detour_active_ = false;
    }
    Pose2D target = filterTarget(raw_target);

    const double dx = target.x - follower.x;
    const double dy = target.y - follower.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    const double cos_f = std::cos(follower.yaw);
    const double sin_f = std::sin(follower.yaw);
    const double forward_err = dx * cos_f + dy * sin_f;
    const double lateral_err = -dx * sin_f + dy * cos_f;
    double yaw_err = cluster_common::normalizeAngle(target.yaw - follower.yaw);
    const double yaw_cmd_err = cluster_common::clamp(
        yaw_err, -yaw_error_linear_limit_, yaw_error_linear_limit_);
    double yaw_gain_scale = 1.0;
    if (distance < final_yaw_distance_) {
      yaw_gain_scale = final_yaw_gain_scale_;
      if (disable_final_spin_ && std::fabs(leader_vx) < min_linear_speed_ &&
          std::fabs(leader_wz) < min_angular_speed_ &&
          distance < static_position_tolerance_) {
        yaw_err = 0.0;
      }
    }
    const double heading_err = std::atan2(lateral_err,
        std::max(std::fabs(forward_err), heading_lookahead_));
    const double heading_weight = cluster_common::clamp(
        (distance - pos_deadband_) /
        std::max(final_align_distance_ - pos_deadband_, 0.01),
        0.0, 1.0);
    const bool leader_static =
        std::fabs(leader_vx) < static_leader_v_threshold_ &&
        std::fabs(leader_wz) < static_leader_w_threshold_;

    const bool position_settled = distance <= settle_pos_tolerance_;
    const bool position_released = distance > settle_release_distance_;
    if (!leader_static || position_released ||
        std::fabs(yaw_err) <= settle_yaw_tolerance_) {
      final_yaw_timed_out_ = false;
    }
    const bool yaw_needs_alignment =
        !disable_final_spin_ && !final_yaw_timed_out_ &&
        std::fabs(yaw_err) > settle_yaw_tolerance_;
    const bool yaw_released =
        !disable_final_spin_ && !final_yaw_timed_out_ &&
        std::fabs(yaw_err) > settle_yaw_release_;

    if (!leader_static || position_released || yaw_released) {
      settled_ = false;
    }

    if (!leader_static || position_released) {
      final_yaw_active_ = false;
    }

    if (leader_static && position_settled) {
      if (!yaw_needs_alignment) {
        settled_ = true;
        final_yaw_active_ = false;
      } else {
        if (!final_yaw_active_) {
          final_yaw_active_ = true;
          final_yaw_start_time_ = ros::Time::now();
        }
        const double align_time =
            (ros::Time::now() - final_yaw_start_time_).toSec();
        if (final_yaw_align_timeout_ > 0.0 &&
            align_time > final_yaw_align_timeout_) {
          ROS_WARN_THROTTLE(2.0,
              "Final yaw alignment timeout: yaw_err=%.2f rad, dist=%.2f m",
              yaw_err, distance);
          settled_ = true;
          final_yaw_active_ = false;
          final_yaw_timed_out_ = true;
        }
      }
    }

    if (settled_ && leader_static) {
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_IDLE,
                    lateral_err, forward_err, yaw_err, distance, true);
      return;
    }

    if (distance < pos_deadband_ && std::fabs(yaw_err) < yaw_deadband_ &&
        leader_static) {
      settled_ = true;
      stop();
      publishStatus(cluster_msgs::FollowerStatus::STATE_IDLE,
                    lateral_err, forward_err, yaw_err, distance, true);
      return;
    }

    double vx = 0.0;
    double wz = 0.0;
    if (leader_static && position_settled) {
      vx = 0.0;
      if (yaw_needs_alignment && !settled_) {
        wz = yaw_gain_scale * k_a_ * yaw_cmd_err;
        wz = cluster_common::clamp(wz,
                                   -final_spin_max_angular_speed_,
                                   final_spin_max_angular_speed_);
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

      // Wheeltec-style pose-error control: lateral error and yaw error are
      // corrected together instead of rotating to the target point first.
      double lateral_gain = k_l_;
      if (vx < -min_linear_speed_) {
        lateral_gain = -k_l_;
      }
      const double lateral_weight = std::max(heading_weight, 0.35);
      wz = leader_wz_gain_ * leader_wz +
          lateral_weight * lateral_gain * lateral_err +
          k_heading_ * heading_weight * heading_err +
          yaw_gain_scale * k_a_ * yaw_cmd_err;

      const bool leader_turning = std::fabs(leader_wz) > turn_in_place_wz_threshold_;
      const bool yaw_priority = std::fabs(yaw_err) > yaw_priority_threshold_;
      if (leader_turning || yaw_priority) {
        vx *= yaw_priority_vx_scale_;
        vx = cluster_common::clamp(vx, -max_turn_linear_speed_, max_turn_linear_speed_);
      }
    }

    const bool orbit_reverse_allowed = control_mode_ == "body_orbit" &&
        allow_orbit_reverse_ && std::fabs(leader_wz) > min_angular_speed_;
    if (!allow_reverse_while_leader_forward_ && !orbit_reverse_allowed &&
        (leader_vx > min_linear_speed_ || std::fabs(leader_wz) > min_angular_speed_) &&
        vx < 0.0) {
      vx = 0.0;
    }

    if (leader_static && distance < final_align_distance_) {
      const double near_cap = cluster_common::clamp(
          near_target_speed_gain_ * distance, 0.0, near_target_max_speed_);
      vx = cluster_common::clamp(vx, -near_cap, near_cap);
    }

    vx = cluster_common::clamp(vx, -max_linear_speed_, max_linear_speed_);
    wz = cluster_common::clamp(wz, -max_angular_speed_, max_angular_speed_);

    if (std::fabs(vx) < min_linear_speed_) vx = 0.0;
    if (std::fabs(wz) < min_angular_speed_) wz = 0.0;

    geometry_msgs::Twist cmd = filterCommand(vx, wz);
    cmd_vel_pub_.publish(cmd);
    if (debug_enabled_) {
      ROS_INFO_THROTTLE(debug_period_,
          "[FOLLOW_DBG] mode=%s form=%u assigned=%d offset(x=%.3f,y=%.3f,yaw=%.3f) leader(x=%.3f,y=%.3f,yaw=%.3f,vx=%.3f,wz=%.3f) follower(x=%.3f,y=%.3f,yaw=%.3f) target(final_x=%.3f,final_y=%.3f,raw_x=%.3f,raw_y=%.3f,raw_yaw=%.3f,x=%.3f,y=%.3f,yaw=%.3f,detour=%d) err(fwd=%.3f,lat=%.3f,yaw=%.3f,dist=%.3f,heading=%.3f,weight=%.3f) cmd_raw(vx=%.3f,wz=%.3f) cmd_out(vx=%.3f,wz=%.3f)",
          control_mode_.c_str(), latest_leader_cmd_.formation,
          assigned_goal_fresh ? 1 : 0,
          active_offset_x, active_offset_y, active_offset_yaw,
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
      ROS_WARN_THROTTLE(2.0, "Target pose jumped %.2f m, accepting new target", jump);
      filtered_target_ = raw_target;
      settled_ = false;
      final_yaw_active_ = false;
      final_yaw_timed_out_ = false;
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
      ROS_DEBUG("Path keepout detour active: line_clearance=%.2f, target_dist=%.2f",
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
  ros::Subscriber assigned_goal_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber leader_imu_sub_;
  ros::Subscriber follower_imu_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher status_pub_;
  ros::Timer control_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cluster_msgs::LeaderCmd latest_leader_cmd_;
  geometry_msgs::PoseStamped latest_assigned_goal_;
  bool leader_cmd_received_{false};
  bool assigned_goal_received_{false};
  ros::Time last_leader_cmd_time_;
  ros::Time last_assigned_goal_time_;

  std::string map_frame_;
  std::string leader_frame_;
  std::string follower_frame_;
  std::string leader_odom_frame_;
  std::string follower_odom_frame_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string assigned_goal_topic_;
  std::string leader_imu_topic_;
  std::string follower_imu_topic_;
  std::string control_mode_topic_;
  std::string control_mode_;

  int follower_index_;
  double formation_spacing_;
  double circle_show_radius_;
  bool use_assigned_goal_;
  double assigned_goal_timeout_;
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
  double yaw_error_linear_limit_;
  double k_heading_;
  double k_approach_heading_;
  double leader_vx_gain_;
  double leader_wz_gain_;
  double orbit_v_gain_;
  double target_filter_alpha_;
  double cmd_filter_alpha_;
  double near_target_speed_gain_;
  double near_target_max_speed_;
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
  bool use_imu_yaw_assist_;
  double imu_max_age_;
  double imu_offset_alpha_;
  double tf_max_age_;
  double map_tf_future_tolerance_;
  bool enable_odom_prediction_;
  double prediction_max_age_;
  double prediction_odom_max_age_;
  double final_yaw_distance_;
  double final_yaw_gain_scale_;
  bool disable_final_spin_;
  double settle_pos_tolerance_;
  double settle_release_distance_;
  double settle_yaw_tolerance_;
  double settle_yaw_release_;
  double final_spin_max_angular_speed_;
  double final_yaw_align_max_error_;
  double final_yaw_align_timeout_;
  double static_yaw_blend_distance_;
  bool path_keepout_enabled_;
  double path_keepout_radius_;
  double path_detour_radius_;
  double path_detour_step_;
  double path_detour_goal_tolerance_;
  bool use_leader_offsets_;
  bool control_mode_changed_{false};
  bool target_initialized_{false};
  bool settled_{false};
  bool final_yaw_active_{false};
  bool final_yaw_timed_out_{false};
  bool path_detour_active_{false};
  uint8_t last_formation_{255};
  double formation_anchor_yaw_{0.0};
  ros::Time final_yaw_start_time_;
  Pose2D filtered_target_{0.0, 0.0, 0.0};
  PoseCache leader_cache_;
  PoseCache follower_cache_;
  ImuYawCache leader_imu_;
  ImuYawCache follower_imu_;
  geometry_msgs::Twist last_cmd_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_follower_controller");
  MapFollowerController controller;
  ros::spin();
  return 0;
}
