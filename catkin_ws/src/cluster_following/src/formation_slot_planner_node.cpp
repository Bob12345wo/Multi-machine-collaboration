#include <cstdint>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cluster_common/pose_utils.h"
#include "cluster_msgs/LeaderCmd.h"

class FormationSlotPlanner {
public:
  FormationSlotPlanner() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("leader_frame", leader_frame_, "robot1/base_link");
    pnh_.param<std::string>("robot2_frame", robot2_frame_, "robot2/base_link");
    pnh_.param<std::string>("robot3_frame", robot3_frame_, "robot3/base_link");
    pnh_.param<std::string>("leader_odom_frame", leader_odom_frame_, "robot1/odom");
    pnh_.param<std::string>("robot2_odom_frame", robot2_odom_frame_, "robot2/odom");
    pnh_.param<std::string>("robot3_odom_frame", robot3_odom_frame_, "robot3/odom");
    pnh_.param<std::string>("leader_cmd_topic", leader_cmd_topic_, "/robot1/leader_cmd");
    pnh_.param<std::string>("leader_assist_goal_topic",
                            leader_assist_goal_topic_,
                            "/robot1/formation_assist_goal");
    pnh_.param<std::string>("robot2_goal_topic", robot2_goal_topic_, "/robot2/assigned_goal");
    pnh_.param<std::string>("robot3_goal_topic", robot3_goal_topic_, "/robot3/assigned_goal");
    pnh_.param("enabled", enabled_, true);
    pnh_.param("loop_rate", loop_rate_, 10.0);
    pnh_.param("leader_cmd_timeout", leader_cmd_timeout_, 0.6);
    pnh_.param("formation_spacing", formation_spacing_, 0.8);
    pnh_.param("circle_show_radius", circle_show_radius_, 0.5);
    pnh_.param("circle_entry_blend_duration",
               circle_entry_blend_duration_, 4.0);
    pnh_.param("circle_goal_max_step", circle_goal_max_step_, 0.06);
    pnh_.param("circle_goal_yaw_step", circle_goal_yaw_step_, 0.20);
    int circle_prepare_formation = cluster_msgs::LeaderCmd::FORMATION_TRIANGLE;
    pnh_.param("circle_prepare_formation", circle_prepare_formation,
               static_cast<int>(cluster_msgs::LeaderCmd::FORMATION_TRIANGLE));
    circle_prepare_formation_ = static_cast<uint8_t>(circle_prepare_formation);
    pnh_.param("assignment_switch_margin", assignment_switch_margin_, 0.15);
    pnh_.param("lock_robot_slots", lock_robot_slots_, false);
    pnh_.param("prevent_crossing_assignment", prevent_crossing_assignment_, true);
    pnh_.param("assignment_min_path_separation",
               assignment_min_path_separation_, 0.55);
    pnh_.param("allow_runtime_reassignment", allow_runtime_reassignment_, false);
    pnh_.param("waypoint_planning_enabled", waypoint_planning_enabled_, true);
    pnh_.param("leader_keepout_radius", leader_keepout_radius_, 0.60);
    pnh_.param("leader_detour_radius", leader_detour_radius_, 0.90);
    pnh_.param("peer_keepout_radius", peer_keepout_radius_, 0.55);
    pnh_.param("peer_detour_radius", peer_detour_radius_, 0.85);
    pnh_.param("waypoint_step", waypoint_step_, 0.65);
    pnh_.param("waypoint_goal_tolerance", waypoint_goal_tolerance_, 0.18);
    pnh_.param("published_goal_min_separation",
               published_goal_min_separation_, 0.70);
    pnh_.param("waypoint_leader_static_v_threshold",
               waypoint_leader_static_v_threshold_, 0.04);
    pnh_.param("waypoint_leader_static_w_threshold",
               waypoint_leader_static_w_threshold_, 0.04);
    pnh_.param("waypoint_leader_static_dwell",
               waypoint_leader_static_dwell_, 1.0);
    pnh_.param("debug_enabled", debug_enabled_, false);
    pnh_.param("debug_period", debug_period_, 0.8);
    pnh_.param("tf_max_age", tf_max_age_, 1.0);
    pnh_.param("map_tf_future_tolerance", map_tf_future_tolerance_, 1.2);
    pnh_.param("enable_odom_prediction", enable_odom_prediction_, true);
    pnh_.param("prediction_max_age", prediction_max_age_, 6.0);
    pnh_.param("prediction_odom_max_age", prediction_odom_max_age_, 1.0);
    pnh_.param("leader_assist_enabled", leader_assist_enabled_, true);
    pnh_.param("leader_assist_transition_timeout",
               leader_assist_transition_timeout_, 5.0);
    pnh_.param("leader_assist_goal_tolerance",
               leader_assist_goal_tolerance_, 0.12);
    pnh_.param("leader_assist_yaw_tolerance",
               leader_assist_yaw_tolerance_, 0.20);
    pnh_.param("leader_assist_participation",
               leader_assist_participation_, 0.45);
    pnh_.param("leader_assist_max_shift",
               leader_assist_max_shift_, 0.25);

    leader_cmd_sub_ = nh_.subscribe(leader_cmd_topic_, 1,
                                    &FormationSlotPlanner::leaderCmdCallback, this);
    leader_assist_goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(leader_assist_goal_topic_, 1);
    robot2_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot2_goal_topic_, 1);
    robot3_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot3_goal_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
                             &FormationSlotPlanner::controlLoop, this);

    ROS_INFO("FormationSlotPlanner: %s -> %s, %s, %s",
             leader_cmd_topic_.c_str(), leader_assist_goal_topic_.c_str(),
             robot2_goal_topic_.c_str(),
             robot3_goal_topic_.c_str());
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTwoPi = 2.0 * kPi;
  static constexpr double kHalfPi = 0.5 * kPi;

  struct Pose2D {
    double x;
    double y;
    double yaw;
  };

  struct Slot {
    double offset_x;
    double offset_y;
    double offset_yaw;
    Pose2D pose;
  };

  struct PlannedGoal {
    Pose2D pose;
    bool detour;
    std::string reason;
    double clearance;
  };

  struct WaypointState {
    bool active{false};
    Pose2D pose{0.0, 0.0, 0.0};
    std::string reason;
    double clearance{999.0};
  };

  struct PoseCache {
    bool valid{false};
    Pose2D map_pose{0.0, 0.0, 0.0};
    Pose2D odom_pose{0.0, 0.0, 0.0};
    ros::Time stamp;
  };

  struct AssistTransition {
    bool active{false};
    uint8_t formation{255};
    ros::Time start_time;
    Pose2D leader_goal{0.0, 0.0, 0.0};
    Pose2D robot2_goal{0.0, 0.0, 0.0};
    Pose2D robot3_goal{0.0, 0.0, 0.0};
  };

  void leaderCmdCallback(const cluster_msgs::LeaderCmd::ConstPtr& msg) {
    latest_leader_cmd_ = *msg;
    leader_cmd_received_ = true;
    last_leader_cmd_time_ = ros::Time::now();
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
        "Slot planner using short odom prediction for %s: cached map TF age %.2fs",
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
      ROS_WARN_THROTTLE(2.0, "Slot planner TF failed %s -> %s: %s",
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
          "Slot planner TF stale for %s: age %.2fs exceeds %.2fs",
          frame.c_str(), age, tf_max_age_);
      return false;
    }
    const double future_tolerance =
        allow_amcl_future ? map_tf_future_tolerance_ : 0.2;
    if (age < -future_tolerance) {
      ROS_WARN_THROTTLE(1.0,
          "Slot planner TF from future for %s: age %.2fs exceeds %.2fs, check clock sync",
          frame.c_str(), age, future_tolerance);
      return false;
    }
    return true;
  }

  Pose2D offsetToMap(const Pose2D& leader, double offset_x,
                     double offset_y, double offset_yaw) const {
    const double c = std::cos(leader.yaw);
    const double s = std::sin(leader.yaw);
    Pose2D pose;
    pose.x = leader.x + offset_x * c - offset_y * s;
    pose.y = leader.y + offset_x * s + offset_y * c;
    pose.yaw = cluster_common::normalizeAngle(leader.yaw + offset_yaw);
    return pose;
  }

  std::vector<Slot> buildSlots(const Pose2D& leader, uint8_t formation) const {
    const double d = formation_spacing_;
    std::vector<Slot> slots;
    if (formation == cluster_msgs::LeaderCmd::FORMATION_LINE) {
      slots.push_back({0.0, -d, 0.0, Pose2D()});
      slots.push_back({0.0, d, 0.0, Pose2D()});
    } else if (formation == cluster_msgs::LeaderCmd::FORMATION_TRIANGLE) {
      slots.push_back({-d, -d, 0.0, Pose2D()});
      slots.push_back({-d, d, 0.0, Pose2D()});
    } else {
      slots.push_back({-d, 0.0, 0.0, Pose2D()});
      slots.push_back({-2.0 * d, 0.0, 0.0, Pose2D()});
    }

    for (auto& slot : slots) {
      slot.pose = offsetToMap(leader, slot.offset_x, slot.offset_y, slot.offset_yaw);
    }
    return slots;
  }

  Pose2D computeSharedLeaderGoal(const Pose2D& leader,
                                 const Pose2D& robot2,
                                 const Pose2D& robot3,
                                 const Slot& robot2_slot,
                                 const Slot& robot3_slot) const {
    const double centroid_x = (leader.x + robot2.x + robot3.x) / 3.0;
    const double centroid_y = (leader.y + robot2.y + robot3.y) / 3.0;
    const double mean_offset_x =
        (robot2_slot.offset_x + robot3_slot.offset_x) / 3.0;
    const double mean_offset_y =
        (robot2_slot.offset_y + robot3_slot.offset_y) / 3.0;
    const double c = std::cos(leader.yaw);
    const double s = std::sin(leader.yaw);

    Pose2D goal;
    const double shared_x = centroid_x - (mean_offset_x * c - mean_offset_y * s);
    const double shared_y = centroid_y - (mean_offset_x * s + mean_offset_y * c);
    const double participation =
        cluster_common::clamp(leader_assist_participation_, 0.0, 1.0);
    double dx = (shared_x - leader.x) * participation;
    double dy = (shared_y - leader.y) * participation;
    const double shift = std::hypot(dx, dy);
    if (leader_assist_max_shift_ > 0.0 && shift > leader_assist_max_shift_) {
      const double scale = leader_assist_max_shift_ / shift;
      dx *= scale;
      dy *= scale;
    }
    goal.x = leader.x + dx;
    goal.y = leader.y + dy;
    goal.yaw = leader.yaw;
    return goal;
  }

  void startLeaderAssistTransition(const Pose2D& leader,
                                   const Pose2D& robot2,
                                   const Pose2D& robot3,
                                   const Slot& robot2_slot,
                                   const Slot& robot3_slot,
                                   const ros::Time& now) {
    leader_assist_.active = true;
    leader_assist_.formation = latest_leader_cmd_.formation;
    leader_assist_.start_time = now;
    leader_assist_.leader_goal = computeSharedLeaderGoal(
        leader, robot2, robot3, robot2_slot, robot3_slot);
    leader_assist_.robot2_goal = offsetToMap(
        leader_assist_.leader_goal, robot2_slot.offset_x,
        robot2_slot.offset_y, robot2_slot.offset_yaw);
    leader_assist_.robot3_goal = offsetToMap(
        leader_assist_.leader_goal, robot3_slot.offset_x,
        robot3_slot.offset_y, robot3_slot.offset_yaw);
    ROS_INFO("Leader assist transition: center formation=%u leader=(%.2f,%.2f) "
             "r2=(%.2f,%.2f) r3=(%.2f,%.2f) assist_shift=%.2fm",
             leader_assist_.formation,
             leader_assist_.leader_goal.x, leader_assist_.leader_goal.y,
             leader_assist_.robot2_goal.x, leader_assist_.robot2_goal.y,
             leader_assist_.robot3_goal.x, leader_assist_.robot3_goal.y,
             std::hypot(leader_assist_.leader_goal.x - leader.x,
                        leader_assist_.leader_goal.y - leader.y));
  }

  bool leaderAssistSettled(const Pose2D& leader,
                           const Pose2D& robot2,
                           const Pose2D& robot3,
                           const ros::Time& now) const {
    if (!leader_assist_.active) {
      return true;
    }
    if ((now - leader_assist_.start_time).toSec() >
        leader_assist_transition_timeout_) {
      return true;
    }
    const double leader_dist = std::hypot(
        leader_assist_.leader_goal.x - leader.x,
        leader_assist_.leader_goal.y - leader.y);
    const double leader_yaw = std::fabs(cluster_common::normalizeAngle(
        leader_assist_.leader_goal.yaw - leader.yaw));
    const double robot2_dist = std::hypot(
        leader_assist_.robot2_goal.x - robot2.x,
        leader_assist_.robot2_goal.y - robot2.y);
    const double robot3_dist = std::hypot(
        leader_assist_.robot3_goal.x - robot3.x,
        leader_assist_.robot3_goal.y - robot3.y);
    return leader_dist <= leader_assist_goal_tolerance_ &&
           leader_yaw <= leader_assist_yaw_tolerance_ &&
           robot2_dist <= waypoint_goal_tolerance_ &&
           robot3_dist <= waypoint_goal_tolerance_;
  }

  Pose2D circleCenterFromLeader(const Pose2D& leader) const {
    Pose2D center;
    center.x = leader.x - circle_show_radius_ * std::sin(leader.yaw);
    center.y = leader.y + circle_show_radius_ * std::cos(leader.yaw);
    center.yaw = 0.0;
    return center;
  }

  double phaseFromRobotOrFallback(const Pose2D& robot,
                                  const Pose2D& center,
                                  double fallback_phase) const {
    const double dx = robot.x - center.x;
    const double dy = robot.y - center.y;
    if (std::hypot(dx, dy) < 0.10) {
      return fallback_phase;
    }
    return std::atan2(dy, dx);
  }

  void initializeCirclePhaseSlots(const Pose2D& leader,
                                  const Pose2D& robot2,
                                  const Pose2D& robot3,
                                  const ros::Time& now) {
    circle_center_ = circleCenterFromLeader(leader);
    circle_robot2_entry_slot_ = poseAsLeaderRelativeSlot(leader, robot2);
    circle_robot3_entry_slot_ = poseAsLeaderRelativeSlot(leader, robot3);
    circle_entry_slots_initialized_ = true;
    const double leader_phase = phaseFromRobotOrFallback(
        leader, circle_center_, cluster_common::normalizeAngle(leader.yaw - kHalfPi));
    const double direction = latest_leader_cmd_.leader_vyaw >= 0.0 ? 1.0 : -1.0;
    const double phase_a = cluster_common::normalizeAngle(
        leader_phase - direction * kTwoPi / 3.0);
    const double phase_b = cluster_common::normalizeAngle(
        leader_phase - direction * 2.0 * kTwoPi / 3.0);

    Slot slot_a = buildCirclePhaseSlot(leader, phase_a,
                                       latest_leader_cmd_.leader_vyaw);
    Slot slot_b = buildCirclePhaseSlot(leader, phase_b,
                                       latest_leader_cmd_.leader_vyaw);
    const double keep_cost =
        distance2(robot2, slot_a.pose) + distance2(robot3, slot_b.pose);
    const double swap_cost =
        distance2(robot2, slot_b.pose) + distance2(robot3, slot_a.pose);
    const bool keep_safe = assignmentSafe(robot2, robot3, slot_a, slot_b);
    const bool swap_safe = assignmentSafe(robot2, robot3, slot_b, slot_a);
    const bool swap = swap_safe && (!keep_safe || swap_cost < keep_cost);
    circle_robot2_phase_ = swap ? phase_b : phase_a;
    circle_robot3_phase_ = swap ? phase_a : phase_b;
    circle_phase_last_time_ = now;
    circle_phase_slots_initialized_ = true;
    ROS_INFO("CIRCLE_SHOW phase slots locked: robot2_phase=%.2f robot3_phase=%.2f "
             "assign=%s cost(keep=%.2f,swap=%.2f) safe(keep=%d,swap=%d) "
             "center=(%.2f,%.2f)",
             circle_robot2_phase_, circle_robot3_phase_, swap ? "swap" : "keep",
             keep_cost, swap_cost, keep_safe ? 1 : 0, swap_safe ? 1 : 0,
             circle_center_.x, circle_center_.y);
  }

  Slot buildCirclePhaseSlot(const Pose2D& leader,
                            double phase,
                            double angular_speed) const {
    Slot slot;
    slot.pose.x = circle_center_.x + circle_show_radius_ * std::cos(phase);
    slot.pose.y = circle_center_.y + circle_show_radius_ * std::sin(phase);
    const double tangent_sign = angular_speed >= 0.0 ? 1.0 : -1.0;
    slot.pose.yaw = cluster_common::normalizeAngle(
        phase + tangent_sign * kHalfPi);

    const double dx = slot.pose.x - leader.x;
    const double dy = slot.pose.y - leader.y;
    const double c = std::cos(leader.yaw);
    const double s = std::sin(leader.yaw);
    slot.offset_x = dx * c + dy * s;
    slot.offset_y = -dx * s + dy * c;
    slot.offset_yaw = cluster_common::normalizeAngle(slot.pose.yaw -
                                                     leader.yaw);
    return slot;
  }

  Slot poseAsLeaderRelativeSlot(const Pose2D& leader,
                                const Pose2D& pose) const {
    Slot slot;
    slot.pose = pose;
    const double dx = pose.x - leader.x;
    const double dy = pose.y - leader.y;
    const double c = std::cos(leader.yaw);
    const double s = std::sin(leader.yaw);
    slot.offset_x = dx * c + dy * s;
    slot.offset_y = -dx * s + dy * c;
    slot.offset_yaw = cluster_common::normalizeAngle(pose.yaw - leader.yaw);
    return slot;
  }

  std::vector<Slot> buildCircleShowSlots(const Pose2D& leader,
                                         const ros::Time& now) {
    if (!circle_phase_slots_initialized_) {
      return buildSlots(leader, circle_prepare_formation_);
    }

    double dt = 0.0;
    if (!circle_phase_last_time_.isZero()) {
      dt = std::max(0.0, (now - circle_phase_last_time_).toSec());
    }
    circle_phase_last_time_ = now;
    const double angular_speed = latest_leader_cmd_.leader_vyaw;
    circle_robot2_phase_ = cluster_common::normalizeAngle(
        circle_robot2_phase_ + angular_speed * dt);
    circle_robot3_phase_ = cluster_common::normalizeAngle(
        circle_robot3_phase_ + angular_speed * dt);

    std::vector<Slot> slots;
    slots.reserve(2);
    // Circle-show uses fixed robot-specific phase ownership. Robot2 and robot3
    // keep the phase captured at entry, so neither follower crosses through the
    // other follower to claim a different orbit slot.
    slots.push_back(buildCirclePhaseSlot(leader, circle_robot2_phase_,
                                         angular_speed));
    slots.push_back(buildCirclePhaseSlot(leader, circle_robot3_phase_,
                                         angular_speed));
    return slots;
  }

  std::vector<Slot> blendSlots(const std::vector<Slot>& from,
                               const std::vector<Slot>& to,
                               double ratio) const {
    if (from.size() != to.size()) {
      return to;
    }
    ratio = cluster_common::clamp(ratio, 0.0, 1.0);
    std::vector<Slot> blended;
    blended.reserve(to.size());
    for (std::size_t i = 0; i < to.size(); ++i) {
      Slot slot;
      slot.offset_x = from[i].offset_x +
          (to[i].offset_x - from[i].offset_x) * ratio;
      slot.offset_y = from[i].offset_y +
          (to[i].offset_y - from[i].offset_y) * ratio;
      slot.offset_yaw = cluster_common::normalizeAngle(
          from[i].offset_yaw +
          cluster_common::normalizeAngle(to[i].offset_yaw -
                                         from[i].offset_yaw) * ratio);
      slot.pose.x = from[i].pose.x + (to[i].pose.x - from[i].pose.x) * ratio;
      slot.pose.y = from[i].pose.y + (to[i].pose.y - from[i].pose.y) * ratio;
      slot.pose.yaw = cluster_common::normalizeAngle(
          from[i].pose.yaw +
          cluster_common::normalizeAngle(to[i].pose.yaw -
                                         from[i].pose.yaw) * ratio);
      blended.push_back(slot);
    }
    return blended;
  }

  std::vector<Slot> buildCurrentSlots(const Pose2D& leader,
                                      const ros::Time& now) {
    if (latest_leader_cmd_.formation !=
        cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW) {
      return buildSlots(leader, latest_leader_cmd_.formation);
    }

    std::vector<Slot> entry_slots = buildSlots(leader, circle_prepare_formation_);
    if (circle_entry_slots_initialized_) {
      entry_slots.clear();
      entry_slots.push_back(circle_robot2_entry_slot_);
      entry_slots.push_back(circle_robot3_entry_slot_);
    }
    if (!circle_show_started_) {
      return entry_slots;
    }

    std::vector<Slot> orbit_slots = buildCircleShowSlots(leader, now);
    if (circle_entry_blend_duration_ <= 0.0 ||
        circle_show_started_time_.isZero()) {
      return orbit_slots;
    }

    const double age = (now - circle_show_started_time_).toSec();
    if (age >= circle_entry_blend_duration_) {
      return orbit_slots;
    }
    return blendSlots(entry_slots, orbit_slots, age / circle_entry_blend_duration_);
  }

  static double distance2(const Pose2D& a, const Pose2D& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
  }

  geometry_msgs::PoseStamped toPoseStamped(const Slot& slot,
                                           const ros::Time& stamp) const {
    return toPoseStamped(slot.pose, stamp);
  }

  geometry_msgs::PoseStamped toPoseStamped(const Pose2D& pose,
                                           const ros::Time& stamp) const {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);
    return msg;
  }

  PlannedGoal planWaypoint(const Pose2D& robot, const Slot& slot,
                            const Pose2D& leader, const Pose2D& peer,
                            bool leader_static, WaypointState& state) const {
    PlannedGoal goal;
    goal.pose = slot.pose;
    goal.detour = false;
    goal.reason = "final";
    goal.clearance = 999.0;

    if (!waypoint_planning_enabled_ || !leader_static) {
      state.active = false;
      return goal;
    }

    const double target_dist = std::hypot(slot.pose.x - robot.x,
                                          slot.pose.y - robot.y);
    if (target_dist <= waypoint_goal_tolerance_) {
      state.active = false;
      return goal;
    }

    if (state.active) {
      const double waypoint_dist = std::hypot(state.pose.x - robot.x,
                                              state.pose.y - robot.y);
      if (waypoint_dist > waypoint_goal_tolerance_) {
        goal.pose = state.pose;
        goal.detour = true;
        goal.reason = state.reason;
        goal.clearance = state.clearance;
        return goal;
      }
      state.active = false;
    }

    struct Obstacle {
      std::string name;
      Pose2D pose;
      double keepout_radius;
      double detour_radius;
    };
    const Obstacle obstacles[] = {
      {"leader", leader, leader_keepout_radius_, leader_detour_radius_},
      {"peer", peer, peer_keepout_radius_, peer_detour_radius_},
    };

    const Obstacle* selected = nullptr;
    double selected_clearance = 999.0;
    for (const auto& obs : obstacles) {
      const double final_clearance =
          std::hypot(slot.pose.x - obs.pose.x, slot.pose.y - obs.pose.y);
      if (final_clearance < obs.keepout_radius * 0.8) {
        continue;
      }
      const double clearance = distancePointToSegment(
          obs.pose.x, obs.pose.y, robot.x, robot.y, slot.pose.x, slot.pose.y);
      if (clearance < obs.keepout_radius && clearance < selected_clearance) {
        selected = &obs;
        selected_clearance = clearance;
      }
    }

    if (!selected) {
      return goal;
    }

    const double from_angle = std::atan2(robot.y - selected->pose.y,
                                         robot.x - selected->pose.x);
    const double target_angle = std::atan2(slot.pose.y - selected->pose.y,
                                           slot.pose.x - selected->pose.x);
    const double delta = cluster_common::normalizeAngle(target_angle - from_angle);
    const double step = cluster_common::clamp(delta, -waypoint_step_, waypoint_step_);
    const double radius = std::max(selected->detour_radius, selected->keepout_radius);

    goal.pose.x = selected->pose.x + radius * std::cos(from_angle + step);
    goal.pose.y = selected->pose.y + radius * std::sin(from_angle + step);
    // Intermediate points use the path tangent. Applying the final formation
    // yaw here makes a differential-drive follower fight the detour path.
    goal.pose.yaw = std::atan2(goal.pose.y - robot.y, goal.pose.x - robot.x);
    goal.detour = true;
    goal.reason = selected->name;
    goal.clearance = selected_clearance;
    state.active = true;
    state.pose = goal.pose;
    state.reason = goal.reason;
    state.clearance = goal.clearance;
    return goal;
  }

  void resolveGoalConflict(PlannedGoal& robot2_goal,
                            PlannedGoal& robot3_goal,
                            const Slot& robot2_slot,
                            const Slot& robot3_slot,
                            const Pose2D& robot2,
                            const Pose2D& robot3,
                            bool paths_safe,
                            bool leader_static) {
    if (!waypoint_planning_enabled_ || !leader_static) {
      yielding_robot_ = 0;
      return;
    }

    const double robot2_remaining = std::hypot(
        robot2_slot.pose.x - robot2.x, robot2_slot.pose.y - robot2.y);
    const double robot3_remaining = std::hypot(
        robot3_slot.pose.x - robot3.x, robot3_slot.pose.y - robot3.y);
    if (yielding_robot_ != 0) {
      const double moving_remaining =
          yielding_robot_ == 2 ? robot3_remaining : robot2_remaining;
      const double held_remaining =
          yielding_robot_ == 2 ? robot2_remaining : robot3_remaining;
      if (moving_remaining <= waypoint_goal_tolerance_) {
        if (held_remaining <= waypoint_goal_tolerance_) {
          yielding_robot_ = 0;
        } else {
          // The first robot has cleared the corridor. Hold it at its slot and
          // release the robot that yielded during the first stage.
          yielding_robot_ = yielding_robot_ == 2 ? 3 : 2;
        }
      }
    }

    const double published_dist = std::hypot(robot2_goal.pose.x - robot3_goal.pose.x,
                                             robot2_goal.pose.y - robot3_goal.pose.y);
    if (yielding_robot_ == 0 && paths_safe &&
        published_dist >= published_goal_min_separation_) {
      return;
    }

    if (yielding_robot_ == 0) {
      // Let the robot already closer to its slot clear the shared corridor.
      yielding_robot_ = robot2_remaining > robot3_remaining ? 2 : 3;
      ROS_WARN("Coordinating formation transition: robot%d yielding "
               "(goal separation %.2fm, paths_safe=%d)",
               yielding_robot_, published_dist, paths_safe ? 1 : 0);
    }

    PlannedGoal& held_goal = yielding_robot_ == 2 ? robot2_goal : robot3_goal;
    const Pose2D& held_robot = yielding_robot_ == 2 ? robot2 : robot3;
    held_goal.pose = held_robot;
    held_goal.detour = true;
    held_goal.reason = "yield";
    held_goal.clearance = published_dist;
  }

  Pose2D limitCircleGoalStep(const Pose2D& robot,
                             const Pose2D& requested,
                             Pose2D& last_goal,
                             bool& has_last_goal) const {
    if (!has_last_goal) {
      last_goal = robot;
      has_last_goal = true;
    }

    Pose2D limited = requested;
    const double dx = requested.x - last_goal.x;
    const double dy = requested.y - last_goal.y;
    const double dist = std::hypot(dx, dy);
    if (circle_goal_max_step_ > 0.0 && dist > circle_goal_max_step_) {
      const double scale = circle_goal_max_step_ / dist;
      limited.x = last_goal.x + dx * scale;
      limited.y = last_goal.y + dy * scale;
    }

    const double yaw_delta =
        cluster_common::normalizeAngle(requested.yaw - last_goal.yaw);
    if (circle_goal_yaw_step_ > 0.0 &&
        std::fabs(yaw_delta) > circle_goal_yaw_step_) {
      limited.yaw = cluster_common::normalizeAngle(
          last_goal.yaw + std::copysign(circle_goal_yaw_step_, yaw_delta));
    }

    last_goal = limited;
    return limited;
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

  static double orient2d(const Pose2D& a, const Pose2D& b, const Pose2D& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  static bool segmentsIntersect(const Pose2D& a, const Pose2D& b,
                                const Pose2D& c, const Pose2D& d) {
    const double o1 = orient2d(a, b, c);
    const double o2 = orient2d(a, b, d);
    const double o3 = orient2d(c, d, a);
    const double o4 = orient2d(c, d, b);
    return (o1 * o2 < 0.0) && (o3 * o4 < 0.0);
  }

  static double segmentSeparation(const Pose2D& a, const Pose2D& b,
                                  const Pose2D& c, const Pose2D& d) {
    if (segmentsIntersect(a, b, c, d)) {
      return 0.0;
    }
    return std::min(
        std::min(distancePointToSegment(a.x, a.y, c.x, c.y, d.x, d.y),
                 distancePointToSegment(b.x, b.y, c.x, c.y, d.x, d.y)),
        std::min(distancePointToSegment(c.x, c.y, a.x, a.y, b.x, b.y),
                 distancePointToSegment(d.x, d.y, a.x, a.y, b.x, b.y)));
  }

  bool assignmentSafe(const Pose2D& robot2, const Pose2D& robot3,
                      const Slot& robot2_slot, const Slot& robot3_slot) const {
    if (!prevent_crossing_assignment_) {
      return true;
    }
    const double separation = segmentSeparation(
        robot2, robot2_slot.pose, robot3, robot3_slot.pose);
    return separation >= assignment_min_path_separation_;
  }

  void controlLoop(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!enabled_) return;
    if (!leader_cmd_received_ ||
        (now - last_leader_cmd_time_).toSec() > leader_cmd_timeout_) {
      return;
    }
    if (latest_leader_cmd_.mode != cluster_msgs::LeaderCmd::MODE_FORMATION) {
      formation_active_ = false;
      static_candidate_since_ = ros::Time(0);
      yielding_robot_ = 0;
      circle_show_started_ = false;
      circle_show_started_time_ = ros::Time(0);
      circle_phase_slots_initialized_ = false;
      circle_entry_slots_initialized_ = false;
      circle_robot2_goal_initialized_ = false;
      circle_robot3_goal_initialized_ = false;
      circle_phase_last_time_ = ros::Time(0);
      leader_assist_.active = false;
      return;
    }

    if (!formation_active_) {
      formation_active_ = true;
      assignment_initialized_ = false;
      robot2_waypoint_.active = false;
      robot3_waypoint_.active = false;
      yielding_robot_ = 0;
      static_candidate_since_ = ros::Time(0);
      leader_assist_.active = false;
    }

    Pose2D leader;
    Pose2D robot2;
    Pose2D robot3;
    if (!lookupPose(leader_frame_, leader_odom_frame_, leader_cache_, leader) ||
        !lookupPose(robot2_frame_, robot2_odom_frame_, robot2_cache_, robot2) ||
        !lookupPose(robot3_frame_, robot3_odom_frame_, robot3_cache_, robot3)) {
      return;
    }

    const bool formation_changed =
        latest_leader_cmd_.formation != last_formation_;
    if (formation_changed) {
      assignment_initialized_ = false;
      robot2_waypoint_.active = false;
      robot3_waypoint_.active = false;
      yielding_robot_ = 0;
      static_candidate_since_ = ros::Time(0);
      leader_assist_.active = false;
      last_formation_ = latest_leader_cmd_.formation;
      circle_show_started_ = false;
      circle_show_started_time_ = ros::Time(0);
      circle_phase_slots_initialized_ = false;
      circle_entry_slots_initialized_ = false;
      circle_robot2_goal_initialized_ = false;
      circle_robot3_goal_initialized_ = false;
      circle_phase_last_time_ = ros::Time(0);
    }
    const bool circle_show =
        latest_leader_cmd_.formation == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW;
    if (circle_show &&
        !circle_show_started_ &&
        std::fabs(latest_leader_cmd_.leader_vyaw) >= waypoint_leader_static_w_threshold_) {
      circle_show_started_ = true;
      circle_show_started_time_ = now;
      initializeCirclePhaseSlots(leader, robot2, robot3, now);
    }
    std::vector<Slot> slots = buildCurrentSlots(leader, now);
    if (slots.size() < 2) return;

    const double keep_order_cost =
        distance2(robot2, slots[0].pose) + distance2(robot3, slots[1].pose);
    const double swap_cost =
        distance2(robot2, slots[1].pose) + distance2(robot3, slots[0].pose);
    const bool keep_safe = assignmentSafe(robot2, robot3, slots[0], slots[1]);
    const bool swap_safe = assignmentSafe(robot2, robot3, slots[1], slots[0]);
    bool swap = false;
    if (lock_robot_slots_) {
      assignment_initialized_ = true;
    } else if (circle_show) {
      // Circle-show uses fixed robot-specific phase ownership. Dynamic
      // swapping here makes robot2 cut through robot3 when entering the orbit.
      swap = false;
      assignment_initialized_ = true;
    } else if (!assignment_initialized_) {
      swap = (swap_safe && (!keep_safe ||
          swap_cost + assignment_switch_margin_ < keep_order_cost));
      assignment_initialized_ = true;
    } else if (!allow_runtime_reassignment_) {
      // Keep the assignment selected on formation entry. Path-separation
      // tests fluctuate while robots move; using them to swap slots here
      // produces metre-scale target jumps and makes followers cross paths.
      swap = last_swap_assignment_;
    } else {
      swap = last_swap_assignment_;
      if (last_swap_assignment_) {
        if (!swap_safe && keep_safe) {
          swap = false;
        } else if (keep_safe) {
          swap = !(keep_order_cost + assignment_switch_margin_ < swap_cost);
        }
      } else {
        if (!keep_safe && swap_safe) {
          swap = true;
        } else if (swap_safe) {
          swap = swap_cost + assignment_switch_margin_ < keep_order_cost;
        }
      }
    }
    last_swap_assignment_ = swap;

    Slot robot2_slot = swap ? slots[1] : slots[0];
    Slot robot3_slot = swap ? slots[0] : slots[1];
    if (leader_assist_enabled_ && !circle_show) {
      if (formation_changed) {
        startLeaderAssistTransition(leader, robot2, robot3,
                                    robot2_slot, robot3_slot, now);
      }
      if (leaderAssistSettled(leader, robot2, robot3, now)) {
        leader_assist_.active = false;
      } else {
        robot2_slot.pose = leader_assist_.robot2_goal;
        robot3_slot.pose = leader_assist_.robot3_goal;
        leader_assist_goal_pub_.publish(
            toPoseStamped(leader_assist_.leader_goal, now));
      }
    }
    const bool raw_leader_static =
        std::fabs(latest_leader_cmd_.leader_vx) < waypoint_leader_static_v_threshold_ &&
        std::fabs(latest_leader_cmd_.leader_vyaw) < waypoint_leader_static_w_threshold_;
    if (!raw_leader_static) {
      static_candidate_since_ = ros::Time(0);
    } else if (static_candidate_since_.isZero()) {
      static_candidate_since_ = now;
    }
    const bool leader_static = raw_leader_static &&
        (now - static_candidate_since_).toSec() >= waypoint_leader_static_dwell_;
    const bool planning_static = leader_static || circle_show;
    PlannedGoal robot2_goal =
        planWaypoint(robot2, robot2_slot, leader, robot3, planning_static,
                     robot2_waypoint_);
    PlannedGoal robot3_goal =
        planWaypoint(robot3, robot3_slot, leader, robot2, planning_static,
                     robot3_waypoint_);
    const bool selected_paths_safe = swap ? swap_safe : keep_safe;
    if (circle_show) {
      // Circle-show is a moving orbit, not a static formation switch. The
      // waypoint/yielding layer intentionally holds one follower still during
      // static slot changes; applying it here breaks phase synchronization and
      // makes one follower cut across or stop while the others orbit.
      yielding_robot_ = 0;
      robot2_waypoint_.active = false;
      robot3_waypoint_.active = false;
      robot2_goal.pose = robot2_slot.pose;
      robot2_goal.detour = false;
      robot2_goal.reason = "circle";
      robot3_goal.pose = robot3_slot.pose;
      robot3_goal.detour = false;
      robot3_goal.reason = "circle";
      ROS_INFO_THROTTLE(5.0,
          "CIRCLE_SHOW direct orbit goals: r2=(%.2f,%.2f) r3=(%.2f,%.2f)",
          robot2_goal.pose.x, robot2_goal.pose.y,
          robot3_goal.pose.x, robot3_goal.pose.y);
      robot2_goal.pose = limitCircleGoalStep(
          robot2, robot2_goal.pose, circle_robot2_last_goal_,
          circle_robot2_goal_initialized_);
      robot3_goal.pose = limitCircleGoalStep(
          robot3, robot3_goal.pose, circle_robot3_last_goal_,
          circle_robot3_goal_initialized_);
    } else {
      resolveGoalConflict(robot2_goal, robot3_goal, robot2_slot, robot3_slot,
                          robot2, robot3, selected_paths_safe, planning_static);
      circle_robot2_goal_initialized_ = false;
      circle_robot3_goal_initialized_ = false;
    }
    robot2_goal_pub_.publish(toPoseStamped(robot2_goal.pose, now));
    robot3_goal_pub_.publish(toPoseStamped(robot3_goal.pose, now));

    if (debug_enabled_) {
      ROS_INFO_THROTTLE(debug_period_,
          "[SLOT_DBG] form=%u assign=%s safe(keep=%d,swap=%d) r2_slot(offset=%.2f,%.2f,yaw=%.2f final=%.2f,%.2f pub=%.2f,%.2f detour=%d:%s:%.2f) r3_slot(offset=%.2f,%.2f,yaw=%.2f final=%.2f,%.2f pub=%.2f,%.2f detour=%d:%s:%.2f) cost(keep=%.3f,swap=%.3f)",
          latest_leader_cmd_.formation, swap ? "swap" : "keep",
          keep_safe ? 1 : 0, swap_safe ? 1 : 0,
          robot2_slot.offset_x, robot2_slot.offset_y, robot2_slot.offset_yaw,
          robot2_slot.pose.x, robot2_slot.pose.y,
          robot2_goal.pose.x, robot2_goal.pose.y,
          robot2_goal.detour ? 1 : 0, robot2_goal.reason.c_str(),
          robot2_goal.clearance,
          robot3_slot.offset_x, robot3_slot.offset_y, robot3_slot.offset_yaw,
          robot3_slot.pose.x, robot3_slot.pose.y,
          robot3_goal.pose.x, robot3_goal.pose.y,
          robot3_goal.detour ? 1 : 0, robot3_goal.reason.c_str(),
          robot3_goal.clearance,
          keep_order_cost, swap_cost);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber leader_cmd_sub_;
  ros::Publisher leader_assist_goal_pub_;
  ros::Publisher robot2_goal_pub_;
  ros::Publisher robot3_goal_pub_;
  ros::Timer timer_;

  cluster_msgs::LeaderCmd latest_leader_cmd_;
  bool leader_cmd_received_{false};
  bool assignment_initialized_{false};
  bool formation_active_{false};
  bool last_swap_assignment_{false};
  int yielding_robot_{0};
  uint8_t last_formation_{255};
  ros::Time last_leader_cmd_time_;
  ros::Time static_candidate_since_;

  std::string map_frame_;
  std::string leader_frame_;
  std::string robot2_frame_;
  std::string robot3_frame_;
  std::string leader_odom_frame_;
  std::string robot2_odom_frame_;
  std::string robot3_odom_frame_;
  std::string leader_cmd_topic_;
  std::string leader_assist_goal_topic_;
  std::string robot2_goal_topic_;
  std::string robot3_goal_topic_;
  bool enabled_;
  double loop_rate_;
  double leader_cmd_timeout_;
  double formation_spacing_;
  double circle_show_radius_;
  double circle_entry_blend_duration_;
  double circle_goal_max_step_;
  double circle_goal_yaw_step_;
  uint8_t circle_prepare_formation_;
  double assignment_switch_margin_;
  bool lock_robot_slots_;
  bool prevent_crossing_assignment_;
  double assignment_min_path_separation_;
  bool allow_runtime_reassignment_;
  bool waypoint_planning_enabled_;
  double leader_keepout_radius_;
  double leader_detour_radius_;
  double peer_keepout_radius_;
  double peer_detour_radius_;
  double waypoint_step_;
  double waypoint_goal_tolerance_;
  double published_goal_min_separation_;
  double waypoint_leader_static_v_threshold_;
  double waypoint_leader_static_w_threshold_;
  double waypoint_leader_static_dwell_;
  bool debug_enabled_;
  double debug_period_;
  double tf_max_age_;
  double map_tf_future_tolerance_;
  bool enable_odom_prediction_;
  double prediction_max_age_;
  double prediction_odom_max_age_;
  bool leader_assist_enabled_;
  double leader_assist_transition_timeout_;
  double leader_assist_goal_tolerance_;
  double leader_assist_yaw_tolerance_;
  double leader_assist_participation_;
  double leader_assist_max_shift_;
  PoseCache leader_cache_;
  PoseCache robot2_cache_;
  PoseCache robot3_cache_;
  WaypointState robot2_waypoint_;
  WaypointState robot3_waypoint_;
  bool circle_show_started_{false};
  ros::Time circle_show_started_time_;
  bool circle_phase_slots_initialized_{false};
  bool circle_entry_slots_initialized_{false};
  Pose2D circle_center_{0.0, 0.0, 0.0};
  Slot circle_robot2_entry_slot_;
  Slot circle_robot3_entry_slot_;
  bool circle_robot2_goal_initialized_{false};
  bool circle_robot3_goal_initialized_{false};
  Pose2D circle_robot2_last_goal_{0.0, 0.0, 0.0};
  Pose2D circle_robot3_last_goal_{0.0, 0.0, 0.0};
  double circle_robot2_phase_{0.0};
  double circle_robot3_phase_{0.0};
  ros::Time circle_phase_last_time_;
  AssistTransition leader_assist_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_slot_planner");
  FormationSlotPlanner planner;
  ros::spin();
  return 0;
}
