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
#include "cluster_following/circle_slot_orbit.h"
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
    pnh_.param<std::string>("robot2_goal_topic", robot2_goal_topic_, "/robot2/assigned_goal");
    pnh_.param<std::string>("robot3_goal_topic", robot3_goal_topic_, "/robot3/assigned_goal");
    pnh_.param("enabled", enabled_, true);
    pnh_.param("loop_rate", loop_rate_, 10.0);
    pnh_.param("leader_cmd_timeout", leader_cmd_timeout_, 0.6);
    pnh_.param("formation_spacing", formation_spacing_, 0.8);
    pnh_.param("circle_show_radius", circle_show_radius_, 0.5);
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

    leader_cmd_sub_ = nh_.subscribe(leader_cmd_topic_, 1,
                                    &FormationSlotPlanner::leaderCmdCallback, this);
    robot2_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot2_goal_topic_, 1);
    robot3_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(robot3_goal_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_),
                             &FormationSlotPlanner::controlLoop, this);

    ROS_INFO("FormationSlotPlanner: %s -> %s, %s",
             leader_cmd_topic_.c_str(), robot2_goal_topic_.c_str(),
             robot3_goal_topic_.c_str());
  }

private:
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

  std::vector<Slot> buildCircleShowSlots(const Pose2D& leader,
                                         double now_sec) {
    if (!circle_orbit_.active()) {
      circle_orbit_.enter(toOrbitPose(leader), circle_show_radius_,
                          latest_leader_cmd_.leader_vyaw, now_sec);
    }
    circle_orbit_.update(now_sec, latest_leader_cmd_.leader_vyaw);
    const auto orbit_slots = circle_orbit_.slots(now_sec);

    std::vector<Slot> slots;
    slots.reserve(2);
    for (const auto& orbit_slot : orbit_slots) {
      const double dx = orbit_slot.pose.x - leader.x;
      const double dy = orbit_slot.pose.y - leader.y;
      const double c = std::cos(leader.yaw);
      const double s = std::sin(leader.yaw);
      Slot slot;
      slot.offset_x = dx * c + dy * s;
      slot.offset_y = -dx * s + dy * c;
      slot.offset_yaw = cluster_common::normalizeAngle(
          orbit_slot.pose.yaw - leader.yaw);
      slot.pose = fromOrbitPose(orbit_slot.pose);
      slots.push_back(slot);
    }
    return slots;
  }

  static cluster_following::Pose2D toOrbitPose(const Pose2D& pose) {
    cluster_following::Pose2D out;
    out.x = pose.x;
    out.y = pose.y;
    out.yaw = pose.yaw;
    return out;
  }

  static Pose2D fromOrbitPose(const cluster_following::Pose2D& pose) {
    Pose2D out;
    out.x = pose.x;
    out.y = pose.y;
    out.yaw = pose.yaw;
    return out;
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
      return;
    }

    if (!formation_active_) {
      formation_active_ = true;
      assignment_initialized_ = false;
      robot2_waypoint_.active = false;
      robot3_waypoint_.active = false;
      yielding_robot_ = 0;
      static_candidate_since_ = ros::Time(0);
    }

    Pose2D leader;
    Pose2D robot2;
    Pose2D robot3;
    if (!lookupPose(leader_frame_, leader_odom_frame_, leader_cache_, leader) ||
        !lookupPose(robot2_frame_, robot2_odom_frame_, robot2_cache_, robot2) ||
        !lookupPose(robot3_frame_, robot3_odom_frame_, robot3_cache_, robot3)) {
      return;
    }

    if (latest_leader_cmd_.formation != last_formation_) {
      assignment_initialized_ = false;
      robot2_waypoint_.active = false;
      robot3_waypoint_.active = false;
      yielding_robot_ = 0;
      static_candidate_since_ = ros::Time(0);
      last_formation_ = latest_leader_cmd_.formation;
      if (latest_leader_cmd_.formation == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW) {
        circle_orbit_.enter(toOrbitPose(leader), circle_show_radius_,
                            latest_leader_cmd_.leader_vyaw, now.toSec());
      } else {
        circle_orbit_.reset();
      }
    }
    std::vector<Slot> slots =
        latest_leader_cmd_.formation == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW
        ? buildCircleShowSlots(leader, now.toSec())
        : buildSlots(leader, latest_leader_cmd_.formation);
    if (slots.size() < 2) return;

    const double keep_order_cost =
        distance2(robot2, slots[0].pose) + distance2(robot3, slots[1].pose);
    const double swap_cost =
        distance2(robot2, slots[1].pose) + distance2(robot3, slots[0].pose);
    const bool keep_safe = assignmentSafe(robot2, robot3, slots[0], slots[1]);
    const bool swap_safe = assignmentSafe(robot2, robot3, slots[1], slots[0]);
    const bool circle_show =
        latest_leader_cmd_.formation == cluster_msgs::LeaderCmd::FORMATION_CIRCLE_SHOW;
    const bool circle_show_preparing = circle_show &&
        std::fabs(latest_leader_cmd_.leader_vx) < waypoint_leader_static_v_threshold_ &&
        std::fabs(latest_leader_cmd_.leader_vyaw) < waypoint_leader_static_w_threshold_;
    bool swap = false;
    if (lock_robot_slots_) {
      assignment_initialized_ = true;
    } else if (circle_show_preparing) {
      swap = (swap_safe && (!keep_safe ||
          swap_cost + assignment_switch_margin_ < keep_order_cost));
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

    const Slot& robot2_slot = swap ? slots[1] : slots[0];
    const Slot& robot3_slot = swap ? slots[0] : slots[1];
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
    PlannedGoal robot2_goal =
        planWaypoint(robot2, robot2_slot, leader, robot3, leader_static,
                     robot2_waypoint_);
    PlannedGoal robot3_goal =
        planWaypoint(robot3, robot3_slot, leader, robot2, leader_static,
                     robot3_waypoint_);
    const bool selected_paths_safe = swap ? swap_safe : keep_safe;
    resolveGoalConflict(robot2_goal, robot3_goal, robot2_slot, robot3_slot,
                        robot2, robot3, selected_paths_safe, leader_static);
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
  std::string robot2_goal_topic_;
  std::string robot3_goal_topic_;
  bool enabled_;
  double loop_rate_;
  double leader_cmd_timeout_;
  double formation_spacing_;
  double circle_show_radius_;
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
  PoseCache leader_cache_;
  PoseCache robot2_cache_;
  PoseCache robot3_cache_;
  WaypointState robot2_waypoint_;
  WaypointState robot3_waypoint_;
  cluster_following::CircleOrbitSlotPlanner circle_orbit_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_slot_planner");
  FormationSlotPlanner planner;
  ros::spin();
  return 0;
}
