#ifndef CLUSTER_FORMATION_LEADER_CONTROLLER_H
#define CLUSTER_FORMATION_LEADER_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "cluster_msgs/LeaderCmd.h"
#include "cluster_msgs/FollowerStatus.h"
#include "cluster_msgs/SetMode.h"
#include "cluster_msgs/SetFormation.h"
#include "cluster_common/pose_utils.h"

namespace cluster_formation {

struct FormationOffset {
  double x, y, yaw;
};

class LeaderController {
public:
  LeaderController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~LeaderController() = default;

  void spin();

private:
  // Callbacks
  void selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void followerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void followerStatusCallback(const cluster_msgs::FollowerStatus::ConstPtr& msg);
  void teleopVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void returnHomeCallback(const std_msgs::Bool::ConstPtr& msg);

  // Service callbacks
  bool setModeCallback(cluster_msgs::SetMode::Request& req,
                       cluster_msgs::SetMode::Response& res);
  bool setFormationCallback(cluster_msgs::SetFormation::Request& req,
                            cluster_msgs::SetFormation::Response& res);

  // Control loop
  void controlLoop(const ros::TimerEvent& event);

  // Core logic
  void computeFormationTarget();
  void computeFollowTarget();
  void computeReturnHome();
  void checkSafety();
  void publishLeaderCmd();
  geometry_msgs::Pose computeTargetPose(const cluster_common::Pose2D& leader_pose,
                                        const FormationOffset& offset);

  // Helpers
  FormationOffset getFormationOffset(uint8_t formation_type);
  void updatePIDFromReconfig();

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers
  ros::Subscriber self_odom_sub_;
  ros::Subscriber follower_odom_sub_;
  ros::Subscriber follower_status_sub_;
  ros::Subscriber teleop_vel_sub_;
  ros::Subscriber return_home_sub_;

  // Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher leader_cmd_pub_;

  // Services
  ros::ServiceServer set_mode_srv_;
  ros::ServiceServer set_formation_srv_;

  // Timer
  ros::Timer control_timer_;

  // Latest data
  nav_msgs::Odometry latest_self_odom_;
  nav_msgs::Odometry latest_follower_odom_;
  cluster_msgs::FollowerStatus latest_follower_status_;
  geometry_msgs::Twist latest_teleop_cmd_;
  bool self_odom_received_;
  bool follower_odom_received_;
  bool follower_status_received_;
  bool teleop_cmd_received_;
  bool home_pose_received_;
  bool return_home_active_;
  cluster_common::Pose2D home_pose_;

  // State
  uint8_t current_mode_;
  uint8_t current_formation_;
  FormationOffset current_offset_;

  // Safety
  double follower_lost_timeout_;
  double max_formation_error_;
  double max_error_duration_;  // seconds of > max_error before stopping
  ros::Time error_start_time_;
  bool error_exceeded_;
  ros::Time last_follower_status_time_;
  ros::Time last_teleop_cmd_time_;

  // Parameters
  double loop_rate_;
  double max_linear_speed_;
  double max_angular_speed_;
  double speed_limit_;
  double return_home_pos_tolerance_;
  double return_home_yaw_tolerance_;
  double return_home_k_v_;
  double return_home_k_w_;

  // LeaderCmd cache
  cluster_msgs::LeaderCmd cached_leader_cmd_;

  // Custom offsets (from set_mode service)
  bool use_custom_offsets_;
  FormationOffset custom_offset_;
};

}  // namespace cluster_formation

#endif  // CLUSTER_FORMATION_LEADER_CONTROLLER_H
