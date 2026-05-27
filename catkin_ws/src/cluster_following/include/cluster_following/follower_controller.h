#ifndef CLUSTER_FOLLOWING_FOLLOWER_CONTROLLER_H
#define CLUSTER_FOLLOWING_FOLLOWER_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <deque>

#include "cluster_msgs/LeaderCmd.h"
#include "cluster_msgs/FollowerStatus.h"
#include "cluster_common/pid.h"
#include "cluster_common/pose_utils.h"

namespace cluster_following {

struct TrajectoryPoint {
  ros::Time stamp;
  double x, y, theta;
};

class FollowerController {
public:
  FollowerController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FollowerController() = default;

  void spin();

private:
  // Callbacks
  void leaderOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void selfOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void leaderCmdCallback(const cluster_msgs::LeaderCmd::ConstPtr& msg);

  // Control loops
  void controlLoop(const ros::TimerEvent& event);
  void statusLoop(const ros::TimerEvent& event);

  // Tracking methods
  void trackTargetPose();
  void followLeaderPath();
  void stop();
  void tryCalibrate();

  void publishCmdVel(double vx, double vz);
  double computeLookaheadDistance(double current_speed);
  void updateTrackingError(double error_x, double error_y,
                           double error_yaw, double error_dist);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers
  ros::Subscriber leader_odom_sub_;
  ros::Subscriber self_odom_sub_;
  ros::Subscriber leader_cmd_sub_;

  // Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher status_pub_;

  // Timers
  ros::Timer control_timer_;
  ros::Timer status_timer_;

  // Latest data
  nav_msgs::Odometry latest_self_odom_;
  nav_msgs::Odometry latest_leader_odom_;
  cluster_msgs::LeaderCmd latest_leader_cmd_;
  bool self_odom_received_;
  bool leader_odom_received_;
  bool leader_cmd_received_;

  // Current mode
  uint8_t current_mode_;

  // PID controllers
  cluster_common::PID lin_pid_;
  cluster_common::PID ang_pid_;

  // Trajectory buffer for following mode
  std::deque<TrajectoryPoint> trajectory_buffer_;
  double follow_delay_;
  int max_buffer_size_;

  // Parameters
  double loop_rate_;
  double max_linear_speed_;
  double max_angular_speed_;
  double leader_lost_timeout_;
  ros::Time last_leader_odom_time_;
  ros::Time last_leader_cmd_time_;

  // Deadband thresholds
  double pos_deadband_;
  double yaw_deadband_;
  double min_linear_speed_;
  double reverse_entry_distance_;
  double leader_vx_feedforward_gain_;
  double leader_wz_feedforward_gain_;
  double yaw_error_gain_;
  double lateral_error_gain_;
  double turn_yaw_correction_scale_;
  double turn_lateral_correction_scale_;
  double turn_in_place_threshold_;
  bool allow_reverse_while_leader_forward_;

  // Frame calibration
  bool calibrated_;
  double frame_shift_x_;
  double frame_shift_y_;
  double frame_shift_theta_;
  double prev_vz_;

  // Last published tracking error, used by follower status.
  double last_error_x_;
  double last_error_y_;
  double last_error_yaw_;
  double last_error_dist_;
};

}  // namespace cluster_following

#endif  // CLUSTER_FOLLOWING_FOLLOWER_CONTROLLER_H
