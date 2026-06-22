#ifndef CLUSTER_FOLLOWING_CIRCLE_SLOT_ORBIT_H
#define CLUSTER_FOLLOWING_CIRCLE_SLOT_ORBIT_H

#include <algorithm>
#include <array>
#include <cmath>

namespace cluster_following {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Slot {
  double offset_x{0.0};
  double offset_y{0.0};
  double offset_yaw{0.0};
  Pose2D pose;
};

class CircleOrbitSlotPlanner {
 public:
  void enter(const Pose2D& leader, double radius, double angular_speed,
             double now_sec) {
    radius_ = std::max(radius, 0.0);
    angular_speed_ = angular_speed;
    center_.x = leader.x - radius_ * std::sin(leader.yaw);
    center_.y = leader.y + radius_ * std::cos(leader.yaw);
    center_.yaw = 0.0;
    phase_ = leader.yaw - kHalfPi;
    last_update_sec_ = now_sec;
    active_ = true;
  }

  void reset() {
    active_ = false;
    phase_ = 0.0;
    last_update_sec_ = 0.0;
    angular_speed_ = 0.0;
  }

  bool active() const { return active_; }

  void update(double now_sec, double angular_speed) {
    if (!active_) return;
    const double dt = std::max(0.0, now_sec - last_update_sec_);
    phase_ = normalizeAngle(phase_ + angular_speed_ * dt);
    last_update_sec_ = now_sec;
    angular_speed_ = angular_speed;
  }

  std::array<Slot, 2> slots(double now_sec) const {
    const double dt = active_ ? std::max(0.0, now_sec - last_update_sec_) : 0.0;
    const double leader_phase = normalizeAngle(phase_ + angular_speed_ * dt);
    return {slotAt(leader_phase - kTwoPi / 3.0),
            slotAt(leader_phase - 2.0 * kTwoPi / 3.0)};
  }

  std::array<Slot, 2> bodySlots(const Pose2D& leader, double spacing,
                                unsigned int formation) const {
    const double d = spacing;
    std::array<Slot, 2> out;
    if (formation == 1) {
      out = {makeBodySlot(leader, 0.0, -d, 0.0),
             makeBodySlot(leader, 0.0, d, 0.0)};
    } else if (formation == 3) {
      out = {makeBodySlot(leader, -d, -d, 0.0),
             makeBodySlot(leader, -d, d, 0.0)};
    } else {
      out = {makeBodySlot(leader, -d, 0.0, 0.0),
             makeBodySlot(leader, -2.0 * d, 0.0, 0.0)};
    }
    return out;
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTwoPi = 2.0 * kPi;
  static constexpr double kHalfPi = 0.5 * kPi;

  static double normalizeAngle(double angle) {
    while (angle > kPi) angle -= kTwoPi;
    while (angle < -kPi) angle += kTwoPi;
    return angle;
  }

  Slot slotAt(double phase) const {
    Slot slot;
    slot.pose.x = center_.x + radius_ * std::cos(phase);
    slot.pose.y = center_.y + radius_ * std::sin(phase);
    slot.pose.yaw = normalizeAngle(phase + kHalfPi);
    return slot;
  }

  static Slot makeBodySlot(const Pose2D& leader, double offset_x,
                           double offset_y, double offset_yaw) {
    const double c = std::cos(leader.yaw);
    const double s = std::sin(leader.yaw);
    Slot slot;
    slot.offset_x = offset_x;
    slot.offset_y = offset_y;
    slot.offset_yaw = offset_yaw;
    slot.pose.x = leader.x + offset_x * c - offset_y * s;
    slot.pose.y = leader.y + offset_x * s + offset_y * c;
    slot.pose.yaw = normalizeAngle(leader.yaw + offset_yaw);
    return slot;
  }

  bool active_{false};
  Pose2D center_;
  double radius_{0.0};
  double angular_speed_{0.0};
  double phase_{0.0};
  double last_update_sec_{0.0};
};

}  // namespace cluster_following

#endif  // CLUSTER_FOLLOWING_CIRCLE_SLOT_ORBIT_H
