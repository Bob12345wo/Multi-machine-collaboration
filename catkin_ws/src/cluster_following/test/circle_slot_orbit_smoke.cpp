#include <cassert>
#include <cmath>

#include "cluster_following/circle_slot_orbit.h"

namespace {

bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}

}  // namespace

int main() {
  using cluster_following::CircleOrbitSlotPlanner;
  using cluster_following::Pose2D;

  CircleOrbitSlotPlanner planner;
  const Pose2D leader_start{1.0, 2.0, 0.25};
  const double radius = 0.8;
  const double angular_speed = 0.16;

  planner.enter(leader_start, radius, angular_speed, 10.0);
  const auto slots_t0 = planner.slots(10.0);
  const auto slots_t1 = planner.slots(11.0);

  assert(slots_t0.size() == 2);
  assert(slots_t1.size() == 2);

  const double d0 = std::hypot(slots_t0[0].pose.x - slots_t0[1].pose.x,
                              slots_t0[0].pose.y - slots_t0[1].pose.y);
  const double d1 = std::hypot(slots_t1[0].pose.x - slots_t1[1].pose.x,
                              slots_t1[0].pose.y - slots_t1[1].pose.y);
  assert(near(d0, std::sqrt(3.0) * radius));
  assert(near(d1, std::sqrt(3.0) * radius));

  const double moved = std::hypot(slots_t1[0].pose.x - slots_t0[0].pose.x,
                                 slots_t1[0].pose.y - slots_t0[0].pose.y);
  assert(moved > 0.01);

  const Pose2D leader_drifted{4.0, -3.0, 2.5};
  const auto column = planner.bodySlots(leader_drifted, 0.8, 0);
  assert(column.size() == 2);
  assert(near(column[0].pose.x, leader_drifted.x - 0.8 * std::cos(leader_drifted.yaw)));

  return 0;
}
