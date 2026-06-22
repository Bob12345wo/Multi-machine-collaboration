#ifndef CLUSTER_FORMATION_CIRCLE_SHOW_RECOVERY_H
#define CLUSTER_FORMATION_CIRCLE_SHOW_RECOVERY_H

#include <cstdint>

namespace cluster_formation {

enum class CircleShowPhase { NORMAL, PREPARING, ACTIVE, RECOVERING };

struct CircleCommand {
  double linear_x;
  double angular_z;
};

class CircleShowRecovery {
 public:
  static CircleCommand scaleCircleCommand(CircleCommand command, double scale) {
    command.linear_x *= scale;
    command.angular_z *= scale;
    return command;
  }

  bool enter(uint8_t recovery_formation) {
    if (phase_ != CircleShowPhase::NORMAL) {
      return false;
    }
    recovery_formation_ = recovery_formation;
    phase_ = CircleShowPhase::PREPARING;
    settled_since_ = -1.0;
    return true;
  }

  bool updateStart(bool robot2_settled, bool robot3_settled,
                   double now_sec, double dwell_sec) {
    if (phase_ != CircleShowPhase::PREPARING) {
      return false;
    }
    if (!robot2_settled || !robot3_settled) {
      settled_since_ = -1.0;
      return false;
    }
    if (settled_since_ < 0.0) {
      settled_since_ = now_sec;
      if (dwell_sec <= 0.0) {
        phase_ = CircleShowPhase::ACTIVE;
        settled_since_ = -1.0;
        return true;
      }
      return false;
    }
    if (now_sec - settled_since_ < dwell_sec) {
      return false;
    }
    phase_ = CircleShowPhase::ACTIVE;
    settled_since_ = -1.0;
    return true;
  }

  bool requestExit() {
    if (phase_ != CircleShowPhase::PREPARING &&
        phase_ != CircleShowPhase::ACTIVE) {
      return false;
    }
    phase_ = CircleShowPhase::RECOVERING;
    settled_since_ = -1.0;
    return true;
  }

  bool updateRecovery(bool robot2_settled, bool robot3_settled,
                      double now_sec, double dwell_sec) {
    if (phase_ != CircleShowPhase::RECOVERING) {
      return false;
    }
    if (!robot2_settled || !robot3_settled) {
      settled_since_ = -1.0;
      return false;
    }
    if (settled_since_ < 0.0) {
      settled_since_ = now_sec;
      return false;
    }
    if (now_sec - settled_since_ < dwell_sec) {
      return false;
    }
    phase_ = CircleShowPhase::NORMAL;
    settled_since_ = -1.0;
    return true;
  }

  void abort() {
    phase_ = CircleShowPhase::NORMAL;
    settled_since_ = -1.0;
  }

  CircleShowPhase phase() const { return phase_; }
  uint8_t recoveryFormation() const { return recovery_formation_; }

 private:
  CircleShowPhase phase_{CircleShowPhase::NORMAL};
  uint8_t recovery_formation_{0};
  double settled_since_{-1.0};
};

}  // namespace cluster_formation

#endif  // CLUSTER_FORMATION_CIRCLE_SHOW_RECOVERY_H
