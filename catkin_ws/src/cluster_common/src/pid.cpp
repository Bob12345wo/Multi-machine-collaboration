#include "cluster_common/pid.h"
#include <algorithm>
#include <cmath>

namespace cluster_common {

PID::PID()
  : kp_(0.0), ki_(0.0), kd_(0.0)
  , i_max_(0.0), output_max_(0.0)
  , integral_(0.0), prev_error_(0.0)
  , first_update_(true) {}

PID::PID(double kp, double ki, double kd, double i_max, double output_max)
  : kp_(kp), ki_(ki), kd_(kd)
  , i_max_(i_max), output_max_(output_max)
  , integral_(0.0), prev_error_(0.0)
  , first_update_(true) {}

void PID::setGains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::setLimits(double i_max, double output_max) {
  i_max_ = i_max;
  output_max_ = output_max;
}

double PID::update(double error, double dt) {
  if (first_update_) {
    prev_error_ = error;
    first_update_ = false;
  }

  // Proportional
  double p_term = kp_ * error;

  // Integral with anti-windup
  integral_ += error * dt;
  if (i_max_ > 0.0) {
    integral_ = std::max(-i_max_, std::min(i_max_, integral_));
  }
  double i_term = ki_ * integral_;

  // Derivative (on measurement, not error, to avoid derivative kick)
  double derivative = (error - prev_error_) / std::max(dt, 0.001);
  double d_term = kd_ * derivative;

  prev_error_ = error;

  double output = p_term + i_term + d_term;

  // Output limiting
  if (output_max_ > 0.0) {
    output = std::max(-output_max_, std::min(output_max_, output));
  }

  return output;
}

void PID::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_update_ = true;
}

}  // namespace cluster_common
