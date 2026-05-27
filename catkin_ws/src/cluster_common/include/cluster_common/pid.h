#ifndef CLUSTER_COMMON_PID_H
#define CLUSTER_COMMON_PID_H

namespace cluster_common {

class PID {
public:
  PID();
  PID(double kp, double ki, double kd, double i_max, double output_max);

  void setGains(double kp, double ki, double kd);
  void setLimits(double i_max, double output_max);

  double update(double error, double dt);
  void reset();

  double getKp() const { return kp_; }
  double getKi() const { return ki_; }
  double getKd() const { return kd_; }

private:
  double kp_, ki_, kd_;
  double i_max_, output_max_;
  double integral_;
  double prev_error_;
  bool first_update_;
};

}  // namespace cluster_common

#endif  // CLUSTER_COMMON_PID_H
