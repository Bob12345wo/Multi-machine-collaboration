#include <cassert>
#include <cmath>

#include "cluster_formation/circle_show_recovery.h"

int main() {
  cluster_formation::CircleShowRecovery state;

  assert(state.enter(3U));
  assert(state.phase() == cluster_formation::CircleShowPhase::PREPARING);
  assert(!state.enter(1U));
  assert(!state.updateStart(true, false, 0.0, 0.5));
  assert(!state.updateStart(true, true, 0.1, 0.5));
  assert(state.updateStart(true, true, 0.6, 0.5));
  assert(state.phase() == cluster_formation::CircleShowPhase::ACTIVE);
  assert(state.requestExit());
  assert(!state.requestExit());

  assert(!state.updateRecovery(true, true, 0.0, 1.0));
  assert(!state.updateRecovery(true, false, 0.6, 1.0));
  assert(!state.updateRecovery(true, true, 0.7, 1.0));
  assert(state.updateRecovery(true, true, 1.7, 1.0));
  assert(state.phase() == cluster_formation::CircleShowPhase::NORMAL);
  assert(state.recoveryFormation() == 3U);

  assert(state.enter(3U, 10.0));
  assert(!state.updateStart(false, false, 12.0, 0.5, 3.0));
  assert(state.updateStart(false, false, 13.0, 0.5, 3.0));
  assert(state.phase() == cluster_formation::CircleShowPhase::ACTIVE);

  state.abort();
  assert(!state.updateRecovery(true, true, 10.0, 1.0));

  const auto circle_cmd = cluster_formation::CircleShowRecovery::scaleCircleCommand(
      {0.08, 0.16}, 0.35);
  assert(std::fabs(circle_cmd.linear_x - 0.028) < 1e-12);
  assert(std::fabs(circle_cmd.angular_z - 0.056) < 1e-12);
  assert(std::fabs(circle_cmd.linear_x / circle_cmd.angular_z - 0.5) < 1e-12);
  return 0;
}
