#include <gtest/gtest.h>

#include "cluster_formation/circle_show_recovery.h"

TEST(CircleShowRecovery, exitsOnlyAfterBothFollowersSettleForDwell) {
  cluster_formation::CircleShowRecovery state;
  ASSERT_TRUE(state.enter(3U));
  ASSERT_EQ(state.phase(), cluster_formation::CircleShowPhase::ACTIVE);
  ASSERT_TRUE(state.requestExit());
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::RECOVERING);
  EXPECT_FALSE(state.updateRecovery(true, true, 0.0, 1.0));
  EXPECT_FALSE(state.updateRecovery(true, true, 0.8, 1.0));
  EXPECT_TRUE(state.updateRecovery(true, true, 1.0, 1.0));
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::NORMAL);
  EXPECT_EQ(state.recoveryFormation(), 3U);
}

TEST(CircleShowRecovery, ignoresRepeatedEnterAndExitAndAbortsOnIdle) {
  cluster_formation::CircleShowRecovery state;
  ASSERT_TRUE(state.enter(1U));
  EXPECT_FALSE(state.enter(3U));
  EXPECT_TRUE(state.requestExit());
  EXPECT_FALSE(state.requestExit());
  state.abort();
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::NORMAL);
  EXPECT_FALSE(state.updateRecovery(true, true, 10.0, 1.0));
}

TEST(CircleShowRecovery, resetsDwellWhenEitherFollowerLosesSettlement) {
  cluster_formation::CircleShowRecovery state;
  ASSERT_TRUE(state.enter(0U));
  ASSERT_TRUE(state.requestExit());
  EXPECT_FALSE(state.updateRecovery(true, true, 0.0, 1.0));
  EXPECT_FALSE(state.updateRecovery(true, false, 0.6, 1.0));
  EXPECT_FALSE(state.updateRecovery(true, true, 0.7, 1.0));
  EXPECT_TRUE(state.updateRecovery(true, true, 1.7, 1.0));
}
