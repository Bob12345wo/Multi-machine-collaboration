# Circle Show Safe Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make CIRCLE_SHOW continuously orbit at a safe radius and recover to the pre-circle formation only after an explicit `e` exit and verified follower settling.

**Architecture:** Keep `/robot1/leader_cmd` and existing formation messages unchanged. Add a ROS-independent `CircleShowRecovery` state helper to `cluster_formation`, then use it from `LeaderController` to select circle, recovery, and normal commands. Recovery uses the existing formation-slot planner: car1 is held stationary while the controller publishes the stored ordinary formation.

**Tech Stack:** ROS1 Melodic, roscpp, std_msgs, catkin, gtest, Python rospy.

---

## File structure

- `catkin_ws/src/cluster_formation/include/cluster_formation/circle_show_recovery.h`: ROS-independent state machine and settle-timer decision logic.
- `catkin_ws/src/cluster_formation/src/leader_controller.cpp`: subscribes to explicit circle exit, delegates transition decisions to the helper, and preserves circle geometry under adaptive slowing.
- `catkin_ws/src/cluster_formation/include/cluster_formation/leader_controller.h`: declares circle-exit callback, subscriber, helper state, and parameters.
- `catkin_ws/src/cluster_formation/test/circle_show_recovery_test.cpp`: gtest coverage for entry, exit, dwell, repeated commands, and abort.
- `catkin_ws/src/cluster_formation/CMakeLists.txt`: builds the test only when `CATKIN_ENABLE_TESTING` is enabled.
- `catkin_ws/src/cluster_bringup/scripts/teleop_keyboard.py`: publishes `/robot1/circle_exit` when `e` is pressed and blocks manual velocity commands while a local circle/recovery state is active.
- `catkin_ws/src/cluster_bringup/config/formation_params.yaml`: supplies safe radius and recovery thresholds.
- `README.md`: documents `e`, changed circle radius/speed, and the safe recovery flow.

### Task 1: Add a testable circle-recovery state helper

**Files:**
- Create: `catkin_ws/src/cluster_formation/include/cluster_formation/circle_show_recovery.h`
- Create: `catkin_ws/src/cluster_formation/test/circle_show_recovery_test.cpp`
- Modify: `catkin_ws/src/cluster_formation/CMakeLists.txt`

- [ ] **Step 1: Write the failing state-machine test**

```cpp
#include <gtest/gtest.h>
#include "cluster_formation/circle_show_recovery.h"

TEST(CircleShowRecovery, exitsOnlyAfterBothFollowersSettleForDwell) {
  cluster_formation::CircleShowRecovery state;
  state.enter(3U);
  ASSERT_EQ(state.phase(), cluster_formation::CircleShowPhase::ACTIVE);
  ASSERT_TRUE(state.requestExit());
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::RECOVERING);
  EXPECT_FALSE(state.updateRecovery(true, true, 0.0));
  EXPECT_FALSE(state.updateRecovery(true, true, 0.8));
  EXPECT_TRUE(state.updateRecovery(true, true, 1.0));
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::NORMAL);
  EXPECT_EQ(state.recoveryFormation(), 3U);
}

TEST(CircleShowRecovery, ignoresRepeatedEnterAndExitAndAbortsOnIdle) {
  cluster_formation::CircleShowRecovery state;
  state.enter(1U);
  EXPECT_FALSE(state.enter(3U));
  EXPECT_TRUE(state.requestExit());
  EXPECT_FALSE(state.requestExit());
  state.abort();
  EXPECT_EQ(state.phase(), cluster_formation::CircleShowPhase::NORMAL);
  EXPECT_FALSE(state.updateRecovery(true, true, 10.0));
}
```

- [ ] **Step 2: Enable and run the test to verify it fails because the helper is missing**

Add this CMake block after `add_dependencies(leader_controller_node ...)`:

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(circle_show_recovery_test
    test/circle_show_recovery_test.cpp
  )
  if(TARGET circle_show_recovery_test)
    target_link_libraries(circle_show_recovery_test ${catkin_LIBRARIES})
  endif()
endif()
```

Run:

```bash
cd ~/agilex_ws
catkin_make --pkg cluster_formation
catkin_make run_tests_cluster_formation
```

Expected: compilation fails with `cluster_formation/circle_show_recovery.h: No such file or directory`.

- [ ] **Step 3: Implement the minimal header-only helper**

```cpp
#ifndef CLUSTER_FORMATION_CIRCLE_SHOW_RECOVERY_H
#define CLUSTER_FORMATION_CIRCLE_SHOW_RECOVERY_H

#include <cstdint>

namespace cluster_formation {

enum class CircleShowPhase { NORMAL, ACTIVE, RECOVERING };

class CircleShowRecovery {
 public:
  bool enter(uint8_t formation) {
    if (phase_ != CircleShowPhase::NORMAL) return false;
    recovery_formation_ = formation;
    phase_ = CircleShowPhase::ACTIVE;
    settled_since_ = -1.0;
    return true;
  }

  bool requestExit() {
    if (phase_ != CircleShowPhase::ACTIVE) return false;
    phase_ = CircleShowPhase::RECOVERING;
    settled_since_ = -1.0;
    return true;
  }

  bool updateRecovery(bool robot2_settled, bool robot3_settled,
                      double now_sec, double dwell_sec = 1.0) {
    if (phase_ != CircleShowPhase::RECOVERING) return false;
    if (!robot2_settled || !robot3_settled) {
      settled_since_ = -1.0;
      return false;
    }
    if (settled_since_ < 0.0) {
      settled_since_ = now_sec;
      return false;
    }
    if (now_sec - settled_since_ < dwell_sec) return false;
    phase_ = CircleShowPhase::NORMAL;
    settled_since_ = -1.0;
    return true;
  }

  void abort() { phase_ = CircleShowPhase::NORMAL; settled_since_ = -1.0; }
  CircleShowPhase phase() const { return phase_; }
  uint8_t recoveryFormation() const { return recovery_formation_; }

 private:
  CircleShowPhase phase_{CircleShowPhase::NORMAL};
  uint8_t recovery_formation_{0};
  double settled_since_{-1.0};
};

}  // namespace cluster_formation

#endif
```

- [ ] **Step 4: Re-run the focused test and verify it passes**

Run:

```bash
cd ~/agilex_ws
catkin_make --pkg cluster_formation
catkin_make run_tests_cluster_formation
catkin_test_results build/test_results
```

Expected: `circle_show_recovery_test` passes with zero failures.

- [ ] **Step 5: Commit the isolated state-machine change**

```bash
git add catkin_ws/src/cluster_formation/include/cluster_formation/circle_show_recovery.h catkin_ws/src/cluster_formation/test/circle_show_recovery_test.cpp catkin_ws/src/cluster_formation/CMakeLists.txt
git commit -m "test: cover circle show recovery state"
```

### Task 2: Integrate explicit exit and settled recovery in the leader controller

**Files:**
- Modify: `catkin_ws/src/cluster_formation/include/cluster_formation/leader_controller.h`
- Modify: `catkin_ws/src/cluster_formation/src/leader_controller.cpp`
- Test: `catkin_ws/src/cluster_formation/test/circle_show_recovery_test.cpp`

- [ ] **Step 1: Extend the failing test with fresh-status and interrupted-settle cases**

```cpp
TEST(CircleShowRecovery, resetsDwellWhenEitherFollowerLosesSettlement) {
  cluster_formation::CircleShowRecovery state;
  state.enter(0U);
  ASSERT_TRUE(state.requestExit());
  EXPECT_FALSE(state.updateRecovery(true, true, 0.0));
  EXPECT_FALSE(state.updateRecovery(true, false, 0.6));
  EXPECT_FALSE(state.updateRecovery(true, true, 0.7));
  EXPECT_TRUE(state.updateRecovery(true, true, 1.7));
}
```

- [ ] **Step 2: Run it and verify the current helper fails on this missing behaviour**

Run:

```bash
cd ~/agilex_ws
catkin_make run_tests_cluster_formation
```

Expected: the test exposes any incorrect settle-timer reset.

- [ ] **Step 3: Add the controller callback and recovery decision**

In `leader_controller.h`, include the new helper and add:

```cpp
void circleExitCallback(const std_msgs::Bool::ConstPtr& msg);
bool followerSettled(const cluster_msgs::FollowerStatus& status,
                     const ros::Time& receipt_time, const ros::Time& now) const;

ros::Subscriber circle_exit_sub_;
CircleShowRecovery circle_recovery_;
double circle_exit_settle_error_;
double circle_exit_settle_dwell_;
```

In the constructor, subscribe and load parameters:

```cpp
pnh_.param("circle_exit_settle_error", circle_exit_settle_error_, 0.12);
pnh_.param("circle_exit_settle_dwell", circle_exit_settle_dwell_, 1.0);
circle_exit_sub_ = nh_.subscribe("/robot1/circle_exit", 1,
    &LeaderController::circleExitCallback, this);
```

The callback calls `circle_recovery_.requestExit()` only for `true` messages,
logs the saved formation, and publishes no direct velocity.  In
`computeFormationTarget()`, choose formation and command as follows:

```cpp
if (circle_recovery_.phase() == CircleShowPhase::RECOVERING) {
  current_formation_ = circle_recovery_.recoveryFormation();
  geometry_msgs::Twist stop;
  publishCmdVel(stop, true, false);
  const ros::Time now = ros::Time::now();
  const bool robot2_settled = followerSettled(
      latest_follower_status_, last_follower_status_time_, now);
  const bool robot3_settled = followerSettled(
      latest_follower3_status_, last_follower3_status_time_, now);
  circle_recovery_.updateRecovery(robot2_settled, robot3_settled,
      now.toSec(), circle_exit_settle_dwell_);
}
```

`followerSettled` must return true only when status was received, is fresher
than `formation_status_timeout_`, `error_dist` is finite, and its magnitude is
at or below `circle_exit_settle_error_`.  Any transition to IDLE and any
non-formation `setMode` request must call `circle_recovery_.abort()`.

- [ ] **Step 4: Preserve circle geometry while adaptive slowing is active**

In the circle branch of `computeFormationTarget()`, replace
`applyAdaptiveFormationSpeed(show_cmd)` with equal scaling:

```cpp
const double circle_scale = adaptive_formation_speed_enabled_
    ? adaptive_formation_speed_scale_ : 1.0;
show_cmd.linear.x *= circle_scale;
show_cmd.angular.z *= circle_scale;
```

Use `circle_recovery_.enter(last_non_circle_formation_)` whenever a valid
`CIRCLE_SHOW` request transitions from normal formation.  Remove the existing
implicit exit blocks in `teleopVelCallback()` and `navVelCallback()`; while the
state is ACTIVE or RECOVERING they must return without publishing non-zero
manual/navigation commands.

- [ ] **Step 5: Run package tests and build**

Run:

```bash
cd ~/agilex_ws
catkin_make --pkg cluster_formation
catkin_make run_tests_cluster_formation
catkin_test_results build/test_results
```

Expected: build succeeds and every `cluster_formation` test passes.

- [ ] **Step 6: Commit controller integration**

```bash
git add catkin_ws/src/cluster_formation/include/cluster_formation/leader_controller.h catkin_ws/src/cluster_formation/src/leader_controller.cpp catkin_ws/src/cluster_formation/test/circle_show_recovery_test.cpp
git commit -m "feat: recover safely from circle show"
```

### Task 3: Expose the `e` exit command and safe defaults

**Files:**
- Modify: `catkin_ws/src/cluster_bringup/scripts/teleop_keyboard.py`
- Modify: `catkin_ws/src/cluster_bringup/config/formation_params.yaml`
- Modify: `README.md`

- [ ] **Step 1: Add a failing keyboard-source regression check**

Create `catkin_ws/src/cluster_bringup/test/test_teleop_circle_exit.py`:

```python
from pathlib import Path


def test_keyboard_declares_explicit_circle_exit():
    source = Path(__file__).parents[1] / 'scripts' / 'teleop_keyboard.py'
    text = source.read_text(encoding='utf-8')
    assert "'/robot1/circle_exit'" in text
    assert "elif key == 'e':" in text
```

- [ ] **Step 2: Run the check and verify it fails before the keyboard change**

Run:

```bash
python3 -m pytest catkin_ws/src/cluster_bringup/test/test_teleop_circle_exit.py -q
```

Expected: FAIL because the source does not yet contain `/robot1/circle_exit`.

- [ ] **Step 3: Implement the keyboard exit publisher**

Add `self.circle_exit_pub = rospy.Publisher('/robot1/circle_exit', Bool,
queue_size=1)` in `TeleopKeyboard.__init__`.  Add `e` to the module docstring
and `print_help()`.  In `run()`, insert this branch before normal formation
keys:

```python
elif key == 'e':
    if self.service_key_ready(key):
        self.vx = 0.0
        self.vz = 0.0
        self.last_motion_key_time = rospy.Time(0)
        self.circle_exit_pub.publish(Bool(data=True))
        rospy.loginfo("Request CIRCLE_SHOW safe exit")
```

`LeaderController` remains the authority for ignoring `w/a/s/d` in circle or
recovery phases; the keyboard retains normal teleoperation semantics outside
those phases.

- [ ] **Step 4: Configure safe defaults and document operation**

Change this block in `formation_params.yaml`:

```yaml
# Key 3 circle-show formation.  Key e stops car1 and safely restores the
# ordinary formation active before entry.
circle_show_radius: 0.80
circle_show_angular_speed: 0.16
circle_exit_settle_error: 0.12
circle_exit_settle_dwell: 1.0
```

In `README.md`, update the key table with `e`, describe the explicit exit
sequence, replace the old 0.5 m display radius, and state that `w/a/s/d` do
not cancel circle mode.

- [ ] **Step 5: Run source regression checks and package build**

Run:

```bash
python3 -m pytest catkin_ws/src/cluster_bringup/test/test_teleop_circle_exit.py -q
cd ~/agilex_ws
catkin_make --pkg cluster_formation
catkin_make run_tests_cluster_formation
```

Expected: pytest reports one passing test; catkin build and test commands pass.

- [ ] **Step 6: Commit controls, configuration, and documentation**

```bash
git add catkin_ws/src/cluster_bringup/scripts/teleop_keyboard.py catkin_ws/src/cluster_bringup/config/formation_params.yaml catkin_ws/src/cluster_bringup/test/test_teleop_circle_exit.py README.md
git commit -m "feat: add explicit circle show exit control"
```

### Task 4: Validate on three vehicles

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Build and deploy the changed packages to each vehicle**

Run on each car after copying the changed source into `~/agilex_ws/src`:

```bash
cd ~/agilex_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_formation;cluster_following;cluster_bringup" -j2
source devel/setup.bash
```

Expected: all five packages compile successfully.

- [ ] **Step 2: Run the documented low-speed recovery test**

Start the ordinary three-car stack, select TRIANGLE with `4`, wait for both
`/robotX/follower_status.error_dist` values to settle, press `3` for ten
seconds, then press `e`.  Observe that car1 stops, each follower receives
assigned goals, and both return to the prior TRIANGLE slots.

Run on car1 during and after exit:

```bash
rostopic echo /robot1/leader_cmd -n 1 --noarr
rostopic echo /robot2/follower_status -n 3 --noarr
rostopic echo /robot3/follower_status -n 3 --noarr
rostopic info /robot2/cmd_vel
rostopic info /robot3/cmd_vel
```

Expected: final `leader_cmd.formation` is TRIANGLE, each `/cmd_vel` has only
its safety-filter publisher, and no persistent `leader keepout danger` or
`peer keepout danger` warning appears after settling.

- [ ] **Step 3: Test abort and normal modes**

Repeat the circle test, press `z` during both active circle and recovery, then
restart normal FORMATION and test `1`, `2`, and `4`.  Record that `z` stops all
commands immediately and each ordinary formation still reaches its slots.

- [ ] **Step 4: Commit the validated operator instructions**

```bash
git add README.md
git commit -m "docs: add circle show recovery validation"
```
