$planner = Get-Content -Raw -LiteralPath "$PSScriptRoot/../src/formation_slot_planner_node.cpp"
$follower = Get-Content -Raw -LiteralPath "$PSScriptRoot/../src/map_follower_controller_node.cpp"

if ($planner -notmatch "initializeCirclePhaseSlots") {
  throw "formation_slot_planner must initialize robot-specific CIRCLE_SHOW phases"
}
if ($planner -notmatch "phase_a = cluster_common::normalizeAngle") {
  throw "CIRCLE_SHOW must build ideal 120-degree phase targets"
}
if ($planner -notmatch "swap_safe && \(!keep_safe \|\| swap_cost < keep_cost\)") {
  throw "CIRCLE_SHOW must assign ideal phases by entry cost and path safety"
}
if ($planner -notmatch "limitCircleGoalStep") {
  throw "CIRCLE_SHOW assigned goals must be step-limited to avoid metre-scale jumps"
}
if ($planner -notmatch "Circle-show is a moving orbit, not a static formation switch") {
  throw "CIRCLE_SHOW must bypass static waypoint/yield coordination"
}
if ($planner -notmatch "robot2_goal\.pose = robot2_slot\.pose") {
  throw "CIRCLE_SHOW must publish direct orbit slot goals"
}
if ($planner -notmatch "phase ownership") {
  throw "CIRCLE_SHOW must document robot-specific phase ownership"
}
if ($planner -match "CircleOrbitSlotPlanner") {
  throw "formation_slot_planner must not use fixed ideal orbit slots for CIRCLE_SHOW entry"
}
if ($planner -notmatch "Circle-show uses fixed robot-specific phase ownership[\s\S]*slots.push_back\(buildCirclePhaseSlot\(leader, circle_robot2_phase_") {
  throw "CIRCLE_SHOW must keep robot2 and robot3 on their own captured phases"
}
if ($planner -notmatch "Circle-show uses fixed robot-specific phase ownership[\s\S]*swap = false") {
  throw "CIRCLE_SHOW must keep fixed robot-specific slot ownership"
}
if ($planner -notmatch "if \(circle_show\)[\s\S]*yielding_robot_ = 0;[\s\S]*robot2_waypoint_\.active = false;[\s\S]*robot3_waypoint_\.active = false;") {
  throw "CIRCLE_SHOW must clear yielding and waypoint state"
}
if ($planner -match "circle_show_preparing[\s\S]*swap_cost") {
  throw "CIRCLE_SHOW must not dynamically swap slots during entry"
}
if ($planner -notmatch "circle_show_started_") {
  throw "CIRCLE_SHOW must not infer preparing from zero velocity after the orbit has started"
}
if ($follower -notmatch "circle_assigned_goal") {
  throw "map follower must guard the assigned-goal handling specifically for CIRCLE_SHOW"
}
if ($follower -notmatch "control_mode_ == `"wheeltec_global`" && !circle_assigned_goal") {
  throw "wheeltec_global anchor recompute must be skipped only for CIRCLE_SHOW assigned goals"
}

$leader = Get-Content -Raw -LiteralPath "$PSScriptRoot/../../cluster_formation/src/leader_controller.cpp"
if ($leader -match "circle_scale\s*=\s*0\.0") {
  throw "CIRCLE_SHOW must slow down instead of hard-pausing the orbit"
}
if ($leader -notmatch "circle_slow_scale") {
  throw "CIRCLE_SHOW must use a configured slow scale when followers lag"
}

Write-Host "circle_show_isolation=PASS"
