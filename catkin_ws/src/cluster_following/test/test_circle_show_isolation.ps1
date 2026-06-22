$planner = Get-Content -Raw -LiteralPath "$PSScriptRoot/../src/formation_slot_planner_node.cpp"
$follower = Get-Content -Raw -LiteralPath "$PSScriptRoot/../src/map_follower_controller_node.cpp"

if ($planner -notmatch "CircleOrbitSlotPlanner") {
  throw "formation_slot_planner must use the CIRCLE_SHOW-only virtual orbit helper"
}
if ($planner -notmatch "FORMATION_CIRCLE_SHOW[\s\S]*circle_orbit_") {
  throw "virtual orbit helper must be isolated to FORMATION_CIRCLE_SHOW"
}
if ($planner -notmatch "circle_show_preparing") {
  throw "CIRCLE_SHOW must keep adaptive slot assignment during its preparing phase"
}
if ($planner -notmatch "circle_show_preparing[\s\S]*swap_cost") {
  throw "CIRCLE_SHOW preparing assignment must be based on closest slot cost"
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
