$root = Split-Path -Parent $PSScriptRoot
$files = @(
  (Join-Path $root 'config\formation_params.yaml'),
  (Join-Path $root 'config\formation_slot_planner_params.yaml'),
  (Join-Path $root 'config\map_follower_params.yaml')
)

foreach ($file in $files) {
  $text = Get-Content -Raw $file
  if ($text -notmatch '(?m)^circle_show_radius:\s*0\.80\s*$') {
    throw "circle_show_radius must be 0.80 in $file"
  }
}

$formation = Get-Content -Raw (Join-Path $root 'config\formation_params.yaml')
foreach ($param in @(
    'circle_start_settle_error',
    'circle_start_settle_dwell',
    'circle_pause_error',
    'circle_exit_settle_error',
    'circle_exit_settle_dwell')) {
  if ($formation -notmatch "(?m)^$($param):\s*[0-9.]+\s*$") {
    throw "$param must be configured for synchronized CIRCLE_SHOW"
  }
}
