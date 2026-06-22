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
