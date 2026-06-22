$source = Get-Content -Raw (Join-Path $PSScriptRoot '..\scripts\teleop_keyboard.py')

if ($source -notmatch "'/robot1/circle_exit'") {
  throw 'teleop keyboard does not declare the circle-exit publisher'
}
if ($source -notmatch "elif key == 'e':") {
  throw 'teleop keyboard does not handle the explicit circle-exit key'
}
