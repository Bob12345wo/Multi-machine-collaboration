# Circle Show Safe Recovery Design

## Goal

Make `CIRCLE_SHOW` a continuous three-vehicle display mode with an explicit,
safe exit: `3` starts circling and `e` returns car2/car3 to the ordinary
formation that was active before circling began.

## Current Problem

The existing circle mode starts car1 immediately and treats any non-zero
manual command as an implicit exit.  Its 0.5 m circle radius also coincides
with the follower-to-leader danger threshold.  Switching directly from that
moving formation to another formation can leave followers pursuing rapidly
changing targets or held by safety filtering.

## User-Facing Behaviour

- `3` enters continuous `CIRCLE_SHOW` from a normal formation and records the
  current non-circle formation as the recovery formation.
- `e` is the only normal exit command while circle mode is active.  It starts
  recovery instead of allowing an arbitrary manual velocity command to change
  mode.
- During circle mode and recovery, `w`, `a`, `s`, and `d` do not publish a
  non-zero leader velocity.  `z` retains its existing emergency/IDLE meaning.
- Recovery stops car1, switches the published formation back to the saved
  formation, and lets the existing slot planner use its static-leader detour
  and path-conflict logic.
- Recovery completes only after both followers report a finite position error
  at or below the configured tolerance continuously for a configured dwell
  period.  The state then becomes ordinary `FORMATION` using the saved
  formation.
- A second `e` during recovery is ignored.  A `3` during recovery is ignored.
- If the user requests IDLE or the leader controller safety logic enters IDLE,
  recovery is cancelled and all vehicles receive the existing stop behaviour.

## Controller State Model

`LeaderController` keeps its external ROS mode as `FORMATION` so that existing
followers and the slot planner keep consuming `LeaderCmd`.  Internally it adds
three display states:

```text
NORMAL
  -- set CIRCLE_SHOW --> CIRCLE_ACTIVE

CIRCLE_ACTIVE
  -- press e --> CIRCLE_RECOVERING
  -- IDLE/safety --> NORMAL (IDLE ROS mode)

CIRCLE_RECOVERING
  -- both follower errors settled for dwell --> NORMAL (saved formation)
  -- IDLE/safety --> NORMAL (IDLE ROS mode)
```

While `CIRCLE_ACTIVE`, car1 commands the circular velocity.  While
`CIRCLE_RECOVERING`, car1 commands zero velocity and publishes the saved
ordinary formation.  The slot planner sees a stationary leader and therefore
uses its pre-existing intermediate waypoint, crossing-prevention, and staged
yield logic.

## Geometry and Safety

The default circle radius is increased to 0.80 m.  It is greater than the
leader safety distance (0.55 m) and danger distance (0.50 m), while the chord
distance between followers becomes about 1.39 m.  The circle angular speed
remains low at 0.16 rad/s, so car1's nominal speed becomes 0.128 m/s.

The current adaptive speed layer scales linear and angular circle commands
differently.  For circle mode, both components must use the same scale so the
actual circle radius remains equal to the configured radius when follower
tracking error triggers coordination slowing.

## Interfaces and Parameters

No custom ROS messages or services are introduced.

- Add `/robot1/circle_exit` (`std_msgs/Bool`) subscribed by
  `leader_controller`.
- Add a keyboard publisher for that topic and bind it to `e`.
- Add parameters under `formation_params.yaml`:
  - `circle_exit_settle_error: 0.12` metres
  - `circle_exit_settle_dwell: 1.0` seconds
- Keep the existing `/robot1/leader_cmd` message and formation values.

## Error Handling

- Recovery must not declare completion from stale follower status.  The same
  status freshness timeout used by adaptive formation slowing applies.
- Non-finite or missing follower status prevents completion and keeps car1
  stopped in recovery.
- `z`, watchdog/safety transition to IDLE, and node shutdown bypass recovery
  and preserve the existing immediate-stop behaviour.

## Tests

Extract the state-transition and circle-command selection logic into a small
ROS-independent helper so it can be compiled and tested without hardware.
The tests must cover entry, explicit exit, ignoring repeated exit/start events,
IDLE cancellation, recovery completion after dwell, and equal velocity scaling
for circle geometry.

Hardware validation will verify one clean entry, a ten-second circle, explicit
`e` exit, safety-filter warnings, return to the saved formation, and normal
operation of COLUMN/LINE/TRIANGLE afterwards.
