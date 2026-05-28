# Multi-machine Collaboration

ROS1 Melodic multi-robot collaboration project for two AgileX LIMO S2 cars.

This workspace focuses on shared-map two-car formation following:

- car1 runs the ROS master, leader controller, map server and AMCL stack.
- car2 runs the follower base, lidar and follower localization.
- Both cars localize against the same `cluster_map.yaml` map.
- The follower controller uses a shared `map` frame and supports a Wheeltec-style global formation mode.

## Repository Layout

```text
catkin_ws/src/cluster_bringup     Launch files, startup scripts and runtime configs
catkin_ws/src/cluster_common      Shared PID and pose utilities
catkin_ws/src/cluster_formation   Leader controller and formation commands
catkin_ws/src/cluster_following   Odom and map based follower controllers
catkin_ws/src/cluster_msgs        ROS messages and services
limo_ros_ref                      LIMO reference launch/config/map files
ros1_melodic/wheeltec_multi       Reference ROS1 multi-machine formation package
```

## Current Startup Flow

Run `roscore` on car1 first:

```bash
cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
roscore
```

Start car2:

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car2_slave.sh /dev/ttyUSB0
```

Start car1:

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car1_master.sh /dev/ttyUSB0
```

Verify localization from car1:

```bash
rostopic hz /robot1/scan
rostopic hz /robot2/scan
rostopic echo /robot1/amcl_pose -n 1 --noarr
rostopic echo /robot2/amcl_pose -n 1 --noarr
rosrun tf tf_echo map robot1/base_link
rosrun tf tf_echo map robot2/base_link
```

Start follower and keyboard control:

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_map_follower.sh
~/agilex_ws/src/cluster_bringup/scripts/start_keyboard.sh
```

Keyboard controls:

```text
c      FORMATION mode
1      COLUMN
2      LINE
3      TRIANGLE_LEFT
4      TRIANGLE_RIGHT
w/s    forward/backward
a/d    turn left/right
space  stop
```

## Notes

- `cluster_map.yaml` and `cluster_map.pgm` should be stored in `~/agilex_ws/src/limo_ros/limo_bringup/maps/` on car1.
- If lidar has no data, try switching the startup script argument between `/dev/ttyUSB0` and `/dev/ttyUSB1`.
- The current follower mode is configured in `catkin_ws/src/cluster_bringup/config/map_follower_params.yaml`.
- `control_mode: body_orbit` keeps robot2 in the leader body frame, so side and triangle formations rotate with car1.
- `cmd_safety_filter_params.yaml` configures the robot2 lidar safety filter between `/robot2/cmd_vel_raw` and `/robot2/cmd_vel`.

## Build

On the car:

```bash
cd ~/agilex_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_formation;cluster_following;cluster_bringup" -j2
source devel/setup.bash
```
