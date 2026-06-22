# AgileX LIMO 三车协同编队

本项目基于 ROS1 Melodic 和三台 AgileX LIMO，实现共享地图定位、动态槽位分配、三车编队、车间防碰撞、网页可视化，以及由 car1 执行路径规划、car2/car3 自适应跟随的导航方案。

> 当前推荐方案：只让 car1 运行 `move_base`。car2 和 car3 使用专用编队控制器跟随 car1，不分别运行 `move_base`。近距离编队中，这种方案比三车各自导航更稳定、响应更快。

## 1. 系统架构

```text
键盘 / 网页手动控制 / car1 move_base
                 |
                 v
       /robot1/leader_controller
          |                 |
          |                 +--> /robot1/cmd_vel --> car1 底盘
          v
   /robot1/leader_cmd
          |
          v
 formation_slot_planner
    |                 |
    v                 v
/robot2/assigned_goal  /robot3/assigned_goal
    |                 |
    v                 v
map_follower_robot2   map_follower_robot3
    |                 |
    v                 v
/robot2/cmd_vel_raw   /robot3/cmd_vel_raw
    |                 |
    v                 v
cmd_safety_filter    cmd_safety_filter
    |                 |
    v                 v
/robot2/cmd_vel       /robot3/cmd_vel
```

主要职责：

- car1：ROS Master、地图、三车 AMCL、主车控制器、编队规划、网页服务和可选导航。
- car2/car3：底盘、雷达和原始传感器数据发布。
- `formation_slot_planner`：根据当前队形和从车位置分配槽位，切换队形时避免两台从车互相穿越。
- `map_follower_controller`：根据共享地图中的目标槽位生成从车速度。
- `cmd_safety_filter`：结合雷达和三车 TF 做障碍物限速及车间防碰撞。

## 2. 固定网络配置

项目默认使用以下固定地址：

| 车辆 | 主机名 | IP | ROS 命名空间 |
| --- | --- | --- | --- |
| car1 | `car1` | `192.168.137.248` | `/robot1` |
| car2 | `car2` | `192.168.137.133` | `/robot2` |
| car3 | `car3` | `192.168.137.99` | `/robot3` |

ROS Master 固定为：

```bash
http://192.168.137.248:11311
```

任意车辆的新终端都先执行：

```bash
cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
```

脚本会根据本机 IP/主机名设置 `ROS_MASTER_URI`、`ROS_IP` 和 `ROBOT_NS`。如果本机没有真正持有预期 IP，脚本会直接报错，避免 ROS 节点注册成功但 TCPROS 数据无法连接。

启动前检查：

```bash
hostname -I
ping -c 3 192.168.137.248
ping -c 3 192.168.137.133
ping -c 3 192.168.137.99
timedatectl
```

三车系统时间应同步。时间差过大会出现 `TF_OLD_DATA`、TF extrapolation、AMCL 断树和控制卡顿。

## 3. 软件与地图准备

### 3.1 ROS 依赖

car1 需要 ROS 导航组件：

```bash
sudo apt-get update
sudo apt-get install ros-melodic-navigation \
  ros-melodic-move-base \
  ros-melodic-global-planner \
  ros-melodic-dwa-local-planner
```

三台车都需要原厂 `limo_base`、LD14 雷达驱动和本项目五个包：

```text
cluster_common
cluster_msgs
cluster_formation
cluster_following
cluster_bringup
```

### 3.2 共享地图

默认地图配置：

```text
~/agilex_ws/src/limo_ros/limo_bringup/maps/cluster_map.yaml
~/agilex_ws/src/limo_ros/limo_bringup/maps/cluster_map.pgm
```

`cluster_map.yaml` 中的 `image:` 必须指向实际存在的 PGM 文件。地图服务器只在 car1 上运行，因此地图文件至少必须存在于 car1。

### 3.3 编译

首次部署或 C++ 文件、消息、CMake 发生变化后，在每台包含相应源码的车辆上编译：

```bash
cd ~/agilex_ws
catkin_make -j2
source devel/setup.bash
```

也可以只编译本项目：

```bash
cd ~/agilex_ws
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_formation;cluster_following;cluster_bringup" \
  -j2
source devel/setup.bash
```

脚本权限：

```bash
chmod +x ~/agilex_ws/src/cluster_bringup/scripts/*.sh
chmod +x ~/agilex_ws/src/cluster_bringup/scripts/*.py
```

## 4. 固定起点与初始位姿

默认初始位姿位于：

```text
cluster_bringup/config/initial_poses.yaml
```

当前默认摆放为三角形，三车车头朝向相同：

```text
                 地图 +X / 车头方向
                         ^
                         |
                     car1 (0.0, 0.0, 0.0)

car3 (-0.8, +0.8, 0.0)       car2 (-0.8, -0.8, 0.0)
```

这里的坐标是地图坐标，不是“看起来差不多”的相对位置。建议在地面标记三台车的固定起点和统一朝向。每次启动前将车放回标记点，保持三台车车头方向一致。

如果实际起点改变，应修改 `initial_poses.yaml` 中的 `x/y/yaw`，否则网页位置、AMCL 位置和真实摆放会不一致。`yaw` 单位是弧度。

## 5. 完整启动流程

每条启动命令保持在独立终端中，不要关闭。

### 5.1 启动 car1 主系统

先启动 car1，因为 ROS Master 位于 car1：

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car1_master.sh /dev/ttyUSB0
```

该命令启动：

- ROS Master（本机没有运行 master 时由 `roslaunch` 自动启动）
- car1 底盘和雷达
- 三车 namespaced TF 转发
- `map_server`
- 三车 AMCL
- 固定初始位姿发布器
- car1 `leader_controller`

正常情况下不需要单独运行 `roscore`。如果手动运行 `roscore`，必须在 car1 上、使用相同的 `ROS_MASTER_URI`，并且只运行一个。

### 5.2 启动 car2

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car2_slave.sh /dev/ttyUSB0
```

### 5.3 启动 car3

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car3_slave.sh /dev/ttyUSB0
```

如果雷达不是 `/dev/ttyUSB0`，先检查：

```bash
ls -l /dev/ttyUSB*
dmesg | grep -i ttyUSB | tail -20
```

然后把正确端口作为脚本第一个参数传入。

### 5.4 系统自检

在 car1 新终端执行：

```bash
~/agilex_ws/src/cluster_bringup/scripts/check_cluster_system.sh
```

启动编队前必须确认：

- `/robot1/scan`、`/robot2/scan`、`/robot3/scan` 有数据。
- `/robot1/odom`、`/robot2/odom`、`/robot3/odom` 有数据。
- `map -> robotX/base_link` 三条 TF 连通且不是 NaN。
- 不存在 `FAIL`。

AMCL 是事件驱动的。车辆完全静止时，`rostopic hz /robotX/amcl_pose` 可能显示 `no new messages`，不能单独据此判断 AMCL 故障。应同时检查：

```bash
rostopic echo /robot1/amcl_pose -n 1 --noarr
rosrun tf tf_echo map robot1/base_link
```

### 5.5 启动稳定编队控制

在 car1 新终端执行：

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_map_follower.sh
```

默认启用：

- 动态槽位分配
- 队形切换防交叉路径
- car1、car2、car3 车间防碰撞
- 雷达障碍物安全过滤
- IMU 航向辅助

确认控制链路：

```bash
rostopic info /robot2/cmd_vel
rostopic info /robot3/cmd_vel
```

正常情况下 `/robot2/cmd_vel` 和 `/robot3/cmd_vel` 各自只有对应的 `cmd_safety_filter` 发布者。

## 6. 控制方式

键盘控制和网页控制二选一，不要同时启动。

### 6.1 键盘控制

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_keyboard.sh
```

| 按键 | 功能 |
| --- | --- |
| `w/s` | car1 前进/后退 |
| `a/d` | car1 左转/右转 |
| `Space` | 停止 |
| `z` | IDLE |
| `x` | TELEOP，仅控制 car1 |
| `c` | FORMATION，进入柱形编队 |
| `v` | FOLLOW |
| `1` | COLUMN：两台从车依次位于 car1 后方 |
| `2` | LINE：两台从车位于 car1 左右两侧 |
| `3` | CIRCLE_SHOW：三车闭环绕圈表演 |
| `4` | TRIANGLE：两台从车位于 car1 后方左右两侧 |
| `e` | 安全退出 CIRCLE_SHOW：car1 停车，从车恢复进入转圈前的普通队形 |
| `5` | 切换从车控制模式 |
| `6` | 开关避障安全层 |
| `0` | car1 返回本次启动时记录的 home pose |
| `q` / `Ctrl-C` | 退出 |

队形槽位使用 car1 车体坐标系：X 向前、Y 向左。默认队形间距为 `0.8 m`。CIRCLE_SHOW 的圆半径也是 `0.8 m`；进入后持续绕圈，按 `e` 才会安全退出。转圈和恢复期间 `w/a/s/d` 不会抢占 car1 的自动控制，`z` 仍可立即进入 IDLE。

### 6.2 网页控制与可视化

确保键盘节点没有运行，然后在 car1 启动：

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_web_visualizer.sh
```

同一局域网的电脑浏览器访问：

```text
http://192.168.137.248:8080
```

网页提供：

- 三车实时位置、朝向、目标槽位和轨迹。
- `/map` 栅格地图显示。
- 全局路径和局部路径显示。
- `Fit Map`、`Fit Robots`、鼠标滚轮缩放和拖动。
- IDLE、TELEOP、FORMATION、FOLLOW 和四种队形切换。
- WASD 手动控制 car1。
- car1 导航目标下发。

网页手动控制会主动取消 car1 当前导航目标。使用网页时不要同时运行 `start_keyboard.sh`。

## 7. car1 导航、从车编队跟随

推荐的导航结构是：

```text
car1 move_base -> /robot1/nav_vel -> leader_controller -> /robot1/cmd_vel
                                               |
                                               +-> 从车编队跟随
```

先完成第 5 节的三车启动、自检和 `start_map_follower.sh`，再在 car1 新终端启动：

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car1_nav.sh 3
```

参数 `3` 表示启动时保持 TRIANGLE 队形。可用值：

```text
0 COLUMN
1 LINE
2 CIRCLE_SHOW（不建议用于导航）
3 TRIANGLE
```

导航运行时不要启动键盘节点。随后启动网页，在网页中点击 `Set Goal`，再点击地图上的空闲区域给 car1 下发目标。

检查导航：

```bash
rostopic echo /robot1/move_base/status -n 1
rostopic hz /robot1/nav_vel
rostopic info /robot1/cmd_vel
```

`leader_controller` 必须仍是 `/robot1/cmd_vel` 的唯一发布者。`move_base` 输出应位于 `/robot1/nav_vel`。

导航点无法执行时：

```bash
rosrun cluster_bringup check_nav_map.py GOAL_X GOAL_Y
rosservice call /robot1/move_base/clear_costmaps "{}"
```

不要使用从车各自导航的旧入口：

```text
start_nav_follower.sh
```

该脚本默认拒绝启动。旧方案仅保留作对比实验，不用于当前近距离编队。

## 8. 常用诊断

### 8.1 雷达

```bash
rostopic hz /robot1/scan
rostopic hz /robot2/scan
rostopic hz /robot3/scan

rostopic info /robot1_raw_scan
rostopic info /robot2_raw_scan
rostopic info /robot3_raw_scan
```

原始雷达有数据但 `/robotX/scan` 无数据时，检查对应的 `scan_frame_republisher` 节点。

### 8.2 里程计与 IMU

```bash
rostopic hz /robot1/odom
rostopic hz /robot2/odom
rostopic hz /robot3/odom

rostopic hz /robot1/imu_fixed
rostopic hz /robot2/imu_fixed
rostopic hz /robot3/imu_fixed
```

### 8.3 定位与 TF

```bash
rosrun tf tf_echo map robot1/base_link
rosrun tf tf_echo map robot2/base_link
rosrun tf tf_echo map robot3/base_link

rosrun tf tf_echo robot1/odom robot1/base_link
rosrun tf tf_echo robot2/odom robot2/base_link
rosrun tf tf_echo robot3/odom robot3/base_link
```

### 8.4 编队控制

```bash
rostopic hz /robot2/assigned_goal
rostopic hz /robot3/assigned_goal
rostopic echo /robot2/follower_status -n 3
rostopic echo /robot3/follower_status -n 3
rostopic hz /robot2/cmd_vel_raw
rostopic hz /robot3/cmd_vel_raw
rostopic hz /robot2/cmd_vel
rostopic hz /robot3/cmd_vel
```

## 9. 常见故障

### `Unable to communicate with master`

1. 先在 car1 启动 `start_car1_master.sh`。
2. 检查三车是否能 ping 通 `192.168.137.248`。
3. 重新 `source setup_network.sh`。

### `Unable to contact my own server at http://错误IP:端口`

本机 `ROS_IP` 设置成了别的车辆 IP。不要手工复制旧终端变量，重新执行：

```bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
echo $ROS_IP
hostname -I
```

两者必须对应。

### AMCL 为 NaN 或 `map -> base_link` 断开

不要继续启动编队。依次检查该车 `/scan`、`/odom`、`odom -> base_link`，确认初始位姿与真实摆放一致，然后重启 car1 主系统以重新建立 AMCL。

### `TF_OLD_DATA` 或 extrapolation

同步三台车系统时间，确认没有重复的旧节点向 `/tf` 发布数据：

```bash
sudo timedatectl set-ntp true
rosnode list
rostopic info /tf
```

### 从车不动

按顺序检查：

```bash
rostopic hz /robotX/assigned_goal
rostopic hz /robotX/cmd_vel_raw
rostopic hz /robotX/cmd_vel
rostopic info /robotX/cmd_vel
```

- 没有 `assigned_goal`：检查 `formation_slot_planner` 和三车 TF。
- 有目标但没有 `cmd_vel_raw`：检查 `map_follower_robotX`。
- 有 `cmd_vel_raw` 但最终速度为零：查看安全层终端警告和雷达障碍距离。
- `/cmd_vel` 有多个发布者：停止重复控制节点。

### 网页位置与实车不一致

网页显示的是 `map -> robotX/base_link`，不是摄像头画面推断的位置。检查固定起点、统一朝向、`initial_poses.yaml` 和地图原点。不要通过前端平移图标来掩盖错误定位。

## 10. 关键文件

```text
cluster_bringup/config/initial_poses.yaml
  固定起点和 AMCL 初始位姿

cluster_bringup/config/formation_slot_planner_params.yaml
  队形间距、动态槽位分配、防交叉路径和中间航点

cluster_bringup/config/map_follower_params.yaml
  从车跟随控制器增益、限速、航向控制和 IMU 辅助

cluster_bringup/config/cmd_safety_filter_params.yaml
  雷达避障和车间防碰撞参数

cluster_bringup/config/nav/
  car1 move_base、costmap 和 DWA 配置

cluster_bringup/launch/car1_master.launch
  car1 主系统、共享地图和三车 AMCL

cluster_bringup/launch/map_follower.launch
  car2/car3 稳定编队控制链路

cluster_bringup/launch/car1_nav.launch
  car1-only move_base

cluster_bringup/scripts/web_visualizer_server.py
  网页地图、状态、控制和导航目标接口

cluster_following/src/formation_slot_planner_node.cpp
  动态编队槽位分配和切换路径规划

cluster_following/src/map_follower_controller_node.cpp
  从车闭环运动控制

cluster_following/src/cmd_safety_filter_node.cpp
  障碍物和车辆间安全过滤

cluster_formation/src/leader_controller.cpp
  car1 模式、手动速度、导航速度和编队命令管理
```

## 11. 安全建议

- 首次测试使用空旷区域和低速度，保证随时可急停。
- 启动编队前必须完成三车 TF/定位自检。
- 不要同时运行键盘控制与网页控制。
- 不要同时运行稳定编队和旧的从车 `move_base`。
- 不要通过减小车间安全距离来掩盖定位或目标分配错误。
- 发生 NaN、TF 断树、重复速度发布者或雷达断流时立即停止车辆。
