# 集群小车多车协同项目

本项目基于 ROS1 Melodic 和 AgileX LIMO S2，实现 car1、car2、car3 在同一张地图下的多车定位、编队跟随、队形切换、安全限速和调试输出。

当前方案不是各车使用各自局部坐标系协同，而是三台车共用 `cluster_map.yaml`，通过 AMCL 定位到同一个 `map` 坐标系：

```text
robotX/odom + robotX/scan
  -> car1 上统一 map_server + multi_amcl
  -> map -> robotX/odom -> robotX/base_link
  -> map_follower_robot2 / map_follower_robot3 计算各自目标点
  -> cmd_safety_filter_node 输出到底盘 cmd_vel
```

## 目录结构

```text
catkin_ws/src/cluster_bringup     启动文件、脚本、参数配置
catkin_ws/src/cluster_common      通用位姿、限幅、PID 工具
catkin_ws/src/cluster_formation   car1 主车控制、队形命令、键盘模式
catkin_ws/src/cluster_following   从车跟随控制、安全过滤、调试输出
catkin_ws/src/cluster_msgs        自定义消息
limo_ros_ref                      LIMO 原始参考文件
ros1_melodic/wheeltec_multi       参考用 ROS1 多机编队功能包
```

## 固定 IP

```text
car1  192.168.137.248
car2  192.168.137.133
car3  192.168.137.99
```

如果 IP 变化，优先在系统网络里固定回上面的地址，减少 ROS 配置修改。

## 启动流程

### 1. car1 启动 roscore

```bash
cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
roscore
```

### 2. car2 启动从车

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car2_slave.sh /dev/ttyUSB0
```

如果 `/robot2/scan` 没频率，把雷达端口换成 `/dev/ttyUSB1` 测试。

### 3. car3 启动从车

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car3_slave.sh /dev/ttyUSB0
```

car3 当前使用原厂底盘节点发布全局 `/odom`、`/limo_status`，再 relay 到 `/robot3/odom`、`/robot3/limo_status`；同时 relay `/robot3/cmd_vel -> /cmd_vel` 让原厂底盘接收控制指令。因此 car3 上不要再额外启动其它全局 `/cmd_vel` 控制器。

### 4. car1 启动主车、地图和三车 AMCL

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car1_master.sh /dev/ttyUSB0
```

等价 launch：

```bash
roslaunch cluster_bringup car1_master.launch lidar_port:=/dev/ttyUSB0
```

该启动会默认启动 robot1、robot2、robot3 的 odom TF republisher 和 AMCL，car3 已经是默认必选节点。

### 5. 检查定位

在 car1 上检查：

```bash
rostopic hz /robot1/scan
rostopic hz /robot2/scan
rostopic hz /robot3/scan
rostopic echo /robot1/amcl_pose -n 1 --noarr
rostopic echo /robot2/amcl_pose -n 1 --noarr
rostopic echo /robot3/amcl_pose -n 1 --noarr
rosrun tf tf_echo map robot1/base_link
rosrun tf tf_echo map robot2/base_link
rosrun tf tf_echo map robot3/base_link
```

三台车都能持续输出 `map -> robotX/base_link`，才说明它们在同一张地图里。

### 6. 启动三车编队跟随

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_map_follower.sh
```

默认会启动：

```text
map_follower_robot2
map_follower_robot3
robot2_cmd_safety_filter
robot3_cmd_safety_filter
```

robot2 输出 `/robot2/cmd_vel_raw`，经过安全层变为 `/robot2/cmd_vel`；robot3 输出 `/robot3/cmd_vel_raw`，经过安全层变为 `/robot3/cmd_vel`。

### 7. 启动键盘控制

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_keyboard.sh
```

## 键盘功能

```text
c      进入 FORMATION 编队模式
1      COLUMN：robot2 在 car1 后方 0.8m，robot3 在 car1 后方 1.6m
2      LINE：robot2 在 car1 右侧 0.8m，robot3 在 car1 左侧 0.8m
3      CIRCLE_SHOW：三车闭环绕圈表演
4      TRIANGLE：robot2 后右，robot3 后左
5      切换跟随控制模式 body_orbit / wheeltec_global
6      开关避障安全层
0      car1 返回启动时记录的初始点
w/s    car1 前进 / 后退
a/d    car1 左转 / 右转
space  停车
```

`CIRCLE_SHOW` 会让 car1 自动低速画圆，robot2 和 robot3 分别跟随圆周上相隔 120 度的目标点。默认圆半径 `0.5m`，car1 线速度约 `0.16m/s`。退出表演请按 `z` 切回 IDLE，或按 `1/2/4` 切到其它队形。

## 安全与调试

雷达安全层订阅 `/robot2/scan`、`/robot3/scan`，分别过滤 `/robot2/cmd_vel_raw`、`/robot3/cmd_vel_raw`。

TF 防撞层会查询从车和 `robot1/base_link` 的相对距离，作为雷达识别不到主车时的最后保护。同时 robot2 与 robot3 也会互相查询：

```text
robot2 safety: robot2/base_link -> robot3/base_link
robot3 safety: robot3/base_link -> robot2/base_link
```

默认从车互防撞距离：

```yaml
peer_safe_distance: 0.75
peer_danger_distance: 0.60
```

这样低于 0.75m 开始限速，低于 0.60m 且继续朝对方靠近时停车；如果正在远离对方，不会硬拦，避免卡死。

调试日志：

```text
[FOLLOW_DBG]  跟随控制：leader/follower 位姿、目标点、误差、原始速度、绕行状态
[SAFETY_DBG]  安全过滤：雷达障碍、leader/peer TF 距离、是否介入、最终速度
```

调试完成后可在参数文件里关闭：

```yaml
debug_enabled: false
```

## 地图文件

三车共用地图放在 car1：

```text
~/agilex_ws/src/limo_ros/limo_bringup/maps/cluster_map.yaml
~/agilex_ws/src/limo_ros/limo_bringup/maps/cluster_map.pgm
```

`multi_amcl.launch` 默认加载 `cluster_map.yaml`。

## 编译

在小车上编译：

```bash
cd ~/agilex_ws
source devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_formation;cluster_following;cluster_bringup" -j2
source devel/setup.bash
```

如果只改了跟随控制、安全层和主车控制：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_formation;cluster_following" -j2
```

## 常见问题

- `/robotX/scan` 没频率：检查雷达端口 `/dev/ttyUSB0` / `/dev/ttyUSB1`，以及是否有同名雷达节点抢占。
- `map -> robotX/base_link` 不存在：检查 `/robotX/odom`、`/robotX/scan`、`namespaced_tf.launch`、`multi_amcl.launch`。
- car3 `/robot3/odom` 没发布：先确认原厂 `/odom` 是否发布，再检查 `robot3_odom_relay`。
- 从车被安全层卡住：看 `[SAFETY_DBG]` 的 `leader(...)` 和 `peer(...)` 字段。
- 切换队形时从车直冲 car1：看 `[FOLLOW_DBG]` 是否出现 `detour=1`。
