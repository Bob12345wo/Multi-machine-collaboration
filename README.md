# 集群小车双车协同项目

本仓库是基于 ROS1 Melodic 的 AgileX LIMO S2 双车协同项目，当前重点是两车在同一张地图下进行编队跟随、队形切换、安全限速和调试分析。

## 当前方案

当前定位和跟随方案如下：

```text
car1 / car2 各自底盘里程计 + 各自 LD14 雷达
        ↓
/robot1/odom、/robot2/odom
/robot1/scan、/robot2/scan
        ↓
car1 上统一启动 map_server + 双 AMCL
        ↓
map -> robot1/odom -> robot1/base_link
map -> robot2/odom -> robot2/base_link
        ↓
map_follower_controller 在同一个 map 坐标系里计算 car2 目标点
        ↓
cmd_safety_filter_node 做雷达避障和两车 TF 防撞限速
```

也就是说，两车不是各自用局部坐标系跟随，而是共用 `cluster_map.yaml`，通过 AMCL 定位到同一个 `map` 坐标系。

## 目录结构

```text
catkin_ws/src/cluster_bringup     启动文件、脚本、参数配置
catkin_ws/src/cluster_common      通用姿态、限幅、PID 工具
catkin_ws/src/cluster_formation   car1 主车控制、队形命令、键盘模式
catkin_ws/src/cluster_following   car2 跟随控制、安全过滤、调试输出
catkin_ws/src/cluster_msgs        自定义消息
limo_ros_ref                      LIMO 原始参考文件
ros1_melodic/wheeltec_multi       参考用 ROS1 多机编队功能包
```

## 启动流程

### 1. car1 启动 roscore

```bash
cd ~/agilex_ws
source devel/setup.bash
source ~/agilex_ws/src/cluster_bringup/scripts/setup_network.sh
roscore
```

### 2. car2 启动从车

car2 雷达端口目前常用 `/dev/ttyUSB0`，如果没有雷达数据，换成 `/dev/ttyUSB1` 测试。

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car2_slave.sh /dev/ttyUSB0
```

### 3. car1 启动主车、地图和 AMCL

car1 雷达端口根据实际情况选择 `/dev/ttyUSB0` 或 `/dev/ttyUSB1`。

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_car1_master.sh /dev/ttyUSB0
```

### 4. 检查定位

在 car1 上检查：

```bash
rostopic hz /robot1/scan
rostopic hz /robot2/scan
rostopic echo /robot1/amcl_pose -n 1 --noarr
rostopic echo /robot2/amcl_pose -n 1 --noarr
rosrun tf tf_echo map robot1/base_link
rosrun tf tf_echo map robot2/base_link
```

两个 `tf_echo` 都能持续输出，说明两车已经在同一个地图坐标系里。

### 5. 启动跟随控制

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_map_follower.sh
```

当前 `start_map_follower.launch` 默认会启动安全过滤器：

```text
/map_follower_controller
/robot2_cmd_safety_filter
```

也就是跟随控制器先输出 `/robot2/cmd_vel_raw`，再经过安全层输出 `/robot2/cmd_vel` 给底盘。

### 6. 启动键盘控制

```bash
~/agilex_ws/src/cluster_bringup/scripts/start_keyboard.sh
```

## 键盘功能

```text
c      进入 FORMATION 编队模式
1      COLUMN：car2 在 car1 后方
2      LINE：car2 在 car1 右侧
3      TRIANGLE_LEFT
4      TRIANGLE_RIGHT
5      切换 car2 跟随控制模式 body_orbit / wheeltec_global
6      开关 car2 避障安全层
0      car1 返回启动时记录的初始点
w/s    car1 前进 / 后退
a/d    car1 左转 / 右转
space  停车
```

## 当前安全和调试功能

### 雷达安全过滤

`cmd_safety_filter_node` 订阅 `/robot2/scan`，对 `/robot2/cmd_vel_raw` 做前方雷达限速，最终输出 `/robot2/cmd_vel`。

参数文件：

```text
catkin_ws/src/cluster_bringup/config/cmd_safety_filter_params.yaml
```

### 两车 TF 防撞

安全层还会通过 TF 查询：

```text
robot2/base_link -> robot1/base_link
```

当 car2 继续朝 car1 靠近，并且两车距离进入保护范围时，安全层会限制 car2 的线速度。

当前默认参数：

```yaml
robot_safe_distance: 0.55
robot_danger_distance: 0.50
robot_turn_gain: 0.0
```

这层保护不依赖雷达，所以即使两车雷达处于同一平面、互相不容易识别，也能用地图 TF 做最后防撞。

### 静态切换队形绕行

当 car1 静止、car2 切换队形时，如果 car2 到目标点的直线路径会穿过 car1 附近，`map_follower_controller` 会先生成一个绕行点，避免直接冲向 car1。

参数文件：

```text
catkin_ws/src/cluster_bringup/config/map_follower_params.yaml
```

当前绕行参数：

```yaml
path_keepout_enabled: true
path_keepout_radius: 0.70
path_detour_radius: 0.85
path_detour_step: 0.55
path_detour_goal_tolerance: 0.18
```

如果日志里出现下面内容，说明绕行逻辑生效：

```text
Path keepout detour active
detour=1
```

### 调试日志

当前默认打开调试输出，方便分析碰撞和队形切换问题。

跟随控制器会打印：

```text
[FOLLOW_DBG]
```

内容包括 car1 位姿、car2 位姿、目标点、误差、原始控制速度和是否启用绕行点。

安全过滤器会打印：

```text
[SAFETY_DBG]
```

内容包括雷达是否检测到障碍物、两车 TF 距离、是否正在靠近 car1、安全层是否介入、最终输出速度。

调试完成后可以在两个参数文件里关闭：

```yaml
debug_enabled: false
```

## 地图文件

双车共用地图应放在 car1：

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

如果只改了跟随和安全层，可以编译：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="cluster_common;cluster_msgs;cluster_following" -j2
```

## 常见问题

- `/robot1/scan` 或 `/robot2/scan` 没频率：检查雷达端口 `/dev/ttyUSB0` / `/dev/ttyUSB1`。
- `map -> robotX/base_link` 不存在：检查 AMCL、map_server、雷达话题和 odom TF。
- car2 被安全层卡住：查看 `[SAFETY_DBG]` 里的 `robot(dist,front,lat,toward,applied)`。
- 切换队形时 car2 直冲 car1：查看 `[FOLLOW_DBG]` 里是否出现 `detour=1`。
