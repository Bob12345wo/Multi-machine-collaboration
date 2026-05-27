# LIMO S2 双车集群控制 — 操作手册

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件准备](#2-硬件准备)
3. [网络配置](#3-网络配置)
4. [代码部署与编译](#4-代码部署与编译)
5. [仿真测试（无需真车）](#5-仿真测试无需真车)
6. [单台真车测试](#6-单台真车测试)
7. [双车联网测试](#7-双车联网测试)
8. [双车编队行驶](#8-双车编队行驶)
9. [双车跟随行驶](#9-双车跟随行驶)
10. [键盘遥控操作说明](#10-键盘遥控操作说明)
11. [参数在线调优](#11-参数在线调优)
12. [常见问题排查](#12-常见问题排查)
13. [附录：文件清单](#13-附录：文件清单)

---

## 1. 系统概述

### 1.1 什么是集群控制

本项目实现两台 LIMO S2 ROS 小车的协同控制，包含两个核心功能：

| 功能 | 说明 |
|------|------|
| **编队行驶** | 两台车保持固定相对位置（纵列、并排、三角形）一起移动 |
| **跟随行驶** | 一台车领航，另一台车沿领航者的路径延时跟随 |

### 1.2 系统架构

```
┌─────────────────────────────────────┐
│  Car1（领航者 / Leader）              │
│  主机名: limo1                        │
│  IP: 192.168.1.101                   │
│  ┌─────────────────────────────┐    │
│  │ roscore（ROS Master）        │    │
│  │ limo_base 底盘驱动            │    │
│  │ leader_controller 领航控制器  │    │
│  └─────────────────────────────┘    │
└──────────┬──────────────────────────┘
           │  WiFi 局域网
           │  ROS_MASTER_URI
           │
┌──────────▼──────────────────────────┐
│  Car2（跟随者 / Follower）            │
│  主机名: limo2                        │
│  IP: 192.168.1.102                   │
│  ┌─────────────────────────────┐    │
│  │ limo_base 底盘驱动            │    │
│  │ follower_controller 跟随控制器│    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

**关键设计：**
- Car1 运行 `roscore` 作为整个系统的 ROS Master
- Car2 所有节点连接到 Car1 的 Master
- 两台车的 ROS 话题通过命名空间隔离：`/robot1/` 和 `/robot2/`
- 领航者主动计算跟随者的目标位置并下发指令
- 跟随者用 PID 控制器闭环跟踪目标

### 1.3 四种编队队形

| 编号 | 队形 | 示意图（L=领航，F=跟随） | offset_x | offset_y |
|------|------|--------------------------|----------|----------|
| 0 | **纵列** | F在L正后方0.8m | -0.8m | 0.0 |
| 1 | **并排** | F在L左侧0.8m | 0.0 | -0.8m |
| 2 | **左三角** | F在L左后方 | -0.8m | -0.8m |
| 3 | **右三角** | F在L右后方 | -0.8m | +0.8m |

> 偏移量在领航者车体坐标系下定义：**X轴=前进方向，Y轴=左方**，可以随时通过 service 调用或参数修改。

### 1.4 四种工作模式

| 模式 | 编号 | 说明 |
|------|------|------|
| **IDLE** | 0 | 空闲，两车静止 |
| **TELEOP** | 1 | 遥控模式，仅控制领航者 |
| **FORMATION** | 2 | 编队模式，领航者移动时跟随者保持编队偏移 |
| **FOLLOW** | 3 | 跟随模式，跟随者延时回放领航者的行驶路径 |

---

## 2. 硬件准备

### 2.1 所需设备

| 设备 | 数量 | 说明 |
|------|------|------|
| LIMO S2 小车 | 2台 | 均已刷好系统，能正常运行 ROS |
| 无线路由器 | 1台 | 创建局域网，两台车连入同一个 WiFi |
| 笔记本电脑（可选） | 1台 | 用于 SSH 登录和监控 |
| 网线（可选） | — | 初次配置时可能用到显示器+键盘 |

### 2.2 确认每台车的基本状态

在每台 LIMO S2 上分别执行以下检查：

```bash
# 1. 确认系统版本
lsb_release -a
# 应输出: Ubuntu 18.04

# 2. 确认 ROS 已安装且环境正常
roscore
# 按 Ctrl+C 停止。应能正常启动，无报错。

# 3. 确认底盘驱动可用
roslaunch limo_base limo_base.launch
# 按 Ctrl+C 停止。车轮应有短暂响应。

# 4. 查看当前主机名
hostname
# 记录两台车的主机名，后续配置会用。
# LIMO 默认主机名通常为 rostest24 或类似名称。
```

---

## 3. 网络配置

这是**最关键的一步**。两台车必须在同一局域网内，能够互相通信。

### 3.1 连接 WiFi

在每台车的 Jetson Nano 上连接同一个 WiFi 路由器：

```bash
# 图形界面方式（推荐）
# 点击桌面右上角网络图标 → 选择 WiFi → 输入密码

# 或者命令行方式
nmcli device wifi list                    # 扫描WiFi
nmcli device wifi connect "WiFi名称" password "密码"
```

### 3.2 设置静态 IP

为了让每次开机后 IP 不变，建议在路由器上绑定 MAC 地址和 IP，或在 Jetson Nano 上设置静态 IP。

**方法一：路由器 DHCP 静态绑定（推荐）**
登录路由器管理页面，找到 DHCP 静态分配功能，将两台车的 MAC 地址分别绑定到：
- Car1（领航者）：`192.168.137.248`
- Car2（跟随者）：`192.168.1.102`

**方法二：Ubuntu 系统设置静态 IP**
```bash
# 编辑网络配置文件
sudo nano /etc/netplan/01-netcfg.yaml

# Car1 的内容示例：
# network:
#   version: 2
#   renderer: networkd
#   wifis:
#     wlan0:
#       dhcp4: no
#       addresses: [192.168.1.101/24]
#       gateway4: 192.168.1.1
#       nameservers:
#         addresses: [8.8.8.8, 114.114.114.114]
#       access-points:
#         "WiFi名称":
#           password: "WiFi密码"

sudo netplan apply
```

### 3.3 配置 hosts 文件（可选但推荐）

在两台车上都编辑 `/etc/hosts`，添加对方的映射：

```bash
sudo nano /etc/hosts
# 在文件末尾添加：
192.168.1.101   car1
192.168.1.102   car2
```

### 3.4 验证网络连通性

在 Car1 上：
```bash
ping 192.168.1.102
# 应该能 ping 通，延迟 < 10ms
```

在 Car2 上：
```bash
ping 192.168.1.101
# 应该能 ping 通
```

### 3.5 配置 SSH 免密登录

在两台车上分别执行：
```bash
# 生成 SSH 密钥（如果还没有的话）
ssh-keygen -t rsa -b 4096 -N "" -f ~/.ssh/id_rsa

# Car1 上：将公钥复制到 Car2
ssh-copy-id agilex@192.168.1.102
# 输入 Car2 的密码（默认: agx）

# Car2 上：将公钥复制到 Car1
ssh-copy-id agilex@192.168.1.101
# 输入 Car1 的密码（默认: agx）
```

验证免密登录：
```bash
# 在 Car1 上，应无需输入密码即可登录 Car2
ssh agilex@192.168.1.102

# 在 Car2 上，同样应免密登录 Car1
ssh agilex@192.168.1.101
```

### 3.6 校准系统时间

ROS 依赖时间戳进行消息同步，两台车的时间必须一致：

```bash
# 在两台车上分别安装 ntpdate
sudo apt-get install -y ntpdate

# 同步时间（需要互联网连接）
sudo ntpdate -u ntp.ubuntu.com

# 或者手动设置（如果无法联网）
# sudo date -s "2026-05-10 15:30:00"
```

---

## 4. 代码部署与编译

### 4.1 将代码复制到 Jetson Nano

将你电脑上 `catkin_ws` 整个文件夹复制到两台车的 Jetson Nano 上。

**方法一：U盘拷贝**
将 `catkin_ws` 文件夹复制到 U盘，再插到 Jetson Nano 上复制到 `~/catkin_ws`。

**方法二：SCP 传输（推荐）**
在你的 Windows 电脑上打开 PowerShell 或 CMD：
```cmd
# 传输到 Car1
scp -r catkin_ws agilex@192.168.1.101:/home/agilex/

# 传输到 Car2
scp -r catkin_ws agilex@192.168.1.102:/home/agilex/
```

### 4.2 编译项目

在**两台车**上都执行以下步骤：

```bash
# 进入工作空间
cd ~/catkin_ws

# 编译所有包
catkin_make

# 如果编译成功，会看到类似输出：
# [100%] Built target leader_controller_node
# [100%] Built target follower_controller_node
```

> **若编译报错**，常见问题：
> - `找不到 cluster_msgs`：先单独编译 cluster_msgs：`catkin_make --pkg cluster_msgs`，再完整编译
> - `找不到 dynamic_reconfigure`：`sudo apt-get install ros-melodic-dynamic-reconfigure`
> - 如果 catkin_ws 是从 Windows 复制过来的，可能需要修复文件权限：`chmod +x src/cluster_bringup/scripts/*.py src/cluster_bringup/scripts/*.sh`

### 4.3 设置环境变量

在两台车上，将工作空间添加到 `~/.bashrc`：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

验证环境：
```bash
echo $ROS_PACKAGE_PATH
# 应该包含 /home/agilex/catkin_ws/src
```

### 4.4 修改网络配置脚本

在两台车上分别编辑 `setup_network.sh`，确保 hostname 匹配：

```bash
nano ~/catkin_ws/src/cluster_bringup/scripts/setup_network.sh
```

找到这一行：
```bash
if [ "$HOSTNAME" = "limo1" ] || [ "$HOSTNAME" = "rostest24" ] || [ "$HOSTNAME" = "car1" ]; then
```

将 `rostest24` 替换为你的 Car1 实际主机名。同理修改 Car2 的检测条件。

---

## 5. 仿真测试（无需真车）

在将代码部署到真车之前，建议先在单机上进行仿真测试，验证控制逻辑是否正确。

### 5.1 启动仿真

```bash
# 确保已 source 工作空间
source ~/catkin_ws/devel/setup.bash

# 先启动 roscore（如果还没启动）
roscore &
sleep 2

# 启动仿真环境
roslaunch cluster_bringup sim_test.launch
```

此时系统启动了：
- **fake_odom_robot1**：模拟领航者里程计，初始位置 (0, 0)
- **fake_odom_robot2**：模拟跟随者里程计，初始位置 (-0.8, 0)
- **leader_controller**：领航控制器
- **follower_controller**：跟随控制器
- **teleop_keyboard**：键盘遥控

### 5.2 仿真测试步骤

打开新终端，用 `rqt_graph` 查看节点和话题连接：
```bash
rqt_graph
```

打开新终端，用 `rviz` 可视化两车位置：
```bash
rviz
# 在 rviz 中：
# 1. Fixed Frame 设为 "robot1/odom"
# 2. 添加两个 Odometry 显示，分别订阅 /robot1/odom 和 /robot2/odom
```

在 teleop_keyboard 终端中按键操作：
1. 按 `x` 进入 TELEOP 模式
2. 按 `w` 让领航者前进
3. 你会看到 /robot1/odom 的位姿在变化
4. 按 `c` 进入 FORMATION 模式
5. 再按 `w` 前进，观察 follower 是否跟踪

用 `rqt_plot` 查看跟踪误差：
```bash
rqt_plot /robot2/follower_status/error_dist
```

### 5.3 预期行为

| 操作 | 预期结果 |
|------|----------|
| 按 `x` 后按 `w` | 只有 robot1 前进，robot2 不动 |
| 按 `c` 后按 `w` | robot1 前进，robot2 自动跟上，保持在 robot1 后方 0.8m |
| 按 `v` 后按 `w` | robot1 前进，robot2 延时 2 秒后沿相同路径跟随 |
| 按 `z` | 全部停止 |

---

## 6. 单台真车测试

只用一台真车，同时运行 leader 和 follower 控制器，验证控制逻辑与真车底盘的交互。

### 6.1 启动步骤

```bash
# 在 Car1 上执行
source setup_network.sh   # 设置环境变量
roscore &
sleep 2
roslaunch cluster_bringup car1_bringup.launch
```

> 这时你只有 Car1 在运行。如果你想同时测试 follower 逻辑：
> 在另一个终端启动 follower_controller（它会用同一台车的 odom 作为 leader 和 follower 的输入）

### 6.2 虚拟 Follower 测试

新建终端：
```bash
roslaunch cluster_bringup car2_bringup.launch
```

然后在 teleop 终端：
1. 按 `x` → 进入遥操模式，手动开车验证底盘响应
2. 按 `c` → 进入编队模式，手动开车，观察 `/robot2/cmd_vel` 是否在自动输出速度指令
3. 用 `rostopic echo /robot2/cmd_vel` 查看

> **注意**：此时 Car2 的 limo_base 也在本机运行，如果 Car2 的 cmd_vel 有非零值，它会尝试控制同一台车的底盘。在实际测试中，你可以不启动第二个 limo_base，只启动 follower_controller 节点来观察输出。

### 6.3 安全检查

真车测试前，务必确认：
- [ ] 小车放在地上，周围有足够空间（至少 3m × 3m）
- [ ] 紧急停止方式已知：按 `Ctrl+C` 或 APP 急停
- [ ] 速度限制参数设为保守值（默认 0.3m/s）
- [ ] 有人随时准备抓住小车

---

## 7. 双车联网测试

### 7.1 启动流程

**步骤 1：Car1（领航者）启动 roscore**

```bash
# SSH 登录 Car1
ssh agilex@192.168.1.101

# 设置网络
source ~/catkin_ws/src/cluster_bringup/scripts/setup_network.sh

# 确保之前的 roscore 已关闭
pkill roscore
sleep 1

# 启动 roscore
roscore &
sleep 3
```

**步骤 2：Car2（跟随者）连接 Master**

```bash
# SSH 登录 Car2
ssh agilex@192.168.1.102

# 设置网络（这会把 ROS_MASTER_URI 指向 Car1）
source ~/catkin_ws/src/cluster_bringup/scripts/setup_network.sh

# 验证连接
rostopic list
# 应该能看到 /rosout 等话题，说明已连接到 Car1 的 Master
```

**步骤 3：Car1 启动底盘和控制器**

```bash
# 在 Car1 的终端中
roslaunch cluster_bringup car1_bringup.launch mode:=teleop
```

**步骤 4：Car2 启动底盘和控制器**

```

### 7.2 验证双车联通

在 Car1 上：
```bash
# 查看所有话题
rostopic list

# 应该看到以下话题：
# /robot1/odom
# /robot1/cmd_vel
# /robot1/leader_cmd
# /robot1/teleop_vel
# /robot2/odom
# /robot2/cmd_vel
# /robot2/follower_status

# 查看 Car2 的 odom 是否有数据
rostopic echo /robot2/odom -n 1
```

如果 `/robot2/odom` 有数据，说明双车联通成功。

---

## 8. 双车编队行驶

### 8.1 准备工作

1. 将两台车并排放置，Car2 在 Car1 后方约 0.8m
2. 确保前方有至少 5m 的直线行驶空间
3. 两台车均已按照[第7章](#7-双车联网测试)启动并验证联通

### 8.2 启动编队

在 Car1 的终端中，使用键盘遥控切换模式：

```
==================================================
  LIMO S2 Cluster Control - Keyboard Teleop
==================================================
  Driving:
    w/s  : forward / backward
    a/d  : turn left / right
  Modes:
    z    : IDLE
    x    : TELEOP
    c    : FORMATION (COLUMN)
    v    : FOLLOW
  Formations:
    1    : COLUMN
    2    : LINE
    3    : TRIANGLE_LEFT
    4    : TRIANGLE_RIGHT
  q / CTRL-C : quit
==================================================
```

1. 按 `c` 进入编队模式（默认纵队）
2. 按 `w` 缓慢前进，观察 Car2 是否自动跟上
3. 按 `s` 后退，观察 Car2 是否后退保持距离
4. 按 `a` / `d` 转弯，观察 Car2 是否绕弯保持队形

### 8.3 切换编队队形

在 FORMATION 模式下，按数字键切换队形：

| 按键 | 队形 |
|------|------|
| `1` | 纵列（跟随者在领航者正后方） |
| `2` | 并排（跟随者在领航者左侧） |
| `3` | 左三角（跟随者在领航者左后方） |
| `4` | 右三角（跟随者在领航者右后方） |

### 8.4 命令行切换模式

也可以通过 ROS 服务直接切换，不需要键盘：

```bash
# 切换到编队模式（纵列）
rosservice call /robot1/set_mode "mode: 2
formation: 0
offset_x: 0.0
offset_y: 0.0
offset_yaw: 0.0"

# 切换到编队模式（左三角，自定义偏移 -1.0m 后方，-0.5m 左侧）
rosservice call /robot1/set_mode "mode: 2
formation: 2
offset_x: -1.0
offset_y: -0.5
offset_yaw: 0.0"

# 切换回 IDLE
rosservice call /robot1/set_mode "mode: 0
formation: 0
offset_x: 0.0
offset_y: 0.0
offset_yaw: 0.0"

# 在编队模式下单独切换队形
rosservice call /robot1/set_formation "formation: 1"   # 并排
```

### 8.5 监控编队状态

在新终端中实时查看跟随者的跟踪状态：

```bash
# 查看跟踪误差
rostopic echo /robot2/follower_status

# 关键字段：
# error_dist: 距离误差（理想为 0）
# error_yaw:  角度误差（理想为 0）
# leader_visible: 是否正常接收领航者数据
```

---

## 9. 双车跟随行驶

### 9.1 与编队模式的区别

| | 编队模式 | 跟随模式 |
|------|----------|----------|
| **跟踪目标** | 固定相对位置（实时计算） | 领航者历史路径（延时回放） |
| **Follower 行为** | 保持 (dx, dy) 偏移 | 沿领航者的轨迹行驶 |
| **适用场景** | 队形保持、协同移动 | 路径跟随、循迹导航 |
| **延时** | 无延时、实时跟踪 | 默认 2 秒延时 |

### 9.2 启动跟随模式

1. 先按 `x` 进入 TELEOP 模式
2. 手动驾驶领航者沿一个路径行驶（如走一个 S 形）
3. 将跟随者停在领航者的出发点
4. 按 `v` 进入 FOLLOW 模式
5. 再按 `w` 让领航者沿路径前进，观察跟随者是否 2 秒后经过相同路径

### 9.3 调整跟随延时

默认延时为 2 秒，可以通过 dynamic_reconfigure 调整：

```bash
rosrun rqt_reconfigure rqt_reconfigure
# 在界面中找到 follower_controller → follow_delay_seconds
# 调大：跟随距离更远，更安全
# 调小：跟随更紧密
```

---

## 10. 键盘遥控操作说明

### 10.1 启动键盘遥控

键盘遥控随 `sim_test.launch` 和 `car1_bringup.launch` 自动启动。
也可以单独启动：

```bash
rosrun cluster_bringup teleop_keyboard.py
```

### 10.2 按键速查表

| 按键 | 功能 | 适用模式 |
|------|------|----------|
| `w` | 前进 | TELEOP / FORMATION / FOLLOW |
| `s` | 后退 | TELEOP / FORMATION / FOLLOW |
| `a` | 左转（逆时针） | TELEOP / FORMATION / FOLLOW |
| `d` | 右转（顺时针） | TELEOP / FORMATION / FOLLOW |
| `空格` | 停止（松手即停） | 所有模式 |
| `z` | 切换 IDLE 模式 | 所有模式 |
| `x` | 切换 TELEOP 模式 | 所有模式 |
| `c` | 切换 FORMATION 模式 | 所有模式 |
| `v` | 切换 FOLLOW 模式 | 所有模式 |
| `1` | 纵列队形 | FORMATION |
| `2` | 并排队形 | FORMATION |
| `3` | 左三角队形 | FORMATION |
| `4` | 右三角队形 | FORMATION |
| `q` | 退出程序 | — |

> **注意**：键盘控制使用的是"按下持续、松开停止"模式。按 `w` 时持续前进，松开 `w` 按空格即可停止。

---

## 11. 参数在线调优

无需重启节点即可调整 PID 参数和其他配置。

### 11.1 启动调参工具

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

### 11.2 领航者参数（leader_controller）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `speed_limit` | 0.6 m/s | 编队/跟随模式下的最大速度 |
| `max_formation_error` | 2.0 m | 编队误差超过此值触发保护 |
| `follower_lost_timeout` | 3.0 s | 跟随者失联超时 |
| `max_error_duration` | 5.0 s | 编队误差持续超限时间 |
| `offset_x` | 0.0 | 自定义编队 X 偏移 |
| `offset_y` | 0.0 | 自定义编队 Y 偏移 |
| `offset_yaw` | 0.0 | 自定义编队角度偏移 |

### 11.3 跟随者参数（follower_controller）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lin_kp` | 0.6 | 线速度 PID 比例增益 |
| `lin_ki` | 0.01 | 线速度 PID 积分增益 |
| `lin_kd` | 0.1 | 线速度 PID 微分增益 |
| `ang_kp` | 1.2 | 角速度 PID 比例增益 |
| `ang_ki` | 0.0 | 角速度 PID 积分增益 |
| `ang_kd` | 0.3 | 角速度 PID 微分增益 |
| `max_linear_speed` | 0.8 m/s | 跟随者最大线速度 |
| `max_angular_speed` | 1.0 rad/s | 跟随者最大角速度 |
| `pos_deadband` | 0.05 m | 位置死区 |
| `yaw_deadband` | 0.05 rad | 角度死区 |
| `follow_delay_seconds` | 2.0 s | 跟随延时 |

### 11.4 调参建议

| 现象 | 调整方法 |
|------|----------|
| 跟随者跟不上，距离误差大 | 增大 `lin_kp`、增大 `max_linear_speed` |
| 跟随者来回振荡 | 减小 `lin_kp`、增大 `lin_kd`、增大 `pos_deadband` |
| 跟随者转向不到位 | 增大 `ang_kp` |
| 转向时左右摇摆 | 减小 `ang_kp`、增大 `ang_kd` |
| 到达目标后不断微调 | 增大 `pos_deadband` 和 `yaw_deadband` |
| 编队拐弯时跟随者切角 | 增大 `ang_kp`、加快领航者转弯速度 |

---

## 12. 常见问题排查

### 12.1 网络问题

**Q: Car2 执行 `rostopic list` 看不到 Car1 的话题**

```bash
# 1. 检查 ROS_MASTER_URI 是否正确
echo $ROS_MASTER_URI
# Car2 应显示: http://192.168.1.101:11311

# 2. 检查能否 ping 通 Car1
ping 192.168.1.101

# 3. 检查 Car1 的 roscore 是否在运行
# 在 Car1 上：
ps aux | grep roscore

# 4. 检查防火墙（Ubuntu 18.04 默认无防火墙，如有则关闭）
sudo ufw disable
```

**Q: Car1 收不到 Car2 的 odom 数据**

```bash
# 在 Car1 上检查话题连接
rostopic info /robot2/odom
# 查看 Publishers 是否有 Car2 的节点

# 检查两台车的 ROS_IP 是否设对
echo $ROS_IP

# 检查时间同步
date
# 两台车的时间差应在 1 秒以内
```

### 12.2 编译问题

**Q: `catkin_make` 报找不到 cluster_msgs**

```bash
# 先单独编译消息包
catkin_make --pkg cluster_msgs

# 再完整编译
catkin_make
```

**Q: Python 脚本权限不足**

```bash
chmod +x ~/catkin_ws/src/cluster_bringup/scripts/*.py
chmod +x ~/catkin_ws/src/cluster_bringup/scripts/*.sh
```

### 12.3 运行问题

**Q: 编队模式下跟随者不动**

1. 检查模式是否正确：`rostopic echo /robot1/leader_cmd`，看 `mode` 是否为 2
2. 检查跟随者是否收到指令：在 Car2 上 `rostopic echo /robot1/leader_cmd`，看是否有数据
3. 检查 cmd_vel 是否发出：`rostopic echo /robot2/cmd_vel`
4. 增大 PID 增益试试

**Q: 跟随者乱跑**

1. 检查两车里程计坐标系是否对齐（两车启动时方向应一致）
2. 先按 `z` 回到 IDLE，重新对准两车位置再按 `c`
3. 检查 leader_cmd 中的 target_pose 是否合理

**Q: 键盘遥控没反应**

1. 确认 teleop_keyboard 节点在运行：`rosnode list | grep teleop`
2. 确认终端焦点在 teleop 窗口
3. 检查是否在 IDLE 模式（IDLE 模式下 teleop 不生效，按 `x` 切换到 TELEOP）

### 12.4 安全相关

**紧急停止方法（任选其一）：**
1. 在 teleop 终端按 `z`（切换到 IDLE，立即停止）
2. 按 `Ctrl+C` 停止所有 ROS 节点
3. 直接用手拿起小车（轮子离地即失去驱动力）
4. 使用 LIMO APP 远程急停

---

## 13. 附录：文件清单

### 13.1 完整文件列表

```
catkin_ws/src/
├── cluster_msgs/                         # 消息定义包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   ├── LeaderCmd.msg                 # 控制指令消息
│   │   └── FollowerStatus.msg            # 状态反馈消息
│   └── srv/
│       ├── SetMode.srv                   # 模式切换服务
│       └── SetFormation.srv              # 队形切换服务
│
├── cluster_common/                       # 共享工具库
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/cluster_common/
│   │   ├── pid.h                         # PID 控制器头文件
│   │   └── pose_utils.h                  # 位姿工具函数头文件
│   └── src/
│       ├── pid.cpp                       # PID 控制器实现
│       └── pose_utils.cpp                # 位姿工具函数实现
│
├── cluster_formation/                    # 领航控制器
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── cfg/
│   │   └── FormationParams.cfg           # 动态参数配置
│   ├── include/cluster_formation/
│   │   └── leader_controller.h           # 领航控制器头文件
│   └── src/
│       ├── leader_controller.cpp         # 核心逻辑实现
│       └── leader_controller_node.cpp    # 节点入口
│
├── cluster_following/                    # 跟随控制器
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── cfg/
│   │   └── FollowerParams.cfg            # 动态参数配置
│   ├── include/cluster_following/
│   │   └── follower_controller.h         # 跟随控制器头文件
│   └── src/
│       ├── follower_controller.cpp       # 核心逻辑实现
│       └── follower_controller_node.cpp  # 节点入口
│
└── cluster_bringup/                      # 启动与配置
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── formation_params.yaml          # 领航者参数
    │   └── follower_params.yaml           # 跟随者参数
    ├── launch/
    │   ├── sim_test.launch               # 单机仿真启动文件
    │   ├── car1_bringup.launch           # Car1 启动文件
    │   ├── car2_bringup.launch           # Car2 启动文件
    │   └── cluster_system.launch         # 双车系统启动文件
    └── scripts/
        ├── setup_network.sh              # 网络配置脚本
        ├── fake_odom_publisher.py        # 仿真里程计
        └── teleop_keyboard.py            # 键盘遥控
```

### 13.2 话题速查表

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| `/robot1/odom` | `nav_msgs/Odometry` | limo_base (Car1) | leader_controller, follower_controller |
| `/robot2/odom` | `nav_msgs/Odometry` | limo_base (Car2) | leader_controller, follower_controller |
| `/robot1/cmd_vel` | `geometry_msgs/Twist` | leader_controller | limo_base (Car1) |
| `/robot2/cmd_vel` | `geometry_msgs/Twist` | follower_controller | limo_base (Car2) |
| `/robot1/teleop_vel` | `geometry_msgs/Twist` | teleop_keyboard | leader_controller |
| `/robot1/leader_cmd` | `cluster_msgs/LeaderCmd` | leader_controller | follower_controller |
| `/robot2/follower_status` | `cluster_msgs/FollowerStatus` | follower_controller | leader_controller |

### 13.3 服务速查表

| 服务 | 类型 | 用途 |
|------|------|------|
| `/robot1/set_mode` | `cluster_msgs/SetMode` | 切换工作模式 |
| `/robot1/set_formation` | `cluster_msgs/SetFormation` | 切换编队队形 |

---

## 快速上手 Checklist

对于第一次使用，按以下顺序操作：

- [ ] **硬件**：两台车充满电，放到平整地面
- [ ] **网络**：两台车连入同一 WiFi，互相能 ping 通
- [ ] **SSH**：两台车互相免密登录
- [ ] **时间**：两台车时间同步
- [ ] **部署**：代码复制到两台车并编译通过
- [ ] **配置**：修改 `setup_network.sh` 中的主机名
- [ ] **仿真**：先在单机上运行 `sim_test.launch`，确认逻辑正确
- [ ] **单真车**：在一台车上运行 `car1_bringup.launch`，确认底盘响应
- [ ] **双车联网**：两台车分别启动，确认话题互通
- [ ] **编队测试**：进入 FORMATION 模式，低速行驶测试
- [ ] **跟随测试**：进入 FOLLOW 模式，直线路径测试
- [ ] **调参优化**：用 `rqt_reconfigure` 在线调整 PID

---

> 文档版本：v1.0  
> 适用硬件：LIMO S2（NVIDIA Jetson Nano）  
> 适用系统：Ubuntu 18.04 + ROS Melodic  
> 最后更新：2026-05-10
