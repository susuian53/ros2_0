# Cyberdog SIM 仿真上下文

## 1. 环境概述

- **操作系统**：Ubuntu 20.04 LTS
- **ROS2**：Galactic（已安装）
- **Gazebo**：11.14.0（已安装）
- **LCM**：1.5.0（已安装）
- **Eigen**：3.3.7（已安装）
- **Python**：3.8.10
- **工具链**：xacro、vcstool、colcon 均已安装

> 所有依赖已完全具备，无需手动安装。

---

## 2. 代码仓库结构

```
cyberdog_sim/
├── src/
│   ├── cyberdog_simulator/   # Gazebo 仿真器与可视化工具
│   │   ├── cyberdog_example/ # 官方仿真实例（本文档重点）
│   │   │   ├── src/
│   │   │   │   ├── keybroad_commander.cpp  # 键盘控制例程
│   │   │   │   └── cyberdogmsg_sender.cpp  # 参数/外力控制例程
│   │   │   ├── include/cyberdog_example/
│   │   │   │   └── gamepad_lcmt.hpp        # LCM 游戏手柄消息定义
│   │   │   └── CMakeLists.txt
│   │   └── cyberdog_gazebo/script/
│   │       ├── launchsim.py          # 原始一键启动脚本
│   │       ├── launchsim_safe.py     # 增强版（自动清理进程）
│   │       ├── launchgazebo.sh       # 启动 Gazebo
│   │       ├── launchvisual.sh       # 启动 Rviz2
│   │       ├── launchcontrol.sh      # 启动控制程序
│   │       └── gazebolauncher.py     # Gazebo 进程管理器
│   └── cyberdog_locomotion/  # 运动控制算法
├── cyberdog_sim.repos
└── ...
```

---

## 3. 仿真启动流程

### 3.1 一键启动（推荐）

```bash
cd /home/cyberdog_sim
python3 launchsim_safe.py
```

`launchsim_safe.py` 执行逻辑：
1. 调用 `pkill` 清理所有仿真相关进程（`gzclient`、`gzserver`、`rviz2`、`ros2 launch` 等）
2. 启动 Gazebo 仿真（`launchgazebo.sh` → `gazebolauncher.py`）
3. 启动 Rviz2 可视化（`launchvisual.sh`）
4. 启动控制程序（`launchcontrol.sh`）

### 3.2 分别启动（调试用）

```bash
# 终端1：Gazebo
source /opt/ros/galactic/setup.bash && source install/setup.bash
ros2 launch cyberdog_gazebo gazebo.launch.py

# 终端2：控制程序
source /opt/ros/galactic/setup.bash && source install/setup.bash
ros2 launch cyberdog_gazebo cyberdog_control_launch.py

# 终端3：可视化
source /opt/ros/galactic/setup.bash && source install/setup.bash
ros2 launch cyberdog_visual cyberdog_visual.launch.py
```

---

## 4. 通信架构

仿真环境由三个核心模块组成：

| 模块 | 职责 | 通信方式 |
|------|------|----------|
| `cyberdog_control` | 运动控制算法 | ↔ Gazebo：共享内存；↔ 外部：LCM |
| `legged_plugins` | Gazebo 物理仿真与传感器模拟 | ↔ ROS2：Topic |
| `cyberdog_visual` | Rviz2 数据可视化与 Topic 转发 | ↔ ROS2：Topic |

### 4.1 控制程序 ↔ Gazebo（共享内存）

- Gazebo 创建 host 共享内存，控制程序 attach 进行数据交换
- 数据结构：`robotToSim` / `simToRobot`

### 4.2 控制程序 ↔ 外部（LCM 通信）

#### 高层接口 —— 基本控制指令

- **通道**：`robot_control_cmd`
- **URL**：`udpm://239.255.76.67:7671?ttl=255`
- **频率**：2~500 Hz（超时 500ms 触发趴下保护）
- **数据结构**：`robot_control_cmd_lcmt`

常用模式：

| mode | gait_id | 动作 |
|------|---------|------|
| 7 | 0 | 缓慢趴下 |
| 11 | 3 | 中速行走 (TROT_MEDIUM) |
| 11 | 10 | 快速行走 |
| 11 | 27 | 慢速行走 |
| 12 | 0 | 恢复站立 |
| 21 | 0 | 位控姿态模式 |

#### 高层接口 —— 状态反馈

- **通道**：`robot_control_response`
- **URL**：`udpm://239.255.76.67:7670?ttl=255`
- **频率**：50 Hz

#### 底层接口 —— 电机直接控制

- **通道**：`motor_ctrl`
- **指令**：`MotorCmd`（`q_des[12]`、`qd_des[12]`、`kp_des[12]`、`kd_des[12]`、`tau_des[12]`）
- **反馈**：`RobotData`（`q[12]`、`qd[12]`、`tau[12]`、`quat[4]`、`rpy[3]`、`acc[3]`、`omega[3]`）

### 4.3 仿真程序 ↔ ROS2 Topic

#### 发布的 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达（需 `use_lidar:=true`） |
| `/joint_states` | `sensor_msgs/JointState` | 12 关节位置/速度/力矩 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |

#### 接收的 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/yaml_parameter` | `YamlParam` | 实时修改控制参数 |
| `/apply_force` | `ApplyForce` | 施加外部力（仅仿真） |

### 4.4 仿真程序 ↔ LCM（仿真器数据）

- **通道**：`simulator_lcmt`
- 包含：机身位姿、速度、关节状态、足端力/位置等

---

## 5. 官方仿真实例详解（cyberdog_example）

### 5.1 键盘控制例程（keybroad_commander）

**功能**：通过键盘发送 `gamepad_lcmt` LCM 消息，模拟手柄控制机器人。

**启动**：
```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
./build/cyberdog_example/keybroad_commander
```

**键位映射**：

| 键位 | 作用 | 数据变化 |
|------|------|----------|
| `w` / `s` | x 方向速度 ±0.1 | `leftStickAnalog[1]` |
| `a` / `d` | y 方向速度 ±0.1 | `leftStickAnalog[0]` |
| `i` / `k` | pitch 速度 ±0.1 | `rightStickAnalog[1]` |
| `j` / `l` | yaw 速度 ±0.1 | `rightStickAnalog[0]` |
| `e` | QP 站立模式 | `x = 1` |
| `r` | locomotion 模式 | `y = 1` |
| `t` | 缓慢趴下 | `a = 1` |
| `y` | 恢复站立 | `b = 1` |
| `c` | 清零所有速度 | 摇杆归零 |

**实现要点**：
- 先通过 ROS2 Topic `/yaml_parameter` 发送 `use_rc = 0`，切换到 gamepad 控制模式
- 然后进入循环，读取键盘输入，构造 `gamepad_lcmt` 对象
- 通过 LCM 通道 `gamepad_lcmt` 发布消息
- 每次发布后将按钮状态（`x/y/a/b`）清零，避免持续触发

**`gamepad_lcmt` 数据结构**：
```cpp
class gamepad_lcmt {
    int32_t leftBumper, rightBumper;
    int32_t leftTriggerButton, rightTriggerButton;
    int32_t back, start;
    int32_t a, b, x, y;
    int32_t leftStickButton, rightStickButton;
    float leftTriggerAnalog, rightTriggerAnalog;
    float leftStickAnalog[2];   // [0]=x, [1]=y
    float rightStickAnalog[2];  // [0]=yaw, [1]=pitch
};
```

### 5.2 参数与外力例程（cyberdogmsg_sender）

**功能**：演示通过 ROS2 Topic 修改参数和施加外力。

**启动**：
```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
./build/cyberdog_example/cyberdogmsg_sender
```

**执行流程**：
1. 发送 `use_rc = 0`（切换到 gamepad 控制模式）
2. 发送 `control_mode = 12`（恢复站立）
3. 等待 5 秒
4. 发送 `control_mode = 11`（locomotion 行走模式）
5. 等待 1 秒
6. 通过 `/apply_force` 对 `FL_knee` 施加 20N 的 z 方向力，持续 2 秒
7. 等待 5 秒
8. 发送 `des_roll_pitch_height` 向量，设置 roll=0.2、height=0.25
9. 等待 7 秒
10. 发送 `control_mode = 12`（恢复站立）

**关键代码模式**：
```cpp
// 修改参数
auto param = cyberdog_msg::msg::YamlParam();
param.name = "control_mode";
param.kind = 2;  // kS64
param.s64_value = 11;
param.is_user = 0;
para_pub_->publish(param);

// 施加外力
auto force = cyberdog_msg::msg::ApplyForce();
force.link_name = "FL_knee";
force.force = {0, 0, 20};
force.time = 2;
force_pub_->publish(force);
```

---

## 6. 坐标系与关节定义

### 6.1 机身坐标系

- **x**：前方
- **y**：左侧
- **z**：上方
- 右手法则

### 6.2 腿与关节顺序

| 编号 | 腿 | 关节1 | 关节2 | 关节3 |
|------|-----|-------|-------|-------|
| 0-2 | FR（右前） | 侧摆髋 | 前摆髋 | 膝 |
| 3-5 | FL（左前） | 侧摆髋 | 前摆髋 | 膝 |
| 6-8 | RR（右后） | 侧摆髋 | 前摆髋 | 膝 |
| 9-11 | RL（左后） | 侧摆髋 | 前摆髋 | 膝 |

### 6.3 关节范围

| 关节 | 范围 [rad] | 最大速度 | 最大力矩 |
|------|-----------|---------|---------|
| 侧摆髋关节 | [-0.68, 0.68] | 38.19 | 12 Nm |
| FR/FL 前摆髋关节 | [2.79, -1.33] | 38.19 | 12 Nm |
| RR/RL 前摆髋关节 | [3.14, -0.98] | 38.19 | 12 Nm |
| 膝关节 | [-0.52, -2.53] | 38.19 | 12 Nm |

---

## 7. 开发建议

### 7.1 控制机器人运动

1. **先启动仿真**：`python3 launchsim_safe.py`
2. **再运行控制程序**：如 `keybroad_commander` 或自定义 LCM/ROS2 节点
3. **关键前提**：必须先发送 `use_rc = 0`（通过 `/yaml_parameter`），否则控制指令无效

### 7.2 自定义控制程序

参考 `keybroad_commander.cpp`：
- 初始化 ROS2 节点
- 发布 `/yaml_parameter` 切换 `use_rc = 0`
- 初始化 LCM
- 构造 `gamepad_lcmt` 或 `robot_control_cmd_lcmt` 消息
- 循环发布

参考 `cyberdogmsg_sender.cpp`：
- 初始化 ROS2 节点
- 创建 `/yaml_parameter` 和 `/apply_force` 发布者
- 按序发送参数/力指令

### 7.3 常见问题

| 问题 | 解决 |
|------|------|
| Gazebo 残留进程 | 使用 `launchsim_safe.py` 或手动 `pkill -9 gzclient gzserver` |
| 控制指令无响应 | 检查是否已发送 `use_rc = 0`；检查 `life_count` 是否递增 |
| yaml-cpp 冲突 | 确保无其他版本 yaml-cpp |
| 编译失败 | 确认 `BUILD_ROS=ON`，已 source ROS2 环境 |

---

## 8. 参考路径

- 开发指南：`/home/cyberdog_sim/文档/开发指南.md`
- 一键启动脚本：`/home/cyberdog_sim/launchsim_safe.py`
- 官方例程源码：`/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_example/src/`
- LCM 消息定义：`/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_example/include/cyberdog_example/`
