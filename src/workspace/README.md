# Cyberdog 仿真导航框架 V3 — 架构文档

## 一、整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                      config_v3.py                           │
│  路线文件 · 步态区域 · 触发器 · 跟踪参数                       │
└──────────────────────┬──────────────────────────────────────┘
                       │ 读取配置
┌──────────────────────▼──────────────────────────────────────┐
│              scheme1_step_tracking_v3.py                    │
│  引擎: 加载路径 → 定位 → 区域匹配 → 航向计算 → 步态执行       │
└──────────────────────┬──────────────────────────────────────┘
                       │ 调用步态
┌──────────────────────▼──────────────────────────────────────┐
│                   gait_lib_v2.py                            │
│  步态积木库: init/finish · 离散步态 · 斜坡力控 · 播报          │
└──────────────────────┬──────────────────────────────────────┘
                       │ LCM / ROS2
┌──────────────────────▼──────────────────────────────────────┐
│              仿真环境 (Gazebo + cyberdog_locomotion)          │
│  收 simulator_lcmt (位姿) · 发 robot_control_cmd (运动)      │
└─────────────────────────────────────────────────────────────┘
```

## 二、文件清单

| 文件 | 角色 | 是否需改 |
|------|------|---------|
| `path_config.py` | **路线关键点** | ✅ 经常改 |
| `config_v3.py` | **步态区域 / 触发器 / 参数** | ✅ 经常改 |
| `build_path.py` | 从关键点生成 CSV + 预览图 | 不改 |
| `scheme1_step_tracking_v3.py` | 导航引擎 | 不改 |
| `gait_lib_v2.py` | 步态库 + 力控 | 不改 |

备份：
| `gait_lib.py` | 原始步态库 (无斜坡力控) |
| `scheme1_step_tracking.py` | V1 引擎 (区域硬编码版) |
| `scheme1_step_tracking_v2.py` | V2 引擎 (区域硬编码 + 力控) |

数据文件（不改）：
| `usergait_def.toml` | 自定义步态定义 (低姿态高抬腿) |
| `usergait_param_full.toml` | 自定义步态参数 |
| `file_send_lcmt.py` | LCM 类型: 文件发送 |

生成物：
| `track_path.csv` | 插值后的完整路径 (3320 路点) |
| `track_path_preview.png` | 路径预览图 |

## 三、通信机制

### 输入 (读取状态)
- **LCM `simulator_state`** (端口 7667, UDP 组播 239.255.76.67): Gazebo 真值位姿
  - `p[3]`: x, y, z (世界坐标, 米)
  - `rpy[3]`: roll, pitch, yaw (弧度)
  - 频率: ~500Hz
  - Yaw 修复: 趴下时 roll≈±π, yaw 需 +π 翻转 (gait_lib_v2.py `_on_state`)

### 输出 (发送指令)
- **LCM `robot_control_cmd`** (端口 7671, UDP 组播): 运动控制
  - `mode`: 11=locomotion, 12=stand, 7=prone
  - `gait_id`: 27=TROT_SLOW, 6=WALK
  - `vel_des[3]`: 前向/横向/角速度 (m/s, rad/s)
  - `step_height[2]`: 抬腿高度 (0~0.08m)
  - `pos_des[2]`: 质心高度 (0.08~0.20m)
  - `duration`: 0=持续, >0=定时 (ms)
  - `life_count`: 心跳 (int8, 变化时命令生效)
  - **500ms 超时机制**: 无新命令则自动趴下 → 必须 50Hz 持续发布

- **LCM `user_gait_file`** (端口 7671): 加载自定义步态参数 (mode=62 gait=110)
- **ROS2 `/yaml_parameter`**: 模式切换 + 身体倾斜
- **ROS2 `/apply_force`**: 斜坡外力补偿 (V2 功能, subprocess 非阻塞)

## 四、导航算法

### 4.1 主循环 (20~50Hz)
```
while running:
    x, y, yaw = gl.get_position()           # 读位姿
    zone = match_zone(x, y)                 # 匹配步态区域
    check_triggers(x, y)                    # 检查一次性触发器
    check_slope(y)                          # 斜坡力控 (滞后)
    check_goal(x, y)                        # 终点判定

    idx = nearest_waypoint(x, y)            # 最近路点 (前搜500点)
    la  = lookahead(idx, L=0.5m)            # 前视点
    target = atan2(ty-y, tx-x)              # 几何方向
    if backward: target += π                # S1 背身
    angle_err = target - yaw               # 航向偏差

    if |angle_err| > 15°: turn(angle_err)   # 先转
    else: execute_gait(zone)                # 再走
```

### 4.2 航向计算
- **几何方向**: `atan2(ty-y, tx-x)` — 狗 → 前视点的方向角
- **背身**: zone gait = `backward` 时 `target += π`
- 退出阈值: 15° (大于此值先旋转, 小于则前行)

### 4.3 离散步态执行
- 所有步态通过 `_step_run()` 以 **50Hz 持续发布** `robot_control_cmd`
- 每步时长 = `distance / speed * 1.3` (安全系数补偿加减速)
- 步完成后自动回零 `vel_des=[0,0,0]`

## 五、路径系统

### 5.1 path_config.py 结构
```python
WAYPOINTS = [
    ("SPAWN",    0.00, 0.00),   # (名称, x, y) 世界坐标
    ("ROCK",     3.00, 0.00),
    ...
]
STEP = 0.02   # 插值间距 (米)
```

### 5.2 build_path.py 流程
1. 读取 `path_config.py` 的 WAYPOINTS
2. 线性插值 (间距 0.02m) → 生成稠密路点
3. 计算每个路点的 yaw (`atan2(dy, dx)` 前向差分)
4. 输出 `track_path.csv` (x, y, yaw 三列)
5. 输出 `track_path_preview.png` (可视化预览)

## 六、区域系统 (config_v3.py)

### 6.1 步态区域 (ZONES)
按优先级排序, 第一个匹配的生效:

```python
dict(name="名称", gait="步态类型",
     step_m=步距, speed_ms=速度,
     x_min=x下界, x_max=x上界,
     y_min=y下界, y_max=y上界)
```

可用步态类型:
| gait | 对应方法 | 效果 |
|------|---------|------|
| `forward` | `step_forward()` | 普通前进 0.2m/s |
| `backward` | `step_backward()` | 倒退 0.15m/s (S1 背身) |
| `high_forward` | `step_high_forward()` | 高抬腿 0.15m/s (石板路/坎) |
| `crouch` | `crouch_step_forward()` | 低姿态高抬腿 0.08m/s (mode=62 gait=110, 过限高杆) |
| `slope` | `step_forward()` + 力控 | 斜坡慢走 0.08m/s |

坐标边界用 ±99 表示不限。

### 6.2 触发器 (TRIGGERS)
进入区域时**触发一次** (防重复):

```python
dict(name="名称",
     x_min=, x_max=, y_min=, y_max=,
     action="动作", arg="参数")
```

可用动作:
| action | 效果 |
|--------|------|
| `announce` | 语音播报 (espeak-ng) + 终端打印 |
| `print` | 仅终端打印 |
| `jump` | 原地跳跃 (mode=22) |
| `custom` | 自定义函数 `arg(gl)` |

### 6.3 斜坡力控
- 进入 y>12.2 开启, 退出 y<11.8 关闭 (1m 滞后带)
- 原理: 读 IMU roll/pitch → 低通滤波 → 反向 `apply_force` + `des_roll_pitch_height` 外倾
- 力控参数: `SLOPE_FORCE_GAIN=100` (N/rad), `SLOPE_LEAN_GAIN=0.8`
- 节流: 每 5 步 (~2.5s) 更新一次, 非阻塞 subprocess.Popen

## 七、步态库 API (gait_lib_v2.py)

### 初始化和收尾
```python
gl = GaitLib()
gl.init()      # 抢占总线 → 站起(mode=12,7s) → 沉降2s → locomotion(mode=11) → 沉降2s → 加载低姿态步态
gl.finish()    # 停车 → 趴下 (mode=7)
```

### 离散步态 (阻塞, 50Hz 持续发布)
```python
gl.step_forward(distance=0.08, speed=0.2)     # 前进
gl.step_backward(distance=0.05, speed=0.1)    # 后退
gl.step_turn(degrees, rate=None)              # 旋转 (正=左)
gl.step_high_forward(distance=0.08, speed=0.15)  # 高抬腿
gl.crouch_step_forward(distance=0.06, speed=0.10) # 低姿态高抬腿 (mode=62 gait=110)
gl.step_shift(distance=0.03, speed=0.05)      # 横移
```

### 持续运动 (需手动 stop)
```python
gl.forward(0.2) / gl.backward(0.1) / gl.turn_left(0.5) / gl.turn_right(0.5)
gl.shift_left(0.05) / gl.shift_right(0.05) / gl.stop()
```

### 特殊动作
```python
gl.jump()                       # 原地跳跃 (mode=22)
gl.announce("文字")              # 语音播报 (espeak-ng, 非阻塞)
gl.stuck_recover()              # 后退 0.5s 脱困
gl.get_position()               # → (x, y, z, roll, pitch, yaw)
```

### 斜坡力控 (V2 特有)
```python
gl.enable_slope_comp(force_gain=100, lean_gain=0.8)
gl.disable_slope_comp()
gl._slope_tick()                # 每步调用, 内部节流
```

## 八、关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| LOOKAHEAD | 0.50m | 前视距离 |
| LOOKAHEAD_MIN | 0.25m | 弯道近距前视 |
| ANGLE_THRESH | 15° | 航向修正阈值 |
| STEP_FAR | 0.10m | 直道步距 |
| STEP_NEAR | 0.05m | 弯道步距 |
| STUCK_LIMIT | 12 | 卡住步数 |
| STUCK_EPS | 0.012m | 卡住位移阈值 |
| GOAL_TOL | 0.35m | 终点半径 |

## 九、常见问题

1. **狗站起就侧移**: 有残留进程在发 `robot_control_cmd` → `pkill -9 -f full_track_nav`
2. **狗站起就趴**: LCM 500ms 超时 → 检查步态是否 50Hz 持续发布
3. **无意义旋转**: 几何方向法在偏轨时自动修正, 是正常的; 放宽 ANGLE_THRESH 可减少
4. **斜坡摔倒**: V2 的 `apply_force` 依赖 ROS2 环境 → 确保 `source install/setup.bash`
5. **路径不对**: 编辑 `path_config.py` → 跑 `build_path.py` → 检查预览图
6. **匍匐过限高杆卡住**: mode=62 gait=110 加载失败 → 检查 `usergait_def.toml` 和 `usergait_param_full.toml` 是否在 workspace 目录

## 十、运行命令

```bash
# 启动仿真 (一个终端)
cd /home/cyberdog_sim
python3 launchsim_safe.py --world race

# 编辑路线 → 生成
python3 src/workspace/build_path.py

# 运行导航 (另一个终端)
python3 src/workspace/scheme1_step_tracking_v3.py
```
