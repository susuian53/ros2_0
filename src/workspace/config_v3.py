#!/usr/bin/env python3
"""
config_v3.py — 赛道配置文件

修改路线、步态区域、交互触发只需改此文件，无需动引擎代码。
引擎: scheme1_step_tracking_v3.py
"""

# ═══════════════════════════════════════════════════════
# 路线文件
# ═══════════════════════════════════════════════════════
PATH_CSV = "/home/cyberdog_sim/src/workspace/track_path.csv"

# ═══════════════════════════════════════════════════════
# 跟踪参数
# ═══════════════════════════════════════════════════════
START_INDEX = 5000     # 起点路点索引 (由 susuian1.py 设置)
LOOKAHEAD       = 0.50      # 前视距离 (m)
LOOKAHEAD_MIN   = 0.25      # 弯道近距前视
ANGLE_THRESH    = 15.0      # 航向修正阈值 (°) — 放宽减少无意义旋转
TURN_FAST_THRES = 30.0      # 快转阈值 (°)
STEP_FAR        = 0.07      # 直道步距 (m), 缩小防摔倒
STEP_NEAR       = 0.04      # 弯道步距 (m)
CURVE_ANGLE     = 25.0      # 弯道判断角 (°)
STUCK_LIMIT     = 25        # 卡住步数上限, 放宽
STUCK_EPS       = 0.008     # 卡住位移阈值 (m), 放宽
GOAL_TOL        = 0.35      # 终点判定半径 (m)

# ═══════════════════════════════════════════════════════
# 步态切换段 (按路点顺序, 第一个匹配的生效)
#
# gait 类型:
#   forward       — 普通前进
#   backward      — 倒退 (S1 背身)
#   high_forward  — 高抬腿 (石板路、坎)
#   crouch        — 匍匐前进 (限高杆)
#   slope         — 斜坡慢走 + 力控补偿
#   jump          — 跳跃动作
#
# 每个段: dict(name, gait, step_m, speed_ms, start, end)
#   start/end 写 WAYPOINTS 里的路点名，按路径顺序切换
# ═══════════════════════════════════════════════════════
GAIT_STAGES = [
    dict(name="S1_back", gait="backward",
         step_m=0.08, speed_ms=0.15,
         start="SPAWN", end="PRE_TURN"),

    dict(name="bridge_bump", gait="high_forward",
         step_m=0.06, speed_ms=0.12,
         start="BR_BOT", end="BR_TOP"),

    dict(name="crouch_bar1", gait="crouch",
         step_m=0.04, speed_ms=0.13,
         start="CH1_IN", end="S4_MID"),

    dict(name="crouch_bar2", gait="crouch",
         step_m=0.04, speed_ms=0.13,
         start="BAR2_MID", end="CH3_IN_RET"),

    # ── 斜坡段 ──
    dict(name="slope_top", gait="slope",
         step_m=0.03, speed_ms=0.03,
         start="BR_TOP", end="FINISH"),

    # ── FINISH 跳跃 → START2 ──
    dict(name="jump_finish", gait="jump",
         step_m=0.08, speed_ms=0.20,
         start="FINISH", end="START2"),

    # ── 跳跃后正常寻迹 (START2 → A1 → A2) ──
    dict(name="post_jump", gait="high_forward",
         step_m=0.09, speed_ms=0.30,
         start="START2", end="A1"),

    dict(name="default", gait="forward",
         step_m=STEP_FAR, speed_ms=0.22,
         start=None, end=None),
]

# ═══════════════════════════════════════════════════════
# 一次性触发器 (进入区域时触发一次)
#
# action: announce(text) | print(text) | jump | custom(func)
# ═══════════════════════════════════════════════════════
TRIGGERS = [
    dict(name="s1_done",
         x_min=2.5, x_max=99,  y_min=-99, y_max=0.5,
         action="announce", arg="石板路通过"),
    
     dict(name="crouch_bar1", 
         x_min=-2, x_max=0.5,   y_min=9.3, y_max=10.2,
         action="announce", arg="识别到限高杆"),
     
     dict(name="obstacle", skip=3,
         x_min=0.5, x_max=2.0,   y_min=8.3, y_max=10.2,
         action="announce", arg="识别到障碍物"),
    
     dict(name="crouch_bar2",
          x_min=1.9, x_max=2.5,y_min=8.5, y_max=11.1,
          action="announce", arg="识别到限高杆"),

    dict(name="bridge_enter",
         x_min=3.0, x_max=99,  y_min=7.0, y_max=7.5,
         action="announce", arg="进入独木桥"),

    dict(name="finish_line",
         x_min=-99, x_max=99,  y_min=12.5, y_max=99,
         action="announce", arg="到达终点"),
]
