#!/usr/bin/env python3
"""
scheme1_step_tracking_v3.py — 离散步态寻迹引擎

所有路线、区域、交互在 config_v3.py 中配置，只改配置不动引擎。
航向使用路径 yaw + 横向偏差修正，避免几何方向引起的无意义旋转。

用法:
    python3 /home/cyberdog_sim/src/workspace/scheme1_step_tracking_v3.py
"""

import sys, os, time, math, signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gait_lib_v2 import GaitLib
import config_v3 as cfg
import path_config as path_cfg
import importlib.machinery, importlib.util

# 动态加载第六赛段专用步态库（文件名为 6.gait.lib.py）
six_lib = None
try:
    six_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '6.gait.lib.py')
    loader = importlib.machinery.SourceFileLoader('six_gait_lib', six_path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    six_lib = importlib.util.module_from_spec(spec)
    loader.exec_module(six_lib)
    print('[INFO] Loaded sixth-segment gait lib:', six_path)
except Exception as e:
    six_lib = None
    print('[WARN] Could not load sixth gait lib:', e)

running = True
_triggers_fired = set()
_trigger_enters = {}     # trigger_name → 进入次数 (配合 skip=N 使用)
_stage_enters = {}       # stage_name → 进入次数 (可选)

def on_sigint(*_):
    global running; running = False
    print("\n\n⚠  Ctrl+C — 安全退出中...")

def normalize_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

# ═══════════════════════════════════════════════════════
def load_path(csv_path):
    path = []
    with open(csv_path) as f:
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 2:
                yaw = float(p[2]) if len(p) >= 3 else 0.0
                path.append((float(p[0]), float(p[1]), yaw))
    print(f"[PATH] {len(path)} waypoints from {csv_path}")
    return path

def nearest_index(path, x, y, start):
    """只往前搜 200 点 (~4m), 防止交叉路口回跳"""
    start = max(0, start)
    if start >= len(path): start = 0
    best, best_d = start, math.hypot(path[start][0]-x, path[start][1]-y)
    end = min(start + 200, len(path))
    for i in range(start, end):
        d = math.hypot(path[i][0]-x, path[i][1]-y)
        if d < best_d: best_d = d; best = i
    return best

def lookahead_index(path, x, y, start, L):
    for i in range(start, len(path)):
        if math.hypot(path[i][0]-x, path[i][1]-y) >= L: return i
    return len(path)-1

def build_waypoint_index_map():
    """把 WAYPOINTS 里的路点名映射到 track_path.csv 的行号。"""
    indices = {}
    total = 0
    points = path_cfg.WAYPOINTS
    for i, (name, x, y) in enumerate(points):
        indices[name.upper()] = total
        if i + 1 >= len(points):
            continue
        _, x1, y1 = points[i + 1]
        dist = math.hypot(x1 - x, y1 - y)
        total += max(1, int(dist / path_cfg.STEP))
    return indices

def is_curve(path, idx, n=10):
    if idx+n >= len(path): return False
    a0 = math.atan2(path[min(idx+n,len(path)-1)][1]-path[idx][1],
                    path[min(idx+n,len(path)-1)][0]-path[idx][0])
    a1 = math.atan2(path[min(idx+n+5,len(path)-1)][1]-path[min(idx+n,len(path)-1)][1],
                    path[min(idx+n+5,len(path)-1)][0]-path[min(idx+n,len(path)-1)][0])
    return math.degrees(abs(a1-a0)) > cfg.CURVE_ANGLE

def match_stage(idx, waypoint_index_map):
    """按路点区间匹配步态段。"""
    default_stage = cfg.GAIT_STAGES[-1]
    for stage in cfg.GAIT_STAGES:
        if stage['name'] == 'default':
            continue

        start_name = stage.get('start')
        end_name = stage.get('end')
        start_idx = waypoint_index_map.get(start_name.upper()) if start_name else None
        end_idx = waypoint_index_map.get(end_name.upper()) if end_name else None
        if start_idx is None:
            continue
        if end_idx is None:
            end_idx = 10**9

        if start_idx <= idx < end_idx:
            return stage

    return default_stage

def match_triggers(x, y, la=None, path_len=None):
    matched = []
    for t in cfg.TRIGGERS:
        if t['name'] in _triggers_fired:
            continue
        if not (t['x_min'] <= x <= t['x_max'] and t['y_min'] <= y <= t['y_max']):
            continue

        # 特殊逻辑：终点触发器只有在前视目标点到达路径末尾时才允许触发
        if t['name'] == 'finish_line' and la is not None and path_len is not None:
            if la < path_len - 1:
                continue

        # skip=N: 跳过前N次, 第N+1次触发
        skip = t.get('skip', 0)
        if skip > 0:
            cnt = _trigger_enters.get(t['name'], 0) + 1
            _trigger_enters[t['name']] = cnt
            if cnt <= skip:
                continue  # 还没到触发次数
        matched.append(t)
    return matched

def fire_trigger(gl, t):
    _triggers_fired.add(t['name'])
    skip = t.get('skip', 0)
    tag = f" (after {skip} skips)" if skip else ""
    print(f"\n  ⚡ [{t['name']}]{tag}", end='')
    if t['action'] == 'announce':
        print(f" 📢 {t['arg']}"); gl.announce(t['arg'])
    elif t['action'] == 'print':
        print(f" ℹ️  {t['arg']}")
    elif t['action'] == 'jump':
        print(" 🤸"); gl.jump()
    elif t['action'] == 'custom' and callable(t['arg']):
        print(" 🔧"); t['arg'](gl)

def execute_gait(gl, stage, dg):
    g, d, s = stage['gait'], stage['step_m'], stage['speed_ms']
    if dg < 1.0: d = max(0.03, d*(dg/1.0))
    if   g == 'backward':      gl.step_backward(d, speed=s)
    elif g == 'high_forward':  gl.step_high_forward(d, speed=s)
    elif g == 'crouch':        gl.crouch_step_forward(d, speed=s)
    elif g == 'slope':         gl.slope_step_forward(d, speed=s)
    elif g == 'jump':          _do_jump_and_stabilize(gl, d, s)
    else:                      gl.step_forward(d, speed=s)


_jumped = False  # 全局标记：跳跃只执行一次

def _do_jump_and_stabilize(gl, d, s):
    """跳跃 + 稳住身体，只执行一次"""
    global _jumped
    if _jumped:
        # 跳跃后正常前进
        gl.step_forward(d, speed=s)
        return
    _jumped = True
    print("\n  🤸 执行跳跃！")
    gl.jump()
    # 跳跃后稳住身体：短暂站立
    print("  🧍 稳住身体中...")
    for _ in range(15):
        gl.stop()
        gl._pump()
        time.sleep(0.15)
    print("  ✅ 跳跃+稳住完成，继续寻迹")

# ═══════════════════════════════════════════════════════
def run():
    global running
    if not os.path.exists(cfg.PATH_CSV):
        print(f"[ERROR] {cfg.PATH_CSV} not found!"); return
    path = load_path(cfg.PATH_CSV)
    waypoint_index_map = build_waypoint_index_map()
    # 特殊区间索引（按路点名）
    def _idx(name):
        v = waypoint_index_map.get(name.upper())
        return v if v is not None else -1
    START2_i = _idx('START2')

    # 特殊节点 A1/A2 的索引（对应 path_config.py 中的命名）
    IDX_1 = _idx('A1')
    IDX_2 = _idx('A2')

    # ── 第六赛段状态机 ──
    sexta_phase = 0
    # 0 = START2→A1 先转后直行
    # 1 = A1 执行逻辑函数U
    sexta_A1_turned = False

    # U 函数子阶段状态
    u_sub_phase = 0       # 0=前进矫正(0.19,14.67),1=左侧移0.1m,2=转向0°,3=前往(2.50,14.85),4=等20s,5=右移(2.50,13.50),6=后退寻路(2.0,12.80),7=右移(3.50,12.80),8=左侧移0.2m,9=转向0°,10=停止
    u_start_x = 0.0
    u_start_y = 0.0
    u_wait_start = 0.0

    # run 卡住检测
    low_stuck_cnt = 0
    low_stuck_last_x = 0.0
    low_stuck_last_y = 0.0

    # 斜面4段: 全部侧移, 拐弯渐进调角, XY偏移大时停下修正
    SLOPE_SIDE_SPEED  = 0.05    # 侧移速度
    SLOPE_TURN_MAX_DEG = 4.0    # 每次最大转角 (°)
    SLOPE_TURN_RATE    = 0.12   # 转角速率
    XY_OFFSET_MAX      = 0.06   # XY偏移阈值 (m)
    slope_seg = 0               # 当前段 0-3
    slope_seg_targets = ['TOP_L', 'TOP_LL', 'TOP_RR', 'FINISH']
    slope_seg_starts  = ['BR_TOP', 'TOP_L', 'TOP_LL', 'TOP_RR']  # 每段起点路点名
    slope_turning = False       # 拐弯调角阶段
    slope_turn_target_yaw = 0.0 # 拐弯目标yaw
    slope_final_turn = False    # 斜面完成后的最终回正调角
    slope_anti_g_vf = 0.0       # 动态重力对抗分量 (积累)
    slope_last_x = 0.0
    slope_last_y = 0.0

    # 跳跃下坡状态机 (FINISH → START2)
    # Phase: 0=转到-90°, 1=走到Y=13.00, 2=跳跃, 3=调整到(3.20,12.90), 4=走到X≈2.50, 5=完成
    jump_phase = 0
    jump_turned = False
    jump_done = False
    jump_start_z = 0.0
    jump_last_z = 0.0
    jump_stable_cnt = 0
    jump_drop_cnt = 0
    jump_phase2_start = 0
    s1_back_yaw_ok = False      # S1_back 矫正朝向标志

    gl = GaitLib()
    gl.init()
    if not gl.pose_valid: print("[ERROR] No pose"); return

    x, y, z, roll, pitch, yaw = gl.get_position()
    idx = nearest_index(path, x, y, cfg.START_INDEX)
    last_x, last_y = x, y
    stuck_cnt = step = 0
    total_dist = 0.0
    last_stage = None

    goal = path[-1]
    print(f"\n═══ V3 ═══")
    print(f"  Start: ({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}°")
    print(f"  Goal:  ({goal[0]:.2f},{goal[1]:.2f})")
    print(f"  Stages:{len(cfg.GAIT_STAGES)}  Triggers:{len(cfg.TRIGGERS)}\n")
    # 调试：打印第六赛段关键索引
    print(f"  IDXs: A1={IDX_1} A2={IDX_2} START2={START2_i}")

    # ── 第六赛段 低姿态辅助 (仿 crouch_step_forward, pos_z 比默认低3cm) ──
    LOW_MODE = 62       # MODE_MOTION
    LOW_GAIT = 110      # GAIT_USER
    LOW_POS_Z = 0.17    # 3cm lower than default 0.20
    LOW_STEP_H = 0.08

    def run_stuck_check(cx, cy):
        nonlocal low_stuck_cnt, low_stuck_last_x, low_stuck_last_y
        if low_stuck_last_x == 0.0 and low_stuck_last_y == 0.0:
            low_stuck_last_x, low_stuck_last_y = cx, cy
            return False
        d = math.hypot(cx - low_stuck_last_x, cy - low_stuck_last_y)
        low_stuck_last_x, low_stuck_last_y = cx, cy
        if d < 0.005:
            low_stuck_cnt += 1
        else:
            low_stuck_cnt = 0
        if low_stuck_cnt > 8:
            gl.stop()
            gl._send(mode=12, gait_id=0, contact=0, step_h=0.0, pos_z=0.25)
            time.sleep(0.3)
            gl.step_backward(0.10, speed=0.15)
            gl._pump()
            low_stuck_cnt = 0
            print(f"  ⚡ run_unstuck: stand + back 0.10m")
            return True
        return False

    def run_forward(dist=0.05, speed=0.30):
        gl.step_forward(dist, speed=speed)

    def run_stop():
        gl.stop()

    def u_forward(dist=0.05, speed=0.15):
        dur_ms = max(100, int(dist / (speed * 1.0) * 1000 * 1.3))
        gl._step_run(mode=LOW_MODE, gait_id=LOW_GAIT, vf=speed,
                     step_h=LOW_STEP_H, pos_z=LOW_POS_Z, dur_ms=dur_ms)

    def u_stop():
        gl._send(mode=LOW_MODE, gait_id=LOW_GAIT, pos_z=LOW_POS_Z)

    def u_backward(dist=0.05, speed=0.10):
        dur_ms = max(100, int(dist / (speed * 1.0) * 1000 * 1.3))
        gl._step_run(mode=LOW_MODE, gait_id=LOW_GAIT, vf=-speed,
                     step_h=LOW_STEP_H, pos_z=LOW_POS_Z, dur_ms=dur_ms)

    while running:
        x, y, z, roll, pitch, yaw = gl.get_position()

        # ── 路点段 ──
        stage = match_stage(idx, waypoint_index_map)
        if stage != last_stage:
            # 离开旧段
            if last_stage:
                if last_stage['gait'] == 'slope':
                    gl._restore_default_gait()
                    gl.disable_slope_comp()
            # 进入新段
            print(f"\n  ➜ [{stage['name']}] gait={stage['gait']}  wp={idx}")
            if stage['gait'] == 'slope':
                gl._load_gecko_slope_gait()
                # 不用力控, 壁虎低姿态自带防滑
            last_stage = stage

        # ── 斜坡力控心跳 (非斜坡区自动跳过) ──
        gl._slope_tick()

        # ── 路点 + 前视 ──
        idx = nearest_index(path, x, y, idx)
        curve = is_curve(path, idx)
        L = cfg.LOOKAHEAD_MIN if curve else cfg.LOOKAHEAD
        la = lookahead_index(path, x, y, idx, L)
        tx, ty, _ = path[la]

        # ── 触发器 ──
        for t in match_triggers(x, y, la, len(path)): fire_trigger(gl, t)

        # ═══════════════════════════════════════════════════════════
        # 第六赛段: START2→A1→A2 → 逻辑函数U
        # ═══════════════════════════════════════════════════════════
        in_sexta = (jump_done and START2_i >= 0 and IDX_1 >= 0 and (idx >= START2_i or sexta_phase > 0))
        if in_sexta and IDX_1 >= 0:

            # ── 阶段0: START2→A1，先定向再直行 (run) ──
            if sexta_phase == 0:
                tx1, ty1, _ = path[IDX_1]
                dist_a1 = math.hypot(tx1 - x, ty1 - y)
                if not sexta_A1_turned:
                    target0 = math.atan2(ty1 - y, tx1 - x)
                    err0 = math.degrees(normalize_angle(target0 - yaw))
                    if abs(err0) > 8.0:
                        turn_deg = max(-45, min(45, err0))
                        gl.step_turn(turn_deg, rate=0.25)
                        print(f"  ⇢ START2 orient to A1: err={err0:+4.1f}° cur={math.degrees(yaw):.0f}°")
                        gl._pump(); time.sleep(0.02)
                        continue
                    sexta_A1_turned = True
                    print(f"  ✅ START2 oriented. Run to A1.")
                    continue
                if run_stuck_check(x, y): continue
                if dist_a1 > 0.10:
                    run_forward(dist=0.05, speed=0.30)
                    print(f"  → S6-0: pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}° d={dist_a1:.2f}")
                    continue
                sexta_phase = 1
                u_sub_phase = 0
                run_stop()
                print(f"  ✅ Reached A1 (dist={dist_a1:.3f}m).")
                print("\n  ══ S6 Phase 1: 逻辑函数U (前进矫正(0.19,14.67)→左侧移0.1m→转向0°→前往(2.50,14.85)→等20s→右移(2.50,13.50)→后退寻路(2.0,12.80)→右移(3.50,12.80)→左侧移0.2m→转向0°→停止)")

            # ── 阶段1: A1 处执行逻辑函数U ──
            #   U: 前进矫正(0.19,14.67) → 左侧移0.1m → 转向0° → 前往(2.50,14.85)yaw≈10° → 等20s → 右移(2.50,13.50) → 后退寻路(2.0,12.80) → 右移(3.50,12.80) → 左侧移0.2m → 转向0° → 停止
            if sexta_phase == 1:
                if run_stuck_check(x, y): continue

                # 1a: 前进到 (0.19, 14.67)
                if u_sub_phase == 0:
                    tx, ty = 0.19, 14.67
                    target_angle = math.atan2(ty - y, tx - x)
                    err = math.degrees(normalize_angle(target_angle - yaw))
                    dist = math.hypot(tx - x, ty - y)
                    err_x, err_y = abs(tx - x), abs(ty - y)
                    if dist < 0.03 and err_x < 0.02 and err_y < 0.02:
                        gl.stop()
                        u_sub_phase = 1
                        print(f"  \u2705 到达(0.19,14.67). 位置:({x:.3f},{y:.3f}), 循环前进+右转找0\u00b0.")
                        continue
                    if dist < 0.10:
                        if abs(err) > 5.0:
                            turn_deg = max(-30, min(30, err))
                            gl.step_turn(turn_deg, rate=0.20)
                            gl._pump(); time.sleep(0.02)
                            print(f"  \u21bb U-微调(0.19,14.67): err={err:+4.1f}\u00b0 pos=({x:.3f},{y:.3f}) d={dist:.3f}")
                            continue
                        step = min(0.03, dist + 0.01)
                        gl.step_forward(step, speed=0.08)
                        gl._pump()
                        print(f"  \u2192 U-微进(0.19,14.67): pos=({x:.3f},{y:.3f}) d={dist:.3f}")
                        continue
                    if abs(err) > 5.0:
                        turn_deg = max(-45, min(45, err))
                        gl.step_turn(turn_deg, rate=0.25)
                        gl._pump(); time.sleep(0.02)
                        print(f"  \u21bb U-转向(0.19,14.67): err={err:+4.1f}\u00b0 pos=({x:.3f},{y:.3f})")
                        continue
                    gl.step_forward(0.05, speed=0.20)
                    gl._pump()
                    print(f"  \u2192 U-前进\u2192(0.19,14.67): pos=({x:.3f},{y:.3f}) dist={dist:.3f}m")
                    continue

                # 1b: 循环: 前进+右侧移, 直到yaw接近0\u00b0 (身体与球平行)
                if u_sub_phase == 1:
                    err_to_zero = math.degrees(normalize_angle(0.0 - yaw))
                    if abs(err_to_zero) > 8.0:
                        # 前进 + 右侧移 (用侧移推球, 身体自然转正)
                        gl._step_run(vf=0.04, vl=0.0, vy=-0.12, dur_ms=400)
                        gl._pump()
                        print(f"  \u21c9 U-前进+右侧移\u21920\u00b0: err={err_to_zero:+4.1f}\u00b0 yaw={math.degrees(yaw):.0f}\u00b0 pos=({x:.2f},{y:.2f})")
                        continue
                    gl.stop()
                    u_sub_phase = 2
                    u_start_x, u_start_y = x, y
                    print(f"  \u2705 yaw\u22480\u00b0, 身体平行于球. 凶猛侧移撞球1.5m.")
                    continue

                # 1c: 凶猛右侧移1.5m (TrotFast gait=10 撞球)
                if u_sub_phase == 2:
                    moved = math.hypot(x - u_start_x, y - u_start_y)
                    if moved < 1.5:
                        gl._step_run(vf=0.0, vl=-0.55, vy=0.0, dur_ms=200, gait_id=10)
                        gl._pump()
                        print(f"  \u21c9 U-凶猛侧移撞球(TrotFast): moved={moved:.2f}m / 1.5m pos=({x:.2f},{y:.2f})")
                        continue
                    gl.stop()
                    u_sub_phase = 3
                    print(f"  \u2705 侧移1.5m完成. 导航到(0.19,12.90).")
                    continue

                # 1d: 导航到 (0.19, 12.90), 转到0\u00b0
                if u_sub_phase == 3:
                    tx, ty = 0.19, 12.90
                    target_angle = math.atan2(ty - y, tx - x)
                    err = math.degrees(normalize_angle(target_angle - yaw))
                    dist = math.hypot(tx - x, ty - y)
                    if dist < 0.10:
                        err0 = math.degrees(normalize_angle(0.0 - yaw))
                        if abs(err0) > 5.0:
                            turn_deg = max(-30, min(30, err0))
                            gl.step_turn(turn_deg, rate=0.20)
                            gl._pump(); time.sleep(0.02)
                            print(f"  \u21bb U-调yaw\u21920\u00b0: err={err0:+3.1f}\u00b0")
                            continue
                        gl.stop()
                        u_sub_phase = 4
                        u_start_x, u_start_y = x, y
                        print(f"  \u2705 到达(0.19,12.90) yaw\u22480\u00b0. 左侧移0.5m.")
                        continue
                    if abs(err) > 5.0:
                        turn_deg = max(-45, min(45, err))
                        gl.step_turn(turn_deg, rate=0.25)
                        gl._pump(); time.sleep(0.02)
                        print(f"  \u21bb U-导航(0.19,12.90): err={err:+4.1f}\u00b0 dist={dist:.2f}m")
                        continue
                    gl.step_forward(0.05, speed=0.20)
                    gl._pump()
                    print(f"  \u2192 U-前往(0.19,12.90): pos=({x:.2f},{y:.2f}) dist={dist:.2f}m")
                    continue

                # 1e: 左侧移0.5m
                if u_sub_phase == 4:
                    moved = math.hypot(x - u_start_x, y - u_start_y)
                    if moved < 0.5:
                        gl._step_run(vf=0.0, vl=0.20, vy=0.0, dur_ms=400)
                        gl._pump()
                        print(f"  \u21c6 U-左侧移: moved={moved:.2f}m / 0.5m pos=({x:.2f},{y:.2f})")
                        continue
                    gl.stop()
                    u_sub_phase = 5
                    print(f"  \u2705 左侧移0.5m完成. 转身冲刺Trot\u2192(3.5,12.90).")
                    continue

                # 1f: Trot 冲刺到 (3.5, 12.90) (gait_id=3 Trot medium)
                if u_sub_phase == 5:
                    tx, ty = 3.5, 12.90
                    target_angle = math.atan2(ty - y, tx - x)
                    err = math.degrees(normalize_angle(target_angle - yaw))
                    dist = math.hypot(tx - x, ty - y)
                    if dist < 0.10:
                        gl.stop()
                        u_sub_phase = 6
                        print(f"  \u2705 到达(3.5,12.90). 趴下.")
                        continue
                    if abs(err) > 5.0:
                        turn_deg = max(-30, min(30, err))
                        gl.step_turn(turn_deg, rate=0.30)
                        gl._pump(); time.sleep(0.02)
                        print(f"  \u21bb U-Trot冲刺: err={err:+4.1f}\u00b0 dist={dist:.2f}m")
                        continue
                    gl._step_run(vf=0.30, vl=0.0, vy=0.0, dur_ms=200, gait_id=3)
                    gl._pump()
                    print(f"  \u2192 U-Trot冲刺\u2192(3.5,12.90): pos=({x:.2f},{y:.2f}) dist={dist:.2f}m")
                    continue

                # 1g: 趴下 (MODE_PRONE=7)
                if u_sub_phase == 6:
                    gl._send(mode=7, gait_id=0, contact=0)
                    gl._pump()
                    time.sleep(3)
                    print("\n  \U0001f3af U函数完成。")
                    print(f"     位置: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}\u00b0")
                    running = False
                    continue

        # ── 终点判定 ──
        dg = math.hypot(goal[0]-x, goal[1]-y)
        if la == len(path) - 1 and dg < cfg.GOAL_TOL:
            print(f"\n🎯 GOAL reached. d={dg:.2f}m steps={step} dist={total_dist:.1f}m")

        # ── 航向: 几何方向 (狗→前视点) ──
        is_backward = (stage['gait'] == 'backward')
        geo_angle = math.atan2(ty - y, tx - x)
        if is_backward:
            target = normalize_angle(geo_angle + math.pi)
        else:
            target = geo_angle
        a_err = math.degrees(normalize_angle(target - yaw))

        # 在第六a段执行期间，完全禁止默认的基于角度的转向
        suppress_turn_until_2A = False
        try:
            if IDX_1 >= 0 and IDX_2 >= 0 and START2_i >= 0 and START2_i <= idx <= IDX_2 and sexta_phase < 3:
                suppress_turn_until_2A = True
        except NameError:
            suppress_turn_until_2A = False
        # 调试：只在抑制激活时打印（减少日志噪音）
        if suppress_turn_until_2A:
            print(f"  [DBG] suppress_turn=True idx={idx} sexta_phase={sexta_phase}")

        # ── 默认行为：按角度判断是否旋转（若被抑制则跳过转向） ──
        if stage['gait'] == 'slope':
            # ── 斜面4段: 全部侧移, 拐弯渐进调角, XY偏移大时停下修正 ──
            # 根据当前位置自动跳到对应段位
            for seg_i in range(len(slope_seg_targets)):
                t_idx = _idx(slope_seg_targets[seg_i])
                if t_idx >= 0 and idx > t_idx and slope_seg <= seg_i:
                    slope_seg = seg_i + 1
                    slope_turning = False
                    slope_last_x, slope_last_y = 0.0, 0.0
                    print(f'  ⏭ 跳过段{seg_i+1}/4({slope_seg_targets[seg_i]}) → 当前段{slope_seg+1}/4')
            if slope_seg < len(slope_seg_targets):
                tgt_name = slope_seg_targets[slope_seg]
                tgt_idx = _idx(tgt_name)
                if tgt_idx >= 0:
                    nx, ny = path[tgt_idx][0], path[tgt_idx][1]
                    dist_tgt = math.hypot(nx - x, ny - y)
                    target_dir = math.atan2(ny - y, nx - x)

                    # ── 拐弯调角阶段: 小步渐进旋转, 不移动 ──
                    if slope_turning:
                        yaw_err = normalize_angle(slope_turn_target_yaw - yaw)
                        if abs(yaw_err) < math.radians(3):
                            # 调角完成, 进入下一段
                            slope_turning = False
                            slope_seg += 1
                            slope_last_x, slope_last_y = x, y
                            if slope_seg < len(slope_seg_targets):
                                step += 1
                                print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                      f"wp={idx:5d} ✓ 拐弯完成 → 段{slope_seg+1}/4 → {slope_seg_targets[slope_seg]}")
                            else:
                                step += 1
                                print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                      f"wp={idx:5d} ✓ 斜面4段完成!")
                            continue
                        else:
                            turn_deg = max(-SLOPE_TURN_MAX_DEG, min(SLOPE_TURN_MAX_DEG, math.degrees(yaw_err)))
                            gl.step_turn(turn_deg, rate=SLOPE_TURN_RATE)
                            step += 1
                            print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                  f"wp={idx:5d} ↻ 调角 {turn_deg:+5.1f}° [slope_turn]")
                            continue

                    # ── 到达路点, 开始拐弯 ──
                    if dist_tgt < 0.30:
                        if slope_seg + 1 < len(slope_seg_targets):
                            next_name = slope_seg_targets[slope_seg + 1]
                            next_idx = _idx(next_name)
                            if next_idx >= 0:
                                next_dir = math.atan2(
                                    path[next_idx][1] - y, path[next_idx][0] - x)
                                # vl方向=yaw+90°, 要让vl指向目标 → yaw=目标方向-90°
                                slope_turn_target_yaw = normalize_angle(next_dir - math.pi / 2)
                                slope_turning = True
                                step += 1
                                print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                      f"wp={idx:5d} 📍 到达 {tgt_name} 拐弯→目标yaw={math.degrees(slope_turn_target_yaw):.0f}°")
                        else:
                            slope_seg += 1
                            step += 1
                            print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                  f"wp={idx:5d} 📍 斜面完成!")
                        slope_last_x, slope_last_y = x, y
                        continue

                    # 侧移理想朝向: yaw = 目标方向 - 90°, 使vl指向目标
                    desired_yaw = normalize_angle(target_dir - math.pi / 2)

                    # ── XY偏移检测: 计算到理想路径线的垂直距离 ──
                    start_name = slope_seg_starts[slope_seg]
                    start_idx = _idx(start_name)
                    if start_idx >= 0:
                        sx, sy = path[start_idx][0], path[start_idx][1]
                        vx, vy = nx - sx, ny - sy
                        cx, cy = x - sx, y - sy
                        v_len = math.hypot(vx, vy)
                        if v_len > 0.001:
                            cross_track = abs(vx * cy - vy * cx) / v_len
                            if cross_track > XY_OFFSET_MAX:
                                # 停下侧移修正, 但保持重力对抗分量
                                sign = -1.0 if (vx * cy - vy * cx) > 0 else 1.0
                                vl = sign * SLOPE_SIDE_SPEED * 0.8
                                yaw_err = normalize_angle(desired_yaw - yaw)
                                vy_cmd = max(-0.30, min(0.30, yaw_err * 2.5))
                                gl._step_run(vf=slope_anti_g_vf, vl=vl, vy=vy_cmd, dur_ms=300)
                                slope_last_x, slope_last_y = x, y
                                step += 1
                                print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                                      f"wp={idx:5d} ⚡ XY修正 cross={cross_track:.3f}m [slope]")
                                continue

                    # ── 侧移 + Y连续修正: 同时进行, 不分开 ──
                    step_m = stage['step_m']
                    a_err = normalize_angle(target_dir - yaw)
                    shift_dir = 1.0 if math.sin(a_err) > 0 else -1.0

                    y_err = ny - y
                    x_err = nx - x

                    # 基础侧移速度
                    vl = shift_dir * SLOPE_SIDE_SPEED

                    # Y连续P修正: 强增益, 与y误差成正比, 始终叠加在侧移上
                    vf = y_err * 4.0  # 高增益: 0.05m偏差→vf=0.20
                    vf = max(-SLOPE_SIDE_SPEED * 1.2, min(SLOPE_SIDE_SPEED * 1.2, vf))

                    # Y偏差大时减小侧移速度(专注修正), 但不停
                    if abs(y_err) > 0.03:
                        vl *= 0.3

                    # X方向已越过目标: 反向侧移
                    if x_err * shift_dir > 0 and abs(x_err) > 0.02:
                        vl = -shift_dir * SLOPE_SIDE_SPEED * 0.6

                    # 动态重力对抗: 检测实际后滑, 积累对抗分量
                    if slope_last_x != 0 or slope_last_y != 0:
                        dx_r = x - slope_last_x; dy_r = y - slope_last_y
                        fwd_slide = dx_r * math.cos(yaw) + dy_r * math.sin(yaw)
                        # 后滑 → 加大对抗分量; 前移正常 → 缓慢衰减
                        if fwd_slide < -0.001:
                            slope_anti_g_vf += abs(fwd_slide) * 15.0
                        else:
                            slope_anti_g_vf *= 0.92
                        slope_anti_g_vf = max(0.0, min(0.25, slope_anti_g_vf))
                    # 将动态对抗分量叠加到vf
                    vf += slope_anti_g_vf

                    desired_yaw = normalize_angle(target_dir - math.pi / 2)
                    yaw_err = normalize_angle(desired_yaw - yaw)
                    vy_cmd = max(-0.2, min(0.2, yaw_err * 2.0))
                    step_d = min(dist_tgt, step_m)
                    dur_ms = max(200, int(step_d/(SLOPE_SIDE_SPEED*1.0)*1000*1.3))
                    gl._step_run(vf=vf, vl=vl, vy=vy_cmd, dur_ms=dur_ms)
                    slope_last_x, slope_last_y = x, y
            else:
                # 斜面段全部完成, 直接跳到 FINISH 触发 jump_finish 阶段
                finish_idx = _idx('FINISH')
                if finish_idx >= 0:
                    idx = finish_idx
                    print(f"  斜面段完成, 跳至 FINISH(wp={finish_idx}), 进入下斜面")
                else:
                    idx += 1
                continue
        elif stage['gait'] == 'jump' and not jump_done:
            # ── FINISH: 直接跳跃 (kJump3d/JumpDownStair) → 落地恢复 → A1 ──

            if jump_phase == 0:
                jump_start_z = z
                jump_last_z = z
                jump_stable_cnt = 0
                gl.enable_slope_comp(force_gain=120.0, lean_gain=1.0, body_height=0.08)
                # ── 跳跃前矫正yaw到180° (面朝下坡方向) ──
                target_yaw = math.radians(180)
                yaw_err = math.degrees(normalize_angle(target_yaw - yaw))
                if abs(yaw_err) > 10.0:
                    turn_deg = max(-30, min(30, yaw_err))
                    gl.step_turn(turn_deg, rate=0.25)
                    gl._pump()
                    time.sleep(0.02)
                    print(f'  [FINISH] 矫正yaw: err={yaw_err:+5.1f}° cur={math.degrees(yaw):.0f}° → 目标180°')
                    continue
                gl.stop()
                jump_phase = 1
                print(f'  [FINISH] yaw={math.degrees(yaw):.0f}° 已矫正, 力控已开启 z0={z:.3f}, 准备跳跃...')
                continue

            elif jump_phase == 1:
                # ── 直接跳跃: mode=16 gait_id=9 kJumpDownStair ──
                gl.disable_slope_comp()
                print(f'  🚀 kJumpDownStair 触发!')
                gl.jump_down()
                jump_phase = 2
                jump_phase2_start = step
                jump_last_z = z
                jump_stable_cnt = 0
                gl.enable_slope_comp(force_gain=100.0, lean_gain=0.8, body_height=0.10)
                print(f'  ✓ 跳跃完成 z={z:.3f}, 落地恢复中...')
                continue

            elif jump_phase == 2:
                # ── 落地恢复: 每步0.15s, 给身体真实恢复时间 ──
                gl._slope_tick()
                gl._send(mode=12, gait_id=0, contact=0, step_h=0.0, pos_z=0.25)
                gl._pump()
                time.sleep(0.15)
                step += 1
                dz = abs(z - jump_last_z)
                if dz < 0.005:
                    jump_stable_cnt += 1
                else:
                    jump_stable_cnt = max(0, jump_stable_cnt - 1)
                jump_last_z = z
                if jump_stable_cnt > 15 and step - jump_phase2_start > 20:
                    jump_phase = 3
                    # 重新进入 locomotion 模式, 先站稳
                    for _ in range(20):
                        gl._send(mode=11, gait_id=0, vf=0, vl=0, vy=0, pos_z=0.25)
                        gl._pump()
                        time.sleep(0.05)
                    # 重开力控 (斜面防滑)
                    gl.enable_slope_comp(force_gain=80.0, lean_gain=0.6, body_height=0.12)
                    print(f'  ✓ 恢复完成 z={z:.3f} stable={jump_stable_cnt} steps={step}, 进入Locomotion+力控, 前往A1')
                    continue
                print(f'  [{step:5d}] ({x:6.2f},{y:6.2f}) z={z:.3f} y={math.degrees(yaw):5.0f}°  '
                      f'wp={idx:5d} ↣ 恢复 stable={jump_stable_cnt}')
                continue

            elif jump_phase == 3:
                # ── 导航到 A1 (力控防滑) ──
                a1_idx = _idx('A1')
                if a1_idx >= 0:
                    sx, sy = path[a1_idx][0], path[a1_idx][1]
                    dist_to_a1 = math.hypot(sx - x, sy - y)
                    if dist_to_a1 < 0.50:
                        jump_done = True
                        idx = nearest_index(path, x, y, a1_idx)
                        print(f'  ✓ 到达A1 d={dist_to_a1:.2f}m, 下斜面完成')
                        continue
                    gl._slope_tick()
                    tdir = math.atan2(sy - y, sx - x)
                    a_err = normalize_angle(tdir - yaw)
                    if abs(a_err) > math.radians(20):
                        turn_deg = max(-20, min(20, math.degrees(a_err)))
                        gl.step_turn(turn_deg, rate=0.35)
                    elif abs(a_err) > math.radians(8):
                        vy_cmd = max(-0.2, min(0.2, math.degrees(a_err) * 0.02))
                        gl._step_run(vf=0.04, vl=0.0, vy=vy_cmd, dur_ms=400)
                    else:
                        gl._step_run(vf=0.05, vl=0.0, vy=0.0, dur_ms=400)
                    step += 1
                    print(f'  [{step:5d}] ({x:6.2f},{y:6.2f}) z={z:.3f} y={math.degrees(yaw):5.0f}°  '
                          f'wp={idx:5d} → A1 d={dist_to_a1:.2f}m a_err={math.degrees(a_err):+.0f}°')
                else:
                    jump_done = True
                continue
        elif abs(a_err) > cfg.ANGLE_THRESH and not suppress_turn_until_2A:
            # S1_back: 先矫正朝向再后退 (背对ROCK, 面向180°)
            if stage['name'] == 'S1_back' and not s1_back_yaw_ok:
                turn_deg = max(-45, min(45, a_err))
                gl.step_turn(turn_deg, rate=0.25)
                print(f"  \u21bb S1_back\u77eb\u6b63: err={a_err:+4.1f}\u00b0 yaw={math.degrees(yaw):.0f}\u00b0 \u2192 \u76ee\u6807{math.degrees(normalize_angle(target)):.0f}\u00b0")
            # backward/high_forward: 直走不调朝向不侧移 (独木桥等场景必须直线前进)
            elif stage['gait'] in ('backward', 'high_forward'):
                execute_gait(gl, stage, dg)
            else:
                turn_deg = max(-45, min(45, a_err))
                rate = 0.35 if abs(a_err) > cfg.TURN_FAST_THRES else 0.25
                gl.step_turn(turn_deg, rate=rate)
        elif suppress_turn_until_2A:
            # 被抑制时输出简短日志以便调试
            print("  (turn suppressed until 2A)")
        else:
            if stage["name"] == 'S1_back':
                s1_back_yaw_ok = True
            execute_gait(gl, stage, dg)

        # ── 卡住 ──
        moved = math.hypot(x-last_x, y-last_y)
        total_dist += moved
        if moved < cfg.STUCK_EPS:
            stuck_cnt += 1
            if stuck_cnt > cfg.STUCK_LIMIT:
                print(f"\n⚠️  STUCK ({x:.2f},{y:.2f}) wp={idx}")
                gl.stuck_recover(); stuck_cnt = 0
        else:
            if stuck_cnt: stuck_cnt -= 1
            last_x, last_y = x, y

        step += 1
        if step % 1 == 0:  # 每一步都打印
            print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                  f"wp={idx:5d} target_wp={la:5d} a_err={a_err:+5.1f}° dg={dg:4.1f}m [{stage['name']}]")

    gl.finish()
    print(f"\nDone. steps={step} dist={total_dist:.1f}m")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_sigint)
    try:
        run()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback; traceback.print_exc()
