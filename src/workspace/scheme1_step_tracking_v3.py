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
                gl.enable_slope_comp()
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
        in_sexta = (START2_i >= 0 and IDX_2 >= 0 and (START2_i <= idx <= IDX_2 or sexta_phase > 0))
        if in_sexta and IDX_1 >= 0 and IDX_2 >= 0:

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
                    # 到达: 距离和xy偏差都达标
                    if dist < 0.03 and err_x < 0.02 and err_y < 0.02:
                        gl.stop()
                        u_sub_phase = 1
                        print(f"  ✅ 到达(0.19,14.67). 位置:({x:.3f},{y:.3f}), 左侧移0.1m.")
                        continue
                    # 靠近目标: 小步微调, 先转向再前进
                    if dist < 0.10:
                        if abs(err) > 5.0:
                            turn_deg = max(-30, min(30, err))
                            gl.step_turn(turn_deg, rate=0.20)
                            gl._pump(); time.sleep(0.02)
                            print(f"  ↻ U-微调(0.19,14.67): err={err:+4.1f}° pos=({x:.3f},{y:.3f}) d={dist:.3f}")
                            continue
                        step = min(0.03, dist + 0.01)
                        gl.step_forward(step, speed=0.08)
                        gl._pump()
                        print(f"  → U-微进(0.19,14.67): pos=({x:.3f},{y:.3f}) d={dist:.3f}")
                        continue
                    # 远离目标: 正常转向+前进
                    if abs(err) > 5.0:
                        turn_deg = max(-45, min(45, err))
                        gl.step_turn(turn_deg, rate=0.25)
                        gl._pump(); time.sleep(0.02)
                        print(f"  ↻ U-转向(0.19,14.67): err={err:+4.1f}° pos=({x:.3f},{y:.3f})")
                        continue
                    gl.step_forward(0.05, speed=0.20)
                    gl._pump()
                    print(f"  → U-前进→(0.19,14.67): pos=({x:.3f},{y:.3f}) dist={dist:.3f}m")
                    continue

                # 1b: 左侧移 0.10m (单次执行)
                if u_sub_phase == 1:
                    gl.step_shift(0.10, speed=0.06)
                    gl._pump()
                    u_sub_phase = 2
                    print(f"  ✅ U-左侧移0.1m完成. 位置:({x:.3f},{y:.3f}), 转向0°.")
                    continue

                # 1c: 转向到 yaw=0°
                if u_sub_phase == 2:
                    err_to_zero = math.degrees(normalize_angle(0.0 - yaw))
                    if abs(err_to_zero) > 5.0:
                        turn_deg = max(-45, min(45, err_to_zero))
                        gl.step_turn(turn_deg, rate=0.25)
                        print(f"  ↻ U-turn2zero: err={err_to_zero:+4.1f}° pos=({x:.3f},{y:.3f})")
                        gl._pump(); time.sleep(0.02)
                        continue
                    gl.stop()
                    u_sub_phase = 3
                    print(f"  ✅ 转向完成 yaw≈0°. 前往(2.50,14.85),目标yaw≈10°.")
                    continue

                # 1d: 前往 (2.50, 14.85), yaw≈10°
                if u_sub_phase == 3:
                    tx, ty = 2.50, 14.85
                    target_yaw = math.radians(10)
                    target_angle = math.atan2(ty - y, tx - x)
                    err = math.degrees(normalize_angle(target_angle - yaw))
                    dist = math.hypot(tx - x, ty - y)
                    if dist < 0.10:
                        # 到位后微调yaw到10°
                        err10 = math.degrees(normalize_angle(target_yaw - yaw))
                        if abs(err10) > 3.0:
                            turn_deg = max(-30, min(30, err10))
                            gl.step_turn(turn_deg, rate=0.20)
                            gl._pump(); time.sleep(0.02)
                            print(f"  ↻ U-调yaw→10°: err={err10:+3.1f}° pos=({x:.3f},{y:.3f})")
                            continue
                        gl.stop()
                        u_sub_phase = 4
                        u_wait_start = time.time()
                        print(f"  ✅ 到达(2.50,14.85) yaw≈10°. 等待20s.")
                        continue
                    if abs(err) > 5.0:
                        turn_deg = max(-45, min(45, err))
                        gl.step_turn(turn_deg, rate=0.25)
                        gl._pump(); time.sleep(0.02)
                        print(f"  ↻ U-转向(2.50,14.85): err={err:+4.1f}° pos=({x:.3f},{y:.3f})")
                        continue
                    gl.step_forward(0.05, speed=0.20)
                    gl._pump()
                    print(f"  → U-前往(2.50,14.85): pos=({x:.3f},{y:.3f}) dist={dist:.3f}m")
                    continue

                # 1e: 等待 20s
                if u_sub_phase == 4:
                    elapsed = time.time() - u_wait_start
                    if elapsed < 20.0:
                        gl._pump()
                        if int(elapsed) != int(elapsed - 0.1):
                            print(f"  ⏳ U-等待: {elapsed:.0f}s / 20s")
                        time.sleep(0.5)
                        continue
                    u_sub_phase = 5
                    u_start_x, u_start_y = x, y
                    print(f"  ✅ 等待20s完成. 右侧移到(2.50,13.50)，以y判断.")
                    continue

                # 1f: 右侧移到 (2.50, 13.50), 判断y
                if u_sub_phase == 5:
                    tx, ty = 2.50, 13.50
                    err_y = abs(ty - y)
                    if err_y < 0.03:
                        gl.stop()
                        u_sub_phase = 6
                        u_start_x, u_start_y = x, y
                        print(f"  ✅ 右移完成 y={y:.3f}. 后退寻路到(2.0,12.80).")
                        continue
                    gl.step_shift(-0.03, speed=0.06)
                    gl._pump()
                    print(f"  ⇉ U-右移→(2.50,13.50): y_err={err_y:.3f}m pos=({x:.3f},{y:.3f})")
                    continue

                # 1g: 后退+旋转寻路到 (2.0, 12.80)
                if u_sub_phase == 6:
                    tx, ty = 2.0, 12.80
                    back_angle = normalize_angle(math.atan2(ty - y, tx - x) + math.pi)
                    err = math.degrees(normalize_angle(back_angle - yaw))
                    dist = math.hypot(tx - x, ty - y)
                    if dist < 0.10:
                        gl.stop()
                        u_sub_phase = 7
                        u_start_x, u_start_y = x, y
                        print(f"  ✅ 到达(2.0,12.80). 右侧移到(3.50,12.80).")
                        continue
                    if abs(err) > 5.0:
                        turn_deg = max(-45, min(45, err))
                        gl.step_turn(turn_deg, rate=0.25)
                        gl._pump(); time.sleep(0.02)
                        print(f"  ↻ U-后退寻路(2.0,12.80): err={err:+4.1f}° pos=({x:.3f},{y:.3f})")
                        continue
                    gl.step_backward(0.05, speed=0.10)
                    gl._pump()
                    print(f"  ← U-后退→(2.0,12.80): pos=({x:.3f},{y:.3f}) dist={dist:.3f}m")
                    continue

                # 1h: 右侧移到 (3.50, 12.80)
                if u_sub_phase == 7:
                    tx, ty = 3.50, 12.80
                    err_y = abs(ty - y)
                    if err_y < 0.03:
                        gl.stop()
                        u_sub_phase = 8
                        print(f"  ✅ 右移完成 y={y:.3f}. 位置:({x:.3f},{y:.3f}), 左侧移0.2m.")
                        continue
                    gl.step_shift(-0.03, speed=0.06)
                    gl._pump()
                    print(f"  ⇉ U-右移→(3.50,12.80): y_err={err_y:.3f}m pos=({x:.3f},{y:.3f})")
                    continue

                # 1i: 左侧移 0.20m (单次执行)
                if u_sub_phase == 8:
                    gl.step_shift(0.20, speed=0.06)
                    gl._pump()
                    u_sub_phase = 9
                    print(f"  ✅ U-左侧移0.2m完成. 位置:({x:.3f},{y:.3f}), 转向0°.")
                    continue

                # 1j: 转向到 yaw=0°
                if u_sub_phase == 9:
                    err_to_zero = math.degrees(normalize_angle(0.0 - yaw))
                    if abs(err_to_zero) > 5.0:
                        turn_deg = max(-45, min(45, err_to_zero))
                        gl.step_turn(turn_deg, rate=0.25)
                        print(f"  ↻ U-turn2zero: err={err_to_zero:+4.1f}° pos=({x:.3f},{y:.3f})")
                        gl._pump(); time.sleep(0.02)
                        continue
                    gl.stop()
                    u_sub_phase = 10
                    print(f"  ✅ 转向完成 yaw≈0°. 位置:({x:.3f},{y:.3f})")
                    continue

                # 1k: 停止
                if u_sub_phase == 10:
                    gl.stop()
                    gl._pump()
                    print("\n  🎯 U函数完成。")
                    print(f"     位置: ({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}°")
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
        if abs(a_err) > cfg.ANGLE_THRESH and not suppress_turn_until_2A:
            # 单次旋转不超过 45°, 分多步完成防摔倒
            turn_deg = max(-45, min(45, a_err))
            rate = 0.35 if abs(a_err) > cfg.TURN_FAST_THRES else 0.25
            if stage['gait'] == 'slope':
                gl.step_turn_low(turn_deg, rate=rate)
            else:
                gl.step_turn(turn_deg, rate=rate)
        elif suppress_turn_until_2A:
            # 被抑制时输出简短日志以便调试
            print("  (turn suppressed until 2A)")
        else:
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
