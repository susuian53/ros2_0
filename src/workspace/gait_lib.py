#!/usr/bin/env python3
"""
gait_lib.py — 步态积木库

基于 robot_control_cmd_lcmt 的简洁步态接口，支持连续运动和离散步态。
可用于寻迹、状态机或开环兜底。

用法:
    from gait_lib import GaitLib
    gl = GaitLib()
    gl.init()           # 站起 + 进入 locomotion
    gl.forward(0.2)     # 持续前进
    gl.step_forward()   # 前进一小步 (~8cm)
    gl.step_turn(30)    # 右转约30度
    gl.finish()         # 停车趴下
"""

import sys
import time
import math
import threading

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from simulator_lcmt import simulator_lcmt

# ── 校准参数 ─────────────────────────────────────────
MPS_PER_SEC  = 1.0
RAD_PER_SEC  = 1.0
STEP_MARGIN  = 1.3

# ── 运动原语常量 ──────────────────────────────────────
GAIT_TROT_SLOW  = 27
GAIT_TROT_MED   = 3
GAIT_WALK       = 6
GAIT_USER       = 110
CONTACT_ALL     = 15
MODE_LOCOMOTION = 11
MODE_STAND      = 12
MODE_PRONE      = 7
MODE_MOTION     = 62

_TICK_US = 20000  # 50Hz


class GaitLib:
    """步态积木库 — 机器人运动控制接口"""

    def __init__(self):
        self.lc_tx = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lc_rx = lcm.LCM()
        self._msg = robot_control_cmd_lcmt()
        self._life = 0

        self._lock = threading.Lock()
        self.x = 0.0; self.y = 0.0; self.z = 0.0
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self._valid = False
        self.lc_rx.subscribe("simulator_state", self._on_state)

    # ── 位姿 ──────────────────────────────────────────
    def _on_state(self, channel, data):
        m = simulator_lcmt().decode(data)
        with self._lock:
            self.x = m.p[0]; self.y = m.p[1]; self.z = m.p[2]
            roll_deg = math.degrees(m.rpy[0])
            is_upright = abs(abs(roll_deg) - 0) < abs(abs(roll_deg) - 180)
            self.roll  = m.rpy[0]; self.pitch = m.rpy[1]
            self.yaw   = m.rpy[2] if is_upright else m.rpy[2] + math.pi
            self._valid = True

    def _pump(self):
        self.lc_rx.handle_timeout(10)

    # ── 发送 ──────────────────────────────────────────
    def _send(self, mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
              vf=0.0, vl=0.0, vy=0.0,
              step_h=0.03, pos_z=0.20, contact=CONTACT_ALL, duration=0):
        self._life = (self._life + 1) % 127
        m = self._msg
        m.mode = mode; m.gait_id = gait_id; m.contact = contact
        m.life_count = self._life; m.duration = duration
        m.vel_des[0] = vf; m.vel_des[1] = vl; m.vel_des[2] = vy
        m.step_height[0] = step_h; m.step_height[1] = step_h
        m.pos_des[2] = pos_z
        m.rpy_des[0] = 0.0; m.rpy_des[1] = 0.0; m.rpy_des[2] = 0.0
        m.value = 0
        for i in range(3):
            m.pos_des[i] = 0.0 if i != 2 else pos_z
            m.acc_des[i] = 0.0; m.acc_des[i+3] = 0.0
            m.ctrl_point[i] = 0.0
        for i in range(6):
            m.foot_pose[i] = 0.0
        self.lc_tx.publish("robot_control_cmd", m.encode())

    # ═══════════════════════════════════════════════════
    # 初始化 / 收尾
    # ═══════════════════════════════════════════════════
    def init(self, timeout=15.0):
        """站起 → locomotion"""
        print("[GaitLib] Flushing stale commands...")
        for _ in range(10):
            self._send(mode=MODE_LOCOMOTION, vf=0, vl=0, vy=0, step_h=0.0, pos_z=0.0)
            self._pump(); time.sleep(0.05)

        print("[GaitLib] Waiting for initial pose...")
        waited = 0.0
        while not self._valid and waited < timeout:
            self._pump(); time.sleep(0.1); waited += 0.1

        print("[GaitLib] Standing up (mode=12)...")
        t0 = time.time()
        while time.time() - t0 < 7.0:
            self._send(mode=MODE_STAND, gait_id=0, contact=0, step_h=0.0, pos_z=0.0)
            self._pump(); time.sleep(0.2)

        print("[GaitLib] Settling after stand...")
        sx, sy = self.x, self.y
        for _ in range(10):
            self._send(mode=MODE_STAND, gait_id=0, contact=0, step_h=0.0, pos_z=0.0)
            self._pump(); time.sleep(0.2)
        print(f"[GaitLib]   drift={math.hypot(self.x-sx,self.y-sy):.3f}m  yaw={math.degrees(self.yaw):.0f}°")

        print("[GaitLib] Entering locomotion...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self._send(vf=0, vl=0, vy=0)
            self._pump(); time.sleep(0.2)

        print("[GaitLib] Settling in locomotion...")
        sx, sy = self.x, self.y
        for _ in range(10):
            self._send(vf=0, vl=0, vy=0)
            self._pump(); time.sleep(0.2)
        print(f"[GaitLib]   drift={math.hypot(self.x-sx,self.y-sy):.3f}m  yaw={math.degrees(self.yaw):.0f}°")

        self._pump()
        x, y, _, _, _, yaw = self.get_position()
        print(f"[GaitLib] Ready. pose=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}°")

    def finish(self):
        """停车 → 趴下"""
        print("[GaitLib] Stopping...")
        self._send(vf=0, vl=0, vy=0); time.sleep(0.6)
        print("[GaitLib] Prone...")
        self._send(mode=MODE_PRONE, gait_id=0, contact=0); time.sleep(3)
        print("[GaitLib] Finished.")

    # ═══════════════════════════════════════════════════
    # 持续运动
    # ═══════════════════════════════════════════════════
    def forward(self, speed=0.2):       self._send(vf=speed)
    def backward(self, speed=0.1):      self._send(vf=-speed)
    def turn_left(self, rate=0.5):      self._send(vy=rate)
    def turn_right(self, rate=0.5):     self._send(vy=-rate)
    def shift_left(self, speed=0.05):   self._send(vl=speed)
    def shift_right(self, speed=0.05):  self._send(vl=-speed)
    def stop(self):                     self._send()
    def low_walk(self, speed=0.15):
        self._send(mode=MODE_MOTION, gait_id=GAIT_USER, vf=speed, step_h=0.08)

    # ═══════════════════════════════════════════════════
    # 离散步态 (50Hz 持续发布，防 500ms 超时)
    # ═══════════════════════════════════════════════════
    def _step_run(self, vf=0.0, vl=0.0, vy=0.0, step_h=0.03, dur_ms=1000, pos_z=0.20):
        ticks = max(1, dur_ms * 1000 // _TICK_US)
        for _ in range(ticks):
            self._send(vf=vf, vl=vl, vy=vy, step_h=step_h, pos_z=pos_z)
            self._pump()
            time.sleep(_TICK_US / 1_000_000)
        self._send(vf=0, vl=0, vy=0); self._pump()

    def step_forward(self, distance=0.08, speed=0.2):
        if distance <= 0 or speed <= 0: return
        dur_ms = max(100, int(distance / (speed * MPS_PER_SEC) * 1000 * STEP_MARGIN))
        self._step_run(vf=speed, dur_ms=dur_ms)

    def step_backward(self, distance=0.05, speed=0.1):
        if distance <= 0 or speed <= 0: return
        dur_ms = max(100, int(distance / (speed * MPS_PER_SEC) * 1000 * STEP_MARGIN))
        self._step_run(vf=-speed, dur_ms=dur_ms)

    def step_turn(self, degrees, rate=None):
        if abs(degrees) < 1: return
        if rate is None: rate = 0.5 if abs(degrees) > 30 else 0.25
        rad = math.radians(abs(degrees))
        dur_ms = max(150, int(rad / (rate * RAD_PER_SEC) * 1000 * STEP_MARGIN))
        vy = rate if degrees > 0 else -rate
        self._step_run(vy=vy, step_h=0.05, dur_ms=dur_ms)

    def step_high_forward(self, distance=0.08, speed=0.15):
        if distance <= 0 or speed <= 0: return
        dur_ms = max(100, int(distance / (speed * MPS_PER_SEC) * 1000 * STEP_MARGIN))
        self._step_run(vf=speed, step_h=0.08, dur_ms=dur_ms)

    def crouch_step_forward(self, distance=0.06, speed=0.10):
        if distance <= 0 or speed <= 0: return
        dur_ms = max(100, int(distance / (speed * MPS_PER_SEC) * 1000 * STEP_MARGIN))
        self._step_run(vf=speed, step_h=0.08, dur_ms=dur_ms, pos_z=0.08)

    def step_shift(self, distance=0.03, speed=0.05):
        if abs(distance) < 0.005: return
        dur_ms = max(100, int(abs(distance) / (speed * MPS_PER_SEC) * 1000 * STEP_MARGIN))
        self._step_run(vl=speed if distance > 0 else -speed, dur_ms=dur_ms)

    def jump(self):
        self._send(mode=22, gait_id=0, contact=0, step_h=0.0, pos_z=0.0)
        time.sleep(0.5); self._pump()

    def announce(self, text):
        import subprocess
        try:
            subprocess.Popen(["espeak-ng", "-v", "zh", text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass
        print(f"  📢 {text}")

    # ═══════════════════════════════════════════════════
    # 位姿 / 恢复
    # ═══════════════════════════════════════════════════
    def get_position(self):
        self._pump()
        with self._lock:
            return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

    @property
    def pose_valid(self):
        with self._lock: return self._valid

    def stuck_recover(self):
        print("  [RECOVER] back 0.5s...")
        self._step_run(vf=-0.15, dur_ms=500)
