#!/usr/bin/env python3
"""
gait_lib_v2.py — 步态积木库 V2 (+斜坡力控 +身体倾斜)

继承 gait_lib 全部功能，新增:
  enable_slope_comp()   — 开启 IMU 反馈力控 + 身体倾斜
  disable_slope_comp()  — 关闭
  _slope_tick()         — 斜坡补偿心跳 (节流每5步)

力控通过 ros2 topic pub 施加 (非阻塞 subprocess.Popen)。
"""

import sys, os, time, math, threading, subprocess

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from simulator_lcmt import simulator_lcmt
from file_send_lcmt import file_send_lcmt

# ── 与原 gait_lib 相同的常量 ─────────────────────────
MPS_PER_SEC  = 1.0
RAD_PER_SEC  = 1.0
STEP_MARGIN  = 1.3
GAIT_TROT_SLOW = 27; GAIT_TROT_MED = 3; GAIT_WALK = 6; GAIT_USER = 110
CONTACT_ALL = 15
MODE_LOCOMOTION = 11; MODE_STAND = 12; MODE_PRONE = 7; MODE_MOTION = 62
_TICK_US = 20000

# ── ROS2 source ─────────────────────────
def _source_env():
    return ("source /opt/ros/galactic/setup.bash && "
            "source /home/cyberdog_sim/install/setup.bash && ")

class GaitLib:
    """步态积木库 V2"""

    def __init__(self):
        self.lc_tx = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lc_rx = lcm.LCM()
        self._msg = robot_control_cmd_lcmt()
        self._life = 0
        self._lock = threading.Lock()
        self.x = 0.0; self.y = 0.0; self.z = 0.0
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.sim_time = 0.0
        self._valid = False
        self.lc_rx.subscribe("simulator_state", self._on_state)

    def _on_state(self, channel, data):
        m = simulator_lcmt().decode(data)
        with self._lock:
            self.sim_time = m.time
            self.x = m.p[0]; self.y = m.p[1]; self.z = m.p[2]
            roll_deg = math.degrees(m.rpy[0])
            is_upright = abs(abs(roll_deg) - 0) < abs(abs(roll_deg) - 180)
            self.roll = m.rpy[0]; self.pitch = m.rpy[1]
            self.yaw = m.rpy[2] if is_upright else m.rpy[2] + math.pi
            self._valid = True

    def _pump(self):
        self.lc_rx.handle_timeout(10)

    def sleep_sim(self, duration):
        """等待仿真时间流逝指定的秒数"""
        self._pump()
        with self._lock:
            t0 = self.sim_time
        while True:
            self._pump()
            with self._lock:
                if self.sim_time - t0 >= duration:
                    break
            time.sleep(0.01)

    def _send(self, mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW,
              vf=0.0, vl=0.0, vy=0.0, step_h=0.03, pos_z=0.20,
              contact=CONTACT_ALL, duration=0):
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
        for i in range(6): m.foot_pose[i] = 0.0
        self.lc_tx.publish("robot_control_cmd", m.encode())

    # ═══════════════════════════════════════════════════
    # init / finish / 持续运动 / 离散步态 — 同 gait_lib
    # ═══════════════════════════════════════════════════
    def init(self, timeout=15.0):
        print("[GaitLibV2] Initializing...")
        self._load_user_gait()
        
        print("[GaitLibV2] Waiting for pose...")
        waited = 0.0
        while not self._valid and waited < timeout:
            self._pump(); time.sleep(0.1); waited += 0.1
        
        if not self._valid:
            print("[GaitLibV2] ERROR: Timeout waiting for pose!")
            return

        # 检查仿真时间是否在走
        with self._lock:
            t_start_sim = self.sim_time
        time.sleep(0.5)
        self._pump()
        with self._lock:
            sim_time_working = (self.sim_time > t_start_sim)
        
        if not sim_time_working:
            print("[GaitLibV2] WARNING: sim_time is static, falling back to real-time for init")

        print("[GaitLibV2] Standing up (MODE_STAND)...")
        z_before_stand = self.z
        t0_sim = self.sim_time
        t0_real = time.time()
        while True:
            self._send(mode=MODE_STAND, gait_id=0, contact=0, step_h=0.0, pos_z=0.25)
            self._pump()
            with self._lock:
                dz = self.z - z_before_stand
                # 判定标准：身体相对起始位置抬升 > 0.06m (真站立而非在斜坡/桥面等高位)
                if dz > 0.06: break
                if sim_time_working and (self.sim_time - t0_sim > 5.0): break
                if not sim_time_working and (time.time() - t0_real > 10.0): break
            time.sleep(0.05)

        print("[GaitLibV2] Entering locomotion (MODE_LOCOMOTION)...")
        t0_sim = self.sim_time
        t0_real = time.time()
        while True:
            self._send(mode=MODE_LOCOMOTION, vf=0, vl=0, vy=0, pos_z=0.25)
            self._pump()
            with self._lock:
                # 仿真时间 1.5s 或 现实时间 3s
                if sim_time_working and (self.sim_time - t0_sim > 1.5): break
                if not sim_time_working and (time.time() - t0_real > 3.0): break
            time.sleep(0.05)
            
        self._pump()
        x, y, _, _, _, yaw = self.get_position()
        print(f"[GaitLibV2] ✓ Ready to track. pose=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}°")

    def finish(self):
        self._send(vf=0, vl=0, vy=0); time.sleep(0.6)
        self._send(mode=MODE_PRONE, gait_id=0, contact=0); time.sleep(3)
        print("[GaitLibV2] Finished.")

    def forward(self, speed=0.2):       self._send(vf=speed)
    def backward(self, speed=0.1):      self._send(vf=-speed)
    def turn_left(self, rate=0.5):      self._send(vy=rate)
    def turn_right(self, rate=0.5):     self._send(vy=-rate)
    def shift_left(self, speed=0.05):   self._send(vl=speed)
    def shift_right(self, speed=0.05):  self._send(vl=-speed)
    def stop(self):                     self._send()

    def _load_user_gait(self):
        """加载自定义步态参数 (mode=62 gait=110 低姿态高抬腿) — 无阻塞"""
        workdir = os.path.dirname(os.path.abspath(__file__))
        def_file = os.path.join(workdir, "usergait_def.toml")
        param_file = os.path.join(workdir, "usergait_param_full.toml")
        if not os.path.exists(def_file) or not os.path.exists(param_file):
            print("[GaitLibV2] WARNING: user_gait files not found")
            return
        msg = file_send_lcmt()
        for fpath in [def_file, param_file]:
            with open(fpath, 'r') as f:
                msg.data = f.read()
            self.lc_tx.publish("user_gait_file", msg.encode())
        print("[GaitLibV2] User gait loaded (mode=62)")

    def _load_gecko_slope_gait(self):
        """加载壁虎斜坡爬行步态 — Z字下扎足端 + 四点支撑wave gait"""
        workdir = os.path.dirname(os.path.abspath(__file__))
        def_file = os.path.join(workdir, "gecko_slope_def.toml")
        param_file = os.path.join(workdir, "gecko_slope_param.toml")
        if not os.path.exists(def_file) or not os.path.exists(param_file):
            print("[GaitLibV2] WARNING: gecko_slope gait files not found")
            return
        msg = file_send_lcmt()
        for fpath in [def_file, param_file]:
            with open(fpath, 'r') as f:
                msg.data = f.read()
            self.lc_tx.publish("user_gait_file", msg.encode())
        print("[GaitLibV2] Gecko slope gait loaded")

    def _restore_default_gait(self):
        """恢复默认 user gait 参数"""
        self._load_user_gait()

    def low_walk(self, speed=0.15):
        """低姿态高抬腿行走 (过限高杆, mode=62)"""
        self._send(mode=MODE_MOTION, gait_id=GAIT_USER, vf=speed, step_h=0.08, pos_z=0.12)

    def _step_run(self, vf=0.0, vl=0.0, vy=0.0, step_h=0.03, dur_ms=1000, pos_z=0.20,
                  mode=MODE_LOCOMOTION, gait_id=GAIT_TROT_SLOW):
        """执行一个步态阶段，时间以仿真时间为准"""
        self._pump()
        with self._lock:
            t0_sim = self.sim_time
        t0_real = time.time()
        dur_s = dur_ms / 1000.0
        
        # 预检查仿真时钟是否可用
        time.sleep(0.05); self._pump()
        with self._lock:
            sim_clock_working = (self.sim_time > t0_sim)

        while True:
            self._send(mode=mode, gait_id=gait_id, vf=vf, vl=vl, vy=vy, step_h=step_h, pos_z=pos_z)
            self._pump()
            with self._lock:
                # 仿真时间到达或现实时间超时(防止时钟卡死)
                if sim_clock_working and (self.sim_time - t0_sim >= dur_s):
                    break
                if not sim_clock_working and (time.time() - t0_real >= dur_s * 1.5):
                    break
            time.sleep(0.02)
        
        for _ in range(3):
            self._send(mode=mode, gait_id=gait_id, vf=0, vl=0, vy=0, step_h=step_h, pos_z=pos_z)
            self._pump(); time.sleep(0.02)

    def step_forward(self, distance=0.08, speed=0.2):
        if distance <= 0 or speed <= 0: return
        self._step_run(vf=speed, dur_ms=max(100, int(distance/(speed*MPS_PER_SEC)*1000*STEP_MARGIN)))

    def low_forward(self, distance=0.06, speed=0.10):
        """低姿态高抬腿前进：距离制接口。"""
        self.crouch_step_forward(distance=distance, speed=speed)

    def side_step(self, distance=0.03, speed=0.05):
        """侧着走：distance>0 右移，distance<0 左移。"""
        self.step_shift(distance=distance, speed=speed)

    def forward_step(self, distance=0.08, speed=0.2):
        """普通前进：距离制接口。"""
        self.step_forward(distance=distance, speed=speed)

    def step_backward(self, distance=0.05, speed=0.1):
        if distance <= 0 or speed <= 0: return
        self._step_run(vf=-speed, dur_ms=max(100, int(distance/(speed*MPS_PER_SEC)*1000*STEP_MARGIN)))
    def step_turn(self, degrees, rate=None):
        if abs(degrees) < 1: return
        if rate is None: rate = 0.5 if abs(degrees) > 30 else 0.25
        rad = math.radians(abs(degrees))
        vy = rate if degrees > 0 else -rate
        vl_kick = rate * 0.5 if degrees > 0 else -rate * 0.5
        # 先向转向侧踢腿 (侧移), 再旋转
        self._step_run(vl=vl_kick, vy=vy * 0.3, step_h=0.06, dur_ms=100)
        self._step_run(vy=vy, step_h=0.05, dur_ms=max(150, int(rad/(rate*RAD_PER_SEC)*1000*STEP_MARGIN)))
    def step_turn_low(self, degrees, rate=None):
        """低姿旋转 (限高杆/斜坡区域, mode=62 gait=110 保持低重心)"""
        if abs(degrees) < 1: return
        if rate is None: rate = 0.3 if abs(degrees) > 30 else 0.2
        rad = math.radians(abs(degrees))
        vy = rate if degrees > 0 else -rate
        vl_kick = rate * 0.5 if degrees > 0 else -rate * 0.5
        dur_ms = max(150, int(rad/(rate*RAD_PER_SEC)*1000*STEP_MARGIN))
        self._step_run(mode=MODE_MOTION, gait_id=GAIT_USER, vl=vl_kick, vy=vy * 0.3,
                       step_h=0.06, pos_z=0.10, dur_ms=100)
        self._step_run(mode=MODE_MOTION, gait_id=GAIT_USER, vy=vy,
                       step_h=0.06, pos_z=0.10, dur_ms=dur_ms)
    def step_high_forward(self, distance=0.08, speed=0.15):
        if distance <= 0 or speed <= 0: return
        self._step_run(vf=speed, step_h=0.08, dur_ms=max(100, int(distance/(speed*MPS_PER_SEC)*1000*STEP_MARGIN)))
    def crouch_step_forward(self, distance=0.06, speed=0.10):
        """匍匐前进 (mode=62 gait=110 低姿态高抬腿)"""
        if distance <= 0 or speed <= 0: return
        dur_ms = max(300, int(distance/(speed*MPS_PER_SEC)*1000*STEP_MARGIN))
        self._step_run(mode=MODE_MOTION, gait_id=GAIT_USER, vf=speed,
                       step_h=0.08, pos_z=0.12, dur_ms=dur_ms)

    def slope_step_forward(self, distance=0.04, speed=0.06):
        """斜坡防滑: 壁虎爬行步态(Z字下扎+wave gait三点支撑) + IMU力控"""
        if distance <= 0 or speed <= 0: return
        dur_ms = max(500, int(distance/(speed*MPS_PER_SEC)*1000*STEP_MARGIN))
        self._step_run(vf=speed, dur_ms=dur_ms) # 假设 slope 模式已通过 gait_id 或外部配置生效
    def slide_to_target(self, geo_angle, dg, speed=0.05, back_speed=0.0):
        """斜面侧移: 根据目标方向角分解为前进/后退+侧移分量, 不动朝向.
        geo_angle: 目标方向角(rad), dg: 到目标距离(m)
        back_speed: 向后速度分量, 用于远离边缘"""
        if dg <= 0.001 or speed <= 0: return
        x, y, z, roll, pitch, yaw = self.get_position()
        a_err = geo_angle - yaw
        vf = speed * math.cos(a_err) - back_speed
        vl = speed * math.sin(a_err)
        dur_ms = max(150, int(dg/(speed*MPS_PER_SEC)*1000*STEP_MARGIN))
        self._step_run(vf=vf, vl=vl, dur_ms=dur_ms)
    def step_shift(self, distance=0.03, speed=0.05):
        if abs(distance) < 0.005: return
        vl = speed if distance > 0 else -speed
        self._step_run(vl=vl, dur_ms=max(100, int(abs(distance)/(speed*MPS_PER_SEC)*1000*STEP_MARGIN)))
    def jump(self):
        self._send(mode=22, gait_id=0, contact=0, step_h=0.0, pos_z=0.0)
        time.sleep(0.5); self._pump()
    def announce(self, text):
        try:
            subprocess.Popen(["espeak-ng", "-v", "zh", text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError: pass
        print(f"  📢 {text}")

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

    # ═══════════════════════════════════════════════════
    # 斜坡力控 V2 (非阻塞版)
    # ═══════════════════════════════════════════════════
    def _apply_force(self, fx=0.0, fy=0.0, fz=0.0, duration=0.5, link="base_link"):
        # 限幅: 单方向不超过 30N (防弹飞)
        fx = max(-30, min(30, fx))
        fy = max(-30, min(30, fy))
        if abs(fx) < 0.5 and abs(fy) < 0.5:
            return  # 力太小不发, 减少 ROS2 开销
        src = _source_env()
        cmd = (
            f"{src}"
            f"ros2 topic pub -1 /apply_force cyberdog_msg/msg/ApplyForce "
            f"\"{{link_name: '{link}', force: [{fx:.1f},{fy:.1f},{fz:.1f}], "
            f"rel_pos: [0.0,0.0,0.0], time: {duration:.1f}}}\""
        )
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _set_body_lean(self, roll_des=0.0, pitch_des=0.0, height_des=0.15):
        src = _source_env()
        cmd = (
            f"{src}"
            f"ros2 topic pub -1 /yaml_parameter cyberdog_msg/msg/YamlParam "
            f"\"{{name: des_roll_pitch_height, kind: 3, "
            f"vecxd_value: [{roll_des:.3f},{pitch_des:.3f},{height_des:.3f},"
            f"0,0,0,0,0,0,0,0,0], is_user: 1}}\""
        )
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def enable_slope_comp(self, force_gain=100.0, lean_gain=0.8, body_height=0.08):
        self._slope_comp = True
        self._slope_force_gain = force_gain
        self._slope_lean_gain = lean_gain
        self._filt_roll = self.roll; self._filt_pitch = self.pitch
        self._filt_alpha = 0.4; self._slope_tick_cnt = 0
        self._slope_body_height = body_height
        self._set_body_lean(0, 0, body_height)
        print(f"[GaitLibV2] Slope ON (force={force_gain} lean={lean_gain} body_h={body_height:.2f})")

    def disable_slope_comp(self):
        self._slope_comp = False
        self._apply_force(0, 0, 0, 0.1)
        self._set_body_lean(0, 0, 0.20)
        print("[GaitLibV2] Slope OFF")

    def _slope_tick(self):
        if not getattr(self, '_slope_comp', False): return
        self._slope_tick_cnt = getattr(self, '_slope_tick_cnt', 0) + 1
        if self._slope_tick_cnt % 5 != 0: return  # 节流
        a = self._filt_alpha
        self._filt_roll  = a*self.roll  + (1-a)*self._filt_roll
        self._filt_pitch = a*self.pitch + (1-a)*self._filt_pitch
        Kf = self._slope_force_gain; Kl = self._slope_lean_gain
        fx = -Kf * math.sin(self._filt_pitch)
        fy = -Kf * math.sin(self._filt_roll)
        lr =  Kl * math.sin(self._filt_roll)
        lp =  Kl * math.sin(self._filt_pitch)
        self._apply_force(fx, fy, 0, 0.5)
        self._set_body_lean(lr, lp, self._slope_body_height)
