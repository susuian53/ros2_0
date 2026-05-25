#!/usr/bin/env python3
"""
scheme1_step_tracking_v2.py — 离散步态寻迹 + 斜坡力控

航向: 路径 yaw + CTE 修正。
斜坡: 非阻塞 force + body lean (via ros2 topic pub)。
"""

import sys, os, time, math, signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gait_lib_v2 import GaitLib

LOOKAHEAD=0.50; LOOKAHEAD_MIN=0.25; ANGLE_THRESH=15.0; TURN_FAST_THRES=30.0
STEP_FAR=0.10; STEP_NEAR=0.05; CURVE_ANGLE=25.0
STUCK_LIMIT=12; STUCK_EPS=0.012; GOAL_TOL=0.35
PATH_CSV="/home/cyberdog_sim/track_path_v11.csv"

S1_BACK_X_MAX=3.0; S1_BACK_Y_MAX=0.2
HIGH_STEP_Y_MIN=7.5; HIGH_STEP_Y_MAX=8.5; HIGH_STEP_X_MIN=3.0
CROUCH_Y_MIN=9.3; CROUCH_Y_MAX=10.8; CROUCH_X_MIN=1.8
SLOPE_FORCE_GAIN=100.0; SLOPE_LEAN_GAIN=0.8

running=True

def on_sigint(*_): global running; running=False; print("\n\n⚠  Ctrl+C")

def normalize_angle(a):
    while a>math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a

def load_path(csv_path):
    path=[]
    with open(csv_path) as f:
        for line in f:
            p=line.strip().split(',')
            if len(p)>=2:
                yaw=float(p[2]) if len(p)>=3 else 0.0
                path.append((float(p[0]),float(p[1]),yaw))
    print(f"[PATH] {len(path)} waypoints"); return path

def nearest_index(path,x,y,start):
    """只往前搜 200 点 (~4m), 防止交叉路口回跳"""
    start=max(0,start)
    if start>=len(path): start=0
    best,best_d=start,math.hypot(path[start][0]-x,path[start][1]-y)
    end=min(start+200,len(path))
    for i in range(start,end):
        d=math.hypot(path[i][0]-x,path[i][1]-y)
        if d<best_d: best_d=d; best=i
    return best

def lookahead_index(path,x,y,start,L):
    for i in range(start,len(path)):
        if math.hypot(path[i][0]-x,path[i][1]-y)>=L: return i
    return len(path)-1

def is_curve(path,idx,n=10):
    if idx+n>=len(path): return False
    a0=math.atan2(path[min(idx+n,len(path)-1)][1]-path[idx][1],path[min(idx+n,len(path)-1)][0]-path[idx][0])
    a1=math.atan2(path[min(idx+n+5,len(path)-1)][1]-path[min(idx+n,len(path)-1)][1],path[min(idx+n+5,len(path)-1)][0]-path[min(idx+n,len(path)-1)][0])
    return math.degrees(abs(a1-a0))>CURVE_ANGLE

def run():
    global running
    path=load_path(PATH_CSV)
    gl=GaitLib(); gl.init()
    if not gl.pose_valid: print("[ERROR] No pose"); return

    x,y,z,roll,pitch,yaw=gl.get_position()
    idx=nearest_index(path,x,y,0)
    last_x,last_y=x,y; stuck_cnt=step=0; total_dist=0.0
    slope_active=False; goal=path[-1]
    print(f"\n═══ V2 ═══")
    print(f"  Start:({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}°  Goal:({goal[0]:.2f},{goal[1]:.2f})\n")

    while running:
        x,y,z,roll,pitch,yaw=gl.get_position()

        dg=math.hypot(goal[0]-x,goal[1]-y)
        if dg<GOAL_TOL: print(f"\n🎯 GOAL! d={dg:.2f}m steps={step} dist={total_dist:.1f}m"); break

        idx=nearest_index(path,x,y,idx)
        curve=is_curve(path,idx)
        la=lookahead_index(path,x,y,idx,LOOKAHEAD_MIN if curve else LOOKAHEAD)
        tx,ty,path_yaw=path[la]

        is_backward=(x<S1_BACK_X_MAX and y<S1_BACK_Y_MAX)
        is_crouch=(CROUCH_Y_MIN<y<CROUCH_Y_MAX and x>CROUCH_X_MIN)
        is_high=(HIGH_STEP_Y_MIN<y<HIGH_STEP_Y_MAX and x>HIGH_STEP_X_MIN)
        is_slope_want=(y>12.2) if not slope_active else (y>11.8)

        # 斜坡力控
        if is_slope_want and not slope_active:
            gl.enable_slope_comp(SLOPE_FORCE_GAIN,SLOPE_LEAN_GAIN); slope_active=True
        elif not is_slope_want and slope_active:
            gl.disable_slope_comp(); slope_active=False
        if slope_active: gl._slope_tick()

        # 航向: path yaw + CTE
        nx,ny=path[idx][0],path[idx][1]
        sdx,sdy=tx-nx,ty-ny; slen=math.hypot(sdx,sdy)
        if slen>0.02: cte=(x-nx)*(-sdy/slen)+(y-ny)*(sdx/slen)
        else: cte=0.0
        cte_corr=math.atan2(cte,0.40); cte_corr=max(-0.35,min(0.35,cte_corr))
        if is_backward: cte_corr=-cte_corr

        if is_backward: target=normalize_angle(path_yaw+math.pi+cte_corr)
        else: target=normalize_angle(path_yaw+cte_corr)
        a_err=math.degrees(normalize_angle(target-yaw))

        # 执行
        if abs(a_err)>ANGLE_THRESH:
            rate=0.5 if abs(a_err)>TURN_FAST_THRES else 0.25
            gl.step_turn(a_err,rate=rate)
        elif is_slope_want:
            gl.step_forward(0.04,speed=0.08)
        elif is_backward:
            d=STEP_NEAR if curve else STEP_FAR
            if dg<1.0: d=max(0.04,d*(dg/1.0))
            gl.step_backward(d,speed=0.15)
        elif is_crouch:
            gl.crouch_step_forward(0.04,speed=0.08)
        elif is_high:
            gl.step_high_forward(0.06,speed=0.12)
        else:
            d=STEP_NEAR if curve else STEP_FAR
            if dg<1.0: d=max(0.04,d*(dg/1.0))
            if curve: gl.step_high_forward(d,speed=0.15)
            else: gl.step_forward(d,speed=0.2)

        moved=math.hypot(x-last_x,y-last_y); total_dist+=moved
        if moved<STUCK_EPS:
            stuck_cnt+=1
            if stuck_cnt>STUCK_LIMIT:
                print(f"\n⚠️  STUCK ({x:.2f},{y:.2f})"); gl.stuck_recover(); stuck_cnt=0
        else:
            if stuck_cnt: stuck_cnt-=1
            last_x,last_y=x,y

        step+=1
        if step%20==0:
            d="SLOPE" if slope_active else ("HIGH" if is_high else ("CROUCH" if is_crouch else ("BACK" if is_backward else ("CRV" if curve else "FWD"))))
            print(f"  [{step:5d}] ({x:6.2f},{y:6.2f}) y={math.degrees(yaw):5.0f}°  "
                  f"wp={idx:5d} a_err={a_err:+5.1f}° dg={dg:4.1f}m [{d}]")

    if slope_active: gl.disable_slope_comp()
    gl.finish()
    print(f"\nDone. steps={step} dist={total_dist:.1f}m")

if __name__=="__main__":
    signal.signal(signal.SIGINT,on_sigint)
    try: run()
    except Exception as e: print(f"\n[ERROR] {e}"); import traceback; traceback.print_exc()
