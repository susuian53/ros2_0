#添加lcm模块
import sys
sys.path.append("/home/lcm")
sys.path.append("./lcm")

import lcm
import time
import toml
import copy
import math
import threading
import os
import numpy as np
import cv2
from Robot_Ctrl import Robot_Ctrl
from Msg_receive import Pos_msg, Gait_msg
from user_pub import user_pub
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from identify import QRcode,get_ready,arrow,yellow_wait,yellow_light
flags ={
    "ENDING_FLAG1" : False,# 起点前进标志
    "ENDING_FLAG2" : False,# A区配货标志
    "ENDING_FLAG3" : False,# A->B过S弯标志
    "ENDING_FLAG4" : False,# 前往B区标志
    "ENDING_FLAG5" : False,# B区卸货装货标志
    "ENDING_FLAG6" : False,# B区完成回返A区标志
    "ENDING_FLAG7" : False,# B->A过S弯标志
    "ENDING_FLAG8" : False,# 卸货前进入A区标志
    "ENDING_FLAG9" : False,# A区卸货标志
    "ENDING_FLAG10" : False,# 结束标志
    # A区装货标志
    "A_LOAD_ENDING_FLAG1" : False,
    "A_LOAD_ENDING_FLAG2" : False,
    "A_LOAD_ENDING_FLAG3" : False,
    "A_LOAD_ENDING_FLAG4" : False,
    "A_LOAD_ENDING_FLAG5" : False,
    # A区卸货标志
    "A_UNLOAD_ENDING_FLAG1" : False,
    "A_UNLOAD_ENDING_FLAG2" : False,
    "A_UNLOAD_ENDING_FLAG3" : False,
    "A_UNLOAD_ENDING_FLAG4" : False,
    "A_UNLOAD_ENDING_FLAG5" : False,
    "A_UNLOAD_ENDING_FLAG6" : False,
    "A_UNLOAD_ENDING_FLAG7" : False,
    # S弯标志
    "S_ENDING_FLAG1" : False,
    "S_ENDING_FLAG2" : False,
    "S_ENDING_FLAG3" : False,
    "S_ENDING_FLAG4" : False,
    "S_ENDING_FLAG5" : False,
    "S_ENDING_FLAG6" : False,
    # S弯回程标志
    "BACK_S_ENDING_FLAG1" : False,
    "BACK_S_ENDING_FLAG2" : False,
    "BACK_S_ENDING_FLAG3" : False,
    "BACK_S_ENDING_FLAG4" : False,
    "BACK_S_ENDING_FLAG5" : False,
    "BACK_S_ENDING_FLAG6" : False,
    # 前往B区标志
    "GO_ENDING_FLAG1" : False,
    "GO_ENDING_FLAG2" : False,
    "GO_ENDING_FLAG3" : False,
    "GO_ENDING_FLAG4" : False,
    "GO_ENDING_FLAG5" : False,
    "GO_ENDING_FLAG6" : False,
    # B区回返标志
    "BACK_ENDING_FLAG1" : False,
    "BACK_ENDING_FLAG2" : False,
    "BACK_ENDING_FLAG3" : False,
    "BACK_ENDING_FLAG4" : False,
    "BACK_ENDING_FLAG5" : False,
    "BACK_ENDING_FLAG6" : False,
    # S弯标志
    "is_s_direction_ready0" : False,
    "is_s_direction_ready1" : False,
    "is_s_direction_ready2" : False,
    "is_s_direction_ready3" : False,
    # 箭头识别标志
    "ARROW_executed" : False,
    # 黄灯识别标志
    "LIGHT_executed" : False,
    # 黄灯停止标志
    "WAIT_executed" : False,
    # B区卸货标志
    "unload_b_flag1" : False,
    "unload_b_flag2" : False,
    "unload_b_flag3" : False,
    # B区装货标志
    "load_b_flag1" : False,
    # 配送状态标志
    "is_ready_load_a" : False,# A区是否装货完成
    "is_ready_load_b" : False,# B区是否装货完成
    "is_ready_unload_a" : False,# A区是否卸货完成
    "is_ready_unload_b" : False,# B区是否卸货完成
}
results = {
    "A_QR" : 0,# A区二维码识别结果（装货库位）  A-1：1   A-2：2
    "B_QR" : 0,# B区二维码识别结果（卸货库位）  B-1：3   B-2：4
    "ARROW" : 0,# 箭头识别结果  朝左：1     朝右：2
    "yellow_light": 0,# 黄灯识别标志    未识别：0   已识别：1
    "ready_wait" : 0,# 黄灯停止标志     未等待：0   已等待：1
    "ready_load_a" : 0,# A区装货标志     未装货：0   已装货：1
    "ready_unload_a" : 0# A区卸货标志     未卸货：0   已卸货：1
}
original_distance = 100 # 黄灯初始距离
light_distance = 100 # 黄灯距离

# 根据位置和标志决定狗子执行的动作
def select_step_based_on_position(position,gait_mode,rpy):
    global flags,results
    x, y, z = position
    gait, mode = gait_mode
    rpy=rpy
    if gait==0 and mode==0 or gait ==1 and mode==9:
        return 0
    else:
        # 起点前进
        if x< 0.6 and flags["ENDING_FLAG1"] == False:
            return walk_0(rpy)
        
        # 右转A区配货
        elif -1<=x<2.5 and -2.5<=y<0.16 and flags["ENDING_FLAG2"] == False:
            flags["ENDING_FLAG1"] = True
            if -0.3 <y and flags["is_ready_load_a"] == False:
                if rpy>275 or rpy<90:
                    return 12
                else:
                    return walk_270(rpy)
            else:
                if results["A_QR"] == 0:
                    results["A_QR"] = QRcode()
                    return walk_270(rpy)
                else:
                    return load_a(position,gait_mode,rpy,results["A_QR"])
        # 进入三环
        elif y< 5.47 and flags["ENDING_FLAG3"] == False:
            flags["ENDING_FLAG2"] = True
            return pass_s_and_identify_arrow(position,rpy)

        # 前往B区道路（走斜坡、黄灯）
        elif y< 15 and results["ARROW"] ==2 and flags["ENDING_FLAG4"] == False:
            flags["ENDING_FLAG3"] = True
            return go_right(position,rpy)
        
        # B区卸货装货
        elif y > 14 and flags["ENDING_FLAG5"] == False:
            flags["ENDING_FLAG4"] = True
            return unload_b(position,gait_mode,rpy,results["B_QR"])
        
        # B区完成回返A区
        elif y>4.55 and flags["ENDING_FLAG6"] == False:
            flags["ENDING_FLAG5"] = True
            return back_left(position,rpy)
        
        # 进入三环
        elif (y>0.5 or (x>1.2 and y>0.1)) and flags["ENDING_FLAG7"] == False:
            flags["ENDING_FLAG6"] = True
            return pass_s_back(position,rpy)
        
        # 左转进入A区
        elif y>-0.5 and flags["ENDING_FLAG8"] == False:
            flags["ENDING_FLAG7"] = True
            if x>1.1:
                return walk_180(rpy)
            elif rpy<265:
                return 16
            else:
                return walk_270(rpy)
        
        # A区卸货
        elif -2.5<y<=-0.5 and flags["ENDING_FLAG9"] == False:
            flags["ENDING_FLAG8"] = True
            return unload_a(position,gait_mode,rpy,results["A_QR"])
        
        # 结束
        elif flags["ENDING_FLAG10"] == False:
            flags["ENDING_FLAG9"] = True
            if rpy<175:
                return 19
            elif x>-0.15 :
                return walk_180(rpy)
            else:
                flags["ENDING_FLAG10"] = True
                return 4
        else:
            return 4
        
        
# 转向0°方向前进
def walk_0(rpy):
    if rpy<2 or rpy>358:
        return 1
    elif 5<=rpy<180:
        return 15 # 快右转
    elif 180<=rpy<355:
        return 14 # 快左转
    elif 2<=rpy<5:
        return 3 # 右转
    else:
        return 2 # 左转

# 转向90°方向前进
def walk_90(rpy):
    if 88<rpy<92:
        return 1
    elif 92<=rpy<95:
        return 3 # 右转
    elif 95<=rpy<270:
        return 15 # 快右转
    elif 270<=rpy<=360 or 0<=rpy<85:
        return 14 # 快左转
    else:
        return 2 # 左转
    
# 转向90°方向前进
def walk_90_fast(rpy):
    if 88<rpy<92:
        return 28
    elif 92<=rpy<95:
        return 3 # 右转
    elif 95<=rpy<270:
        return 15 # 快右转
    elif 270<=rpy<=360 or 0<=rpy<85:
        return 14 # 快左转
    else:
        return 2 # 左转
    
# 转向180°方向前进
def walk_180(rpy):
    if 178<rpy<182:
        return 1
    elif 182<=rpy<185:
        return 3 # 右转
    elif 185<=rpy<=360:
        return 15 # 快右转
    elif 0<=rpy<=175:
        return 14 # 快左转
    else:
        return 2 # 左转

# 转向270°方向前进
def walk_270(rpy):
    if 268<rpy<272:
        return 1
    elif 265<rpy<=268:
        return 2 # 左转
    elif 90<=rpy<=265:
        return 14 # 快左转
    elif 0<=rpy<90 or 275<=rpy<=360:
        return 15 # 快右转
    else:
        return 3 # 右转
    
# 转向270°方向前进
def walk_270_fast(rpy):
    if 268<rpy<272:
        return 28
    elif 265<rpy<=268:
        return 2 # 左转
    elif 90<=rpy<=265:
        return 14 # 快左转
    elif 0<=rpy<90 or 275<=rpy<=360:
        return 15 # 快右转
    else:
        return 3 # 右转
        
# A区装货
def load_a(position,gait_mode,rpy,QR):
    global flags,results
    x, y, z = position
    gait, mode = gait_mode
    rpy=rpy
    if gait==0 and mode==0 or gait ==1 and mode==9:
        return 0
    else:
        if -1.25 <y and flags["A_LOAD_ENDING_FLAG1"] == False:
            return walk_270(rpy)
        elif -0.5 <x< 2.5 and (rpy>94 or rpy<86) and flags["A_LOAD_ENDING_FLAG2"] == False:
            flags["A_LOAD_ENDING_FLAG1"] = True
            if QR == 1:
                return walk_0(rpy)
            elif QR ==2:
                return 13
            else:
                return walk_180(rpy)
        elif y< -1.1 and flags["A_LOAD_ENDING_FLAG3"] == False:
            flags["A_LOAD_ENDING_FLAG2"] = True
            return walk_90(rpy)
        elif -1.85 <y and flags["A_LOAD_ENDING_FLAG4"] == False:
            flags["A_LOAD_ENDING_FLAG3"] = True
            if results["ready_load_a"] == 0:
                if flags["is_ready_load_a"] == False:
                    flags["is_ready_load_a"] = True
                    return 4# 高阻尼趴下
                else:
                    results["ready_load_a"] = get_ready()
                    return 0
            else:
                if gait == 0 and mode == 7:
                    return 0
                else:
                    return 6#倒退
        elif flags["A_LOAD_ENDING_FLAG5"] == False:
            flags["A_LOAD_ENDING_FLAG4"] = True
            if QR == 1 and 0.95 <x:
                return walk_180(rpy)
            elif QR == 2 and x< 0:
                return walk_0(rpy)
            else:
                flags["A_LOAD_ENDING_FLAG5"] = True
                return walk_90(rpy)
        elif rpy>94 or rpy<86:
            return 11
        else:
            return walk_90(rpy)   

# A区卸货        
def unload_a(position,gait_mode,rpy,QR):
    global flags,results
    x, y, z = position
    gait, mode = gait_mode
    rpy=rpy
    if gait==0 and mode==0 or gait ==1 and mode==9:
        return 0
    else:
        if 1.1 <x and flags["A_UNLOAD_ENDING_FLAG1"] == False:
            return walk_180(rpy)
        elif y>-1.25 and flags["A_UNLOAD_ENDING_FLAG2"] == False:
            flags["A_UNLOAD_ENDING_FLAG1"] = True
            if rpy<265:
                return 16
            else:
                return walk_270(rpy)
        elif -0.5 <x< 2.5 and (rpy>94 or rpy<86) and flags["A_UNLOAD_ENDING_FLAG3"] == False:
            flags["A_UNLOAD_ENDING_FLAG2"] = True
            if QR == 1:
                return walk_0(rpy)
            elif QR ==2:
                return 17
            else:
                return walk_180(rpy)
        elif y< -1.1 and flags["A_UNLOAD_ENDING_FLAG4"] == False:
            flags["A_UNLOAD_ENDING_FLAG3"] = True
            return walk_90(rpy)
        elif -1.85 <y and flags["A_UNLOAD_ENDING_FLAG5"] == False:
            flags["A_UNLOAD_ENDING_FLAG4"] = True
            if results["ready_unload_a"] == 0:
                if flags["is_ready_unload_a"] == False:
                    flags["is_ready_unload_a"] = True
                    return 4# 高阻尼趴下
                else:
                    results["ready_unload_a"] = get_ready()
                    return 0
            else:
                if gait == 0 and mode == 7:
                    return 0
                else:
                    return 6#倒退
        elif x>1.45 and flags["A_UNLOAD_ENDING_FLAG6"] == False:
            flags["A_UNLOAD_ENDING_FLAG5"] = True
            if QR == 2:
                return walk_180(rpy)
            else:
                return walk_180(rpy)
        elif (rpy>95 or rpy<85) and flags["A_UNLOAD_ENDING_FLAG7"] == False:
            flags["A_UNLOAD_ENDING_FLAG6"] = True
            return 18
        else:
            return walk_90(rpy)   

# B区卸货
def unload_b(position,gait_mode,rpy,QR):
    global flags,results
    x, y, z = position
    gait, mode = gait_mode
    rpy=rpy
    if gait==0 and mode==0 or gait ==1 and mode==9:
        return 0
    else:
        if QR ==4:
            if y<15.9 and flags["unload_b_flag1"] == False:
                if x<1.97:
                    return 8
                elif x>2.03:
                    return 7
                else:
                    return walk_90(rpy)
            elif flags["unload_b_flag1"] == False:
                if flags["is_ready_unload_b"] == False:
                    flags["is_ready_unload_b"] = True
                    return 4
                else:
                    flags["unload_b_flag1"] = get_ready()
                    return 0
            elif y>14.87 and flags["unload_b_flag2"] == False and flags["unload_b_flag1"] == True:
                return 6
            elif x>0 and flags["unload_b_flag3"] == False:
                flags["unload_b_flag2"] = True
                return walk_180(rpy)
            else:
                flags["unload_b_flag3"] = True
                if 267<rpy<273:
                    if y<15.85 and flags["load_b_flag1"] == False:
                        if x<-0.03:
                            return 7
                        elif x>0.03:
                            return 8
                        else:
                            return 26
                    elif flags["load_b_flag1"] == False:
                        if flags["is_ready_load_b"] == False:
                            flags["is_ready_load_b"] = True
                            return 4
                        else:
                            flags["load_b_flag1"] = get_ready()
                            return 0
                    else:
                        if x<-0.03:
                            return 7
                        elif x>0.03:
                            return 8
                        else:
                            return 1
                    
                elif 90<rpy<=267:
                    return 14 # 左转
                else:
                    return 3 # 右转
                    

        elif QR ==3:
            return walk_180(rpy)

# 过S弯，并在最后一个S弯识别箭头方向（朝右）
def pass_s_and_identify_arrow(position,rpy):
    global flags,results
    x, y, z = position
    rpy=rpy
    if x< 1.47 and flags["S_ENDING_FLAG1"] == False:
        return walk_0(rpy)
    elif 0.95 <x and flags["S_ENDING_FLAG2"] == False:
        flags["S_ENDING_FLAG1"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 0.95)**2)
        # print(distance)
        target_rpy = math.degrees(math.atan2(y-0.95,x-1.47))+180
        target_rpy -= 90
        delta_rpy = rpy - target_rpy
        # print(delta_rpy)
        if(delta_rpy>5 and delta_rpy<80):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-80):
            return 2
        if(distance<0.72 and distance>=0.68):
            return 21
        elif(distance <0.68):
            return 8
        elif(distance>0.74 and distance<=0.78 ):
            return 20
        elif(distance>0.78):
            return 7
        return 25
    elif x<=0.95 and flags["S_ENDING_FLAG3"] == False:
        flags["S_ENDING_FLAG2"] = True
        if flags["is_s_direction_ready0"] == False:
            if 222<rpy<=228:
                flags["is_s_direction_ready0"] = True
            elif 45<rpy<=222:
                return 2  # 左转
            else:
                return 3  # 右转
        distance = math.sqrt((x - 0.45)**2 + (y - 2)**2)
        # print(distance)
        target_rpy = math.degrees(math.atan2(y-2,x-0.45))+180
        if(target_rpy>=0 and target_rpy<270):
            target_rpy+=90
        else:
            target_rpy-=270
        delta_rpy = rpy-target_rpy
        if(delta_rpy>5 and delta_rpy<80):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-80):
            return 2
        if(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        elif(distance>0.74 and distance<=0.78):
            return 22
        elif(distance>0.78):
            return 8
        return 24
    elif (0.95 < x and y < 3) or 1.5 < x and flags["S_ENDING_FLAG4"] == False:
        flags["S_ENDING_FLAG3"] = True
        if flags["is_s_direction_ready1"] == False:
            if 312 < rpy < 318:
                flags["is_s_direction_ready1"] = True
            elif 135 < rpy <= 312:
                return 2  # 左转
            else:
                return 3  # 右转
        distance = math.sqrt((x - 1.47) ** 2 + (y - 3.05) ** 2)
        target_rpy = math.degrees(math.atan2(y - 3.05, x - 1.47)) + 180
        if target_rpy >= 45 and target_rpy < 90:
            target_rpy += 270
        else:
            target_rpy -= 90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        # print(distance)
        if distance < 0.72 and distance >= 0.68:
            return 21
        elif distance < 0.68:
            return 8
        elif distance > 0.74 and distance <= 0.78:
            return 20
        elif distance > 0.78:
            return 7
        return 25
    elif y < 4.55 and flags["S_ENDING_FLAG5"] == False:
        flags["S_ENDING_FLAG4"] = True
        distance = math.sqrt((x - 1.47) ** 2 + (y - 4.55) ** 2)
        target_rpy = math.degrees(math.atan2(y - 4.55, x - 1.47)) + 180
        target_rpy += 90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        # print(distance)
        if distance > 0.74 and distance <= 0.78:
            return 22
        elif distance > 0.78:
            return 8
        elif distance < 0.72 and distance >= 0.68:
            return 23
        elif distance < 0.68:
            return 7
        return 24
    elif y < 4.9 and flags["S_ENDING_FLAG6"] == False:
        flags["S_ENDING_FLAG5"] = True
        if results["ARROW"] == 0:
            if flags["ARROW_executed"] == False:
                flags["ARROW_executed"] = True
                return 0
            else:
                results["ARROW"] = arrow()
                return 24
        else:
            return 24
    else:
        flags["S_ENDING_FLAG6"] = True
        return walk_90(rpy)

# 回S弯
def pass_s_back(position,rpy):
    global flags,results
    x, y, z = position
    rpy=rpy
    if y> 4.55 and flags["BACK_S_ENDING_FLAG1"] == False:
        return walk_270(rpy)
    elif x<1.5 and flags["BACK_S_ENDING_FLAG2"] == False:
        flags["BACK_S_ENDING_FLAG1"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 4.55)**2)
        #print(distance)
        target_rpy = math.degrees(math.atan2(y - 4.55, x - 1.47)) + 180
        target_rpy+=270
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        if(0.68 <= distance < 0.72):
            return 21
        elif(distance<0.68):
            return 8
        elif(0.74 < distance <= 0.78):
            return 20
        elif(distance>0.78):
            return 7
        return 25
    elif y>3 or x>0.95  and flags["BACK_S_ENDING_FLAG3"] == False:
        flags["BACK_S_ENDING_FLAG2"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 3.05)**2)
        target_rpy = math.degrees(math.atan2(y - 3.05, x - 1.47)) + 180
        target_rpy+=90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        if(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        elif(distance>0.74 and distance <=0.78):
            return 22
        elif(distance>0.78):
            return 8
        return 24
    elif x<=0.95 and flags["BACK_S_ENDING_FLAG4"] == False:
        flags["BACK_S_ENDING_FLAG3"] = True
        if flags["is_s_direction_ready2"] == False:
            if 132<rpy<=138:
                flags["is_s_direction_ready2"] = True
            elif 138<rpy<=315:
                return 3  
            else:
                return 2  
        distance = math.sqrt((x - 0.45)**2 + (y - 2)**2)
        target_rpy = math.degrees(math.atan2(y-2,x-0.45))+180
        if(target_rpy>0 and target_rpy<=90):
            target_rpy+=270
        else:
            target_rpy-=90
        delta_rpy = rpy-target_rpy
        if(delta_rpy>5 and delta_rpy<50):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-50):
            return 2
        #print(distance)
        if(distance>0.74 and distance<=0.78):
            return 20
        elif(distance>0.78):
            return 7
        elif(distance<0.72 and distance>=0.68):
            return 21
        elif(distance<0.68):
            return 8
        return 25
    elif y>0.95 or x>1.47 and flags["BACK_S_ENDING_FLAG5"] == False:
        flags["BACK_S_ENDING_FLAG4"] = True
        if flags["is_s_direction_ready3"] == False:
            if 42<rpy<=48:
                flags["is_s_direction_ready3"] = True
            elif 48<rpy<=225:
                return 3  
            else:
                return 2  
        distance = math.sqrt((x - 1.47)**2 + (y - 0.95)**2)
        target_rpy = math.degrees(math.atan2(y-0.95,x-1.47))+180
        if(target_rpy>90 and target_rpy<=270):
            target_rpy+=90
        else:
            target_rpy-=270
        delta_rpy = rpy - target_rpy
        if(delta_rpy>5 and delta_rpy<50):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-50):
            return 2
        #print(distance)
        if(distance>0.74 and distance<=0.78):
            return 22
        elif(distance > 0.78):
            return 8
        elif(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        return 24
    else:
        return walk_180(rpy)

# B区完成卸货装货后从左边过 限高杆、石板路 回返A区
def back_left(position,rpy):
    global flags
    x, y, z = position
    rpy=rpy
    if y > 13 and flags["BACK_ENDING_FLAG1"] == False:
        if x<-0.03:
            return 7
        elif x>0.03:
            return 8
        else:
            return walk_270_fast(rpy)
    elif y>12 and flags["BACK_ENDING_FLAG2"] == False:
        flags["BACK_ENDING_FLAG1"] = True
        return 5
    elif y>10.5 and flags["BACK_ENDING_FLAG3"] == False:
        flags["BACK_ENDING_FLAG2"] = True
        if x<-0.03:
            return 7
        elif x>0.03:
            return 8
        else:
            return walk_270_fast(rpy)
    elif y>5.9 and flags["BACK_ENDING_FLAG4"] == False:
        flags["BACK_ENDING_FLAG3"] = True
        if x<-0.06:
            return 7
        elif x>0.06:
            return 8
        else:
            return 9
    elif y>5.5 and flags["BACK_ENDING_FLAG5"] == False:
        flags["BACK_ENDING_FLAG4"] = True
        if x<-0.01:
            return 7
        elif x>0.05:
            return 8
        else:
            return walk_270(rpy)
    elif y>4.65 and flags["BACK_ENDING_FLAG6"] == False:
        flags["BACK_ENDING_FLAG5"] = True
        if x<0.45:
            return walk_0(rpy)
        elif ((rpy<30 or rpy>275) or x>0.8 ):
            return 27
        else:
            return walk_270(rpy)
    else:
        flags["BACK_ENDING_FLAG6"] = True
        if x>0.75:
            return 8
        else:
            return walk_270(rpy)
    
# 过S弯后箭头指右，从右边过 斜坡、黄灯 到达B区
def go_right(position,rpy):
    global flags,results,original_distance,light_distance
    x, y, z = position

    rpy=rpy
    # 斜坡前准备
    if x< 2 and flags["GO_ENDING_FLAG1"] == False:
        return walk_0(rpy)
    elif y< 5.75 and flags["GO_ENDING_FLAG2"] == False:
        flags["GO_ENDING_FLAG1"] = True
        return walk_90(rpy)
    elif y< 10.5 and flags["GO_ENDING_FLAG3"] == False:
        flags["GO_ENDING_FLAG2"] = True
        if x<1.97:
            return 8
        elif x>2.03:
            return 7
        else:
            return 10
    # 黄灯路段
    elif y < 12 and flags["GO_ENDING_FLAG4"] == False:
        flags["GO_ENDING_FLAG3"] = True
        if results["yellow_light"] == 0:
            if flags["LIGHT_executed"] == False:
                flags["LIGHT_executed"] = True
                return 0
            else:
                original_distance = yellow_light()
                light_distance = original_distance
                #print(f"当前距离黄灯距离：{light_distance}")
                results["yellow_light"] = 1
                return walk_90(rpy)
        elif results["yellow_light"] == 1 and light_distance > 0.5:
            walk_distance = y - 10.5
            light_distance = original_distance - walk_distance
            #print(f"111111111当前距离黄灯距离：{light_distance}")
            return walk_90(rpy)
        elif light_distance <= 0.5:
            if results["ready_wait"] == 0:
                if flags["WAIT_executed"] == False:
                    flags["WAIT_executed"] = True
                    return 0
                else:
                    results["ready_wait"] = yellow_wait()
                    return walk_90_fast(rpy)
            else:
                return walk_90_fast(rpy)
        else:
            return walk_90_fast(rpy)
    # 通过黄灯后识别二维码
    elif y< 13 and flags["GO_ENDING_FLAG5"] == False:
        flags["GO_ENDING_FLAG4"] = True
        if results["B_QR"] == 0:
            results["B_QR"] = QRcode()
            return walk_90(rpy)
        else:
            return walk_90_fast(rpy)
    else:
        flags["GO_ENDING_FLAG5"] = True
        return walk_90_fast(rpy)


###########################################################################################
def main():
    global turn_count
    turn_count = 0
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()    
    data_lock = threading.Lock()
    
    try:
        user_pub()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)        
        ctrl_thread = threading.Thread(target=my_ctrl.run)
        rec_thread = threading.Thread(target=pos_msg.run)
        gait_thread= threading.Thread(target=gait_msg.run)
            
        ctrl_thread.start()
        time.sleep(4)
        my_ctrl.num = 2# 起步左转一下调正机位
        my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)
        rec_thread.start()
        gait_thread.start()
        def print_worker():
            while True:
                print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} A二维码识别结果{results['A_QR']} B二维码识别结果{results['B_QR']} 箭头识别结果{results['ARROW']} 选择:{my_ctrl.num}")
                print(f"{gait_msg.gait_mode}")
                time.sleep(0.2) 
        thread = threading.Thread(target=print_worker)
        thread.start()

        while True:
            # time.sleep(0.2)
            with data_lock:
                num = select_step_based_on_position(pos_msg.position,gait_msg.gait_mode,pos_msg.rpy[2])
                # print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} A二维码识别结果{results['A_QR']} B二维码识别结果{results['B_QR']} 箭头识别结果{results['ARROW']} 选择:{num}")
                # print(f"{gait_msg.gait_mode}")
            my_ctrl.num = num
            my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
            if num==0:
                print("站立")
                time.sleep(4)           

    except KeyboardInterrupt:
        cmd_msg.mode = 7  # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
        pass
    sys.exit()


if __name__ == '__main__':
    main()
