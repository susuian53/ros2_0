import lcm
import sys
import time
import toml
import copy
import math
import threading
import os
from robot_control_cmd_lcmt import robot_control_cmd_lcmt

class Robot_Ctrl(object):
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        self.steps = toml.load("./toml/usergait.toml")
        self.num = 0  # 初始状态为站立
        self.running = True

    def run(self):
        while self.running:
            self.update_and_publish()
            time.sleep(0.2)

    def update_and_publish(self):
        self.msg.mode = self.steps["step"][self.num]["mode"]
        self.msg.gait_id = self.steps["step"][self.num]["gait_id"]
        self.msg.contact = self.steps["step"][self.num]["contact"]
        self.msg.value = self.steps["step"][self.num]["value"]
        self.msg.duration = self.steps["step"][self.num]["duration"]
        # self.msg.life_count += 1
        for i in range(3):
            self.msg.vel_des[i] = self.steps["step"][self.num]["vel_des"][i]
            self.msg.rpy_des[i] = self.steps["step"][self.num]["rpy_des"][i]
            self.msg.pos_des[i] = self.steps["step"][self.num]["pos_des"][i]
            self.msg.acc_des[i] = self.steps["step"][self.num]["acc_des"][i]
            self.msg.acc_des[i + 3] = self.steps["step"][self.num]["acc_des"][i + 3]
            self.msg.foot_pose[i] = self.steps["step"][self.num]["foot_pose"][i]
            self.msg.ctrl_point[i] = self.steps["step"][self.num]["ctrl_point"][i]
        for i in range(2):
            self.msg.step_height[i] = self.steps["step"][self.num]['step_height'][i]
        self.lc.publish("robot_control_cmd", self.msg.encode())