import lcm
import sys
import time
import toml
import copy
import math
import threading
import os
from simulator_lcmt import simulator_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

class Pos_msg(object):
    def __init__(self, data_lock):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = simulator_lcmt()
        self.lc_r.subscribe("simulator_state", self.msg_handler)
        self.data_lock = data_lock
        self.position = [0,0,0]
        self.rpy=[0.0,0.0,0.0]

    def run(self):
        while True:
            self.lc_r.handle()  # 持续处理消息

    def msg_handler(self, channel, data):
        self.rec_msg = simulator_lcmt().decode(data)
        with self.data_lock:
            self.position[:] = self.rec_msg.p[:]
            self.rec_msg.rpy=list(self.rec_msg.rpy)
            for i in range(len(self.rec_msg.rpy)):
                self.rec_msg.rpy[i]=self.rec_msg.rpy[i]*180/math.pi
            flag=True if abs((abs(self.rec_msg.rpy[0])-0))<abs((abs(self.rec_msg.rpy[0])-180)) else False
            self.rpy=self.rec_msg.rpy[:]
            if (flag==False):
                self.rpy[2]=self.rpy[2]+180

class Gait_msg(object):
    def __init__(self, data_lock):
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.rec_msg = robot_control_response_lcmt()
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.data_lock = data_lock
        self.gait_mode=[0,0]

    def run(self):
        while True:
            self.lc_r.handle()  # 持续处理消息

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        with self.data_lock:
            self.gait_mode = [self.rec_msg.gait_id, self.rec_msg.mode]
