import sys
sys.path.append("./lcm")
import time
import lcm
from threading import Thread, Lock
from simulator_lcmt import simulator_lcmt
import math
class Rec_msg(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = simulator_lcmt()
        self.runing=0
    def run(self):
        self.runing=1
        self.lc_r.subscribe("simulator_state", self.msg_handler)
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = simulator_lcmt().decode(data)
        self.rec_msg.rpy=list(self.rec_msg.rpy)
        for i in range(len(self.rec_msg.rpy)):
            self.rec_msg.rpy[i]=self.rec_msg.rpy[i]*180/math.pi
        flag=True if abs((abs(self.rec_msg.rpy[0])-0))<abs((abs(self.rec_msg.rpy[0])-180)) else False
        print(f"机身位置{self.rec_msg.p}")

    def rec_responce(self): #每隔多少时间打印信息
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.2)

    def quit(self):
        self.rec_thread.join()
        self.runing=0
rec_msg=Rec_msg()
rec_msg.run()