import sys
sys.path.append("./lcm")
import time
import lcm
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from localization_lcmt import localization_lcmt

class Rec_msg(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = localization_lcmt()
        self.running = False

    def run(self):
        self.running = True
        self.lc_r.subscribe("global_to_robot", self.msg_handler)
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = localization_lcmt().decode(data)
        print(f"机身位置{self.rec_msg.xyz}, 机身速度{self.rec_msg.vxyz}, 机身姿态{self.rec_msg.rpy}, 机身角速度{self.rec_msg.omegaBody}, 机身线速度{self.rec_msg.vBody}, 时间戳{self.rec_msg.timestamp}")

    def rec_responce(self): #每隔多少时间打印信息
        while self.running:
            try:
                self.lc_r.handle()
            except:
                print("LCM handle exception occurred")
            time.sleep(0.2)

    def quit(self):
        self.running = False
        self.rec_thread.join()

rec_msg = Rec_msg()
rec_msg.run()