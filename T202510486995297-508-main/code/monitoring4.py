import time
import lcm
from threading import Thread, Lock
from leg_control_data_lcmt import leg_control_data_lcmt
class Rec_msg(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = leg_control_data_lcmt()
        self.runing=0
    def run(self):
        self.runing=1
        self.lc_r.subscribe("leg_control_data", self.msg_handler)
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = leg_control_data_lcmt().decode(data)
        print(f"self.q{self.rec_msg.q}")
    def rec_responce(self): #每隔多少时间打印信息
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.2 )

    def quit(self):
        self.rec_thread.join()
        self.runing=0
rec_msg=Rec_msg()
rec_msg.run()