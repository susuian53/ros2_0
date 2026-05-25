import time
import lcm
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
class Rec_msg(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.rec_msg = robot_control_response_lcmt()
        self.runing=0
    def run(self):
        self.runing=1
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        print(f"模式是{self.rec_msg.mode},步态是{self.rec_msg.gait_id},进度是{self.rec_msg.order_process_bar},切换状态是{self.rec_msg.switch_status}")
    def rec_responce(self): #每隔多少时间打印信息
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.2 )

    def quit(self):
        self.rec_thread.join()
        self.runing=0
rec_msg=Rec_msg()
rec_msg.run()