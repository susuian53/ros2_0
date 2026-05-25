import lcm
import sys
import time
import toml
import copy
import math
import threading
import os
from file_send_lcmt import file_send_lcmt

robot_cmd = {
    'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
    'vel_des': [0.0, 0.0, 0.0],
    'rpy_des': [0.0, 0.0, 0.0],
    'pos_des': [0.0, 0.0, 0.0],
    'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point': [0.0, 0.0, 0.0],
    'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'step_height': [0.0, 0.0],
    'value': 0, 'duration': 0
}

def user_pub():
    lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    usergait_msg = file_send_lcmt()
    # 直行
    steps1 = toml.load("./toml/usergait_param.toml")
    full_steps1 = {'step': [robot_cmd]}
    k = 0
    for i in steps1['step']:
        cmd = copy.deepcopy(robot_cmd)
        cmd['duration'] = i['duration']
        if i['type'] == 'usergait':
            cmd['mode'] = 11  # LOCOMOTION
            cmd['gait_id'] = 110  # USERGAIT
            cmd['vel_des'] = i['body_vel_des']
            cmd['rpy_des'] = i['body_pos_des'][0:3]
            cmd['pos_des'] = i['body_pos_des'][3:6]
            cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
            cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
            cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
            cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
            cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
            cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
            cmd['acc_des'] = i['weight']
            cmd['value'] = i['use_mpc_traj']
            cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
            cmd['ctrl_point'][2] = i['mu']
        if k == 0:
            full_steps1['step'] = [cmd]
        else:
            full_steps1['step'].append(cmd)
        k = k + 1
    f = open("./toml/usergait_param_full.toml", 'w')
    f.write("# Gait Params\n")
    f.writelines(toml.dumps(full_steps1))
    f.close()
    file_obj_gait_def = open("./toml/usergait_def.toml", 'r')
    file_obj_gait_params = open("./toml/usergait_param_full.toml", 'r')
    usergait_msg.data = file_obj_gait_def.read()
    lcm_usergait.publish("user_gait_file", usergait_msg.encode())
    time.sleep(0.5)
    usergait_msg.data = file_obj_gait_params.read()
    lcm_usergait.publish("user_gait_file", usergait_msg.encode())
    time.sleep(0.1)
    file_obj_gait_def.close()
    file_obj_gait_params.close()