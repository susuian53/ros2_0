#!/usr/bin/env python3

# Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import time
import signal
import threading
import subprocess
import argparse

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")


def kill_simulation_processes():
    """杀死所有与仿真相关的进程，防止端口或资源冲突。"""
    processes_to_kill = [
        "gzclient",
        "gzserver",
        "rviz2",
        "cyberdog_control",
        "robot_controller",
        "lcm-.*",
    ]

    print("[launchsim_safe] 正在清理旧的仿真进程...")
    for proc in processes_to_kill:
        subprocess.run(["pkill", "-9", "-f", proc], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "ros2 launch"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "ros2 run"], capture_output=True)
    time.sleep(1)
    print("[launchsim_safe] 进程清理完成。")


def fix_multicast():
    """尝试修复 LCM 组播路由，这是仿真通讯的基础"""
    print("[launchsim_safe] 检查并配置组播路由 (224.0.0.0/4)...")
    # 优先尝试 ip route，它是现代 Linux 的标准
    cmd = "sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo 2>/dev/null || true"
    subprocess.run(cmd, shell=True)

    # 验证是否成功
    try:
        r = subprocess.run("ip route | grep 224.0.0.0",
                           shell=True, capture_output=True, text=True)
        if "224.0.0.0" in r.stdout:
            print("[launchsim_safe] ✓ 组播路由已就绪")
        else:
            print("[launchsim_safe] ⚠️  组播路由配置失败，如果狗趴着不动，请手动执行:")
            print("[launchsim_safe]    sudo ip route add 224.0.0.0/4 dev lo")
    except Exception:
        pass


def run_in_background(script_path, log_path, env=None, extra_args=None):
    """在后台运行脚本，输出重定向到日志文件。"""
    env_vars = env.copy() if env else os.environ.copy()

    log_dir = os.path.dirname(log_path)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)

    cmd = ["bash", script_path]
    if extra_args:
        cmd.extend(extra_args)

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env_vars,
        cwd=BASE_DIR,
    )

    def write_log():
        try:
            with open(log_path, "wb") as f:
                for line in proc.stdout:
                    f.write(line)
                    f.flush()
        except Exception as e:
            print(f"[launchsim_safe] 日志写入失败 ({log_path}): {e}", file=sys.stderr)

    t = threading.Thread(target=write_log, daemon=True)
    t.start()
    return proc


def launchsim():
    parser = argparse.ArgumentParser(description="Cyberdog 仿真启动脚本")
    parser.add_argument("--lidar", action="store_true", help="启用激光雷达")
    parser.add_argument("--camera", action="store_true", help="启用RGB摄像头")
    parser.add_argument("--world", type=str, default="race",
                        help="选择地图: race(默认) / empty(空地+黄线)")
    args = parser.parse_args()

    kill_simulation_processes()
    fix_multicast()

    os.makedirs(LOG_DIR, exist_ok=True)

    my_env = os.environ.copy()

    gazebo_args = ["--world", args.world]
    if args.lidar:
        gazebo_args.append("--lidar")
    if args.camera:
        gazebo_args.append("--camera")

    state_msg = (
        f"地图: {args.world}, "
        f"激光雷达={'开启' if args.lidar else '关闭'}, "
        f"RGB摄像头={'开启' if args.camera else '关闭'}"
    )
    print(f"[launchsim_safe] {state_msg}")

    print("[launchsim_safe] 启动 Gazebo 仿真...")
    gazebo_script = "./src/cyberdog_simulator/cyberdog_gazebo/script/launchgazebo.sh"
    gazebo_proc = run_in_background(
        gazebo_script,
        os.path.join(LOG_DIR, "gazebo.log"),
        env=my_env,
        extra_args=gazebo_args,
    )
    time.sleep(8)

    print("[launchsim_safe] 启动可视化界面...")
    visual_proc = run_in_background(
        "./src/cyberdog_simulator/cyberdog_gazebo/script/launchvisual.sh",
        os.path.join(LOG_DIR, "visual.log"),
        env=my_env,
    )
    time.sleep(2)

    print("[launchsim_safe] 启动控制程序...")
    control_proc = run_in_background(
        "./src/cyberdog_simulator/cyberdog_gazebo/script/launchcontrol.sh",
        os.path.join(LOG_DIR, "control.log"),
        env=my_env,
    )

    web_proc = None
    if args.camera:
        time.sleep(3)
        print("[launchsim_safe] 启动摄像头 Web 服务器...")
        print("[launchsim_safe] 摄像头画面地址: http://localhost:8082")

        web_cmd = (
            "source /opt/ros/galactic/setup.bash && "
            f"source {BASE_DIR}/install/setup.bash && "
            f"python3 {BASE_DIR}/camera_viewer/web_server.py"
        )
        web_proc = subprocess.Popen(
            ["bash", "-c", web_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=my_env,
            cwd=BASE_DIR,
        )

        def write_web_log():
            try:
                with open(os.path.join(LOG_DIR, "camera_web.log"), "wb") as f:
                    for line in web_proc.stdout:
                        f.write(line)
                        f.flush()
            except Exception as e:
                print(
                    f"[launchsim_safe] Web 日志写入失败: {e}", file=sys.stderr)

        threading.Thread(target=write_web_log, daemon=True).start()

    print("[launchsim_safe] 所有进程已启动。")
    print(f"[launchsim_safe] 日志目录: {LOG_DIR}/")
    if args.camera:
        print("[launchsim_safe] 摄像头 Web 界面: http://localhost:8082")
    print("[launchsim_safe] 按 Ctrl+C 停止所有进程。")

    def cleanup():
        procs = [p for p in [gazebo_proc, visual_proc, control_proc, web_proc] if p]
        for proc in procs:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()  # 回收僵尸进程
        kill_simulation_processes()

    signal.signal(signal.SIGINT, lambda sig, frame: None)
    signal.signal(signal.SIGTERM, lambda sig, frame: None)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[launchsim_safe] 收到中断信号，正在停止所有进程...")
        cleanup()
        print("[launchsim_safe] 已停止。")


if __name__ == "__main__":
    launchsim()
