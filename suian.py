#!/usr/bin/env python3
"""suian.py — 清业务进程并重启世界 + 业务。

默认行为：
- 清理旧的 tracker / tracker 输出窗口 / 相关业务节点
- 交给 `susuian.py` 重新启动完整仿真世界和业务

可选：
- `--business-only`：只重建路径并重启 tracker，不重置世界
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
import shutil

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.join(BASE_DIR, "src", "workspace")
LOG_DIR = os.path.join(BASE_DIR, "logs")
TRACKER_SCRIPT = os.path.join(WS_DIR, "scheme1_step_tracking_v3.py")
BUILD_PATH_SCRIPT = os.path.join(WS_DIR, "build_path.py")
FULL_LAUNCHER = os.path.join(BASE_DIR, "susuian.py")

BUSINESS_PATTERNS = [
    "scheme1_step_tracking_v3.py",
    "scheme1_step_tracking",
    "tracker_output",
    "gnome-terminal.*tracker_output",
    "ros2 run.*scheme1_step_tracking",
]

running = True


def run_cmd(cmd, cwd=BASE_DIR, env=None, timeout=None):
    run_env = os.environ.copy()
    if env:
        run_env.update(env)
    try:
        r = subprocess.run(
            cmd,
            shell=True,
            cwd=cwd,
            env=run_env,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"


def _log_thread(proc, log_path):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)

    def _write():
        try:
            with open(log_path, "wb") as f:
                for line in proc.stdout:
                    f.write(line)
                    f.flush()
        except Exception:
            pass

    threading.Thread(target=_write, daemon=True).start()


def kill_business_processes():
    print("[suian] 清理旧业务进程...")
    for pattern in BUSINESS_PATTERNS:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
    time.sleep(1)


def build_path():
    print("[suian] 生成 track_path.csv ...")
    ret, out, err = run_cmd(f"{sys.executable} {BUILD_PATH_SCRIPT}")
    if ret != 0:
        print("[suian] ✗ build_path 失败:")
        print((out + err).strip())
        return False
    return True


def launch_tracker():
    os.makedirs(LOG_DIR, exist_ok=True)
    tracker_log = os.path.join(LOG_DIR, "tracker.log")
    tracker_cmd = (
        "source /opt/ros/galactic/setup.bash && "
        f"source {os.path.join(BASE_DIR, 'install', 'setup.bash')} && "
        f"python3 -u {TRACKER_SCRIPT}"
    )
    print("[suian] 启动业务节点 tracker ...")
    tracker = subprocess.Popen(
        ["bash", "-c", tracker_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=BASE_DIR,
        env=os.environ.copy(),
    )
    _log_thread(tracker, tracker_log)
    print(f"[suian] tracker 日志: {tracker_log}")

    # 如果有桌面环境，顺手打开尾随窗口，方便实时看日志
    if shutil.which("gnome-terminal"):
        print("[suian] 打开 tracker_output 终端窗口...")
        tail_cmd = (
            f"bash -c 'touch {tracker_log}; "
            f"echo \"═══ 实时跟踪数据 ═══\"; "
            f"tail -f {tracker_log}; exec bash'"
        )
        subprocess.Popen(["gnome-terminal", "-t", "tracker_output", "--", "bash", "-c", tail_cmd])
    else:
        print("[suian] 未检测到 gnome-terminal，跳过实时窗口")

    return tracker


def main():
    parser = argparse.ArgumentParser(description="清业务进程并重启世界 + 业务")
    parser.add_argument("--business-only", action="store_true",
                        help="只重建路径并重启 tracker，不重置世界")
    args = parser.parse_args()

    kill_business_processes()

    # 默认模式：重启完整世界（仿真 + 业务）
    if not args.business_only:
        print("[suian] 进入完整重启模式，转交给 susuian.py")
        os.execv(sys.executable, [sys.executable, FULL_LAUNCHER])

    if not build_path():
        sys.exit(1)

    tracker = launch_tracker()

    def _on_signal(sig, frame):
        global running
        print(f"\n[suian] 收到信号 {sig}，准备退出...")
        running = False
        try:
            tracker.terminate()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while running:
            if tracker.poll() is not None:
                print(f"[suian] tracker 已退出，code={tracker.returncode}")
                break
            time.sleep(1)
    finally:
        try:
            tracker.terminate()
        except Exception:
            pass
        try:
            tracker.wait(timeout=3)
        except Exception:
            pass
        print("[suian] 业务已停止")


if __name__ == "__main__":
    main()
