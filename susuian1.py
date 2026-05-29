#!/usr/bin/env python3
"""
susuian1.py — 一键启动：环境检查 → 仿真环境 → 业务节点 (scheme1_step_tracking_v3)

与 susuian.py 的区别:
- 启动前做仿真环境预检查
- 退出时只停止业务节点，不停止仿真环境

用法（在容器内执行）:
    python3 susuian1.py
    python3 susuian1.py --lidar --camera
    python3 susuian1.py --world race
"""

import os
import sys
import math
import time
import signal
import shutil
import threading
import subprocess
import argparse
import importlib.util
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")
INSTALL_DIR = os.path.join(BASE_DIR, "install")
LCM_SRC_DIR = os.path.join(BASE_DIR, "src", "cyberdog_locomotion", "common", "lcm_type", "lcm")
TRACKER_SCRIPT = os.path.join(BASE_DIR, "src", "workspace", "scheme1_step_tracking_v3.py")
BUILD_PATH_SCRIPT = os.path.join(BASE_DIR, "src", "workspace", "build_path.py")


# ── 工具函数 ─────────────────────────────────────────

def run_cmd(cmd, cwd=BASE_DIR, env=None, timeout=None):
    """运行命令，返回 (returncode, stdout, stderr)"""
    run_env = os.environ.copy()
    if env:
        run_env.update(env)
    try:
        r = subprocess.run(
            cmd, shell=True, cwd=cwd, env=run_env,
            capture_output=True, text=True, timeout=timeout
        )
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"


def _is_process_running(pattern):
    """检查是否已有匹配进程在运行。"""
    ret, out, _ = run_cmd(f"pgrep -f \"{pattern}\"")
    return ret == 0 and bool(out.strip())


def ensure_x11_access():
    display = os.environ.get("DISPLAY", "")
    if not display:
        print("[launch] ⚠️  DISPLAY 未设置，图形界面可能无法打开")
        print("[launch]    请在宿主机执行: xhost +")
        return False
    print(f"[launch] DISPLAY={display}")
    return True


def check_sim_environment():
    """检查仿真启动所需的基础条件。"""
    ok = True
    if not shutil.which("lcm-gen"):
        print("[launch] ✗ 未找到 lcm-gen，请确认已安装 LCM")
        ok = False
    if not ensure_x11_access():
        ok = False

    required = [
        os.path.join(BASE_DIR, "src", "cyberdog_simulator", "cyberdog_gazebo", "script", "launchgazebo.sh"),
        os.path.join(BASE_DIR, "src", "cyberdog_simulator", "cyberdog_gazebo", "script", "launchvisual.sh"),
        os.path.join(BASE_DIR, "src", "cyberdog_simulator", "cyberdog_gazebo", "script", "launchcontrol.sh"),
        TRACKER_SCRIPT,
    ]
    for path in required:
        if not os.path.exists(path):
            print(f"[launch] ✗ 缺少必要文件: {path}")
            ok = False
    return ok


def step_fix_multicast():
    """尝试修复 LCM 组播路由，这是仿真通讯的基础"""
    print("[launch] 检查并配置组播路由 (224.0.0.0/4)...")
    cmd = "sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo 2>/dev/null || true"
    subprocess.run(cmd, shell=True)
    ret, out, _ = run_cmd("ip route | grep 224.0.0.0")
    if "224.0.0.0" in out:
        print("[launch] ✓ 组播路由已就绪")
    else:
        print("[launch] ⚠️  组播路由配置失败，如果狗趴着不动，请手动执行:")
        print("[launch]    sudo ip route add 224.0.0.0/4 dev lo")


def step_build_path():
    """运行 build_path.py 生成新的 track_path.csv。"""
    print("[launch] 重新生成路径文件 track_path.csv...")
    ret, out, err = run_cmd(f"{sys.executable} -B {BUILD_PATH_SCRIPT}")
    if ret != 0:
        tail = (out + err)[-800:]
        print(f"[launch] ✗ 路径生成失败:\n{tail}")
        return False
    print("[launch] ✓ 路径已更新")
    return True


def kill_business_nodes():
    """只停止业务节点，不关闭仿真环境。"""
    procs = ["scheme1_step_tracking", "gnome-terminal.*tracker_output"]
    for p in procs:
        subprocess.run(["pkill", "-9", "-f", p], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "ros2 run.*scheme1_step_tracking"], capture_output=True)
    time.sleep(1)


# ── 构建步骤 ─────────────────────────────────────────

def step_generate_lcm():
    """用 lcm-gen 从 .lcm 生成 .hpp (C++) 和 Python 模块"""
    if not os.path.isdir(LCM_SRC_DIR):
        print("[launch] ⚠️  LCM 源目录不存在，跳过 LCM 生成")
        return False

    cpp_ok = os.path.exists(os.path.join(LCM_SRC_DIR, "simulator_lcmt.hpp"))
    py_ok = os.path.exists(os.path.join(LCM_SRC_DIR, "robot_control_cmd_lcmt.py"))
    if cpp_ok and py_ok:
        print("[launch] ✓ LCM 头文件 + Python 模块已存在，跳过生成")
        return True

    print("[launch] 生成 LCM 类型 (C++ + Python)...")
    lcm_files = os.path.join(LCM_SRC_DIR, "*.lcm")

    ret, _, err = run_cmd(f"lcm-gen -x {lcm_files} --cpp-hpath {LCM_SRC_DIR}/")
    if ret != 0:
        print(f"[launch] ✗ lcm-gen C++ 失败:\n{err}")
        return False

    ret, _, err = run_cmd(f"lcm-gen -p {lcm_files} --ppath {LCM_SRC_DIR}/")
    if ret != 0:
        print(f"[launch] ✗ lcm-gen Python 失败:\n{err}")
        return False

    print("[launch] ✓ LCM 类型生成完成 (C++ + Python)")
    return True


def step_build_packages(packages):
    """colcon build 指定包"""
    pkgs = " ".join(packages)
    print(f"[launch] 编译包: {pkgs} ...")
    ret, out, err = run_cmd(
        f"bash -c 'source /opt/ros/galactic/setup.bash && "
        f"cd {BASE_DIR} && colcon build --packages-select {pkgs} "
        f"--merge-install --symlink-install --packages-skip-build-finished'",
        timeout=300,
    )
    if ret != 0:
        tail = (out + err)[-800:]
        print(f"[launch] ✗ 编译失败:\n{tail}")
        return False
    print("[launch] ✓ 编译完成")
    return True


def step_ensure_built():
    """确保仿真依赖已编译 (已编译则跳过)"""
    needed = ["cyberdog_description", "cyberdog_gazebo", "cyberdog_visual", "cyberdog_msg", "cyberdog_locomotion"]
    to_build = []
    for pkg in needed:
        install_pkg = os.path.join(INSTALL_DIR, "share", pkg)
        if os.path.isdir(install_pkg):
            print(f"[launch] ✓ {pkg} 已安装，跳过")
        else:
            to_build.append(pkg)
    if not to_build:
        print("[launch] ✓ 所有包已编译，跳过构建步骤")
        return True
    return step_build_packages(to_build)


# ── 仿真启动 ─────────────────────────────────────────

def _log_thread(proc, log_path):
    """后台线程：把子进程输出写入日志"""
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


def step_launch_sim(args, procs_list, my_env):
    """按顺序启动 Gazebo / RViz / Controller，传递初始坐标"""
    os.makedirs(LOG_DIR, exist_ok=True)

    gazebo_running = _is_process_running("gzserver|gazebo")
    rviz_running = _is_process_running("rviz2")
    ctrl_running = _is_process_running("cyberdog_control|robot_controller|launchcontrol.sh")

    if gazebo_running:
        print("[launch] 检测到 Gazebo 已在运行，跳过重复启动")
    else:
        print(f"[launch] 启动 Gazebo 仿真 (坐标: {args.init_x:.2f}, {args.init_y:.2f}, yaw={args.init_yaw:.3f})...")
    sim_env = my_env.copy()
    sim_env["INIT_X"] = str(args.init_x)
    sim_env["INIT_Y"] = str(args.init_y)
    sim_env["INIT_YAW"] = str(args.init_yaw)

    gazebo_args = ["--world", args.world]
    if args.lidar:
        gazebo_args.append("--lidar")
    if args.camera:
        gazebo_args.append("--camera")

    if not gazebo_running:
        gz = subprocess.Popen(
            ["bash", "./src/cyberdog_simulator/cyberdog_gazebo/script/launchgazebo.sh"] + gazebo_args,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=sim_env, cwd=BASE_DIR,
            start_new_session=True,
        )
        procs_list.append(("gazebo", gz))
        _log_thread(gz, os.path.join(LOG_DIR, "gazebo.log"))

        def _unpause():
            time.sleep(12)
            print("[launch] 解除仿真暂停...")
            run_cmd("source /opt/ros/galactic/setup.bash && ros2 service call /unpause_physics std_srvs/srv/Empty '{}'")

        threading.Thread(target=_unpause, daemon=True).start()

        time.sleep(10)

    if rviz_running:
        print("[launch] 检测到 RViz 已在运行，跳过重复启动")
    else:
        print("[launch] 启动 RViz 可视化...")
        viz = subprocess.Popen(
            ["bash", "./src/cyberdog_simulator/cyberdog_gazebo/script/launchvisual.sh"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=my_env, cwd=BASE_DIR,
            start_new_session=True,
        )
        procs_list.append(("rviz", viz))
        _log_thread(viz, os.path.join(LOG_DIR, "visual.log"))
        time.sleep(3)

    if ctrl_running:
        print("[launch] 检测到底层控制器已在运行，跳过重复启动")
    else:
        print("[launch] 启动底层控制器...")
        ctrl = subprocess.Popen(
            ["bash", "./src/cyberdog_simulator/cyberdog_gazebo/script/launchcontrol.sh"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=my_env, cwd=BASE_DIR,
            start_new_session=True,
        )
        procs_list.append(("controller", ctrl))
        _log_thread(ctrl, os.path.join(LOG_DIR, "control.log"))
        time.sleep(3)

    if args.camera:
        print("[launch] 启动摄像头 Web → http://localhost:8082")
        web_cmd = (
            "source /opt/ros/galactic/setup.bash && "
            f"source {INSTALL_DIR}/setup.bash && "
            f"python3 {BASE_DIR}/camera_viewer/web_server.py"
        )
        web = subprocess.Popen(
            ["bash", "-c", web_cmd],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=my_env, cwd=BASE_DIR,
            start_new_session=True,
        )
        procs_list.append(("camera_web", web))
        _log_thread(web, os.path.join(LOG_DIR, "camera_web.log"))


def step_launch_tracker(procs_list, my_env):
    """启动 workspace Python 寻迹引擎 (scheme1_step_tracking_v3.py)"""
    tracker_log = os.path.join(LOG_DIR, "tracker.log")
    tracker_cmd = (
        "source /opt/ros/galactic/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"python3 -u {TRACKER_SCRIPT}"
    )
    print("[launch] 启动路径跟踪节点 (scheme1_step_tracking_v3)")
    print(f"[launch]   脚本: {TRACKER_SCRIPT}")
    tracker = subprocess.Popen(
        ["bash", "-c", tracker_cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=my_env, cwd=BASE_DIR,
        start_new_session=True,
    )
    procs_list.append(("tracker", tracker))
    _log_thread(tracker, tracker_log)

    print("[launch] 检查并清除旧的打印窗口...")
    subprocess.run(["pkill", "-f", "gnome-terminal.*tracker_output"], capture_output=True, timeout=3)
    time.sleep(0.5)

    print("[launch] 打开实时打印窗口...")
    tail_cmd = f"bash -c 'touch {tracker_log}; echo \"═══ 实时跟踪数据 ═══\"; tail -f {tracker_log}; exec bash'"
    term = subprocess.Popen(
        ["gnome-terminal", "-t", "tracker_output", "--", "bash", "-c", tail_cmd],
        env=my_env,
        start_new_session=True,
    )
    procs_list.append(("tracker_term", term))


# ── 路点名 → CSV 索引解析 ──────────────────────────

def _load_path_config():
    """动态导入 path_config.py，返回 (WAYPOINTS, STEP)"""
    spec = importlib.util.spec_from_file_location(
        "path_config",
        os.path.join(BASE_DIR, "src", "workspace", "path_config.py"),
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.WAYPOINTS, mod.STEP


def _waypoint_name_to_index(name, waypoints):
    """根据路点名找 WAYPOINTS 中的位置"""
    for i, (wname, _, _) in enumerate(waypoints):
        if wname.upper() == name.upper():
            return i
    raise ValueError(f"未找到路点名 '{name}'，可用: {', '.join(w[0] for w in waypoints)}")


def _calc_csv_index(waypoint_idx, waypoints, step):
    """计算第 waypoint_idx 个路点插值后在 CSV 中的行号 (0-indexed)。"""
    total = 0
    for i in range(waypoint_idx):
        _, x0, y0 = waypoints[i]
        _, x1, y1 = waypoints[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(dist / step))
        total += n
    return total


def resolve_start_point(value):
    """支持数字 CSV 行号或路点名，返回 (csv_index, x, y, yaw)"""
    ws_dir = os.path.join(BASE_DIR, "src", "workspace")
    path_csv = os.path.join(ws_dir, "track_path.csv")
    if not os.path.exists(path_csv):
        raise FileNotFoundError(f"未找到路径文件: {path_csv}")

    try:
        idx = int(value)
        with open(path_csv) as f:
            rows = [line.strip() for line in f if line.strip()]
        if idx < 0 or idx >= len(rows):
            raise IndexError(f"CSV 行号 {idx} 越界 (共 {len(rows)} 行)")
        parts = rows[idx].split(",")
        x = float(parts[0])
        y = float(parts[1])
        yaw = float(parts[2]) if len(parts) >= 3 else 0.0
        return idx, x, y, yaw
    except ValueError:
        pass

    waypoints, step = _load_path_config()
    wp_idx = _waypoint_name_to_index(value, waypoints)
    csv_idx = _calc_csv_index(wp_idx, waypoints, step)

    with open(path_csv) as f:
        rows = [line.strip() for line in f if line.strip()]
    if csv_idx >= len(rows):
        raise IndexError(f"路点 '{value}' 插值索引 {csv_idx} 超出 CSV 行数 ({len(rows)})")
    parts = rows[csv_idx].split(",")
    x = float(parts[0])
    y = float(parts[1])
    yaw = float(parts[2]) if len(parts) >= 3 else 0.0
    return csv_idx, x, y, yaw


def write_start_index(idx):
    config_path = os.path.join(BASE_DIR, "src", "workspace", "config_v3.py")
    with open(config_path) as f:
        content = f.read()

    new_line = f"START_INDEX = {idx}     # 起点路点索引 (由 susuian1.py 设置)"
    if re.search(r"^START_INDEX\s*=", content, flags=re.MULTILINE):
        content = re.sub(r"^START_INDEX\s*=.*$", new_line, content, flags=re.MULTILINE)
    else:
        content = content.replace(
            "# ═══════════════════════════════════════════════════════\n"
            "# 跟踪参数\n"
            "# ═══════════════════════════════════════════════════════\n",
            "# ═══════════════════════════════════════════════════════\n"
            "# 跟踪参数\n"
            "# ═══════════════════════════════════════════════════════\n"
            f"{new_line}\n",
        )

    with open(config_path, "w") as f:
        f.write(content)


# ── 主入口 ───────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Cyberdog 一键启动脚本")
    parser.add_argument("start_point", type=str, nargs="?", default="0",
                        help="起点: 数字(CSV行号) 或 路点名(如 CH3_IN, SPAWN, BR_BOT)")
    parser.add_argument("--yaw", type=float, default=None,
                        help="出生朝向 (弧度)，不传则使用 CSV 中的 yaw")
    parser.add_argument("--lidar", action="store_true", help="启用激光雷达")
    parser.add_argument("--camera", action="store_true", help="启用 RGB 摄像头")
    parser.add_argument("--world", type=str, default="race",
                        help="地图: race(默认) / empty")
    parser.add_argument("--no-build", action="store_true", help="跳过编译")
    parser.add_argument("--no-tracker", action="store_true", help="不启动路径跟踪")
    args = parser.parse_args()

    init_x, init_y, init_yaw = 0.0, 0.0, 0.0

    try:
        start_idx, init_x, init_y, csv_yaw = resolve_start_point(args.start_point)
        init_yaw = args.yaw if args.yaw is not None else csv_yaw
        print(f"[launch] 起点: {args.start_point!r} → CSV[{start_idx}] = ({init_x:.2f}, {init_y:.2f}, yaw={init_yaw:.3f})")
        write_start_index(start_idx)
        print(f"[launch] ✓ config_v3.py START_INDEX = {start_idx}")
    except Exception as e:
        print(f"[launch] ⚠️ 起点解析失败: {e}")
        print("[launch] 使用默认坐标 (0,0)")
        init_x, init_y, init_yaw = 0.0, 0.0, args.yaw if args.yaw is not None else 0.0

    if args.yaw is not None:
        init_yaw = args.yaw
        print(f"[launch] yaw 覆盖为 {init_yaw:.3f} 弧度")

    args.init_x = init_x
    args.init_y = init_y
    args.init_yaw = init_yaw

    if not check_sim_environment():
        print("[launch] ✗ 仿真环境检查失败，退出")
        sys.exit(1)

    if not step_build_path():
        print("[launch] ✗ 路径生成失败，退出")
        sys.exit(1)

    kill_business_nodes()
    step_fix_multicast()

    if not step_generate_lcm():
        print("[launch] ✗ LCM 生成失败，退出")
        sys.exit(1)

    if not args.no_build:
        if not step_ensure_built():
            print("[launch] ✗ 编译失败，退出")
            sys.exit(1)

    procs = []
    my_env = os.environ.copy()
    my_env.update({
        "INIT_X": f"{args.init_x:.4f}",
        "INIT_Y": f"{args.init_y:.4f}",
        "INIT_YAW": f"{args.init_yaw:.6f}",
    })

    step_launch_sim(args, procs, my_env)

    if not args.no_tracker:
        print("[launch] 等待仿真就绪 (3s)...")
        time.sleep(3)
        step_launch_tracker(procs, my_env)

    print("\n[launch] ========== 全部启动完成 ==========")
    print(f"[launch] 日志目录: {LOG_DIR}/")
    print("[launch] 按 Ctrl+C 仅停止业务节点，仿真环境将保留")

    stop_event = threading.Event()

    def _on_signal(sig, frame):
        print(f"\n[launch] 收到信号 {sig}，正在停止业务节点...")
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while not stop_event.is_set():
            stop_event.wait(timeout=1)
    finally:
        print("[launch] 清理业务节点...")
        for name, proc in procs:
            if name == "tracker_term":
                print("[launch]   保留打印终端 (tracker_term)，不关闭")
                continue
            if name != "tracker":
                continue
            try:
                proc.terminate()
            except Exception:
                pass
        for name, proc in procs:
            if name != "tracker":
                continue
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                    proc.wait(timeout=3)
                except Exception:
                    pass
        print("[launch] 已停止业务节点，仿真环境保持运行")


if __name__ == "__main__":
    main()
