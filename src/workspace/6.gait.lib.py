#!/usr/bin/env python3
"""
6.gait.lib.py — 第六赛段专用步态封装（侧移 / 快走 / 转向校正）

注意：文件名含数字与点，主模块通过 SourceFileLoader 动态加载。
提供的函数为高层封装，参数采用（gl, ...）形式以便直接调用。
"""

import math


def lateral_shift(gl, distance, speed=0.08):
    """横移：distance 正为右移，负为左移（米）。
    通过调用底层 gl.step_shift 完成短步多次累积移动。
    """
    if abs(distance) < 0.005:
        return
    # 分段执行以提高稳定性（尽量使用 gl.step_shift 的短步）
    remaining = distance
    step = 0.06
    sign = 1 if distance > 0 else -1
    # 注意：底层 gl.step_shift 的正负含义为正=左移、负=右移。
    # 上层约定 distance 正为右移，因此需要对 sign 取反传给 gl.step_shift。
    while abs(remaining) > 0.005:
        cur = sign * min(step, abs(remaining))
        gl.step_shift(-cur, speed=speed)
        gl._pump()
        remaining -= cur


def run_forward(gl, distance, speed=0.30):
    """快速前进封装：使用短步跑/快走语义。
    distance: 米，speed: m/s
    """
    if distance <= 0.0:
        return
    # 使用 gl.step_high_forward 或 step_forward 取决于可用性
    try:
        gl.step_high_forward(distance, speed=speed)
    except Exception:
        gl.step_forward(distance, speed=speed)
    gl._pump()


def adjust_heading(gl, degrees, rate=0.25):
    """调整朝向（相对角度，度数）。正为左转，负为右转。"""
    if abs(degrees) < 1.0:
        return
    gl.step_turn(degrees, rate=rate)
    gl._pump()


def forward_small(gl, distance, speed=0.12):
    """小步前进（用于靠近目标点）"""
    if distance <= 0.0:
        return
    gl.step_forward(distance, speed=speed)
    gl._pump()
