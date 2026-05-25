#!/usr/bin/env python3
"""
path_editor.py — 可视化赛道路径编辑器

在赛道鸟瞰图上鼠标点击编辑关键节点和辅助节点, 定义遍历顺序, 生成 CSV 路径文件。

用法:
  python3 /home/cyberdog_sim/path_planner/path_editor.py

操作:
  左键拖拽    — 移动节点
  右键        — 添加关键节点
  Shift+右键  — 添加辅助节点 (终端交互时用)
  Delete      — 删除选中节点
  C           — 进入连线模式 (左键点节点加入遍历顺序)
  E           — 回到编辑模式
  G           — 切换网格吸附
  Ctrl+Z      — 撤销
  Ctrl+S      — 保存 JSON
  Esc         — 取消选中
"""

import sys
import os
import json
import math
import copy
from typing import Optional, List, Dict, Tuple, Any

import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.patches import Circle

# ── 路径配置 ──────────────────────────────────
WORKSPACE     = "/home/cyberdog_sim"
PLANNER_DIR   = os.path.join(WORKSPACE, "path_planner")
DEFAULT_JSON  = os.path.join(WORKSPACE, "src/Option_One/地图与路径/关键节点.json")
OUTPUT_JSON   = os.path.join(PLANNER_DIR, "path_nodes.json")
OUTPUT_CSV    = os.path.join(PLANNER_DIR, "track_path_edited.csv")

sys.path.insert(0, PLANNER_DIR)
from track_renderer import (
    ImageAdjuster, draw_grid, setup_axes, ViewController,
    BG_COLOR, TEXT_COLOR, SEG_COLORS
)

# ── 常量 ──────────────────────────────────────
SNAP_GRID  = 0.05
PATH_STEP  = 0.02
MAX_UNDO   = 50

SMOOTH_MODES       = ["linear", "catmull_rom", "bspline"]
AVAILABLE_SMOOTH   = ["linear", "catmull_rom"]

try:
    from scipy import interpolate as sci_interp
    AVAILABLE_SMOOTH.append("bspline")
except ImportError:
    pass


# ═══════════════════════════════════════════════════════
# 数据层
# ═══════════════════════════════════════════════════════
class PathEditorData:
    def __init__(self):
        self.nodes = {}          # id -> {name, x, y, type, seg}
        self.traversal = []       # list of node id
        self.next_id = 1
        self.smooth_mode = "linear"
        self.snap_enabled = True
        self.undo_stack = []

    def snapshot(self):
        state = {
            'nodes': copy.deepcopy(self.nodes),
            'traversal': list(self.traversal),
            'next_id': self.next_id,
        }
        self.undo_stack.append(state)
        if len(self.undo_stack) > MAX_UNDO:
            self.undo_stack.pop(0)

    def undo(self) -> bool:
        if not self.undo_stack:
            return False
        state = self.undo_stack.pop()
        self.nodes = state['nodes']
        self.traversal = state['traversal']
        self.next_id = state['next_id']
        return True

    def add_node(self, x: float, y: float, ntype: str = "key", name: str = "") -> int:
        if self.snap_enabled:
            x = round(x / SNAP_GRID) * SNAP_GRID
            y = round(y / SNAP_GRID) * SNAP_GRID
        nid = self.next_id
        self.next_id += 1
        if not name:
            prefix = "N" if ntype == "key" else "A"
            name = f"{prefix}{nid}"
        seg = _infer_segment(x, y)
        self.nodes[nid] = {"name": name, "x": x, "y": y, "type": ntype, "seg": seg}
        return nid

    def remove_node(self, nid: int):
        if nid in self.nodes:
            del self.nodes[nid]
        self.traversal = [i for i in self.traversal if i != nid]

    def move_node(self, nid: int, x: float, y: float):
        if nid not in self.nodes:
            return
        if self.snap_enabled:
            x = round(x / SNAP_GRID) * SNAP_GRID
            y = round(y / SNAP_GRID) * SNAP_GRID
        self.nodes[nid]['x'] = x
        self.nodes[nid]['y'] = y
        self.nodes[nid]['seg'] = _infer_segment(x, y)

    def add_to_traversal(self, nid: int):
        if nid in self.nodes and nid not in self.traversal:
            self.traversal.append(nid)

    def remove_from_traversal(self, nid: int):
        self.traversal = [i for i in self.traversal if i != nid]

    def clear_traversal(self):
        self.traversal = []

    def move_traversal_up(self, idx: int):
        if 0 < idx < len(self.traversal):
            self.traversal[idx], self.traversal[idx - 1] = \
                self.traversal[idx - 1], self.traversal[idx]

    def move_traversal_down(self, idx: int):
        if 0 <= idx < len(self.traversal) - 1:
            self.traversal[idx], self.traversal[idx + 1] = \
                self.traversal[idx + 1], self.traversal[idx]

    def find_node_at(self, x: float, y: float, radius: float = 0.15) -> Optional[int]:
        best_id, best_d = None, radius
        for nid, nd in self.nodes.items():
            d = math.hypot(nd['x'] - x, nd['y'] - y)
            if d < best_d:
                best_d = d
                best_id = nid
        return best_id


def _infer_segment(x: float, y: float) -> str:
    if y < 0.5:   return "S1"
    if y < 3.9:   return "S2"
    if y < 7.5:   return "S3"
    if y < 11.7:  return "S4"
    if y < 12.2 and x > 2.8: return "S5"
    return "S6"


# ═══════════════════════════════════════════════════════
# 路径插值
# ═══════════════════════════════════════════════════════
def interpolate_path(data: PathEditorData) -> list:
    if len(data.traversal) < 2:
        return []

    pts = [(data.nodes[nid]['x'], data.nodes[nid]['y'])
           for nid in data.traversal if nid in data.nodes]

    if data.smooth_mode == "linear":
        waypoints = _interp_linear(pts)
    elif data.smooth_mode == "catmull_rom":
        waypoints = _interp_catmull_rom(pts)
    elif data.smooth_mode == "bspline" and "bspline" in AVAILABLE_SMOOTH:
        waypoints = _interp_bspline(pts)
    else:
        waypoints = _interp_linear(pts)

    result = []
    n = len(waypoints)
    for i in range(n):
        x, y = waypoints[i]
        if i < n - 1:
            dx = waypoints[i + 1][0] - x
            dy = waypoints[i + 1][1] - y
        elif i > 0:
            dx = x - waypoints[i - 1][0]
            dy = y - waypoints[i - 1][1]
        else:
            dx, dy = 1.0, 0.0
        # heading = 面朝方向, move_dir = 运动方向 (倒退时两者差 π)
        tangent = math.atan2(dy, dx)
        result.append((x, y, tangent, tangent))
    return result


def _interp_linear(pts: list) -> list:
    waypoints = [pts[0]]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(dist / PATH_STEP))
        for s in range(1, steps + 1):
            t = s / steps
            waypoints.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return waypoints


def _interp_catmull_rom(pts: list) -> list:
    if len(pts) < 2:
        return list(pts)

    pts_arr = np.array(pts, dtype=float)
    n = len(pts_arr)
    alpha = 0.5

    def _t(idx):
        if idx <= 0: return 0.0
        if idx >= n: return 1.0
        d = 0.0
        for k in range(1, idx + 1):
            d += math.hypot(pts_arr[k][0] - pts_arr[k - 1][0],
                            pts_arr[k][1] - pts_arr[k - 1][1]) ** alpha
        return d

    total_len = _t(n - 1)
    if total_len < 1e-9:
        return _interp_linear(pts)

    num_samples = max(2, int(total_len / PATH_STEP))
    waypoints = [tuple(pts[0])]

    for step in range(1, num_samples):
        t_global = step / num_samples * total_len
        seg = 0
        for i in range(n - 1):
            if _t(i + 1) >= t_global:
                seg = i
                break
        else:
            seg = n - 2

        t0 = _t(seg)
        t1 = _t(seg + 1)
        t_local = 0.5 if (t1 - t0) < 1e-9 else (t_global - t0) / (t1 - t0)

        p0 = pts_arr[max(0, seg - 1)]
        p1 = pts_arr[seg]
        p2 = pts_arr[min(n - 1, seg + 1)]
        p3 = pts_arr[min(n - 1, seg + 2)]

        t, tt, ttt = t_local, t_local * t_local, t_local * t_local * t_local
        x = 0.5 * ((-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * ttt +
                    (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * tt +
                    (-p0[0] + p2[0]) * t + 2*p1[0])
        y = 0.5 * ((-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * ttt +
                    (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * tt +
                    (-p0[1] + p2[1]) * t + 2*p1[1])
        waypoints.append((x, y))

    waypoints.append(tuple(pts[-1]))
    return waypoints


def _interp_bspline(pts: list) -> list:
    if len(pts) < 3:
        return _interp_linear(pts)
    pts_arr = np.array(pts, dtype=float).T
    try:
        k = min(3, len(pts) - 1)
        tck, u = sci_interp.splprep(pts_arr, s=0.05, k=k)
        total = len(pts) * 10
        u_new = np.linspace(0, 1, total)
        x_new, y_new = sci_interp.splev(u_new, tck)
        out = [(float(x_new[0]), float(y_new[0]))]
        for i in range(1, len(x_new)):
            if math.hypot(x_new[i] - out[-1][0], y_new[i] - out[-1][1]) >= PATH_STEP:
                out.append((float(x_new[i]), float(y_new[i])))
        if len(out) > 1:
            out.append((float(x_new[-1]), float(y_new[-1])))
        return out
    except Exception:
        return _interp_linear(pts)


def export_csv(filepath: str, waypoints: list):
    """输出4列: x, y, heading(朝向), move_dir(运动方向)"""
    with open(filepath, 'w') as f:
        for x, y, heading, move_dir in waypoints:
            f.write(f"{x:.4f},{y:.4f},{heading:.6f},{move_dir:.6f}\n")
    print(f"[CSV] {len(waypoints)} waypoints -> {filepath}")


def load_json(filepath: str) -> Optional[dict]:
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load: {e}")
        return None


def save_json(filepath: str, data: PathEditorData):
    nodes_list = []
    for nid in sorted(data.nodes.keys()):
        nd = data.nodes[nid]
        nodes_list.append({
            "id": nid, "name": nd['name'],
            "x": nd['x'], "y": nd['y'],
            "type": nd['type'], "seg": nd['seg'],
            "说明": ""
        })
    out = {
        "_说明": "Path Planner generated",
        "节点": nodes_list,
        "遍历顺序": list(data.traversal),
        "平滑模式": data.smooth_mode,
    }
    with open(filepath, 'w') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[SAVED] {filepath}")


# ═══════════════════════════════════════════════════════
# 编辑器主类
# ═══════════════════════════════════════════════════════
class PathEditor:
    def __init__(self):
        self.data = PathEditorData()
        self.mode = "edit"
        self.dragging: Optional[int] = None
        self.drag_offset = (0.0, 0.0)
        self.selected: Optional[int] = None
        self._click_time = 0.0
        self._last_click_nid: Optional[int] = None

        # 艺术家引用
        self._node_scatters = {}
        self._node_labels = {}
        self._traversal_arrows = []
        self._preview_line = None
        self._selected_ring = None

        # 创建窗口
        self.fig, self.ax = setup_axes(figsize=(10, 20))
        self.fig.patch.set_facecolor(BG_COLOR)
        self.img = ImageAdjuster(self.ax)   # 背景图（可独立调节）
        self.img.draw()
        draw_grid(self.ax)                  # 坐标网格（固定）
        self.view = ViewController(self.ax) # 视图缩放/平移

        # 信息文本
        self._info_text = self.ax.text(
            0.02, 0.02, "", transform=self.ax.transAxes,
            fontsize=8, color=TEXT_COLOR, fontfamily='monospace',
            va='bottom', ha='left', zorder=100,
            bbox=dict(boxstyle='round', fc=BG_COLOR, alpha=0.8, ec='#333333'))
        self._trav_text = None  # removed cluttered traversal list

        # 工具栏
        self._create_toolbar()

        # 事件绑定
        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)
        self.fig.canvas.mpl_connect('close_event', lambda e: print("\nEditor closed."))

    # ── 工具栏 ───────────────────────────────
    def _create_toolbar(self):
        w, h = 0.12, 0.035
        x0 = 0.76
        y = 0.92
        gap = 0.008

        def _btn(label, action, ypos):
            ax_btn = self.fig.add_axes([x0, ypos, w, h])
            btn = Button(ax_btn, label, color='#2a2a4a', hovercolor='#3a3a6a')
            btn.label.set_color(TEXT_COLOR)
            btn.label.set_fontsize(8)
            btn.on_clicked(action)
            return btn

        _btn("Open JSON", self._action_load, y); y -= (h + gap)
        _btn("Save JSON", self._action_save, y); y -= (h + gap)
        _btn("Export CSV", self._action_export_csv, y); y -= (h + gap * 5)

        # 平滑模式
        smooth_labels = [m.replace('_', ' ').title() for m in AVAILABLE_SMOOTH]
        ax_sm = self.fig.add_axes([x0, y - 0.06, w, 0.06])
        self._radio_smooth = RadioButtons(ax_sm, smooth_labels, active=0,
                                          activecolor='#4a4a8a')
        for lbl in self._radio_smooth.labels:
            lbl.set_fontsize(7)
            lbl.set_color(TEXT_COLOR)
        for c in self._radio_smooth.circles:
            c.set_radius(0.6)
        self._radio_smooth.on_clicked(self._on_smooth_change)
        ax_sm.text(-0.15, 1.1, "Smooth:", transform=ax_sm.transAxes,
                   fontsize=7, color=TEXT_COLOR)
        y -= 0.09

        # 模式切换
        ax_md = self.fig.add_axes([x0, y - 0.04, w, 0.04])
        self._radio_mode = RadioButtons(ax_md, ["Edit (E)", "Connect (C)"],
                                        active=0, activecolor='#4a4a8a')
        for lbl in self._radio_mode.labels:
            lbl.set_color(TEXT_COLOR)
            lbl.set_fontsize(7)
        for c in self._radio_mode.circles:
            c.set_radius(0.6)
        self._radio_mode.on_clicked(self._on_mode_change)

        # 吸附
        y -= 0.06
        ax_sn = self.fig.add_axes([x0, y - 0.04, w, 0.04])
        self._radio_snap = RadioButtons(ax_sn, ["Snap ON", "Snap OFF"],
                                        active=0, activecolor='#4a4a8a')
        for lbl in self._radio_snap.labels:
            lbl.set_fontsize(7)
            lbl.set_color(TEXT_COLOR)
        for c in self._radio_snap.circles:
            c.set_radius(0.6)
        self._radio_snap.on_clicked(self._on_snap_change)

        # 遍历操作
        y -= 0.09
        _btn("Clear Trav", lambda e: self._trav_clear(), y)

    # ── 工具栏回调 ───────────────────────────
    def _action_load(self, event):
        self.load_nodes()

    def _action_save(self, event):
        self.data.snapshot()
        save_json(OUTPUT_JSON, self.data)
        self.refresh()

    def _action_export_csv(self, event):
        waypoints = interpolate_path(self.data)
        if waypoints:
            export_csv(OUTPUT_CSV, waypoints)
        else:
            print("[WARN] No traversal defined.")

    def _on_smooth_change(self, label):
        for i, m in enumerate(AVAILABLE_SMOOTH):
            if m.replace('_', ' ').title() == label:
                self.data.smooth_mode = AVAILABLE_SMOOTH[i]
                break
        self.refresh()

    def _on_mode_change(self, label):
        self.mode = "connect" if "Connect" in label else "edit"
        self._update_info()

    def _on_snap_change(self, label):
        self.data.snap_enabled = "ON" in label

    def _trav_move(self, direction):
        if self.selected and self.selected in self.data.traversal:
            idx = self.data.traversal.index(self.selected)
            self.data.snapshot()
            if direction < 0:
                self.data.move_traversal_up(idx)
            else:
                self.data.move_traversal_down(idx)
            self.refresh()

    def _trav_remove_sel(self):
        if self.selected and self.selected in self.data.traversal:
            self.data.snapshot()
            self.data.remove_from_traversal(self.selected)
            self.refresh()

    def _trav_clear(self):
        if self.data.traversal:
            self.data.snapshot()
            self.data.clear_traversal()
            self.refresh()

    # ── 鼠标事件 ─────────────────────────────
    def _on_press(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata

        if event.button == 3:  # 右键 — 添加节点
            self.data.snapshot()
            # Shift+右键 = 辅助节点, 普通右键 = 关键节点
            # matplotlib 不传 modifier, 用简单的交替逻辑:
            # 如果按住选项菜单则用辅助, 默认关键
            ntype = "key"
            nid = self.data.add_node(x, y, ntype)
            self.selected = nid
            self.refresh()
            return

        if event.button == 1:  # 左键
            nid = self.data.find_node_at(x, y)

            if self.mode == "connect":
                if nid is not None:
                    self.data.snapshot()
                    self.data.add_to_traversal(nid)
                    self.selected = nid
                    self.refresh()
                return

            # 编辑模式
            if nid is not None:
                # 双击检测
                now = event.time if hasattr(event, 'time') else 0.0
                if self._last_click_nid == nid and (now - self._click_time) < 0.4:
                    self._edit_node_name(nid)
                    self._last_click_nid = None
                    return
                self._click_time = now
                self._last_click_nid = nid
                self.selected = nid
                self.dragging = nid
                nd = self.data.nodes[nid]
                self.drag_offset = (nd['x'] - x, nd['y'] - y)
                self.data.snapshot()
                self.refresh(skip_path=True)  # 开始拖拽时跳过路径预览
            else:
                self.selected = None
                self._last_click_nid = None
                self.refresh(skip_path=True)  # 取消选中也跳过

    def _on_release(self, event):
        if self.dragging is not None:
            self.dragging = None
            self.refresh()  # 拖完才完整刷新路径预览

    def _on_motion(self, event):
        if self.dragging is not None and event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            self.data.move_node(self.dragging,
                                x + self.drag_offset[0],
                                y + self.drag_offset[1])
            self._update_dragged_node()
            self._update_info()
        elif event.inaxes == self.ax:
            self._update_info()
        else:
            self._info_text.set_text("")

    def _update_dragged_node(self):
        """拖拽时只更新被拖节点的位置, 不重算路径"""
        nid = self.dragging
        if nid is None or nid not in self.data.nodes:
            return
        nd = self.data.nodes[nid]
        x, y = nd['x'], nd['y']
        # 移动 scatter marker
        if nid in self._node_scatters:
            self._node_scatters[nid].set_data([x], [y])
        # 移动标签
        if nid in self._node_labels:
            self._node_labels[nid].set_position((x, y))
        # 移动选中环
        if self._selected_ring:
            self._selected_ring.center = (x, y)
        self.fig.canvas.draw_idle()

    # ── 滚轮缩放 (无极缩放视图) ──────────────
    def _on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        factor = 0.9 if event.button == 'up' else 1.1
        self.view.zoom(factor, event.xdata, event.ydata)
        self._update_info()

    # ── 键盘事件 ─────────────────────────────
    def _on_key(self, event):
        key = event.key
        if key == 'delete':
            if self.selected:
                self.data.snapshot()
                self.data.remove_node(self.selected)
                self.selected = None
                self.refresh()
        elif key == 'c':
            self.mode = "connect"
            self._radio_mode.set_active(1)
            self._update_info()
        elif key == 'e':
            self.mode = "edit"
            self._radio_mode.set_active(0)
            self._update_info()
        elif key == 'g':
            self.data.snap_enabled = not self.data.snap_enabled
            self._radio_snap.set_active(1 if not self.data.snap_enabled else 0)
        elif key == 'ctrl+z':
            if self.data.undo():
                self.refresh()
        elif key == 'ctrl+s':
            self._action_save(None)
        elif key == 'escape':
            self.selected = None
            self.refresh()
        # ── 背景图调节 (独立于坐标网格) ──
        elif key == 'up':
            self.img.nudge(0, 0.05); self._update_info()
        elif key == 'down':
            self.img.nudge(0, -0.05); self._update_info()
        elif key == 'left':
            self.img.nudge(-0.05, 0); self._update_info()
        elif key == 'right':
            self.img.nudge(0.05, 0); self._update_info()
        elif key in ('+', '='):
            self.img.scale(0.98); self._update_info()
        elif key == '-':
            self.img.scale(1.02); self._update_info()
        elif key == 'enter':
            self.img.save(); self._update_info()
        # ── 视图平移 / 复位 ──
        elif key == 'shift+up':
            self.view.pan(0, 0.5); self._update_info()
        elif key == 'shift+down':
            self.view.pan(0, -0.5); self._update_info()
        elif key == 'shift+left':
            self.view.pan(-0.5, 0); self._update_info()
        elif key == 'shift+right':
            self.view.pan(0.5, 0); self._update_info()
        elif key == 'r':
            self.view.reset(); self._update_info()

    def _edit_node_name(self, nid: int):
        if nid not in self.data.nodes:
            return
        old = self.data.nodes[nid]['name']
        try:
            new_name = input(f"  Rename [{old}] -> (Enter to keep): ").strip()
            if new_name:
                self.data.nodes[nid]['name'] = new_name
                self.refresh()
        except Exception:
            pass

    # ── 渲染 (增量更新，不复建 artist) ──────
    def refresh(self, skip_path=False):
        # 删除已移除节点的 artist
        for nid in list(self._node_scatters.keys()):
            if nid not in self.data.nodes:
                self._node_scatters.pop(nid).remove()
                if nid in self._node_labels:
                    self._node_labels.pop(nid).remove()
        for nid in list(self._node_labels.keys()):
            if nid not in self.data.nodes:
                self._node_labels.pop(nid).remove()

        # 更新/创建节点
        for nid, nd in self.data.nodes.items():
            ntype = nd.get('type', 'key')
            seg = nd.get('seg', 'S1')
            color = SEG_COLORS.get(seg, '#888')
            marker, size = ('o', 12) if ntype == 'key' else ('D', 10)

            if nid in self._node_scatters:
                self._node_scatters[nid].set_data([nd['x']], [nd['y']])
            else:
                self._node_scatters[nid] = self.ax.plot(
                    nd['x'], nd['y'], marker=marker, markersize=size,
                    color=color, markeredgecolor='#ffffff',
                    markeredgewidth=1.5, zorder=25, pickradius=8)[0]

            is_sel = (nid == self.selected)
            if nid in self._node_labels:
                lbl = self._node_labels[nid]
                lbl.set_position((nd['x'], nd['y']))
                lbl.set_text(nd['name'])
                lbl.set_color('#ffffff' if is_sel else TEXT_COLOR)
                lbl.set_fontweight('bold' if is_sel else 'normal')
                if is_sel:
                    lbl.set_bbox(dict(boxstyle='round,pad=0.12', fc='#333366',
                                      alpha=0.9, ec=color, lw=1.2))
                else:
                    lbl.set_bbox(None)
            else:
                kw = dict(textcoords="offset points", xytext=(10, 8),
                          fontsize=6, fontweight='bold' if is_sel else 'normal',
                          color='#ffffff' if is_sel else TEXT_COLOR, zorder=26)
                if is_sel:
                    kw['bbox'] = dict(boxstyle='round,pad=0.12', fc='#333366',
                                      alpha=0.9, ec=color, lw=1.2)
                self._node_labels[nid] = self.ax.annotate(
                    nd['name'], (nd['x'], nd['y']), **kw)

        # 遍历连线
        if self._traversal_arrows:
            trav_line = self._traversal_arrows[0]
            if len(self.data.traversal) >= 2:
                xs = [self.data.nodes[nid]['x'] for nid in self.data.traversal
                      if nid in self.data.nodes]
                ys = [self.data.nodes[nid]['y'] for nid in self.data.traversal
                      if nid in self.data.nodes]
                trav_line.set_data(xs, ys)
                trav_line.set_visible(True)
            else:
                trav_line.set_visible(False)
        elif len(self.data.traversal) >= 2:
            xs = [self.data.nodes[nid]['x'] for nid in self.data.traversal
                  if nid in self.data.nodes]
            ys = [self.data.nodes[nid]['y'] for nid in self.data.traversal
                  if nid in self.data.nodes]
            self._traversal_arrows.append(
                self.ax.plot(xs, ys, 'o-', color='#ffcc66', linewidth=1.0,
                             markersize=3, alpha=0.7, zorder=22)[0])

        # 路径预览
        if skip_path:
            if self._preview_line:
                self._preview_line.set_visible(False)
        else:
            waypoints = interpolate_path(self.data)
            if len(waypoints) >= 2:
                xs = [w[0] for w in waypoints]
                ys = [w[1] for w in waypoints]
                if self._preview_line:
                    self._preview_line.set_data(xs, ys)
                    self._preview_line.set_visible(True)
                else:
                    self._preview_line, = self.ax.plot(
                        xs, ys, '-', color='#4ec9b0', linewidth=1.2,
                        alpha=0.6, zorder=21)
            elif self._preview_line:
                self._preview_line.set_visible(False)

        # 选中环
        if self.selected and self.selected in self.data.nodes:
            nd = self.data.nodes[self.selected]
            if self._selected_ring:
                self._selected_ring.center = (nd['x'], nd['y'])
                self._selected_ring.set_visible(True)
            else:
                self._selected_ring = Circle(
                    (nd['x'], nd['y']), 0.18, facecolor='none',
                    edgecolor='#ffcc00', linewidth=2, linestyle='-',
                    alpha=0.9, zorder=24)
                self.ax.add_patch(self._selected_ring)
        elif self._selected_ring:
            self._selected_ring.set_visible(False)

        self._update_info()
        self.fig.canvas.draw_idle()

    def _update_info(self):
        lines = [f"Mode: {self.mode.upper()} | Snap: {'ON' if self.data.snap_enabled else 'OFF'}"]
        if self.selected and self.selected in self.data.nodes:
            nd = self.data.nodes[self.selected]
            in_t = "T" if self.selected in self.data.traversal else "-"
            lines.append(
                f"Sel: [{nd['name']}] ({nd['x']:.3f}, {nd['y']:.3f}) "
                f"type={nd['type']} seg={nd['seg']} trav={in_t}")
        lines.append(
            f"Nodes: {len(self.data.nodes)} | Trav: {len(self.data.traversal)} | "
            f"Smooth: {self.data.smooth_mode}")
        lines.append(self.img.info())
        lines.append("Arrows=nudge img  +/-=scale img  Enter=save calib  Scroll=zoom  R=reset")
        self._info_text.set_text("\n".join(lines))

    # ── 加载 JSON ────────────────────────────
    def load_nodes(self, filepath: str = None):
        fp = filepath or DEFAULT_JSON
        data = load_json(fp)
        if not data:
            print(f"[ERROR] Cannot load {fp}")
            return

        self.data = PathEditorData()
        nodes_list = data.get('节点', [])
        max_id = 0
        for nd in nodes_list:
            nid = nd['id']
            self.data.nodes[nid] = {
                'name': nd.get('name', f'N{nid}'),
                'x': nd['x'], 'y': nd['y'],
                'type': nd.get('type', 'key'),
                'seg': nd.get('seg', _infer_segment(nd['x'], nd['y'])),
            }
            if nid > max_id:
                max_id = nid
        self.data.next_id = max_id + 1

        trav = data.get('遍历顺序', [])
        if trav:
            self.data.traversal = [t for t in trav if t in self.data.nodes]
        else:
            self.data.traversal = sorted(self.data.nodes.keys())

        sm = data.get('平滑模式', 'linear')
        if sm in AVAILABLE_SMOOTH:
            self.data.smooth_mode = sm
            self._radio_smooth.set_active(AVAILABLE_SMOOTH.index(sm))

        self.selected = None
        self.refresh()
        print(f"[LOADED] {fp}: {len(self.data.nodes)} nodes, "
              f"{len(self.data.traversal)} in traversal")


# ═══════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════
def main():
    print("=" * 55)
    print("  Cyberdog Track Path Planner")
    print("=" * 55)
    print(f"  Working dir: {PLANNER_DIR}")
    print(f"  Default JSON: {DEFAULT_JSON}")
    print(f"  Output JSON:  {OUTPUT_JSON}")
    print(f"  Output CSV:   {OUTPUT_CSV}")
    print(f"  Smooth modes: {AVAILABLE_SMOOTH}")
    print()
    print("  Controls:")
    print("    Left drag    — Move node")
    print("    Right click  — Add key node")
    print("    Double-click — Rename node")
    print("    Delete       — Remove selected node")
    print("    C / E        — Connect mode / Edit mode")
    print("    G            — Toggle grid snap")
    print("    Ctrl+Z       — Undo")
    print("    Ctrl+S       — Save JSON")
    print("    Esc          — Deselect")
    print("    Scroll       — Zoom")
    print("=" * 55)

    editor = PathEditor()

    if os.path.exists(DEFAULT_JSON):
        editor.load_nodes(DEFAULT_JSON)

    plt.show()


if __name__ == "__main__":
    main()
