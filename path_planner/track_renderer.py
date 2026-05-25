"""
track_renderer.py — 赛道鸟瞰底图: 坐标系 + 静态背景图
"""
import os, json
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

BG_COLOR    = '#0d1117'
GRID_COLOR  = '#21262d'
AXIS_COLOR  = '#555555'
TEXT_COLOR  = '#c9d1d9'

SEG_COLORS = {
    'S1': '#4ec94e', 'S2': '#569cd6', 'S3': '#c586c0',
    'S4': '#e05555', 'S5': '#d4a84b', 'S6': '#e06c75',
}

BG_IMAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_bg.jpg')

# 默认视图范围
DEFAULT_XMIN, DEFAULT_XMAX = -1.5, 5.5
DEFAULT_YMIN, DEFAULT_YMAX = -1.5, 17.0


CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bg_calib.json')


class ImageAdjuster:
    """背景图独立调节器：平移/缩放图片以对齐坐标网格"""

    def __init__(self, ax):
        self.ax = ax
        self.xmin, self.xmax = DEFAULT_XMIN, DEFAULT_XMAX
        self.ymin, self.ymax = DEFAULT_YMIN, DEFAULT_YMAX
        self._img_artist = None
        self._img_data = None
        if os.path.exists(BG_IMAGE):
            self._img_data = mpimg.imread(BG_IMAGE)
        self._load()

    def _load(self):
        try:
            with open(CALIB_FILE) as f:
                d = json.load(f)
            self.xmin, self.xmax = d['xmin'], d['xmax']
            self.ymin, self.ymax = d['ymin'], d['ymax']
        except Exception:
            pass

    def save(self):
        with open(CALIB_FILE, 'w') as f:
            json.dump({'xmin': self.xmin, 'xmax': self.xmax,
                       'ymin': self.ymin, 'ymax': self.ymax}, f)

    def draw(self):
        if self._img_data is not None:
            self._img_artist = self.ax.imshow(
                self._img_data,
                extent=[self.xmin, self.xmax, self.ymin, self.ymax],
                aspect='equal', zorder=0, origin='upper')

    def nudge(self, dx, dy):
        """平移图片(世界坐标单位)"""
        self.xmin += dx; self.xmax += dx
        self.ymin += dy; self.ymax += dy
        self._apply()

    def scale(self, factor):
        """以图片中心缩放"""
        cx = (self.xmin + self.xmax) / 2
        cy = (self.ymin + self.ymax) / 2
        hw = (self.xmax - self.xmin) / 2 * factor
        hh = (self.ymax - self.ymin) / 2 * factor
        self.xmin = cx - hw; self.xmax = cx + hw
        self.ymin = cy - hh; self.ymax = cy + hh
        self._apply()

    def _apply(self):
        if self._img_artist is not None:
            self._img_artist.set_extent([self.xmin, self.xmax, self.ymin, self.ymax])
            self.ax.figure.canvas.draw_idle()

    def info(self):
        return (f"Img: X[{self.xmin:.2f},{self.xmax:.2f}] "
                f"Y[{self.ymin:.2f},{self.ymax:.2f}] "
                f"({self.xmax-self.xmin:.1f}x{self.ymax-self.ymin:.1f}m)")


def draw_grid(ax):
    """绘制坐标网格 + 原点十字标记"""
    x0, x1 = DEFAULT_XMIN, DEFAULT_XMAX
    y0, y1 = DEFAULT_YMIN, DEFAULT_YMAX
    # 1m 网格
    for x in np.arange(np.floor(x0), x1 + 0.5, 1.0):
        ax.axvline(x, color=GRID_COLOR, linewidth=0.4, zorder=3)
    for y in np.arange(np.floor(y0), y1 + 0.5, 1.0):
        ax.axhline(y, color=GRID_COLOR, linewidth=0.4, zorder=3)
    # 原点坐标轴 (粗线)
    ax.axhline(0, color=AXIS_COLOR, linewidth=1.2, zorder=4)
    ax.axvline(0, color=AXIS_COLOR, linewidth=1.2, zorder=4)
    # 原点大标记
    ax.plot(0, 0, marker='+', color='#ff4444', markersize=20,
            markeredgewidth=2.5, zorder=10)
    ax.plot(0, 0, marker='o', color='#ff4444', markersize=8,
            markerfacecolor='none', markeredgewidth=2, zorder=10)
    # 刻度数字标记
    for x in range(int(np.floor(x0)), int(np.ceil(x1)) + 1):
        if x != 0:
            ax.text(x, 0.08, str(x), fontsize=6, color=AXIS_COLOR,
                    ha='center', va='bottom', zorder=5)
    for y in range(int(np.floor(y0)), int(np.ceil(y1)) + 1):
        if y != 0:
            ax.text(0.08, y, str(y), fontsize=6, color=AXIS_COLOR,
                    ha='left', va='center', zorder=5)
    ax.text(0.08, 0.08, 'O', fontsize=8, color='#ff4444', fontweight='bold',
            ha='left', va='bottom', zorder=10)


def setup_axes(figsize=(10, 20)):
    """创建带坐标系的窗口"""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    fig.subplots_adjust(left=0.06, right=0.94, bottom=0.04, top=0.97)
    ax.set_xlim(DEFAULT_XMIN, DEFAULT_XMAX)
    ax.set_ylim(DEFAULT_YMIN, DEFAULT_YMAX)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)  →', fontsize=9, color=TEXT_COLOR)
    ax.set_ylabel('Y (m)  →', fontsize=9, color=TEXT_COLOR)
    ax.set_title('Click to place nodes  |  Scroll to zoom  |  C=connect  E=edit',
                 fontsize=9, color=TEXT_COLOR)
    ax.tick_params(colors='#666666', labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#333333')
    return fig, ax


class ViewController:
    """管理视图缩放，保持节点世界坐标不变"""

    def __init__(self, ax):
        self.ax = ax
        self.xmin, self.xmax = DEFAULT_XMIN, DEFAULT_XMAX
        self.ymin, self.ymax = DEFAULT_YMIN, DEFAULT_YMAX

    def zoom(self, factor, cx, cy):
        """以世界坐标 (cx,cy) 为中心缩放视图"""
        self.xmin = cx - (cx - self.xmin) * factor
        self.xmax = cx + (self.xmax - cx) * factor
        self.ymin = cy - (cy - self.ymin) * factor
        self.ymax = cy + (self.ymax - cy) * factor
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.figure.canvas.draw_idle()

    def reset(self):
        self.xmin, self.xmax = DEFAULT_XMIN, DEFAULT_XMAX
        self.ymin, self.ymax = DEFAULT_YMIN, DEFAULT_YMAX
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.figure.canvas.draw_idle()

    def pan(self, dx, dy):
        self.xmin += dx; self.xmax += dx
        self.ymin += dy; self.ymax += dy
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.figure.canvas.draw_idle()

    def info(self):
        return (f"View: X[{self.xmin:.1f},{self.xmax:.1f}] "
                f"Y[{self.ymin:.1f},{self.ymax:.1f}] "
                f"({self.xmax-self.xmin:.1f}x{self.ymax-self.ymin:.1f}m)")
