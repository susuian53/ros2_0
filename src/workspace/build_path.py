#!/usr/bin/env python3
"""
build_path.py — 从 path_config.py 生成路径 CSV + 可视化预览

用法:
    python3 /home/cyberdog_sim/src/workspace/build_path.py

输出:
    /home/cyberdog_sim/src/workspace/track_path.csv
    /home/cyberdog_sim/src/workspace/track_path_preview.png   ← 可视化图片
"""

import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from path_config import WAYPOINTS, STEP

WORKDIR  = os.path.dirname(os.path.abspath(__file__))
OUT_CSV  = os.path.join(WORKDIR, "track_path.csv")
OUT_PNG  = os.path.join(WORKDIR, "track_path_preview.png")

# ── 赛道背景 (区域边界, 单位: 米) ──
TRACK_BG = [
    ("Area1",  -0.74, 3.49, -0.63,  6.67, '#FFD700', 0.12),
    ("Area2",  -0.74, 3.49,  6.55, 12.16, '#FFD700', 0.12),
    ("Area3",  -0.74, 3.49, 12.05, 15.78, '#FFD700', 0.12),
    ("Rock",    0.60, 2.40, -0.52,  0.48, '#A0522D', 0.35),
    ("Bridge",  2.87, 3.38,  7.66, 12.16, '#DEB887', 0.50),
    ("Slope",  -0.63, 3.37, 12.16, 15.66, '#A9A9A9', 0.30),
]


def build(points, step=0.02):
    all_pts = [points[0]]
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(dist / step))
        for s in range(1, n + 1):
            t = s / n
            all_pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return all_pts


def main():
    pts = [(x, y) for _, x, y in WAYPOINTS]
    waypoints = build(pts, STEP)
    n = len(waypoints)

    # ── 写 CSV ──
    with open(OUT_CSV, 'w') as f:
        for i in range(n):
            x, y = waypoints[i]
            if i < n - 1:
                dx = waypoints[i + 1][0] - x
                dy = waypoints[i + 1][1] - y
            else:
                dx = x - waypoints[i - 1][0]
                dy = y - waypoints[i - 1][1]
            yaw = math.atan2(dy, dx)
            f.write(f"{x:.4f},{y:.4f},{yaw:.6f}\n")

    print(f"[BUILD] {len(WAYPOINTS)} nodes → {n} waypoints → {OUT_CSV}")
    print(f"  Start: ({waypoints[0][0]:.2f},{waypoints[0][1]:.2f})")
    print(f"  End:   ({waypoints[-1][0]:.2f},{waypoints[-1][1]:.2f})")

    # ── 画图 ──
    try:
        _render(waypoints, pts)
        print(f"[PREVIEW] {OUT_PNG}")
    except Exception as e:
        print(f"[PREVIEW] skipped ({e})")


def _render(waypoints, nodes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(5, 17))
    ax.set_xlim(-1.5, 5.5)
    ax.set_ylim(-1.0, 16.5)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('Track Path Preview', fontweight='bold', fontfamily='monospace')
    ax.grid(True, alpha=0.25)

    # 赛道背景
    for name, x0, x1, y0, y1, color, alpha in TRACK_BG:
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0,
                     fc=color, alpha=alpha, ec='none'))

    # 路径线
    wx = [w[0] for w in waypoints]
    wy = [w[1] for w in waypoints]
    ax.plot(wx, wy, '-', color='#00ccff', linewidth=1.2, alpha=0.8, label=f'{len(waypoints)} waypoints')

    # 关键节点
    for i, (name, x, y) in enumerate(WAYPOINTS):
        ax.plot(x, y, 'o', color='#ff4444', markersize=5, zorder=10)
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(4, 5),
                    fontsize=5, color='white',
                    bbox=dict(boxstyle='round,pad=0.1', fc='#333333', alpha=0.7))

    # 起点/终点
    ax.plot(waypoints[0][0], waypoints[0][1], 'P', color='lime', markersize=14, zorder=15)
    ax.plot(waypoints[-1][0], waypoints[-1][1], 'X', color='red', markersize=14, zorder=15)

    ax.legend(fontsize=6, loc='upper right')
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()


if __name__ == "__main__":
    main()
