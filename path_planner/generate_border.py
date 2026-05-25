#!/usr/bin/env python3
"""从 STL 网格文件提取赛道边界线，输出 border_data.json"""
import struct, math, numpy as np, json, os

MESH_DIR = "/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/model/race2026_meshes"
OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "border_data.json")
BORDER_FILES = ["1_border.stl", "2_border.stl", "3_border.stl"]

def parse_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        verts = []
        for _ in range(n):
            f.read(12)
            for __ in range(3):
                verts.append(struct.unpack('<fff', f.read(12)))
            f.read(2)
    return verts

def cluster_xs(xs, gap_threshold=0.25):
    """按间隙聚簇 X 值，返回各簇均值"""
    xs_sorted = np.sort(xs)
    clusters, current = [], [xs_sorted[0]]
    for i in range(1, len(xs_sorted)):
        if xs_sorted[i] - current[-1] > gap_threshold:
            clusters.append(current)
            current = [xs_sorted[i]]
        else:
            current.append(xs_sorted[i])
    if current:
        clusters.append(current)
    return [float(np.mean(c)) for c in clusters]

def smooth(data, window=3):
    out = []
    for i in range(len(data)):
        lo, hi = max(0, i - window // 2), min(len(data), i + window // 2 + 1)
        avg_x = float(np.mean([d[0] for d in data[lo:hi]]))
        out.append((avg_x, data[i][1]))
    return out

def main():
    yaw, scale = -1.57, 0.01
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    all_pts = []
    for name in BORDER_FILES:
        path = os.path.join(MESH_DIR, name)
        for v in parse_stl(path):
            wx = 1.5 + (v[0] * cos_y - v[1] * sin_y) * scale
            wy = 5.87 + (v[0] * sin_y + v[1] * cos_y) * scale
            all_pts.append((wx, wy))

    pts = np.array(all_pts)
    left_pts, right_pts = [], []
    bins = np.arange(-0.65, 15.80, 0.025)

    for i in range(len(bins) - 1):
        mask = (pts[:, 1] >= bins[i]) & (pts[:, 1] < bins[i + 1])
        if mask.sum() < 4:
            continue
        y = (bins[i] + bins[i + 1]) / 2
        clusters = cluster_xs(pts[mask, 0])
        if len(clusters) >= 2:
            left_pts.append((clusters[0], y))
            right_pts.append((clusters[-1], y))

    left_pts = smooth(left_pts, 3)
    right_pts = smooth(right_pts, 3)

    # 确保 JSON 可序列化
    left_pts = [(float(x), float(y)) for x, y in left_pts]
    right_pts = [(float(x), float(y)) for x, y in right_pts]
    polygon = left_pts + list(reversed(right_pts))

    with open(OUTPUT, 'w') as f:
        json.dump({'left': left_pts, 'right': right_pts, 'polygon': polygon}, f)

    print(f"Generated {OUTPUT}: {len(left_pts)} left + {len(right_pts)} right border points")
    print(f"STL source: {len(pts)} vertices from {len(BORDER_FILES)} files")

if __name__ == '__main__':
    main()
