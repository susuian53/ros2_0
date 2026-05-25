from matplotlib import font_manager
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle, Circle
import numpy as np

# Use the exact Chinese font file
font_path = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
fp = font_manager.FontProperties(fname=font_path)
fp_small = font_manager.FontProperties(fname=font_path, size=7)
fp_mid = font_manager.FontProperties(fname=font_path, size=8)
fp_big = font_manager.FontProperties(fname=font_path, size=14, weight='bold')
fp_legend = font_manager.FontProperties(fname=font_path, size=7)
fp_title = font_manager.FontProperties(fname=font_path, size=14, weight='bold')
fp_label = font_manager.FontProperties(fname=font_path, size=12)

fig, ax = plt.subplots(1, 1, figsize=(10, 32))
ax.set_xlim(-1.5, 5.5)
ax.set_ylim(-1.5, 17)
ax.set_aspect('equal')
ax.set_xlabel('X (m) ->', fontsize=12, fontproperties=fp)
ax.set_ylabel('Y (m) ->', fontsize=12, fontproperties=fp)
ax.set_title('机器狗仿真竞赛 - 赛道地图 (俯视图)', fontsize=14, fontweight='bold', fontproperties=fp_title)
ax.grid(True, alpha=0.3, linestyle='--')

# ============================================================
# Border regions
# ============================================================
border_regions = [
    ("Area1 边界黄线", (-0.73, 3.49), (-0.63, 6.67)),
    ("Area2 边界黄线", (-0.74, 3.49), (6.55, 12.16)),
    ("Area3 边界黄线", (-0.74, 3.48), (12.05, 15.78)),
]

for name, xr, yr in border_regions:
    rect = Rectangle((xr[0], yr[0]), xr[1]-xr[0], yr[1]-yr[0],
                      linewidth=2, edgecolor='#FFD700', facecolor='#FFD700', alpha=0.12, linestyle='-')
    ax.add_patch(rect)

# ============================================================
# Rock road
# ============================================================
rock = Rectangle((0.60, -0.52), 2.40-0.60, 0.48-(-0.52), linewidth=1,
                 edgecolor='#8B4513', facecolor='#A0522D', alpha=0.45, hatch='....')
ax.add_patch(rock)
ax.text(1.5, -0.02, '岩石路 (4块,30cm宽)', fontsize=7, ha='center', va='center',
        color='#5C3317', fontproperties=fp_small)

# ============================================================
# Bridge
# ============================================================
bridge = Rectangle((2.87, 7.66), 3.38-2.87, 12.16-7.66, linewidth=2,
                   edgecolor='#8B4513', facecolor='#DEB887', alpha=0.55, hatch='////')
ax.add_patch(bridge)
ax.text(3.12, 9.9, '独木桥', fontsize=9, ha='center', va='center',
        color='#8B4513', fontweight='bold', fontproperties=fp_mid)

# ============================================================
# Goal
# ============================================================
goal = Rectangle((1.78, 11.34), 2.35-1.78, 11.36-11.34, linewidth=2,
                 edgecolor='#FF4500', facecolor='#FF6347', alpha=0.55)
ax.add_patch(goal)
ax.text(2.07, 11.18, '球门\n50x30cm', fontsize=6, ha='center', va='top',
        color='#FF4500', fontproperties=fp_small)

# ============================================================
# Obstacles
# ============================================================
obs1 = Rectangle((0.72, 8.46), 0.20, 0.20, linewidth=1.5, edgecolor='red', facecolor='red', alpha=0.55)
obs2 = Rectangle((1.02, 8.46), 0.20, 0.20, linewidth=1.5, edgecolor='red', facecolor='red', alpha=0.55)
ax.add_patch(obs1)
ax.add_patch(obs2)
ax.text(0.97, 8.33, '障碍物 (2x20cm)', fontsize=6, ha='center', va='top',
        color='red', fontproperties=fp_small)

# ============================================================
# Height bars
# ============================================================
bar1 = Rectangle((-0.63, 9.55), 0.37-(-0.63), 9.65-9.55, linewidth=2,
                 edgecolor='red', facecolor='red', alpha=0.65)
bar2 = Rectangle((1.57, 10.53), 2.57-1.57, 10.63-10.53, linewidth=2,
                 edgecolor='red', facecolor='red', alpha=0.65)
ax.add_patch(bar1)
ax.add_patch(bar2)
ax.text(-0.13, 9.42, '限高杆1', fontsize=6, ha='center', va='top',
        color='red', fontproperties=fp_small)
ax.text(2.07, 10.40, '限高杆2', fontsize=6, ha='center', va='top',
        color='red', fontproperties=fp_small)

# ============================================================
# Slope
# ============================================================
slope = Rectangle((-0.63, 12.16), 3.37-(-0.63), 15.66-12.16, linewidth=1,
                  edgecolor='#696969', facecolor='#A9A9A9', alpha=0.35, hatch='///')
ax.add_patch(slope)
ax.text(1.37, 13.9, '斜坡', fontsize=10, ha='center', va='center',
        color='#333333', fontweight='bold', fontproperties=fp_mid)

# ============================================================
# Independent objects
# ============================================================
# football2
c = Circle((2.1, 10.8), 0.1, facecolor='white', edgecolor='black', linewidth=2, zorder=10)
ax.add_patch(c)
ax.text(2.1, 10.57, '足球2', fontsize=7, ha='center', va='top',
        color='black', fontproperties=fp_small)

# coke
coke = Rectangle((-0.1-0.065, 11.1-0.17), 0.13, 0.34, linewidth=1.5,
                 edgecolor='#1a1a1a', facecolor='#333333', alpha=0.8, zorder=10)
ax.add_patch(coke)
ax.text(-0.1, 10.80, '可乐', fontsize=7, ha='center', va='top',
        color='#1a1a1a', fontproperties=fp_small)

# football3
c3 = Circle((0.4, 14.7), 0.1, facecolor='white', edgecolor='black', linewidth=2, zorder=10)
ax.add_patch(c3)
ax.text(0.4, 14.47, '足球3', fontsize=7, ha='center', va='top',
        color='black', fontproperties=fp_small)

# ============================================================
# Hanging ball matrix (4x4)
# ============================================================
grid_cols_x = [-0.4, 0.8, 2.0, 3.2]
grid_rows_y = [1.34, 2.18, 3.02, 3.86]
grid_colors = [
    ['蓝', '橙', '蓝', '蓝'],
    ['蓝', '蓝', '橙', '蓝'],
    ['蓝', '蓝', '蓝', '橙'],
    ['橙', '蓝', '蓝', '蓝'],
]

for ri, row_y in enumerate(grid_rows_y):
    for ci, col_x in enumerate(grid_cols_x):
        color_key = grid_colors[ri][ci]
        is_orange = '橙' in color_key
        fc = '#E67E22' if is_orange else '#2980B9'
        ec = '#D35400' if is_orange else '#1F618D'
        c = Circle((col_x, row_y), 0.1, facecolor=fc, edgecolor=ec,
                    linewidth=1.5, zorder=8, alpha=0.85)
        ax.add_patch(c)
        if is_orange:
            c_inner = Circle((col_x, row_y), 0.04, facecolor='yellow',
                              edgecolor='none', alpha=0.7, zorder=9)
            ax.add_patch(c_inner)

ax.text(1.4, 0.7, '悬挂球阵 4x4 (蓝/橙)', fontsize=8, ha='center', va='top',
        color='#2980B9', fontproperties=fp_small)

# Area4 hanging orange ball
c4 = Circle((0.95, 11.1), 0.1, facecolor='#E67E22', edgecolor='#D35400',
             linewidth=2, zorder=10)
ax.add_patch(c4)
ax.text(0.95, 10.80, '悬挂橙球 (距地60cm)', fontsize=6, ha='center', va='top',
        color='#E67E22', fontproperties=fp_small)

# ============================================================
# Channel centerlines
# ============================================================
channels = [
    ("Ch1_左", -0.14, 8.5, 11.5, '#E74C3C'),
    ("Ch2_中", 0.96, 8.5, 11.5, '#2ECC71'),
    ("Ch3_右", 2.06, 8.5, 11.5, '#3498DB'),
]

for ch_name, cx, y0, y1, color in channels:
    ax.axvline(x=cx, ymin=(y0+1.5)/18.5, ymax=(y1+1.5)/18.5,
               color=color, linewidth=2, linestyle='--', alpha=0.7)
    ax.text(cx, y1+0.2, ch_name, fontsize=7, ha='center', va='bottom',
            color=color, fontweight='bold', fontproperties=fp_small)

# 3-channel region highlight
ch_region = Rectangle((-0.64, 8.5), 2.56-(-0.64), 11.5-8.5,
                      linewidth=1, edgecolor='#9B59B6', facecolor='#D2B4DE',
                      alpha=0.12, linestyle='--')
ax.add_patch(ch_region)
ax.text(0.96, 11.9, 'Area4: 三条竖向通道', fontsize=8, ha='center', va='bottom',
        color='#9B59B6', fontproperties=fp_small)

# ============================================================
# Key narrow passage markers
# ============================================================
narrow_points = [
    (5.0, -0.15, '左转弯道 78cm', '#E74C3C'),
    (6.0, 2.60, '右侧通道 104cm', '#E67E22'),
    (12.0, 3.11, '桥面入口 50cm', '#8E44AD'),
    (13.0, -0.39, '左侧窄道 50cm', '#2980B9'),
]

for yy, xx, label, color in narrow_points:
    ax.plot(xx, yy, 's', color=color, markersize=10, zorder=11, markeredgecolor='black', markeredgewidth=0.5)
    ax.text(xx+0.15, yy, label, fontsize=6, ha='left', va='center',
            color=color, fontweight='bold', fontproperties=fp_small)

# ============================================================
# Competition segments
# ============================================================
segments = [
    ("S1 石径探路", 0.5, 1.5, 0.0, 4.5, '#27AE60'),
    ("S2 荒野寻珠", 1.4, 1.4, 1.3, 4.0, '#2980B9'),
    ("S3 曲道冲锋", 1.4, 2.0, 4.0, 7.5, '#8E44AD'),
    ("S4 深隧寻珍", 0.96, 2.0, 7.5, 11.5, '#E74C3C'),
    ("S5 孤梁稳渡", 3.12, 3.12, 7.7, 12.2, '#D35400'),
    ("S6 撷金建功", 1.37, 1.37, 12.2, 15.7, '#C0392B'),
]

for sname, x0, x1, y0, y1, color in segments:
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color=color, lw=3, alpha=0.55,
                               connectionstyle='arc3,rad=0'))
    ax.text(x0+0.15, (y0+y1)/2, sname, fontsize=7, ha='left', va='center',
            color=color, fontweight='bold', fontproperties=fp_small)

# ============================================================
# Start point
# ============================================================
ax.plot(0, 0, 'P', color='green', markersize=20, zorder=15,
        markeredgecolor='darkgreen', markeredgewidth=2)
ax.text(0.2, -0.1, '起点 (0,0)', fontsize=9, ha='left', va='top',
        color='green', fontweight='bold', fontproperties=fp_label)

# ============================================================
# Legend
# ============================================================
legend_elements = [
    mpatches.Patch(facecolor='#FFD700', alpha=0.2, label='边界黄线区域'),
    mpatches.Patch(facecolor='#A0522D', alpha=0.45, label='岩石路'),
    mpatches.Patch(facecolor='#DEB887', alpha=0.55, label='独木桥'),
    mpatches.Patch(facecolor='#A9A9A9', alpha=0.35, label='斜坡'),
    mpatches.Patch(facecolor='#D2B4DE', alpha=0.15, label='三通道区域'),
    mpatches.Patch(facecolor='red', alpha=0.55, label='障碍物/限高杆'),
    mpatches.Patch(facecolor='#E67E22', alpha=0.85, label='橙色球 (悬挂)'),
    mpatches.Patch(facecolor='#2980B9', alpha=0.85, label='蓝色球 (悬挂)'),
    mpatches.Patch(facecolor='white', edgecolor='black', label='足球'),
    mpatches.Patch(facecolor='#333333', alpha=0.8, label='可乐瓶'),
    mpatches.Patch(facecolor='#FF6347', alpha=0.55, label='球门'),
]
ax.legend(handles=legend_elements, loc='upper right', fontsize=7, ncol=2,
          bbox_to_anchor=(1.02, 1.02), framealpha=0.9, prop=fp_legend)

# ============================================================
# Dimension lines
# ============================================================
ax.annotate('', xy=(-0.74, -1.0), xytext=(3.49, -1.0),
            arrowprops=dict(arrowstyle='<->', color='gray', lw=1))
ax.text(1.37, -1.17, '宽 4.2m', fontsize=8, ha='center', color='gray', fontproperties=fp_small)

ax.annotate('', xy=(-1.2, -0.63), xytext=(-1.2, 15.78),
            arrowprops=dict(arrowstyle='<->', color='gray', lw=1))
ax.text(-1.38, 7.5, '长 16.4m', fontsize=8, ha='center', va='center',
        rotation=90, color='gray', fontproperties=fp_small)

# Area labels
ax.text(-0.73+1.5, -0.63+3.5, 'Area 1\n起点区', fontsize=10, ha='center', va='center',
        color='#8B4513', alpha=0.6, fontweight='bold', fontproperties=fp_mid)
ax.text(-0.74+1.5, 6.55+2.5, 'Area 2\n球阵/桥/障碍', fontsize=10, ha='center', va='center',
        color='#2980B9', alpha=0.6, fontweight='bold', fontproperties=fp_mid)
ax.text(-0.74+1.5, 12.05+1.5, 'Area 3\n斜坡/终点', fontsize=10, ha='center', va='center',
        color='#696969', alpha=0.6, fontweight='bold', fontproperties=fp_mid)

plt.tight_layout()
plt.savefig('/home/cyberdog_sim/赛道地图.png', dpi=200, bbox_inches='tight',
            facecolor='white', edgecolor='none')
print("地图已保存到: /home/cyberdog_sim/赛道地图.png")
plt.close()
