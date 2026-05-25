import json
import numpy as np

with open('/home/cyberdog_sim/src/Option_One/地图与路径/关键节点.json', 'r') as f:
    nodes = json.load(f)['节点']
nodes_by_id = {n['id']: n for n in nodes}

traversal = [
    1, 2, 3, 31, 32, 6, 5, 4, 33, 34, 7, 8, 9, 11, 10, 12, 20, 19, 28, 13,
    14, 13, 28, 30, 16, 30, 29, 15, 17, 18, 15, 29, 20, 12, 21, 22, 23, 24,
    25, 27, 26, 27
]

step_distance = 0.02

all_points = []

for i, nid in enumerate(traversal):
    node = nodes_by_id[nid]
    x1, y1 = node['x'], node['y']

    if i == 0:
        all_points.append((x1, y1))
    else:
        prev_nid = traversal[i-1]
        prev_node = nodes_by_id[prev_nid]
        x0, y0 = prev_node['x'], prev_node['y']
        dist = np.sqrt((x1-x0)**2 + (y1-y0)**2)
        num_steps = max(1, int(dist / step_distance))
        for s in range(1, num_steps + 1):
            t = s / num_steps
            all_points.append((x0 + t*(x1-x0), y0 + t*(y1-y0)))

# Compute yaw for each waypoint (direction to next point)
csv_path = '/home/cyberdog_sim/src/full_track_nav/track_path_v11.csv'
with open(csv_path, 'w') as f:
    n = len(all_points)
    for i in range(n):
        x, y = all_points[i]
        # Forward difference: direction from current to next
        if i < n - 1:
            dx = all_points[i+1][0] - x
            dy = all_points[i+1][1] - y
        else:
            dx = x - all_points[i-1][0]
            dy = y - all_points[i-1][1]
        yaw = np.arctan2(dy, dx)
        f.write(f"{x:.4f},{y:.4f},{yaw:.6f}\n")

print(f"v11 path: {len(all_points)} waypoints (x,y,yaw), written to {csv_path}")
