#!/usr/bin/env python3
# main.py
"""
This script implements a robust DP path planner (i hope).
It handles impossible path segments and finds the true optimal path
from the set of all feasible, complete trajectories.
"""

import numpy
import matplotlib.pyplot as plt
import math
import networkx as nx
from pyclothoids import Clothoid
from shapely.geometry import Polygon, LineString
from scipy.optimize import minimize_scalar

def project_point_to_clothoid(x, y, clothoid):
    def distance_sq(s, x_point, y_point, c):
        x_c, y_c = c.X(s), c.Y(s)
        return (x_c - x_point)**2 + (y_c - y_point)**2

    res = minimize_scalar(
        distance_sq, args=(x, y, clothoid),
        bounds=(0, clothoid.length), method='bounded'
    )
    s_star = res.x
    x_clothoid, y_clothoid = clothoid.X(s_star), clothoid.Y(s_star)
    l_dist = math.sqrt((x - x_clothoid)**2 + (y - y_clothoid)**2)
    theta = clothoid.Theta(s_star)
    angle_to_point = math.atan2(y - y_clothoid, x - x_clothoid)
    delta_angle = (angle_to_point - theta + math.pi) % (2 * math.pi) - math.pi
    if delta_angle < 0:
        l_dist *= -1
    return s_star, l_dist

def calculate_cost(path_segment, ref_line, obs_sl_poly):
    W_SMOOTHNESS, W_OBS_DISTANCE, W_GUIDANCE = 0.15, 0.3, 0.2 #0.3, 0.4, 0.3
    path_x, path_y = path_segment.SampleXY(10)
    s_path, l_path = [], []
    for px, py in zip(path_x, path_y):
        s, l = project_point_to_clothoid(px, py, ref_line)
        s_path.append(s); l_path.append(l)
    path_sl_line = LineString(zip(s_path, l_path))
    
    if path_sl_line.intersects(obs_sl_poly): return float('inf')
    cost_smooth = abs(path_segment.dk) * path_segment.length
    dist_to_obs = path_sl_line.distance(obs_sl_poly)
    cost_obs = 1 / (dist_to_obs + 1e-6)
    cost_guide = sum([l**2 for l in l_path]) / len(l_path)
    return (W_SMOOTHNESS * cost_smooth + W_OBS_DISTANCE * cost_obs + W_GUIDANCE * cost_guide)

reference_line = Clothoid.G1Hermite(0, 0, 0, 10, 0, 0) #(0, 0, 0, 4, 0, 0)
obstacle_cartesian_corners = [(2.5, -1.0), (2.5, -1.0), (2.5, 1.0), (2.5, 1.0)] #[(1.5, -0.25), (2.5, -0.25), (2.5, 0.25), (1.5, 0.25)]
obstacle_cartesian_poly = Polygon(obstacle_cartesian_corners)
sl_corners = [project_point_to_clothoid(p[0], p[1], reference_line) for p in obstacle_cartesian_corners]
obstacle_sl_poly = Polygon(sl_corners)


G = nx.DiGraph()
n = 5
G.add_node("start")
for i in range(-n, n+1):
    G.add_node(f"1_{i}"); G.add_node(f"4_{i}")

for l_start_idx in range(-n, n+1):
    y_connection = (l_start_idx / 2.)
    #full_path = Clothoid.G1Hermite(1, l_start_idx/5., path_seg1.ThetaEnd, 5, l_end_idx/5.0,0)
    path_seg1 = Clothoid.G1Hermite(0, 0, 0, 2, y_connection, 0) #(0, 0, 0, 1, y_connection, 0)
    cost1 = calculate_cost(path_seg1, reference_line, obstacle_sl_poly)
    G.add_edge("start", f"1_{l_start_idx}", weight=cost1, path=path_seg1)
    
    for l_end_idx in range(-n, n + 1):
        path_seg2 = Clothoid.G1Hermite(2, y_connection, path_seg1.ThetaEnd, 10, l_end_idx / 10., 0)
        
        if path_seg2 is None:
            cost2 = float('inf')
        else:
            cost2 = calculate_cost(path_seg2, reference_line, obstacle_sl_poly)
        
        G.add_edge(f"1_{l_start_idx}", f"4_{l_end_idx}", weight=cost2, path=path_seg2)

min_cost = float('inf')
best_end_node = None

for i in range(-n, n + 1):
    end_node = f"4_{i}"
    try:
        cost = nx.dijkstra_path_length(G, "start", end_node)
        if cost < min_cost:
            min_cost = cost
            best_end_node = end_node
    except nx.NetworkXNoPath:
        # end node is not reachable, so we ignore it
        continue

if best_end_node is None:
    print("No complete, feasible path could be found by the planner!")
else:
    optimal_path_nodes = nx.dijkstra_path(G, "start", best_end_node)
    print(f"Optimal Path Found: {' -> '.join(optimal_path_nodes)}")

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_title("Final Planner Result")
    ax.set_xlabel("X coordinate"); ax.set_ylabel("Y coordinate")
    ax.plot(*reference_line.SampleXY(500), 'k--', label="Reference Line")
    ax.plot(*obstacle_cartesian_poly.exterior.xy, color='black', linewidth=3, label="Obstacle", zorder=10)

    for u, v, data in G.edges(data=True):
        if 'path' in data and data['path'] is not None:
             path_color = 'red' if data['weight'] == float('inf') else 'green'
             path_alpha = 0.5 if path_color == 'red' else 0.1
             ax.plot(*data['path'].SampleXY(100), color=path_color, alpha=path_alpha)

    for i in range(len(optimal_path_nodes) - 1):
        path_segment = G.edges[optimal_path_nodes[i], optimal_path_nodes[i+1]]['path']
        label = "Optimal Path" if i == 0 else ""
        ax.plot(*path_segment.SampleXY(100), color='blue', linewidth=3, zorder=5, label=label)

    ax.legend(); ax.grid(True); ax.axis('equal')
    plt.show()