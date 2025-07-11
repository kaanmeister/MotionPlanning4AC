#!/usr/bin/env python3
# main.py
"""
This script demonstrates a more complete motion planner prototype by:
-full lattice of candidate paths using pyclothoids instead of creating manual paths.
-added an obstacle to see if it will avoid
-obstacle is also in the frenet frame (SL)
-evaluating the paths in lattice for collision
-green paths are not collides (good to go), reds paths collides
"""

import numpy
import matplotlib.pyplot as plt
import math
from pyclothoids import Clothoid
from shapely.geometry import Polygon, LineString
from scipy.optimize import minimize_scalar

def project_point_to_clothoid(x, y, clothoid):
    """
    projects a Cartesian point (x, y) to a clothoid curve to find its
    frenet coordinates (s, l).
    """
    def distance_sq(s, x_point, y_point, c):
        #get X and Y coordinates separately at a given s
        x_c = c.X(s) 
        y_c = c.Y(s) 
        return (x_c - x_point)**2 + (y_c - y_point)**2

    res = minimize_scalar(
        distance_sq,
        args=(x, y, clothoid),
        bounds=(0, clothoid.length),
        method='bounded'
    )
    s_star = res.x
    
    x_clothoid = clothoid.X(s_star)
    y_clothoid = clothoid.Y(s_star)
    
    l_dist = math.sqrt((x - x_clothoid)**2 + (y - y_clothoid)**2)
    theta = clothoid.Theta(s_star)
    angle_to_point = math.atan2(y - y_clothoid, x - x_clothoid)
    delta_angle = (angle_to_point - theta + math.pi) % (2 * math.pi) - math.pi
    if delta_angle < 0:
        l_dist *= -1
    return s_star, l_dist


reference_line = Clothoid.G1Hermite(0, 0, 0, 4, 0, 0)
obstacle_cartesian_corners = [(1.5, -0.25), (2.5, -0.25), (2.5, 0.25), (1.5, 0.25)]
obstacle_cartesian_poly = Polygon(obstacle_cartesian_corners)
sl_corners = [project_point_to_clothoid(p[0], p[1], reference_line) for p in obstacle_cartesian_corners]
obstacle_sl_poly = Polygon(sl_corners)

plt.figure(figsize=(12, 8))
plt.plot(*reference_line.SampleXY(500), 'k--', label="Reference Line")
plt.plot(*obstacle_cartesian_poly.exterior.xy, 'r-', linewidth=2, label="Obstacle", zorder=30)

colliding_paths = 0
total_paths = 0
n = 3 #5

for l_start in range(-n, n + 1):
    path_segment_1 = Clothoid.G1Hermite(0, 0, 0, 1, l_start/10,0) # 0,0,0,1 10., 0 
    for l_end in range(-n, n + 1):
        total_paths += 1
        full_path = Clothoid.G1Hermite(1, l_start/5., path_segment_1.ThetaEnd, 5, l_end/5.0,0) #10.,           /10., 0
        #full_path = Clothoid.G1Hermite(1, l_start/5., path_segment_1.ThetaEnd, 5, l_end/5,0)
        path_x, path_y = full_path.SampleXY(20) #20
        s_path, l_path = [], []
        for px, py in zip(path_x, path_y):
            s, l = project_point_to_clothoid(px, py, reference_line)
            s_path.append(s)
            l_path.append(l)

        path_sl_line = LineString(zip(s_path, l_path))

        #collision check
        is_colliding = path_sl_line.intersects(obstacle_sl_poly)
        #color = 'black'
        path_color = 'red' if is_colliding else 'green'
        path_alpha = 0.7 if is_colliding else 0.25
        z_order = 1 if is_colliding else 0
        plt.plot(*full_path.SampleXY(100), color=path_color, alpha=path_alpha, zorder=z_order)
        
        if is_colliding:
            colliding_paths += 1

print(f"Total paths generated: {total_paths}")
print(f"Found {colliding_paths} colliding paths.")
print(f"Found {total_paths - colliding_paths} FEASIBLE paths.")

plt.title("Lattice with Collision")
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.legend()
plt.grid(True)
plt.axis('equal')


plt.show()