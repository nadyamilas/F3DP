"""
Experiments and generating the design options for the experiments.

Usage in grasshopper:
    from experiments import experiment_01
"""
from compas.geometry import Point, Vector, Polyline
import Rhino.Geometry as rg # type: ignore
import math 
# from configurations import 

# class Experiments():
#     def __init__(self):
#         pass

# ------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------

def average_point(points):
    """Calculate the average point from a list of points."""
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)
    return Point(x, y, z)

# ------------------------------------------------------------------------
# Main API
# ------------------------------------------------------------------------

def exp_01(radius, num):
    """Experiment with spetial printing"""
    points = []
    all_angles = []
    alpha = (2 * math.pi) / num
    for n in range(num):
        x = math.sin(alpha * n) * radius
        y = math.cos(alpha * n) * radius
        z = 0 
        pt = Point(x, y, z)
        points.append(pt)
        all_angles.append(alpha)
    
    center = average_point(points)
    center_new = Point(center.x, center.y, center.z + 100)
    
    points.extend([center_new, points[1]])
    poly = Polyline(points)

    return poly, center_new
    
    