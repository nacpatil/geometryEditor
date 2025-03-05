from meshClass import *
from primitives import *
import numpy as np
from config import *
 
# Define an oriented cuboid with a custom basis



star_polygon = PolygonSurface(
    x=[0, 1, 3, 1.5, 2, 0, -2, -1.5, -3, -1],
    y=[3, 1, 1, 0, -2, -1, -2, 0, 1, 1],
    z=0,
    title="Star Polygon"
)
star_polygon.translate(4,0,0)
show_all_plots()

obb_cuboid(
    center=(2, 3, 4), 
    size=(2.0, 1.0, 1.5), 
    u=(1, 1, 0),  # Diagonal in X-Y plane
    v=(-1, 1, 0), # Perpendicular to u
    w=(0, 0, 1)   # Stays aligned with Z
)
show_all_plots()

 


PointLine(x=[0, 1, 2], y=[0, 1, 4], z=[0, 1, 8], title="Scatter 1")
PointLine(x=[3, 4, 5], y=[9, 16, 25], z=[27, 64, 125], title="Scatter 2")

# Add spheres (now stored in `mesh_objects`, not plotted immediately)
sphere(center=(0, 0, 0), radius=1.0, title="Sphere 1")
sphere(center=(3, 3, 3), radius=1.5, title="Sphere 2")
sphere(center=(-2, -2, 2), radius=5, title="Sphere 3")

show_all_plots()
for i in range(0,5):
    for j in range(0,5):
        cuboid(center=(5+i*3, 8+j*3, 0), size=(2.0, 2.0, 4.0), title="3D Cuboid")

mesh_objects[-1].rotate('x',45)

# Now plot everything in a **single unified plot**


show_all_plots()
