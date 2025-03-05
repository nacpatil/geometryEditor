from meshClass import *
from primitives import *
import numpy as np
from config import *


if __name__ == "__main__":
    # Add scatter points
    PointLine(x=[0, 1, 2], y=[0, 1, 4], z=[0, 1, 8], title="Scatter 1")
    PointLine(x=[3, 4, 5], y=[9, 16, 25], z=[27, 64, 125], title="Scatter 2")

    # Add spheres (now stored in `mesh_objects`, not plotted immediately)
    sphere(center=(0, 0, 0), radius=1.0, title="Sphere 1")
    sphere(center=(3, 3, 3), radius=1.5, title="Sphere 2")
    sphere(center=(-2, -2, 2), radius=5, title="Sphere 3")

    for i in range(0,5):
        for j in range(0,5):
            cuboid(center=(5+i*3, 8+j*3, 0), size=(2.0, 2.0, 4.0), title="3D Cuboid")
    
    mesh_objects[-1].rotate('x',45)

    # Now plot everything in a **single unified plot**
    show_all_plots()
