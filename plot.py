from utils import *
from primitives import *
import numpy as np
from config import *


if __name__ == "__main__":
    # Add scatter points
    PointLineObject(x=[0, 1, 2], y=[0, 1, 4], z=[0, 1, 8], title="Scatter 1")
    PointLineObject(x=[3, 4, 5], y=[9, 16, 25], z=[27, 64, 125], title="Scatter 2")

    # Add spheres (now stored in `mesh_objects`, not plotted immediately)
    sphere(center=(0, 0, 0), radius=1.0, title="Sphere 1")
    sphere(center=(3, 3, 3), radius=1.5, title="Sphere 2")
    sphere(center=(-2, -2, 2), radius=10, title="Sphere 3")

    # Now plot everything in a **single unified plot**
    show_all_plots()
