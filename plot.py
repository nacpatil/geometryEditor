from utils import *

if __name__ == "__main__":
    import numpy as np

    # Example 1: Rotated Square Points
    x = [0, 1, 1, 0]   # X-coordinates
    y = [0, 0, 1, 1]   # Y-coordinates
    z = [0, 0, 0, 0]   # Z-coordinates

    #rotate_xyz(x, y, z, "z", 45)  # Rotate by 45 degrees around Z-axis
    PointLineObject(x, y, z, title="Rotated Square", lines=True)
    print(x, y, z)

    # Example 2: Pyramid Mesh
    x = [0, 1, 1, 0, 0.5]
    y = [0, 0, 1, 1, 0.5]
    z = [0, 0, 0, 0, 1]  # The peak

    # Define faces using indices (each set forms a triangle)
    i = [0, 0, 0, 1, 1, 2, 2, 3]
    j = [1, 2, 3, 2, 3, 3, 0, 0]
    k = [4, 4, 4, 4, 4, 4, 4, 4]  # The top vertex for each triangle

    MeshObject(x, y, z, i, j, k, title="Pyramid Mesh")

    # Example 3: Generate Sphere Meshes
    create_sphere_mesh(num_spheres=5)

    # Show all stored objects
    show_all_plots()
