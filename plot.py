import plotly.graph_objects as go
import numpy as np
from utils import  plot_3d_scatter, plot_3d_mesh, inner_product, outer_product

# Example Usage
if __name__ == "__main__":
    import numpy as np
    np.random.seed(42)
    x = [0, 0, 2, 2]   # X-coordinates
    y = [0, 1, 1,  0]   # Y-coordinates
    z = [0, 0.5, 1.5, 1.0] # Z-coordinates
 
    plot_3d_scatter(x, y, z)

    exit()

    # Define pyramid vertices
    x = [0, 1, 1, 0, 0.5]
    y = [0, 0, 1, 1, 0.5]
    z = [0, 0, 0, 0, 1]  # The peak

    # Define faces using indices (each set forms a triangle)
    i = [0, 0, 0, 1, 1, 2, 2, 3]
    j = [1, 2, 3, 2, 3, 3, 0, 0]
    k = [4, 4, 4, 4, 4, 4, 4, 4]  # The top vertex for each triangle

    # Call the function
    plot_3d_mesh(x, y, z, i, j, k, title="Pyramid Mesh")