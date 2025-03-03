import plotly.graph_objects as go
from utils import  plot_3d_scatter

# Example Usage
if __name__ == "__main__":
    import numpy as np
    np.random.seed(42)
    x = np.random.rand(50)
    y = np.random.rand(50)
    z = np.random.rand(50)
    
    plot_3d_scatter(x, y, z)
