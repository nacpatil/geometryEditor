import plotly.graph_objects as go

import plotly.graph_objects as go

def plot_3d_scatter(x, y, z, title="3D Scatter Plot", lines=True):
    """
    Plots a 3D scatter plot using Plotly with optional lines connecting the points.

    Args:
        x (array-like): X-coordinates of points.
        y (array-like): Y-coordinates of points.
        z (array-like): Z-coordinates of points.
        title (str): Title of the plot.
        lines (bool): Whether to draw lines connecting the points. Default is True.
    """
    fig = go.Figure()

    # Scatter points
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(size=5, color="red", colorscale='Viridis', opacity=0.8),
        name="Points"
    ))

    # Add lines if enabled
    if lines:
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color='blue', width=2),
            name="Lines"
        ))

    # Set labels and show plot
    fig.update_layout(title=title, scene=dict(
        xaxis_title="X Axis",
        yaxis_title="Y Axis",
        zaxis_title="Z Axis"
    ))
    fig.show()


def plot_3d_mesh(x, y, z, i, j, k, title="3D Mesh Plot"):
    """
    Plots a 3D mesh using Plotly.

    Args:
        x (array-like): X-coordinates of vertices.
        y (array-like): Y-coordinates of vertices.
        z (array-like): Z-coordinates of vertices.
        i (array-like): Indices of the first vertex in each triangle.
        j (array-like): Indices of the second vertex in each triangle.
        k (array-like): Indices of the third vertex in each triangle.
        title (str): Title of the plot.

    Example Usage:
        plot_3d_mesh(x, y, z, i, j, k)
    """
    fig = go.Figure(data=[go.Mesh3d(
        x=x, y=y, z=z,
        i=i, j=j, k=k,
        opacity=0.5,
        color='lightblue'
    )])

    fig.update_layout(title=title, scene=dict(
        xaxis_title="X Axis",
        yaxis_title="Y Axis",
        zaxis_title="Z Axis"
    ))
    
    fig.show()


def inner_product(vec1, vec2):
    """
    Computes the inner (dot) product of two vectors.
    """
    return np.dot(vec1, vec2)

def outer_product(vec1, vec2):
    """
    Computes the outer product of two vectors.
    """
    return np.outer(vec1, vec2)
