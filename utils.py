import plotly.graph_objects as go

def plot_3d_scatter(x, y, z, title="3D Scatter Plot"):
    """
    Plots a 3D scatter plot using Plotly.

    Args:
        x (array-like): X-coordinates of points.
        y (array-like): Y-coordinates of points.
        z (array-like): Z-coordinates of points.
        title (str): Title of the plot.
    """
    fig = go.Figure(data=[go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(size=5, color=z, colorscale='Viridis', opacity=0.8)
    )])

    # Set labels and show plot
    fig.update_layout(title=title, scene=dict(
        xaxis_title="X Axis",
        yaxis_title="Y Axis",
        zaxis_title="Z Axis"
    ))
    fig.show()
 