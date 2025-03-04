import plotly.graph_objects as go
import trimesh
import numpy as np


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

    # Compute axis limits to keep equal scale
    x_range = max(x) - min(x)
    y_range = max(y) - min(y)
    z_range = max(z) - min(z)
    max_range = max(x_range, y_range, z_range) / 2

    # Compute centers
    x_mid = (max(x) + min(x)) / 2
    y_mid = (max(y) + min(y)) / 2
    z_mid = (max(z) + min(z)) / 2

    # Scatter points
    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(size=5, color="red", opacity=0.8),
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

    # Correctly set the scene without nesting `scene` inside another `scene`
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(title="X Axis", range=[x_mid - max_range, x_mid + max_range]),
            yaxis=dict(title="Y Axis", range=[y_mid - max_range, y_mid + max_range]),
            zaxis=dict(title="Z Axis", range=[z_mid - max_range, z_mid + max_range])
        )
    )

    fig.show()


def plot_3d_mesh(x, y, z, i, j, k, title="3D Mesh Plot"):
    """
    Plots a 3D mesh using Plotly with equal aspect ratio for x, y, and z axes.

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
    # Create mesh figure
    fig = go.Figure(data=[go.Mesh3d(
        x=x, y=y, z=z,
        i=i, j=j, k=k,
        opacity=0.5,
        color='lightblue'
    )])

    # Compute axis limits to keep equal scale
    x_range = max(x) - min(x)
    y_range = max(y) - min(y)
    z_range = max(z) - min(z)
    max_range = max(x_range, y_range, z_range) / 2

    # Compute centers
    x_mid = (max(x) + min(x)) / 2
    y_mid = (max(y) + min(y)) / 2
    z_mid = (max(z) + min(z)) / 2

    # Apply equal ranges to all axes
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(title="X Axis", range=[x_mid - max_range, x_mid + max_range]),
            yaxis=dict(title="Y Axis", range=[y_mid - max_range, y_mid + max_range]),
            zaxis=dict(title="Z Axis", range=[z_mid - max_range, z_mid + max_range])
        )
    )

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


def rotate_xyz(x, y, z, axis, angle_degrees):
    """
    Rotates x, y, and z coordinate lists around the specified axis ('x', 'y', or 'z') 
    by the given angle in degrees. Modifies the lists in-place.

    :param x: List of x-coordinates (modified in-place)
    :param y: List of y-coordinates (modified in-place)
    :param z: List of z-coordinates (modified in-place)
    :param axis: Axis to rotate around ('x', 'y', or 'z')
    :param angle_degrees: Rotation angle in degrees
    """
    # Convert angle to radians
    angle = np.radians(angle_degrees)

    # Define rotation matrices
    if axis.lower() == 'x':
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
    elif axis.lower() == 'y':
        rotation_matrix = np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    elif axis.lower() == 'z':
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")

    # Convert input lists to NumPy array (N, 3)
    coords_array = np.array([x, y, z])

    # Apply rotation (matrix multiplication)
    rotated_coords = np.dot(rotation_matrix, coords_array)

    # Modify input lists in-place
    x[:] = rotated_coords[0].tolist()
    y[:] = rotated_coords[1].tolist()
    z[:] = rotated_coords[2].tolist()
 


def create_sphere_mesh(num_spheres=20, subdivisions=2, radius=1.0, spacing=3.0):
    """
    Create multiple sphere meshes and return a single combined mesh.

    Args:
        num_spheres (int): Number of spheres to create.
        subdivisions (int): Number of subdivisions for each icosphere.
        radius (float): Radius of each sphere.
        spacing (float): Distance between sphere centers.

    Returns:
        trimesh.Trimesh: A single mesh combining all spheres.
    """
    all_vertices = []
    all_faces = []
    offset = 0

    for i in range(num_spheres):
        # Create a sphere mesh
        sphere = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)

        # Offset the sphere in the x-direction to space them apart
        sphere.vertices += [i * spacing, 0, 0]

        # Append vertices and faces
        all_vertices.extend(sphere.vertices)
        all_faces.extend(sphere.faces + offset)

        # Update the vertex offset for face indices
        offset += len(sphere.vertices)

    # Create a combined mesh
    combined_mesh = trimesh.Trimesh(vertices=all_vertices, faces=all_faces)
    x, y, z = combined_mesh.vertices[:, 0].tolist(), combined_mesh.vertices[:, 1].tolist(), combined_mesh.vertices[:, 2].tolist()
    i, j, k = combined_mesh.faces[:, 0].tolist(), combined_mesh.faces[:, 1].tolist(), combined_mesh.faces[:, 2].tolist()



    # Return the extracted data as a list
    result = [x, y, z, i, j, k]

    return result
