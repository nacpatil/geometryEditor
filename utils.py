import plotly.graph_objects as go
import trimesh
import numpy as np

# Global figure object to accumulate plots
global_fig = go.Figure()

def update_axis_limits(x, y, z):
    """
    Updates the global axis limits to ensure equal aspect ratio.
    """
    global global_fig

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
    global_fig.update_layout(
        scene=dict(
            xaxis=dict(title="X Axis", range=[x_mid - max_range, x_mid + max_range]),
            yaxis=dict(title="Y Axis", range=[y_mid - max_range, y_mid + max_range]),
            zaxis=dict(title="Z Axis", range=[z_mid - max_range, z_mid + max_range])
        )
    )

def plot_3d_scatter(x, y, z, title="3D Scatter Plot", lines=True):
    """
    Adds a 3D scatter plot to the global figure with optional connecting lines.
    """
    global global_fig

    # Scatter points
    global_fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='markers',
        marker=dict(size=5, color="red", opacity=0.8),
        name=title
    ))

    # Add lines if enabled
    if lines:
        global_fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color='blue', width=2),
            name=f"{title} (Lines)"
        ))

    # Update axis limits
    update_axis_limits(x, y, z)

def plot_3d_mesh(x, y, z, i, j, k, title="3D Mesh Plot"):
    """
    Adds a 3D mesh plot to the global figure.
    """
    global global_fig

    # Add mesh to global figure
    global_fig.add_trace(go.Mesh3d(
        x=x, y=y, z=z,
        i=i, j=j, k=k,
        opacity=0.5,
        color='lightblue',
        name=title
    ))

    # Update axis limits
    update_axis_limits(x, y, z)

def show_all_plots():
    """
    Displays the accumulated 3D plots.
    """
    global global_fig
    global_fig.show()


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
