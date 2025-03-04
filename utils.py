import plotly.graph_objects as go
import trimesh
import numpy as np

# Global figure object to accumulate plots
global_fig = go.Figure()

# Object storage
point_line_objects = []
mesh_objects = []

class PointLineObject:
    def __init__(self, x, y, z, title="Point-Line Object", lines=True):
        self.x = x
        self.y = y
        self.z = z
        self.title = title
        self.lines = lines
        point_line_objects.append(self)
    
    def add_to_plot(self):
        global global_fig
        global_fig.add_trace(go.Scatter3d(
            x=self.x, y=self.y, z=self.z,
            mode='markers',
            marker=dict(size=5, color="red", opacity=0.8),
            name=self.title
        ))
        if self.lines:
            global_fig.add_trace(go.Scatter3d(
                x=self.x, y=self.y, z=self.z,
                mode='lines',
                line=dict(color='blue', width=2),
                name=f"{self.title} (Lines)"
            ))

class MeshObject:
    def __init__(self, x, y, z, i, j, k, title="Mesh Object"):
        self.x = x
        self.y = y
        self.z = z
        self.i = i
        self.j = j
        self.k = k
        self.title = title
        mesh_objects.append(self)
    
    def add_to_plot(self):
        global global_fig
        global_fig.add_trace(go.Mesh3d(
            x=self.x, y=self.y, z=self.z,
            i=self.i, j=self.j, k=self.k,
            opacity=0.5,
            color='lightblue',
            name=self.title
        ))

def update_axis_limits():
    """
    Updates the global axis limits to ensure equal aspect ratio.
    """
    global global_fig
    all_x = []
    all_y = []
    all_z = []
    
    # Collect all points from both objects
    for obj in point_line_objects + mesh_objects:
        all_x.extend(obj.x)
        all_y.extend(obj.y)
        all_z.extend(obj.z)
    
    if not all_x:
        return  # No objects added yet
    
    # Compute axis limits
    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    z_range = max(all_z) - min(all_z)
    max_range = max(x_range, y_range, z_range) / 2

    # Compute centers
    x_mid = (max(all_x) + min(all_x)) / 2
    y_mid = (max(all_y) + min(all_y)) / 2
    z_mid = (max(all_z) + min(all_z)) / 2

    # Apply equal ranges to all axes
    global_fig.update_layout(
        scene=dict(
            xaxis=dict(title="X Axis", range=[x_mid - max_range, x_mid + max_range]),
            yaxis=dict(title="Y Axis", range=[y_mid - max_range, y_mid + max_range]),
            zaxis=dict(title="Z Axis", range=[z_mid - max_range, z_mid + max_range])
        )
    )

def show_all_plots():
    """
    Displays all stored objects as a 3D plot.
    """
    global global_fig
    global_fig = go.Figure()  # Reset plot
    for obj in point_line_objects + mesh_objects:
        obj.add_to_plot()
    update_axis_limits()
    global_fig.show()

def create_sphere_mesh(num_spheres=20, subdivisions=2, radius=1.0, spacing=3.0):
    """
    Creates multiple sphere meshes and stores them in memory as MeshObject.
    """
    for i in range(num_spheres):
        sphere = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
        sphere.vertices += [i * spacing, 0, 0]  # Offset spheres in x-direction
        
        x, y, z = sphere.vertices[:, 0].tolist(), sphere.vertices[:, 1].tolist(), sphere.vertices[:, 2].tolist()
        i, j, k = sphere.faces[:, 0].tolist(), sphere.faces[:, 1].tolist(), sphere.faces[:, 2].tolist()
        
        MeshObject(x, y, z, i, j, k, title=f"Sphere {i}")

# Example Usage
if __name__ == "__main__":
    # Create some point-line objects
    PointLineObject([0, 1, 2], [0, 1, 2], [0, 1, 2], title="Line 1")
    PointLineObject([2, 3, 4], [2, 3, 4], [2, 3, 4], title="Line 2", lines=False)
    
    # Create sphere meshes
    create_sphere_mesh(num_spheres=5)
    
    # Show all objects
    show_all_plots()
