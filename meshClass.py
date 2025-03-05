import open3d as o3d
import trimesh
import numpy as np
from config import *
import math

class TransformableObject:
    """
    Base class that provides transformation functions for rotation and translation.
    Used for both `PointLine` and `MeshObject`.
    """

    def __init__(self, x, y, z, keypoint=None):
        self.x = np.array(x)
        self.y = np.array(y)
        self.z = np.array(z)
        
        # Default keypoint is the first vertex
        self.keyPoint = np.array(keypoint if keypoint else [x[0], y[0], z[0]])

        # Store original object vertices
        self.vertices = np.vstack((self.x, self.y, self.z)).T

    def rotate(self, axis="x", degrees=10):
        """
        Rotates the object around its keypoint.

        Args:
            axis (str): "x", "y", or "z" indicating the rotation axis.
            degrees (float): Rotation angle in degrees.
        """
        radians = np.radians(degrees)  # Convert to radians
        cos_a = math.cos(radians)
        sin_a = math.sin(radians)

        # Define rotation matrices
        if axis == "x":
            rot_matrix = np.array([
                [1, 0,     0],
                [0, cos_a, -sin_a],
                [0, sin_a, cos_a]
            ])
        elif axis == "y":
            rot_matrix = np.array([
                [cos_a,  0, sin_a],
                [0,      1, 0],
                [-sin_a, 0, cos_a]
            ])
        elif axis == "z":
            rot_matrix = np.array([
                [cos_a, -sin_a, 0],
                [sin_a, cos_a,  0],
                [0,     0,      1]
            ])
        else:
            raise ValueError("Invalid axis. Choose from 'x', 'y', or 'z'.")

        # Translate object to keypoint, apply rotation, then translate back
        self.vertices = (self.vertices - self.keyPoint) @ rot_matrix.T + self.keyPoint

        # Update the object
        self.update_geometry()

    def translate(self, dx=0, dy=0, dz=0):
        """
        Moves the object by a given offset.

        Args:
            dx (float): Translation along X-axis.
            dy (float): Translation along Y-axis.
            dz (float): Translation along Z-axis.
        """
        translation_vector = np.array([dx, dy, dz])

        # Apply translation
        self.vertices += translation_vector

        # Move keypoint as well
        self.keyPoint += translation_vector

        # Update the object
        self.update_geometry()

    def update_geometry(self):
        """
        Updates the object's geometry after transformation.
        Must be implemented in subclasses.
        """
        raise NotImplementedError("Subclasses must implement `update_geometry()`")


class PointLine(TransformableObject):
    def __init__(self, x, y, z, title="Point-Line Object", lines=True, keypoint=None):
        super().__init__(x, y, z, keypoint)
        self.title = title
        self.lines = lines
        point_line_objects.append(self)

    def to_open3d(self):
        """
        Converts this object to an Open3D point cloud with optional lines.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.vertices)
        pcd.paint_uniform_color([1, 0, 0])  # Red points

        objects = [pcd]

        if self.lines and len(self.vertices) > 1:
            lines = [[i, i + 1] for i in range(len(self.vertices) - 1)]
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(self.vertices)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.paint_uniform_color([0, 0, 1])  # Blue lines
            objects.append(line_set)

        return objects

    def update_geometry(self):
        """
        Updates the Open3D representation after transformations.
        """
        # Re-create the Open3D objects after transformation
        self.to_open3d()


class MeshObject(TransformableObject):
    def __init__(self, x, y, z, i, j, k, title="Mesh Object", keypoint=None):
        super().__init__(x, y, z, keypoint)
        self.i = np.array(i)
        self.j = np.array(j)
        self.k = np.array(k)
        self.title = title

        # Store face data
        self.triangles = np.vstack((self.i, self.j, self.k)).T

        # Mesh storage
        self.mesh = self.to_open3d()
        mesh_objects.append(self)

    def to_open3d(self):
        """
        Converts this object to an Open3D triangle mesh.
        """
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(self.vertices)
        mesh.triangles = o3d.utility.Vector3iVector(self.triangles)
        mesh.paint_uniform_color([0.5, 0.7, 0.9])  # Light blue color
        mesh.compute_vertex_normals()  # Enables smooth shading
        return mesh

    def update_geometry(self):
        """
        Updates the Open3D mesh after transformations.
        """
        self.mesh.vertices = o3d.utility.Vector3dVector(self.vertices)
        self.mesh.compute_vertex_normals()


def get_scene_size():
    """
    Estimates the size of the scene by checking the bounding box of all objects.
    """
    all_x, all_y, all_z = [], [], []
    
    # Collect all points from both objects
    for obj in point_line_objects + mesh_objects:
        all_x.extend(obj.x)
        all_y.extend(obj.y)
        all_z.extend(obj.z)

    if not all_x:
        return 5  # Default axis length if no objects exist
    
    # Compute scene bounds
    max_extent = max(
        max(all_x) - min(all_x),
        max(all_y) - min(all_y),
        max(all_z) - min(all_z)
    )
    return max_extent * 0.5  # Set axis length to half the max scene size

def create_xyz_axes(length):
    """
    Creates XYZ axes for reference in Open3D, ensuring visibility by moving it outward.
    """
    length = get_scene_size()
    offset = 0  # Move axis slightly outside objects

    axes = o3d.geometry.LineSet()
    axes.points = o3d.utility.Vector3dVector([
        [offset, offset, offset], [length + offset, offset, offset],  # X-axis (red)
        [offset, offset, offset], [offset, length + offset, offset],  # Y-axis (green)
        [offset, offset, offset], [offset, offset, length + offset],  # Z-axis (blue)
    ])
    axes.lines = o3d.utility.Vector2iVector([
        [0, 1], [2, 3], [4, 5]
    ])
    axes.colors = o3d.utility.Vector3dVector([
        [1, 0, 0], [0, 1, 0], [0, 0, 1]  # Red, Green, Blue
    ])
    return axes

def create_grid(full_size=10, step=1):
    """
    Creates a grid (ground plane) for better visualization.
    """
    size = round(0.5*full_size)
    step = round(size/10)+1
    lines = []
    points = []
    for i in range(-size, size + 1, step):
        points.append([i, -size, 0])
        points.append([i, size, 0])
        lines.append([len(points) - 2, len(points) - 1])

        points.append([-size, i, 0])
        points.append([size, i, 0])
        lines.append([len(points) - 2, len(points) - 1])

    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.paint_uniform_color([0.7, 0.7, 0.7])  # Grey color
    return grid

def show_all_plots():
    """
    Displays the accumulated 3D plots using Open3D with visible XYZ axes and a ground grid.
    """
    length = get_scene_size()
    all_objects = [create_xyz_axes(length), create_grid(1.5*length)]
    
    for obj in point_line_objects:
        all_objects.extend(obj.to_open3d())

    for obj in mesh_objects:
        all_objects.append(obj.to_open3d())

    # Display in Open3D viewer
    o3d.visualization.draw_geometries(all_objects)
