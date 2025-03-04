import trimesh
from config import *
from meshClass import *
 

def sphere(center=(0, 0, 0), radius=1.0, subdivisions=3, title="3D Sphere"):
    """
    Adds a single sphere at a given location to `mesh_objects`, but does not plot immediately.
    """
    # Create a single sphere mesh
    sphere = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
    sphere.apply_translation(center)

    # Extract vertex and face data
    vertices = sphere.vertices
    x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    faces = sphere.faces
    i, j, k = faces[:, 0], faces[:, 1], faces[:, 2]

    # Store the mesh object instead of plotting it directly
    mesh_objects.append(MeshObject(x, y, z, i, j, k, title))



def cuboid(center=(0, 0, 0), size=(1.0, 1.0, 1.0), title="3D Cuboid"):
    """
    Adds a single cuboid at a given location to `mesh_objects`, but does not plot immediately.

    Args:
        center (tuple): (x, y, z) coordinates for the cuboid's center.
        size (tuple): (width, height, depth) of the cuboid.
        title (str): Name of the cuboid.
    """
    # Create a single cuboid mesh
    cuboid = trimesh.creation.box(extents=size)

    # Move the cuboid to the specified location
    cuboid.apply_translation(center)

    # Extract vertex and face data
    vertices = cuboid.vertices
    x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    faces = cuboid.faces
    i, j, k = faces[:, 0], faces[:, 1], faces[:, 2]

    # Store the mesh object instead of plotting it directly
    mesh_objects.append(MeshObject(x, y, z, i, j, k, title))
