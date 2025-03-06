import trimesh
from config import *
from meshClass import *
 

def sphere(center=(0, 0, 0), radius=1.0, subdivisions=2, title="3D Sphere"):
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
    myObj = MeshObject(x, y, z, i, j, k, title)
    myObj.volume = size[0]*size[1]*size[2]
    mesh_objects.append(myObj)





def obb_cuboid(center=(0, 0, 0), size=(1.0, 1.0, 1.0), u=(1, 0, 0), v=(0, 1, 0), w=(0, 0, 1), title="OBB Cuboid"):
    """
    Creates a cuboid with an oriented bounding box representation.

    Args:
        center (tuple): (x, y, z) coordinates for the cuboid's center.
        size (tuple): (width, height, depth) of the cuboid.
        u (tuple): First direction vector (local x-axis of the cuboid).
        v (tuple): Second direction vector (local y-axis of the cuboid).
        w (tuple): Third direction vector (local z-axis of the cuboid).
        title (str): Name of the cuboid.

    Returns:
        A trimesh object representing the oriented cuboid.
    """

    # Ensure unit vectors for u, v, w
    u = np.array(u) / np.linalg.norm(u)
    v = np.array(v) / np.linalg.norm(v)
    w = np.array(w) / np.linalg.norm(w)

    # Create a transformation matrix from the basis vectors
    rotation_matrix = np.column_stack((u, v, w))

    # Create a unit cuboid centered at the origin
    cuboid = trimesh.creation.box(extents=size)

    # Apply rotation and translation
    cuboid.apply_transform(np.vstack([
        np.column_stack((rotation_matrix, center)), 
        [0, 0, 0, 1]
    ]))

    # Extract vertex and face data
    vertices = cuboid.vertices
    x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    faces = cuboid.faces
    i, j, k = faces[:, 0], faces[:, 1], faces[:, 2]

    # Store the mesh object instead of plotting it directly
    mesh_objects.append(MeshObject(x, y, z, i, j, k, title))

    return cuboid
