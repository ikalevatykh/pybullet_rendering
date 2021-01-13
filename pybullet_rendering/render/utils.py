import numpy as np
import trimesh

import pybullet_rendering as pr

__all__ = ('decompose', 'mask_to_rgb', 'rgb_to_mask', 'depth_from_zbuffer', 'primitive_mesh')


def decompose(matrix):
    """Extract parameters from projection matrix.

    Arguments:
        matrix {mat44} -- projection matrix 4x4

    Returns:
        tuple -- field of view, z near, z far, aspect ratio
    """
    if matrix[0, 0] == 0.0 or matrix[1, 1] == 0.0 or np.abs(matrix[2, 2]) == 1.0:
        raise ValueError('Invalid projection matrix')

    yfov = 2.0 * np.arctan(1.0 / matrix[1, 1])
    znear = matrix[3, 2] / (matrix[2, 2]-1.0)
    zfar = znear * (matrix[2, 2]-1.0) / (matrix[2, 2]+1.0)
    aspect = matrix[1, 1] / matrix[0, 0]
    return yfov, znear, zfar, aspect


def mask_to_rgb(body_id, link_id):
    """Encode body id and link id as a RGB color.

    Arguments:
        body_id {int} -- body index (0..65535)
        link_id {int} -- link index (0..255)

    Returns:
        tuple -- RGB values
    """
    return (body_id + 1) & 0x00ff, ((body_id + 1) & 0xff00) >> 8, link_id+1


def rgb_to_mask(mask_rgb):
    """Decode segmentation mask value from RGB image.

    Arguments:
        mask_rgb {ndarray} -- RGB-encoded segmentation image

    Returns:
        ndarray -- segmentation mask
    """
    return np.dot(mask_rgb, np.int32([1, 1 << 8, 1 << 24])) - 1


def depth_from_zbuffer(zbuffer, znear, zfar):
    """Convert OpenGL Z-buffer values to metric depth.

    Arguments:
        zbuffer {ndarray} -- Z-buffer image
        znear {float} -- near z limit
        zfar {float} -- far z limit

    Returns:
        ndarray -- metric depth
    """
    if zbuffer.dtype != np.float32:
        zbuffer = np.divide(zbuffer, np.iinfo(zbuffer.dtype).max, dtype=np.float32)

    depth = np.zeros_like(zbuffer)
    noninf = np.not_equal(zbuffer, 1.0)
    depth[noninf] = znear * zfar / (zfar - zbuffer[noninf] * (zfar - znear))
    return depth


def primitive_mesh(shape):
    """Make primitive shape.

    Arguments:
        shape {Shape} -- primitive shape
    """
    if pr.ShapeType.Cube == shape.type:
        return trimesh.creation.box(extents=shape.extents)
    elif pr.ShapeType.Cylinder == shape.type:
        return trimesh.creation.cylinder(height=shape.height, radius=shape.radius)
    elif pr.ShapeType.Plane == shape.type:
        return trimesh.creation.box(extents=(10.0, 10.0, 0.0))
    elif pr.ShapeType.Sphere == shape.type:
        return trimesh.creation.uv_sphere(radius=shape.radius)
    elif pr.ShapeType.Capsule == shape.type:
        mesh = trimesh.creation.capsule(height=shape.height, radius=shape.radius)
        mesh.vertices[:, 2] -= shape.height / 2.0
        return mesh
