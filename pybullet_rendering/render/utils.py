from pathlib import Path
from pybullet_rendering import ShapeType
from pybullet_rendering.data import __file__ as data

__all__ = ['shape_filename']


def shape_filename(shape):
    if shape.type == ShapeType.Mesh:
        return shape.mesh.filename
    elif shape.type == ShapeType.Cube:
        return Path(data).parent / 'primitives' / 'cube.stl'
    elif shape.type == ShapeType.Sphere:
        return Path(data).parent / 'primitives' / 'sphere.stl'
    elif shape.type == ShapeType.Cylinder:
        return Path(data).parent / 'primitives' / 'cylinder.stl'
    else:
        return ''
