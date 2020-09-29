from os.path import dirname, join, abspath
import pybullet_rendering as pr

__all__ = ['shape_filename']


def shape_filename(shape):
    if shape.type == pr.ShapeType.Mesh:
        filename = abspath(shape.mesh.filename)
        return filename

    packagedir = dirname(pr.__path__[0])
    if shape.type == pr.ShapeType.Cube:
        return join(packagedir, 'share', 'primitives', 'cube.stl')
    elif shape.type == pr.ShapeType.Sphere:
        return join(packagedir, 'share', 'primitives', 'sphere.stl')
    elif shape.type == pr.ShapeType.Cylinder:
        return join(packagedir, 'share', 'primitives', 'cylinder.stl')
    elif shape.type == pr.ShapeType.Capsule:
        return join(packagedir, 'share', 'primitives', 'cylinder.stl')
    return ''
