from os.path import dirname, join
import pybullet_rendering as pr

__all__ = ['shape_filename']


def shape_filename(shape):
    if shape.type == pr.ShapeType.Mesh:
        return shape.mesh.filename

    packagedir = dirname(pr.__path__[0])
    if shape.type == pr.ShapeType.Cube:
        return join(packagedir, 'share', 'primitives', 'cube.stl')
    elif shape.type == pr.ShapeType.Sphere:
        return join(packagedir, 'share', 'primitives', 'sphere.stl')
    elif shape.type == pr.ShapeType.Cylinder:
        return join(packagedir, 'share', 'primitives', 'cylinder.stl')

    return ''
