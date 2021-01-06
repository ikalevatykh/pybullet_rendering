import numpy as np
import pickle
import pybullet as pb

from pybullet_rendering import ShapeType
from .base_test_case import BaseTestCase


class SceneGraphTest(BaseTestCase):

    def _test_primitive(self, collision=False, **kwargs):
        if collision:
            col_id = self.client.createCollisionShape(**kwargs)
            vis_id = -1
        else:
            vis_id = self.client.createVisualShape(**kwargs)
            col_id = -1

        body_id = self.client.createMultiBody(
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            baseInertialFramePosition=(3, 2, 1),
            baseInertialFrameOrientation=(0, 0, 0, 1))  # x,y,z,w

        self.client.getCameraImage(320, 240)

        assert self.render.scene_graph
        # nodes map
        nodes = self.render.scene_graph.nodes
        assert len(nodes) == 1
        # body node
        uid, node = next(nodes.items())
        assert node.body == body_id
        assert node.link == -1
        assert len(node.shapes) == 1
        # shape
        shape = node.shapes[0]  # shapes list
        assert np.allclose(shape.pose.origin, (-3, -2, -1)
                           )  # in inertial frame
        assert np.allclose(shape.pose.quat, (1, 0, 0, 0))  # w,x,y,z
        assert shape.has_material == True
        assert shape.material.diffuse_texture == -1
        return shape

    def test_box_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_BOX, halfExtents=[1, 2, 3])
        assert shape.type == ShapeType.Cube
        assert np.allclose(shape.pose.scale, [2, 4, 6])

    def test_sphere_primitive(self):
        shape = self._test_primitive(shapeType=pb.GEOM_SPHERE, radius=1)
        assert shape.type == ShapeType.Sphere
        assert np.allclose(shape.pose.scale, [1, 1, 1])

    def test_cylinder_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_CYLINDER, radius=1, length=2)
        assert shape.type == ShapeType.Cylinder
        assert np.allclose(shape.pose.scale, [1, 1, 2])

    def test_mesh_primitive(self):
        shape = self._test_primitive(shapeType=pb.GEOM_MESH,
                                     fileName='cube.obj',
                                     meshScale=[1, 2, 3])
        assert shape.type == ShapeType.Mesh
        assert np.allclose(shape.pose.scale, [1, 2, 3])
        assert shape.mesh.filename.endswith('cube.obj')
        assert shape.has_material == True

    def test_mesh_vertex_primitive(self):
        scale = [1, 2, 3]
        vertex_num = 100
        faces_num = 30
        rs = np.random.RandomState(77)
        vertices = rs.rand(vertex_num, 3)
        indices = rs.randint(0, vertex_num, size=faces_num*3)
        uvs = rs.rand(vertex_num, 2)
        normals = rs.rand(vertex_num, 3)
        shape = self._test_primitive(shapeType=pb.GEOM_MESH,
                                     meshScale=scale,
                                     vertices=vertices,
                                     indices=indices,
                                     uvs=uvs,
                                     normals=normals)
        assert shape.type == ShapeType.Mesh
        assert np.allclose(shape.pose.scale, scale)
        assert shape.mesh.data is not None
        assert np.allclose(shape.mesh.data.vertices, vertices)
        assert np.allclose(shape.mesh.data.indices, indices)
        assert np.allclose(shape.mesh.data.uvs, uvs)
        assert np.allclose(shape.mesh.data.normals, normals)

    def test_heightfield_primitive(self):
        shape = self._test_primitive(collision=True,
                                     shapeType=pb.GEOM_HEIGHTFIELD,
                                     meshScale=[.5, .5, 2.5],
                                     fileName="heightmaps/ground0.txt",
                                     heightfieldTextureScaling=128)
        assert shape.type == ShapeType.Mesh
        assert shape.mesh.data is not None
        assert shape.mesh.data.vertices.shape == (960000, 3)
        assert shape.mesh.data.uvs.shape == (960000, 2)
        assert shape.mesh.data.normals.shape == (960000, 3)
        assert shape.mesh.data.indices.shape == (960000,)

    def test_load_urdf(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.getCameraImage(320, 240)
        # nodes map
        nodes = self.render.scene_graph.nodes
        assert len(nodes) == 1
        # body node
        uid, node = next(nodes.items())
        assert node.body == body_id
        assert node.link == -1
        assert len(node.shapes) == 5
        for shape in node.shapes:
            assert shape.type == ShapeType.Mesh
            assert shape.mesh.filename.endswith('table.obj')
            assert shape.has_material == True
            assert np.allclose(shape.material.diffuse_texture, -1)

    def test_load_urdf_external_materials(self):
        body_id = self.client.loadURDF("table/table.urdf",
                                       flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        self.client.getCameraImage(320, 240)
        uid, node = next(self.render.scene_graph.nodes.items())
        node.shapes[0].has_material == False

    def test_change_diffuse_color(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.changeVisualShape(
            body_id, -1, shapeIndex=2, rgbaColor=(1.0, 0.5, 0.2, 1.0))
        self.client.getCameraImage(320, 240)
        uid, node = next(self.render.scene_graph.nodes.items())
        assert np.allclose(
            node.shapes[2].material.diffuse_color, (1.0, 0.5, 0.2, 1.0))

    def test_change_diffuse_texture(self):
        body_id = self.client.loadURDF("table/table.urdf")
        tex_uid = self.client.loadTexture("table/table.png")
        self.client.changeVisualShape(body_id, -1, textureUniqueId=tex_uid)
        self.client.getCameraImage(320, 240)
        scene_graph = self.render.scene_graph
        _uid, node = next(scene_graph.nodes.items())
        for shape in node.shapes:
            assert shape.material.diffuse_texture == tex_uid
            assert scene_graph.texture(tex_uid).filename.endswith("table.png")

    def test_update_materials_only(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.getCameraImage(320, 240)
        assert self.render.materials_only == False
        # change materials
        self.client.changeVisualShape(
            body_id, -1, shapeIndex=2, rgbaColor=(1, 1, 1, 1))
        self.client.getCameraImage(320, 240)
        assert self.render.materials_only == True

    def test_scene_graph_pickle(self):
        body_id = self.client.loadURDF("table/table.urdf")
        tex_uid = self.client.loadTexture("table/table.png")
        self.client.changeVisualShape(body_id, -1, textureUniqueId=tex_uid)
        self.client.getCameraImage(320, 240)
        assert self.render.scene_graph

        buffer = pickle.dumps(self.render.scene_graph)
        scene_graph_copy = pickle.loads(buffer)
        assert self.render.scene_graph == scene_graph_copy
