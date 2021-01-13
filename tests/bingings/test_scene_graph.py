import numpy as np
import pickle
import pybullet as pb

from pybullet_rendering import ShapeType
from .base_test_case import BaseTestCase


class SceneGraphTest(BaseTestCase):

    def _test_primitive(self, collision=False, **kwargs):
        col_id, vis_id = -1, -1
        if collision:
            col_id = self.client.createCollisionShape(**kwargs)
        else:
            vis_id = self.client.createVisualShape(**kwargs)

        body_id = self.client.createMultiBody(
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            baseInertialFramePosition=(3, 2, 1),
            baseInertialFrameOrientation=(0, 0, 0, 1))  # x,y,z,w

        self.client.getCameraImage(320, 240)
        self.assertIsNotNone(self.render.scene_graph)

        # nodes map
        nodes = self.render.scene_graph.nodes
        self.assertEqual(len(nodes), 1)
        # body node
        _uid, node = next(nodes.items())
        self.assertEqual(node.body, body_id)
        self.assertEqual(node.link, -1)
        self.assertEqual(len(node.shapes), 1)
        # shape
        shape = node.shapes[0]
        np.testing.assert_almost_equal(
            shape.pose.origin, (-3, -2, -1))  # in inertial frame
        np.testing.assert_almost_equal(
            shape.pose.quat, (1, 0, 0, 0))  # w,x,y,z
        self.assertIsNotNone(shape.material)
        self.assertIsNone(shape.material.diffuse_texture)
        return shape

    def test_box_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_BOX, halfExtents=[1.0, 2.0, 3.0])
        self.assertEqual(shape.type, ShapeType.Cube)
        np.testing.assert_almost_equal(shape.extents, [2.0, 4.0, 6.0])

    def test_sphere_primitive(self):
        shape = self._test_primitive(shapeType=pb.GEOM_SPHERE, radius=1.0)
        self.assertEqual(shape.type, ShapeType.Sphere)
        self.assertEqual(shape.radius, 1.0)

    def test_cylinder_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_CYLINDER, radius=1.0, length=2.0)
        self.assertEqual(shape.type, ShapeType.Cylinder)
        self.assertEqual(shape.radius, 1.0)
        self.assertEqual(shape.height, 2.0)

    def test_capsule_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_CAPSULE, radius=1.0, length=2.0)
        self.assertEqual(shape.type, ShapeType.Capsule)
        self.assertEqual(shape.radius, 1.0)
        self.assertEqual(shape.height, 2.0)

    def test_mesh_primitive(self):
        shape = self._test_primitive(
            shapeType=pb.GEOM_MESH,
            fileName='cube.obj',
            meshScale=[1, 2, 3])
        self.assertEqual(shape.type, ShapeType.Mesh)
        np.testing.assert_almost_equal(shape.pose.scale, [1, 2, 3])
        self.assertTrue(shape.mesh.filename.endswith('cube.obj'))
        self.assertIsNotNone(shape.material)

    def test_mesh_vertex_primitive(self):
        scale = [1, 2, 3]
        vertex_num = 100
        faces_num = 30
        vertices = self.random.rand(vertex_num, 3)
        indices = self.random.randint(0, vertex_num, size=faces_num*3)
        uvs = self.random.rand(vertex_num, 2)
        normals = self.random.rand(vertex_num, 3)
        shape = self._test_primitive(
            shapeType=pb.GEOM_MESH,
            meshScale=scale,
            vertices=vertices,
            indices=indices,
            uvs=uvs,
            normals=normals)
        self.assertEqual(shape.type, ShapeType.Mesh)
        np.testing.assert_almost_equal(shape.pose.scale, scale)
        self.assertIsNotNone(shape.mesh.data)
        np.testing.assert_almost_equal(shape.mesh.data.vertices, vertices)
        np.testing.assert_almost_equal(shape.mesh.data.faces.ravel(), indices)
        np.testing.assert_almost_equal(shape.mesh.data.uvs, uvs)
        np.testing.assert_almost_equal(shape.mesh.data.normals, normals)

    def test_heightfield_primitive(self):
        shape = self._test_primitive(
            collision=True,
            shapeType=pb.GEOM_HEIGHTFIELD,
            meshScale=[.5, .5, 2.5],
            fileName="heightmaps/ground0.txt",
            heightfieldTextureScaling=128)
        self.assertEqual(shape.type, ShapeType.Heightfield)
        self.assertIsNotNone(shape.mesh.data)
        self.assertEqual(shape.mesh.data.vertices.shape, (960000, 3))
        self.assertEqual(shape.mesh.data.uvs.shape, (960000, 2))
        self.assertEqual(shape.mesh.data.normals.shape, (960000, 3))
        self.assertEqual(shape.mesh.data.faces.shape, (320000, 3))

    def test_load_urdf(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.getCameraImage(320, 240)
        # nodes map
        nodes = self.render.scene_graph.nodes
        self.assertEqual(len(nodes), 1)
        # body node
        _uid, node = next(nodes.items())
        self.assertEqual(node.body, body_id)
        self.assertEqual(node.link, -1)
        self.assertEqual(len(node.shapes), 5)
        for shape in node.shapes:
            self.assertEqual(shape.type, ShapeType.Mesh)
            self.assertTrue(shape.mesh.filename.endswith('table.obj'))
            self.assertIsNotNone(shape.material)
            self.assertIsNone(shape.material.diffuse_texture)

    def test_load_urdf_external_materials(self):
        self.client.loadURDF("table/table.urdf",
                             flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        self.client.getCameraImage(320, 240)
        _uid, node = next(self.render.scene_graph.nodes.items())
        self.assertIsNone(node.shapes[0].material)

    def test_change_diffuse_color(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.changeVisualShape(
            body_id, -1, shapeIndex=2, rgbaColor=(1.0, 0.5, 0.2, 1.0))
        self.client.getCameraImage(320, 240)
        _uid, node = next(self.render.scene_graph.nodes.items())
        np.testing.assert_almost_equal(
            node.shapes[2].material.diffuse_color, (1.0, 0.5, 0.2, 1.0))

    def test_change_texture(self):
        body_id = self.client.loadURDF("table/table.urdf")
        tex_uid = self.client.loadTexture("table/table.png")
        self.client.changeVisualShape(body_id, -1, textureUniqueId=tex_uid)
        self.client.getCameraImage(320, 240)
        scene_graph = self.render.scene_graph
        _uid, node = next(scene_graph.nodes.items())
        for shape in node.shapes:
            self.assertIsNotNone(shape.material)
            self.assertIsNotNone(shape.material.diffuse_texture)
            filename = shape.material.diffuse_texture.filename
            self.assertTrue(filename.endswith("table.png"))

    def test_update_materials_only(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.getCameraImage(320, 240)
        self.assertFalse(self.render.materials_only)
        # change materials
        self.client.changeVisualShape(
            body_id, -1, shapeIndex=2, rgbaColor=(1, 1, 1, 1))
        self.client.getCameraImage(320, 240)
        self.assertTrue(self.render.materials_only)

    def test_scene_graph_pickle(self):
        body_id = self.client.loadURDF("table/table.urdf")
        tex_uid = self.client.loadTexture("table/table.png")
        self.client.changeVisualShape(body_id, -1, textureUniqueId=tex_uid)
        self.client.getCameraImage(320, 240)
        self.assertIsNotNone(self.render.scene_graph)

        buffer = pickle.dumps(self.render.scene_graph)
        scene_graph_copy = pickle.loads(buffer)
        self.assertEqual(self.render.scene_graph, scene_graph_copy)
