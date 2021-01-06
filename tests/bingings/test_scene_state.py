import numpy as np
import pickle

from pybullet_rendering import LightType
from .base_test_case import BaseTestCase


class SceneStateTest(BaseTestCase):

    def test_base_state(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.resetBasePositionAndOrientation(
            body_id, (1, 2, 3), (0, 0, 0, 1))
        self.client.getCameraImage(320, 240)
        self.assertIsNotNone(self.render.scene_state)
        self.assertIsNotNone(self.render.scene_graph)

        uid, _node = next(self.render.scene_graph.nodes.items())
        pose = self.render.scene_state.pose(uid)
        np.testing.assert_almost_equal(pose.origin, (1, 2, 3))
        np.testing.assert_almost_equal(pose.quat, (1, 0, 0, 0))  # w,x,y,z

    def test_link_state(self):
        _body_id = self.client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        self.client.getCameraImage(320, 240)
        self.assertIsNotNone(self.render.scene_state)
        self.assertIsNotNone(self.render.scene_graph)
        for uid, node in self.render.scene_graph.nodes.items():
            if node.link == -1:
                state = self.client.getBasePositionAndOrientation(node.body)
            else:
                state = self.client.getLinkState(node.body, node.link)
            linkWorldPosition = state[0]
            linkWorldOrientation = state[1]

            pose = self.render.scene_state.pose(uid)
            np.testing.assert_almost_equal(pose.origin, linkWorldPosition)
            x, y, z, w = linkWorldOrientation
            np.testing.assert_almost_equal(pose.quat, (w, x, y, z))

            matrix = self.client.getMatrixFromQuaternion(linkWorldOrientation)
            np.testing.assert_almost_equal(
                np.asarray(pose.matrix).reshape((4, 4))[:3, :3],
                np.asarray(matrix).reshape((3, 3)).T, decimal=3)

    def test_scene_state_pickle(self):
        self.client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        self.client.getCameraImage(320, 240)
        self.assertIsNotNone(self.render.scene_state)

        buffer = pickle.dumps(self.render.scene_state)
        scene_state_copy = pickle.loads(buffer)
        self.assertEqual(self.render.scene_state, scene_state_copy)
