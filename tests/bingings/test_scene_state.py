import numpy as np
import pickle

from pybullet_rendering import LightType
from .base_test_case import BaseTestCase


class SceneStateTest(BaseTestCase):

    def test_base_state(self):
        body_id = self.client.loadURDF("table/table.urdf")
        self.client.resetBasePositionAndOrientation(body_id, (1, 2, 3), (0, 0, 0, 1))
        self.client.getCameraImage(320, 240)
        assert self.render.scene_state
        assert self.render.scene_graph

        uid, node = next(self.render.scene_graph.nodes.items())
        pose = self.render.scene_state.pose(uid)
        assert np.allclose(pose.origin, (1, 2, 3))
        assert np.allclose(pose.quat, (1, 0, 0, 0))  # w,x,y,z

    def test_link_state(self):
        body_id = self.client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        self.client.getCameraImage(320, 240)
        assert self.render.scene_state
        assert self.render.scene_graph
        for uid, node in self.render.scene_graph.nodes.items():
            if node.link == -1:
                state = self.client.getBasePositionAndOrientation(node.body)
            else:
                state = self.client.getLinkState(node.body, node.link)
            linkWorldPosition = state[0]
            linkWorldOrientation = state[1]

            pose = self.render.scene_state.pose(uid)
            assert np.allclose(pose.origin, linkWorldPosition)
            x, y, z, w = linkWorldOrientation
            assert np.allclose(pose.quat, (w, x, y, z))

            matrix = self.client.getMatrixFromQuaternion(linkWorldOrientation)
            assert np.allclose(
                np.asarray(pose.matrix).reshape((4, 4))[:3, :3],
                np.asarray(matrix).reshape((3, 3)).T, atol=1e-3)

    def test_scene_state_pickle(self):
        self.client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        self.client.getCameraImage(320, 240)
        assert self.render.scene_state

        buffer = pickle.dumps(self.render.scene_state)
        scene_state_copy = pickle.loads(buffer)
        assert self.render.scene_state == scene_state_copy
