import unittest

import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient

from pybullet_rendering import RenderingPlugin, BaseRenderer

__all__ = ['BaseTestCase']


class RendererMock(BaseRenderer):

    def __init__(self):
        super().__init__()
        self.scene_graph = None
        self.materials_only = None
        self.scene_state = None
        self.scene_view = None
        self.render_frame_fn = None

    def update_scene(self, scene_graph, materials_only):
        self.scene_graph = scene_graph
        self.materials_only = materials_only

    def render_frame(self, scene_state, scene_view, frame):
        self.scene_state = scene_state
        self.scene_view = scene_view
        if self.render_frame_fn is not None:
            return self.render_frame_fn(frame)
        return False


class BaseTestCase(unittest.TestCase):

    def setUp(self):
        self.client = BulletClient(pb.DIRECT)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.render = RendererMock()
        self.plugin = RenderingPlugin(self.client, self.render)
        self.random = np.random.RandomState(77)

    def tearDown(self):
        self.plugin.unload()
        self.client.disconnect()
