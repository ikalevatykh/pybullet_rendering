import gc
import unittest

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient

from pybullet_rendering import BaseRenderer, RenderingPlugin


class RendererMock(BaseRenderer):
    def __init__(self):
        super().__init__()

    def update_scene(self, scene_graph, materials_only):
        pass

    def render_frame(self, scene_state, scene_view, frame):
        return False

class PluginTest(unittest.TestCase):

    def test_keep_reference(self):
        self.client = BulletClient(pb.DIRECT)
        RenderingPlugin(self.client, RendererMock())
        gc.collect()
        _ = self.client.getCameraImage(16, 16)
