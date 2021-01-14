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

    def test_load_nonempty_world(self):
        client = BulletClient(pb.DIRECT)
        client.loadURDF("table/table.urdf")
        with self.assertRaises(AssertionError):
            RenderingPlugin(client, RendererMock())

    def test_keep_reference(self):
        client = BulletClient(pb.DIRECT)
        RenderingPlugin(client, RendererMock())
        gc.collect()
        _ = client.getCameraImage(16, 16)
