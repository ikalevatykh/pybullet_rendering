import numpy as np
import pickle

from pybullet_rendering import LightType
from .base_test_case import BaseTestCase


class FrameTest(BaseTestCase):

    def test_skip_frame(self):

        def render_frame_fn(frame):
            return False

        self.render.render_frame_fn = render_frame_fn

        w, h, color, depth, mask = self.client.getCameraImage(16, 8)
        self.assertEqual((w, h), (0, 0))
        self.assertEqual(color.shape, (0, 0, 4))
        self.assertEqual(depth.shape, (0, 0))
        self.assertEqual(mask.shape, (0, 0))

    def test_fill_frame(self):
        width, height = 16, 8
        color_img = self.random.randint(
            0, 255, size=(height, width, 4), dtype=np.uint8)
        depth_img = self.random.random_sample((height, width)).astype(np.float32)
        mask_img = self.random.randint(
            0, 2 ^ 32 - 1, size=(height, width), dtype=np.int32)

        def render_frame_fn(frame):
            frame.color_img[:] = color_img
            frame.depth_img[:] = depth_img
            frame.mask_img[:] = mask_img
            return True

        self.render.render_frame_fn = render_frame_fn

        w, h, color, depth, mask = self.client.getCameraImage(width, height)
        self.assertEqual((w, h), (width, height))
        np.testing.assert_almost_equal(color, color_img)
        np.testing.assert_almost_equal(depth, depth_img)
        np.testing.assert_almost_equal(mask, mask_img)
