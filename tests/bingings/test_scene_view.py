import numpy as np
import pickle

from pybullet_rendering import LightType
from .base_test_case import BaseTestCase


class SceneViewTest(BaseTestCase):

    def test_scene_view(self):
        width, height = 320, 240
        shadow = True
        flags = 0xAFAF
        projectionMatrix = list(self.random.random_sample((4, 4)).flatten())
        viewMatrix = list(self.random.random_sample((4, 4)).flatten())
        lightDirection = self.random.random_sample(3)
        lightColor = self.random.random_sample(3)
        lightDistance = self.random.random_sample()
        lightAmbientCoeff = self.random.random_sample()
        lightDiffuseCoeff = self.random.random_sample()
        lightSpecularCoeff = self.random.random_sample()

        ret = self.client.getCameraImage(  #
            width,
            height,
            projectionMatrix=projectionMatrix,
            viewMatrix=viewMatrix,
            lightDirection=lightDirection,
            lightColor=lightColor,
            lightDistance=lightDistance,
            shadow=shadow,
            lightAmbientCoeff=lightAmbientCoeff,
            lightDiffuseCoeff=lightDiffuseCoeff,
            lightSpecularCoeff=lightSpecularCoeff,
            flags=flags)

        self.assertIsNotNone(self.render.scene_view)

        #flags
        self.assertEqual(self.render.scene_view.flags, flags)

        # viewport
        viewport = self.render.scene_view.viewport
        self.assertEqual(viewport, [width, height])

        # camera
        camera = self.render.scene_view.camera
        np.testing.assert_almost_equal(camera.proj_mat, projectionMatrix)
        np.testing.assert_almost_equal(camera.view_mat, viewMatrix)

        # light
        light = self.render.scene_view.light
        self.assertEqual(light.type, LightType.DirectionalLight)
        np.testing.assert_almost_equal(light.position, lightDirection)
        np.testing.assert_almost_equal(light.direction, -np.array(lightDirection))
        np.testing.assert_almost_equal(light.color, lightColor)
        np.testing.assert_almost_equal(light.distance, lightDistance)
        np.testing.assert_almost_equal(light.ambient_coeff, lightAmbientCoeff)
        np.testing.assert_almost_equal(light.diffuse_coeff, lightDiffuseCoeff)
        np.testing.assert_almost_equal(light.specular_coeff, lightSpecularCoeff)
        np.testing.assert_almost_equal(light.shadow_caster, shadow)

    def test_scene_view_pickle(self):
        projectionMatrix = list(np.eye(4).flatten())
        viewMatrix = list(np.eye(4).flatten())

        self.client.getCameraImage(
            320,  #
            240,
            projectionMatrix=projectionMatrix,
            viewMatrix=viewMatrix)
        self.assertIsNotNone(self.render.scene_view)

        buffer = pickle.dumps(self.render.scene_view)
        scene_view_copy = pickle.loads(buffer)
        self.assertEqual(self.render.scene_view, scene_view_copy)
