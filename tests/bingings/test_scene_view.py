import numpy as np
import pickle

from pybullet_rendering import LightType
from .base_test_case import BaseTestCase


class SceneViewTest(BaseTestCase):

    def test_scene_view(self):
        width, height = 320, 240
        shadow = True
        flags = 0xAFAF
        projectionMatrix = list(np.random.random_sample((4, 4)).flatten())
        viewMatrix = list(np.random.random_sample((4, 4)).flatten())
        lightDirection = np.random.random_sample(3)
        lightColor = np.random.random_sample(3)
        lightDistance = np.random.random_sample()
        lightAmbientCoeff = np.random.random_sample()
        lightDiffuseCoeff = np.random.random_sample()
        lightSpecularCoeff = np.random.random_sample()

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

        assert self.render.scene_view

        #flags
        assert self.render.scene_view.flags == flags

        # viewport
        viewport = self.render.scene_view.viewport
        assert viewport == [width, height]

        # camera
        camera = self.render.scene_view.camera
        assert np.allclose(camera.proj_mat, projectionMatrix)
        assert np.allclose(camera.view_mat, viewMatrix)

        # light
        light = self.render.scene_view.light
        assert light.type == LightType.DirectionalLight
        assert np.allclose(light.position, lightDirection)
        assert np.allclose(light.direction, -np.array(lightDirection))
        assert np.allclose(light.color, lightColor)
        assert np.allclose(light.distance, lightDistance)
        assert np.allclose(light.ambient_coeff, lightAmbientCoeff)
        assert np.allclose(light.diffuse_coeff, lightDiffuseCoeff)
        assert np.allclose(light.specular_coeff, lightSpecularCoeff)
        assert np.allclose(light.shadow_caster, shadow)

    def test_scene_view_pickle(self):
        projectionMatrix = list(np.eye(4).flatten())
        viewMatrix = list(np.eye(4).flatten())

        self.client.getCameraImage(
            320,  #
            240,
            projectionMatrix=projectionMatrix,
            viewMatrix=viewMatrix)
        assert self.render.scene_view

        buffer = pickle.dumps(self.render.scene_view)
        scene_view_copy = pickle.loads(buffer)
        assert self.render.scene_view == scene_view_copy
