import os
os.environ["PYOPENGL_PLATFORM"] = 'egl'

import numpy as np
import pyrender
import trimesh

from pyrender import RenderFlags
from pybullet_rendering import BaseRenderer
from .utils import shape_filename

__all__ = ['Renderer']


class Renderer(BaseRenderer):
    """Pyrender based offscreen renderer
    """

    def __init__(self):
        BaseRenderer.__init__(self)
        self.return_to_bullet = True

        self._scene = pyrender.Scene()
        self._camera = PbCameraNode(self._scene)
        self._light = PbLightNode(self._scene)
        self._node_dict = {}
        self._loader = Loader()

        self._offscreen = pyrender.OffscreenRenderer(0, 0)
        self._flags = pyrender.RenderFlags.NONE
        self._color, self._depth = None, None

    @property
    def color(self):
        """ndarray (h, w, 4) uint8: The color buffer in RGBA format
        """
        return self._color

    @property
    def depth(self):
        """ndarray (h, w) float32: The depth buffer in linear units
        """
        return self._depth

    @property
    def scene(self):
        """pyrender.Scene: Internal scene representation
        """
        return self._scene

    @property
    def flags(self):
        """pyrender.RenderFlags: A render specifications
        """
        return self._flags

    @flags.setter
    def flags(self, values):
        """pyrender.RenderFlags: Set render specifications
        """
        self._flags = values

    def enable_shadows(self, enabel: bool):
        """Enable shadows
        """
        shadow_flags = RenderFlags.SHADOWS_DIRECTIONAL | RenderFlags.SHADOWS_SPOT | RenderFlags.SHADOWS_POINT
        if enable:
            self._flags |= shadow_flags
        else:
            self._flags &= ~shadow_flags

    def enable_pybullet_camera(self, enable: bool):
        """Enable default pybullet camera
        """
        self._camera.enable(enable)

    def enable_pybullet_light(self, enable: bool):
        """Enable default pybullet light
        """
        self._light.enable(enable)

    def update_scene(self, scene_graph, materials_only):
        """Update a scene graph

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for uid, body in scene_graph.nodes.items():
            node = pyrender.Node(uid)
            self._scene.add_node(node)
            self._node_dict[uid] = node

            for shape in body.shapes:
                filename = shape_filename(shape)
                if not filename:
                    continue
                model = self._loader.load(filename, body.no_cache)
                mesh = pyrender.Mesh.from_trimesh(model)
                if shape.has_material:
                    material = PbMaterial(shape.material)
                    for p in mesh.primitives:
                        p.material = material
                pose_mat = np.asarray(shape.pose.matrix).reshape(4, 4).T
                self._scene.add(mesh, pose=pose_mat, parent_node=node)

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} -- scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        for uid, node in self._node_dict.items():
            pose = np.asarray(scene_state.matrix(uid)).reshape(4, 4).T
            self._scene.set_pose(node, np.asarray(pose))

        flags = self._flags | RenderFlags.RGBA

        if self._light.is_enabled:
            self._light.update(scene_view.light)
            if scene_view.light.shadow_caster:
                flags = flags | RenderFlags.SHADOWS_DIRECTIONAL

        if self._camera.is_enabled:
            self._camera.update(scene_view.camera)

        self._scene.bg_color = scene_view.bg_color
        self._offscreen.viewport_width = scene_view.viewport[0]
        self._offscreen.viewport_height = scene_view.viewport[1]
        ret = self._offscreen.render(self._scene, flags)

        if flags & RenderFlags.DEPTH_ONLY:
            self._color, self._depth = None, ret
        else:
            self._color, self._depth = ret

        if self.return_to_bullet:
            if not flags & RenderFlags.DEPTH_ONLY:
                frame.color_img[:] = self._color
            frame.depth_img[:] = self._depth
            #TODO: implement mask
            return True

        return False


class EnableableNode(pyrender.Node):
    """Helper node wrapper
    """

    def __init__(self, scene: pyrender.Scene, **kwargs):
        super().__init__(**kwargs)
        self._scene = scene
        self._enabled = True
        scene.add_node(self)

    @property
    def is_enabled(self):
        return self._enabled

    def enable(self, enable: bool):
        if enable and not self._enabled:
            self._scene.add_node(self)
            self._enabled = True
        elif not enable and self._enabled:
            self._scene.remove_node(self)
            self._enabled = False


class PbCameraNode(EnableableNode):
    """Pybullet-compatible camera node wrapper
    """

    def __init__(self, scene: pyrender.Scene):
        super().__init__(scene, camera=PbCamera())

    def update(self, pb_camera):
        proj_mat = np.asarray(pb_camera.proj_mat).reshape(4, 4).T
        pose_mat = np.asarray(pb_camera.pose_mat).reshape(4, 4).T
        self.camera.set_projection_matrix(proj_mat)
        self._scene.set_pose(self, pose_mat)


class PbLightNode(EnableableNode):
    """Pybullet-compatible light node wrapper
    """

    def __init__(self, scene: pyrender.Scene):
        super().__init__(scene, light=pyrender.DirectionalLight())

    def update(self, pb_light):
        self._scene.ambient_light = np.asarray(pb_light.ambient_color)
        self.light.color = pb_light.diffuse_color
        self.light.intensity = 5.0

        origin = np.asarray(pb_light.position)
        direction = np.asarray(pb_light.direction)
        tmp = [0, 1, 0.3] / np.linalg.norm([0, 1, 0.3])
        forward = -(direction / np.linalg.norm(direction))
        right = np.cross(tmp, forward)
        up = np.cross(forward, right)
        pose_mat = [[*up, origin[0]], [*right, origin[1]], [*forward, origin[2]], [0, 0, 0, 1]]
        self._scene.set_pose(self, np.asarray(pose_mat))


class PbCamera(pyrender.Camera):
    """Pybullet-compatible camera wrapper
    """

    def set_projection_matrix(self, mat):
        self._projection_mat = mat
        # update camera znear, zfar for retrieving depth buffers
        m22, m32 = -mat[2, 2], -mat[3, 2]
        self.zfar = (2.0 * m32) / (2.0 * m22 - 2.0)
        self.znear = ((m22 - 1.0) * self.zfar) / (m22 + 1.0)

    def get_projection_matrix(self, width=None, height=None):
        return self._projection_mat


class PbMaterial(pyrender.MetallicRoughnessMaterial):
    """Pybullet-compatible material wrapper
    """

    def __init__(self, pb_material):
        super().__init__(baseColorFactor=pb_material.diffuse_color,
                         metallicFactor=0.5,
                         roughnessFactor=0.5)


class Loader:
    """Mesh loader with a cache
    """

    def __init__(self):
        self._cache = {}

    def load(self, filename: str, no_cache=False):
        if filename in self._cache:
            return self._trimesh_[filename]
        model = trimesh.load(filename)
        if not no_cache:
            self._cache[filename] = model
        return model
