import os

import numpy as np
import pyrender as pyr
import trimesh
from PIL import Image

import pybullet_rendering as pr

from .utils import decompose, mask_to_rgb, primitive_mesh, rgb_to_mask

__all__ = ('PyrRenderer', 'PyrViewer')


class PyrRenderer(pr.BaseRenderer):
    """Pyrender-based offscreen renderer."""

    def __init__(self,
                 callback_fn=None,
                 render_mask=True,
                 shadows=True,
                 platform=None,
                 device_id=0
                 ):
        """Construct a Renderer.

        Keyword Arguments:
            callback_fn {callable} -- call a function with rendered images instead of passing to bullet (default: {None})
            render_mask {bool} -- render segmentation mask or not (default: {False})
            shadows {bool} -- render shadows for all lights (default: {True})
            platform {str} -- PyOpenGL platform ('egl', 'osmesa', etc.) (default: {None})
            device_id {int} -- EGL device id if platform is 'egl' (default: {0})
        """
        super().__init__()

        self._render_mask = render_mask
        self._flags = pyr.RenderFlags.NONE
        self._callback_fn = callback_fn

        if shadows:
            self._flags |= pyr.RenderFlags.SHADOWS_DIRECTIONAL

        if platform is not None:
            os.environ["PYOPENGL_PLATFORM"] = platform
            os.environ["EGL_DEVICE_ID"] = str(device_id)
        self._renderer = pyr.OffscreenRenderer(0, 0)
        self._scene = Scene()

    @property
    def scene(self):
        """Scene.

        Returns:
            Scene -- internal scene
        """
        return self._scene

    def update_scene(self, scene_graph, materials_only):
        """Update a scene graph.

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        self._scene.update_graph(scene_graph, materials_only)

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings.

        Arguments:
            scene_state {SceneState} -- scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        self._scene.update_state(scene_state)
        self._scene.update_view(scene_view)

        self._renderer.viewport_width = scene_view.viewport[0]
        self._renderer.viewport_height = scene_view.viewport[1]

        # render color and depth
        flags = self._flags | pyr.RenderFlags.RGBA

        if scene_view.light and scene_view.light.shadow_caster:
            flags |= pyr.RenderFlags.SHADOWS_DIRECTIONAL

        ret = self._renderer.render(self._scene, flags)
        color, depth = ret if isinstance(ret, tuple) else (None, ret)

        # render segment mask
        if not self._render_mask:
            mask = None
        else:
            flags |= pyr.RenderFlags.SEG
            flags &= ~pyr.RenderFlags.RGBA
            mask_rgb, _ = self._renderer.render(
                self._scene, flags, self._scene._seg_node_map)
            mask = rgb_to_mask(mask_rgb)

        if self._callback_fn is not None:
            # pass result to a callback function
            self._callback_fn(color, depth, mask)
            return False

        # pass result to bullet
        if color is not None:
            frame.color_img[:] = color
        if depth is not None:
            frame.depth_img[:] = depth
        if mask is not None:
            frame.mask_img[:] = mask
        return True


class PyrViewer(pr.BaseRenderer):
    """Pyrender-based onscreen viewer.

    Use for debug purposes only.
    This class cannot return rendered data.
    You should call pybullet.getCameraImage(...) to trigger window update.
    """

    def __init__(self):
        """Construct a PyrenderViewer."""
        super().__init__()
        self._scene = Scene()
        self._viewer = None

    @property
    def scene(self):
        """Internal scene.

        Returns:
            Scene -- internal scene
        """
        return self._scene

    def update_scene(self, scene_graph, materials_only):
        """Update scene graph.

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        self._scene.update_graph(scene_graph, materials_only)

    def render_frame(self, scene_state, scene_view, frame):
        """Render scene at scene_state with a scene_view settings.

        Arguments:
            scene_state {SceneState} -- scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        self._scene.update_state(scene_state)
        self._scene.update_view(scene_view)

        if self._viewer is None:
            self._viewer = pyr.Viewer(self._scene, run_in_thread=True)

        return False


class Scene(pyr.Scene):
    """Helper Scene wrapper."""

    def __init__(self):
        """Construct a Scene."""
        super().__init__()
        self._bullet_nodes = {}
        self._seg_node_map = {}
        self.bg_color = (0.7, 0.7, 0.8)
        self.ambient_light = (0.2, 0.2, 0.2)

        self._camera_node = pyr.Node(
            camera=pyr.PerspectiveCamera(np.deg2rad(60.0)),
            translation=(0.0, -2.0, 3.0),
            rotation=(-0.472, 0.0, 0.0, 0.882))
        self.add_node(self._camera_node)

        self._light_node = pyr.Node(
            light=pyr.DirectionalLight(color=(0.8, 0.8, 0.8), intensity=5.0),
            translation=(-0.8, -0.2, 2.0),
            rotation=(-0.438, 0.342, -0.511, 0.655))
        self.add_node(self._light_node)

    def update_graph(self, scene_graph, materials_only):
        """Update scene graph.

        This function rebuild scene completely each time when something changed.
        TODO: update changed nodes instead.

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for uid, node in self._bullet_nodes.items():
            self.remove_node(node)
        self._bullet_nodes = {}
        self._seg_node_map = {}

        for uid, body in scene_graph.nodes.items():
            node = pyr.Node(uid)
            self.add_node(node)
            self._bullet_nodes[uid] = node

            for shape in body.shapes:
                if shape.material is None:
                    material = None
                else:
                    material = pyr.MetallicRoughnessMaterial(
                        baseColorFactor=shape.material.diffuse_color,
                        metallicFactor=0.2,
                        roughnessFactor=0.8,
                        alphaMode='BLEND')
                    texture = shape.material.diffuse_texture
                    if texture is not None:
                        if texture.bitmap is not None:
                            image = Image.fromarray(texture.bitmap)
                        else:
                            image = Image.open(os.path.abspath(texture.filename))
                        texture = pyr.Texture(source=image, source_channels=image.mode)
                        material.baseColorTexture = texture

                if shape.mesh is None:
                    mesh = primitive_mesh(shape)
                elif shape.mesh.data is None:
                    mesh = trimesh.load(os.path.abspath(shape.mesh.filename))
                else:
                    data = shape.mesh.data
                    mesh = trimesh.Trimesh(
                        vertices=data.vertices,
                        vertex_normals=data.normals,
                        faces=data.faces,
                        visual=trimesh.visual.TextureVisuals(uv=data.uvs))

                mesh = pyr.Mesh.from_trimesh(mesh, material=material)
                mesh_node = self.add(mesh, pose=shape.pose.matrix.T, parent_node=node)
                self._seg_node_map[mesh_node] = mask_to_rgb(body.body, body.link)

    def update_state(self, scene_state):
        """Apply scene state.

        Arguments:
            scene_state {SceneState} -- transformations of all objects in the scene
        """
        for uid, node in self._bullet_nodes.items():
            self.set_pose(node, scene_state.pose(uid).matrix.T)

    def update_view(self, scene_view):
        """Apply scene state.

        Arguments:
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
        """
        if scene_view.light is not None:
            self.ambient_light = scene_view.light.ambient_color
            self._light_node.light.color = scene_view.light.diffuse_color
            self._light_node.translation = scene_view.light.position

        if scene_view.camera is not None:
            yfov, znear, zfar, aspect = decompose(scene_view.camera.projection_matrix)
            self._camera_node.camera.yfov = yfov
            self._camera_node.camera.znear = znear
            self._camera_node.camera.zfar = zfar
            self._camera_node.camera.aspectRatio = aspect
            self._camera_node.matrix = scene_view.camera.pose_matrix.T
