import numpy as np

from panda3d.core import Vec2, Vec3, Vec4, Quat, Mat4
from panda3d.core import GraphicsPipe, GraphicsPipeSelection
from panda3d.core import GraphicsEngine, GraphicsOutput
from panda3d.core import FrameBufferProperties, WindowProperties
from panda3d.core import ModelNode, NodePath
from panda3d.core import Camera, MatrixLens, PerspectiveLens
from panda3d.core import AmbientLight, DirectionalLight, Spotlight
from panda3d.core import Material, Texture
from panda3d.core import RescaleNormalAttrib, AntialiasAttrib
from panda3d.core import loadPrcFileData

from direct.showbase import Loader

from pybullet_rendering import BaseRenderer
from .utils import shape_filename

__all__ = ['Renderer']


class Renderer(BaseRenderer):

    def __init__(self,
                 frame_size=None,
                 force_hardware=True,
                 MSAA_samples=0,
                 CSAA_samples=0,
                 sRGB_color=False):
        """Renderer based on Panda3D

        Keyword Arguments:
            frame_size {tuple} -- frame size (default: {256, 256})
            force_hardware {bool} -- force hardware rendering (default: {True})
            MSAA_samples {int} -- MSAA (Multi-Sampling Anti-Aliasing) level (default: {0})
            CSAA_samples {int} -- CSAA (Coverage Sampling Antialiasing) level (default: {0})
            sRGB_color {bool} -- apply sRGB colorspace gamma correction (default: {False})
        """
        BaseRenderer.__init__(self)
        self.return_to_bullet = True
        # renderer
        loadPrcFileData("",
            f"""
            gl-compile-and-execute 1
            gl-use-bindless-texture 1
            prefer-texture-buffer 1
            """)

        fbp = FrameBufferProperties(FrameBufferProperties.getDefault())
        fbp.set_force_hardware(force_hardware)
        fbp.set_force_software(not force_hardware)
        fbp.set_multisamples(MSAA_samples)
        fbp.set_coverage_samples(CSAA_samples)
        fbp.set_srgb_color(sRGB_color)
        self._renderer = OffscreenRenderer(frame_size, fbp)
        self._loader = Loader.Loader(None)
        # scene
        self.scene = NodePath('render')
        self.scene.setAttrib(RescaleNormalAttrib.makeDefault())
        self.scene.setTwoSided(False)
        self.scene.setAntialias(AntialiasAttrib.MAuto)
        self.scene.setShaderAuto()
        self._light = PbLightNode(self.scene)
        self._camera = PbCameraNode(self.scene)
        self._node_dict = {}

    @property
    def color(self):
        """ndarray (h, w, 4) uint8: The color buffer in RGBA format
        """
        return self._renderer.extract_color_image()

    @property
    def depth(self):
        """ndarray (h, w) float32: The depth buffer in linear units
        """
        return self._renderer.extract_depth_image()

    def enable_pybullet_camera(self, enable: bool):
        """Enable default pybullet camera
        """
        self._camera.set_active(enable)

    def enable_pybullet_light(self, enable: bool):
        """Enable default pybullet light
        """
        self._light.set_active(enable)

    def update_scene(self, scene_graph, materials_only):
        """Update a scene using scene_graph description

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for uid, pb_node in scene_graph.nodes.items():
            node = NodePath(f'pb_node_{uid:03d}')
            for pb_shape in pb_node.shapes:
                filename = shape_filename(pb_shape)
                if not filename:
                    continue
                shape = self._loader.load_model(filename)
                shape.set_mat(Mat4(*pb_shape.pose.matrix))
                shape.reparent_to(node)
                if shape.has_material:
                    shape.set_material(PbMaterial(pb_shape.material), 1)
                    texture_id = pb_shape.material.diffuse_texture
                    if texture_id > -1:
                        texture = scene_graph.texture(texture_id)
                        shape.set_texture(self._loader.loadTexture(texture.filename))
            node.flatten_light()
            node.reparent_to(self.scene)
            self._node_dict[uid] = node

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} --  scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        for uid, node in self._node_dict.items():
            node.set_mat(Mat4(*scene_state.matrix(uid)))
        if self._camera.is_active():
            self._camera.update(scene_view.camera)
        if self._light.is_active():
            self._light.update(scene_view.light)
        self._renderer.render_frame(self._camera, scene_view.viewport, scene_view.bg_color)
        if self.return_to_bullet:
            print(self.color.shape)
            frame.color_img[:] = self.color
            frame.depth_img[:] = self.depth
            #TODO: implement mask
            return True
        return False

    def __del__(self):
        self._loader.destroy()
        self._node_dict = None


class OffscreenRenderer:
    """Offscreen renderer based on Panda3D

    Arguments:
        frame_size {tuple} -- framebuffer size
        fb_props {FrameBufferProperties} -- framebuffer properties
    """

    def __init__(self, frame_size: tuple, fb_props: FrameBufferProperties):
        self._window = None
        # make default graphics pipe
        pipe = GraphicsPipeSelection.get_global_ptr().make_default_pipe()
        if pipe is None or not pipe.is_valid():
            raise RuntimeError("GraphicsPipe is invalid")
        self._pipe = pipe
        self._engine = GraphicsEngine.get_global_ptr()
        self._fb_props = fb_props
        if frame_size is not None:
            self._configure_window(frame_size)

    def render_frame(self, camera, framesize, bg_color):
        self._configure_window(framesize)
        self._viewport.set_clear_color(Vec3(*bg_color))
        self._viewport.set_camera(camera)
        lens = camera.node().get_lens()
        self._zlimits = lens.get_near(), lens.get_far()
        self._engine.render_frame()

    def extract_color_image(self):
        self._engine.extract_texture_data(self._color_tex, self._window.get_gsg())
        width, height = self._window_size
        color_buf = self._color_tex.get_ram_image_as('RGBA')
        tex_width = self._color_tex.get_x_size()
        tex_height = self._color_tex.get_y_size()
        color_im = np.asarray(color_buf)
        color_im = color_im.reshape((tex_height, tex_width, 4))
        color_im = color_im[:height, :width]
        return np.flipud(color_im)

    def extract_depth_image(self):
        self._engine.extract_texture_data(self._depth_tex, self._window.get_gsg())
        width, height = self._window_size
        depth_buf = self._depth_tex.get_ram_image()
        tex_width = self._color_tex.get_x_size()
        tex_height = self._color_tex.get_y_size()
        # adapted from https://github.com/mmatl/pyrender/pyrender/renderer.py
        depth_im = np.frombuffer(depth_buf, dtype=np.uint16)
        depth_im = depth_im.astype(np.float32) / np.iinfo(np.uint16).max
        depth_im = depth_im.reshape((tex_height, tex_width))
        depth_im = depth_im[:height, :width]
        depth_im = np.flip(depth_im, axis=0)
        inf_inds = (depth_im == 1.0)
        depth_im = 2.0 * depth_im - 1.0
        z_near, z_far = self._zlimits
        noninf = np.logical_not(inf_inds)
        if z_far is None:
            depth_im[noninf] = 2 * z_near / (1.0 - depth_im[noninf])
        else:
            depth_im[noninf] = ((2.0 * z_near * z_far) / (z_far + z_near - depth_im[noninf] *
                                                          (z_far - z_near)))
        depth_im[inf_inds] = 0.0
        return depth_im

    def _configure_window(self, window_size):
        if self._window is not None:
            if not np.array_equal(window_size, self._window_size):
                self._delete_window()

        if self._window is None:
            # framebuffer
            fbp = FrameBufferProperties(self._fb_props)
            fbp.set_rgba_bits(8, 8, 8, 8)
            fbp.set_depth_bits(16)
            self._window = self._engine.make_output(self._pipe, 'buffer', 0, fbp,
                                                    WindowProperties.size(*window_size),
                                                    GraphicsPipe.BFRefuseWindow)
            if self._window is None:
                raise RuntimeError("GraphicsPipe cannot make offscreen buffers")
            self._window_size = window_size
            self._window.set_clear_color_active(False)
            # bind textures
            self._color_tex = Texture("colortex")
            self._depth_tex = Texture("depthtex")
            self._window.add_render_texture(self._color_tex, GraphicsOutput.RTMBindOrCopy,
                                            GraphicsOutput.RTPColor)
            self._window.add_render_texture(self._depth_tex, GraphicsOutput.RTMBindOrCopy,
                                            GraphicsOutput.RTPDepth)
            # viewport
            self._viewport = self._window.make_display_region((0, 1, 0, 1))
            self._viewport.set_sort(-100)
            self._viewport.set_clear_color_active(True)

    def _delete_window(self):
        self._window.clear_render_textures()
        self._color_tex, self._depth_tex = None, None
        self._engine.remove_window(self._window)
        self._window = None
        self._viewport = None
        self._window_size = (0, 0)
        self._engine.remove_all_windows()

    def cleanup(self):
        """ Remove framebuffer and cleanup resources
        """
        if self._window is not None:
            self._delete_window()

    def __del__(self):
        self.cleanup()


class PbCameraNode(NodePath):
    """Pybullet-compatible camera node wrapper
    """

    Z2Y = Mat4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1)

    def __init__(self, render: NodePath):
        self._camera = Camera('pb_camera')
        self._lens = MatrixLens()
        self._camera.set_lens(self._lens)
        super().__init__(self._camera)
        self.reparent_to(render)

    def set_active(self, active: bool):
        self._camera.set_active(active)

    def is_active(self):
        return self._camera.is_active()

    def update(self, pb_camera):
        self.set_mat(self.Z2Y * Mat4(*pb_camera.pose_mat))
        mat = np.asarray(pb_camera.proj_mat).reshape(4, 4)
        m22, m32 = -mat[2, 2], -mat[3, 2]
        zfar = (2.0 * m32) / (2.0 * m22 - 2.0)
        znear = ((m22 - 1.0) * zfar) / (m22 + 1.0)
        self._lens.set_near_far(znear, zfar)
        self._lens.set_user_mat(self.Z2Y * Mat4(*pb_camera.proj_mat))


class PbLightNode():
    """Pybullet-compatible light node wrapper
    """

    def __init__(self, render: NodePath):
        self._alight = AmbientLight('pb_alight')
        self._dlight = DirectionalLight('pb_dlight')
        self._anode = render.attach_new_node(self._alight)
        self._dnode = render.attach_new_node(self._dlight)
        self._render = render
        self._is_active = False
        self.set_active(True)

    def set_active(self, active: bool):
        if active and not self._is_active:
            self._render.set_light(self._anode)
            self._render.set_light(self._dnode)
        elif not active and self._is_active:
            self._render.clear_light(self._anode)
            self._render.clear_light(self._dnode)
        self._is_active = active

    def is_active(self):
        return self._is_active

    def update(self, pb_light):
        self._alight.set_color(Vec3(*pb_light.ambient_color))
        self._dlight.set_color(Vec3(*pb_light.diffuse_color))
        self._dlight.set_specular_color(Vec3(*pb_light.specular_color))
        self._dlight.set_shadow_caster(pb_light.shadow_caster)
        self._dnode.set_pos(Vec3(*pb_light.position))
        self._dnode.look_at(0, 0, 0)


class PbMaterial(Material):
    """Pybullet-compatible material wrapper
    """

    def __init__(self, pb_material):
        super().__init__()
        self.set_ambient(Vec4(*pb_material.diffuse_color))
        self.set_diffuse(Vec4(*pb_material.diffuse_color))
        self.set_specular(Vec3(*pb_material.specular_color))
        self.set_shininess(15.0)
