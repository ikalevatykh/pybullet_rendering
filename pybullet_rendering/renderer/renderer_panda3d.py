import numpy as np
import pybullet as pb
import pybullet_data
import pybullet_rendering

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

from pybullet_rendering import Renderer, ShapeType, LightType

__all__ = ['RendererP3D']


class RendererPanda3D(Renderer):

    def __init__(self,
                 frame_size_max=(256, 256),
                 force_hardware=True,
                 MSAA_samples=0,
                 CSAA_samples=0,
                 sRGB_color=False,
                 fbp=None):
        """Offscreen renderer based on Panda3D

        Keyword Arguments:
            frame_size_max {tuple} -- maximal frame size (default: {256, 256})
            force_hardware {bool} -- force hardware rendering (default: {True})
            MSAA_samples {int} -- MSAA (Multi-Sampling Anti-Aliasing) level (default: {0})
            CSAA_samples {int} -- CSAA (Coverage Sampling Antialiasing) level (default: {0})
            sRGB_color {bool} -- apply sRGB colorspace gamma correction (default: {False})
            fbp {FrameBufferProperties} -- framebuffer properties to overide
        """
        Renderer.__init__(self)
        self.win = None
        self.use_bullet_camera = True
        self.use_bullet_light = True
        self.auto_copy_to_ram = True
        self.return_to_bullet = True

        # configure Panda3D
        loadPrcFileData("", f"""
            model-path {pybullet_data.getDataPath()}
        """)
        # make default graphics pipe
        pipe = GraphicsPipeSelection.get_global_ptr().make_default_pipe()
        if pipe is None or not pipe.is_valid():
            raise RuntimeError("GraphicsPipe is invalid")
        self.pipe = pipe
        self.engine = GraphicsEngine.get_global_ptr()
        # make offscreen framebuffer
        if fbp is None:
            fbp = FrameBufferProperties(FrameBufferProperties.getDefault())
            fbp.set_force_hardware(force_hardware)
            fbp.set_force_software(not force_hardware)
            fbp.set_multisamples(MSAA_samples)
            fbp.set_coverage_samples(CSAA_samples)
            fbp.set_srgb_color(sRGB_color)
            fbp.set_rgba_bits(8, 8, 8, 8)
            fbp.set_depth_bits(1)
        self.win = self.engine.make_output(self.pipe, 'buffer', 0, fbp,
                                           WindowProperties.size(*frame_size_max),
                                           GraphicsPipe.BFRefuseWindow)
        if self.win is None:
            raise RuntimeError("GraphicsPipe cannot make offscreen buffers")
        self.win_size = frame_size_max
        self.win.set_clear_color_active(False)
        # bind textures
        self.color_tex = Texture("colortex")
        self.depth_tex = Texture("depthtex")
        self.mask_tex = None  #TODO: implement mask rendering
        self.win.add_render_texture(self.color_tex, GraphicsOutput.RTMBindOrCopy,
                                    GraphicsOutput.RTPColor)
        self.win.add_render_texture(self.depth_tex, GraphicsOutput.RTMBindOrCopy,
                                    GraphicsOutput.RTPDepth)
        # scene root node
        self.render = NodePath('render')
        self.render.setAttrib(RescaleNormalAttrib.makeDefault())
        self.render.setTwoSided(False)
        self.render.setAntialias(AntialiasAttrib.MAuto)
        self.render.setDepthOffset(1)
        self.render.setShaderAuto()
        # camera
        self.cam = self.render.attach_new_node(Camera('cam'))
        self.cam.node().set_scene(self.render)
        # viewport
        self.viewport = self.win.make_display_region((0, 1, 0, 1))
        self.viewport.set_sort(-100)
        self.viewport.set_camera(self.cam)
        self.viewport.set_clear_color_active(True)
        # light
        self.light = BulletLight(self.render)
        # loader
        self.loader = Loader.Loader(None)
        self.engine.set_default_loader(self.loader.loader)
        # scene
        self.scene = BulletScene(self.render, self.loader)

    def update_scene(self, scene_graph, materials_only):
        """ Update a scene graph

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        self.scene.update(scene_graph, materials_only)

    def render_frame(self, scene_state, scene_view, frame):
        """ Update a scene state

        Arguments:
            scene_state {SceneState} --  scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        self.scene.set_state(scene_state)

        if self.use_bullet_camera:
            BulletCamera.update(self.cam, scene_view.camera)

        if self.use_bullet_light:
            self.light.update(scene_view.light)
        self.light.enable(self.use_bullet_light)

        frame_size = scene_view.viewport
        r = frame_size[0] / self.win_size[0]
        b = frame_size[1] / self.win_size[1]
        self.viewport.set_dimensions(0, r, 0, b)
        self.viewport.set_clear_color(Vec3(*scene_view.background_color))

        self.engine.render_frame()

        if self.auto_copy_to_ram or self.return_to_bullet:
            self.engine.extract_texture_data(self.color_tex, self.win.get_gsg())
            #TODO: implement depth and mask

        if self.return_to_bullet:
            color_ram = self.color_tex.getRamImageAs('RGBA')
            color_img = np.asarray(color_ram).reshape(*self.win_size, 4)
            frame.color_img[:] = np.flipud(color_img[:frame_size[0], :frame_size[1]])
            #TODO: implement depth and mask
            return True

        return False

    def cleanup(self):
        """ Remove framebuffer and cleanup resources
        """
        if self.win is not None:
            self.camera = None
            self.region = None
            self.win.clear_render_textures()
            self.textures = None
            self.engine.remove_window(self.win)
            self.win = None

    def __del__(self):
        self.cleanup()


class BulletCamera:
    """ Bullet camera helper
    """

    PB2PD = Mat4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1)

    @staticmethod
    def update(cam, cam_desc):
        """ Set camera matrices in PyBullet format
        """
        proj_mat = Mat4(*cam_desc.proj_mat)
        lens = MatrixLens()
        lens.set_user_mat(BulletCamera.PB2PD * proj_mat)
        cam.node().set_lens(lens)

        view_mat_inv = Mat4(*cam_desc.view_mat)
        view_mat_inv.invert_in_place()
        cam.set_mat(BulletCamera.PB2PD * view_mat_inv)


class BulletLight:
    """ Bullet light helper
    """

    def __init__(self, root_node):
        self.root = root_node
        self.enabled = False
        # ambient light
        alight = AmbientLight('alight')
        alight.set_color((0.6, 0.6, 0.6, 1.0))
        self.ambiet = self.root.attach_new_node(alight)
        # directional light
        dlight = DirectionalLight('dlight')
        dlight.set_color((0.35, 0.35, 0.35, 1.0))
        dlight.set_specular_color((0.05, 0.05, 0.05, 1.0))
        self.direct = self.root.attach_new_node(dlight)

    def enable(self, enable):
        """ Enable or disable lights
        """
        if enable and not self.enabled:
            self.root.set_light(self.ambiet)
            self.root.set_light(self.direct)
            self.enabled = True
        elif not enable and self.enabled:
            self.root.clear_light(self.ambiet)
            self.root.clear_light(self.direct)
            self.enabled = False

    def update(self, light_desc):
        """ Update lights using PyBullet light decription
        """
        self.ambiet.node().set_color(Vec3(*light_desc.ambient_color))
        self.direct.node().set_color(Vec3(*light_desc.diffuse_color))
        self.direct.node().set_specular_color(Vec3(*light_desc.specular_color))
        self.direct.node().set_shadow_caster(light_desc.shadow_caster)
        self.direct.look_at(Vec3(*light_desc.direction))


class BulletScene:
    """ Bullet scene helper
    """

    def __init__(self, root_node, loader):
        self.root_node = root_node
        self.loader = loader
        self.node_dict = {}

    def update(self, scene_graph, materials_only):
        """ Update scene
        """
        if not materials_only:
            # remove all current nodes
            for uid, body_node in self.node_dict.items():
                body_node.remove_node()
            self.node_dict = {}

        for uid, node_desc in scene_graph.nodes.items():
            body_node = NodePath(f'pb_{uid:03d}')

            for shape in node_desc.shapes:
                # load model
                if shape.type == ShapeType.Mesh:
                    filename = shape.mesh.filename
                elif shape.type == ShapeType.Cube:
                    filename = 'cube.obj'
                else:
                    continue
                model = self.loader.load_model(filename)
                model.reparent_to(body_node)
                # set relative position
                pose = shape.pose
                model.set_pos_quat_scale(Vec3(*pose.origin), Quat(*pose.quat), Vec3(*pose.scale))
                # fill material
                if shape.has_material:
                    mat_desc = shape.material
                    material = Material()
                    material.set_ambient(Vec4(*mat_desc.diffuse_color))
                    material.set_diffuse(Vec4(*mat_desc.diffuse_color))
                    material.set_specular(Vec3(*mat_desc.specular_color))
                    material.set_shininess(15.0)
                    model.set_material(material, 1)
                    # set texture
                    if mat_desc.diffuse_texture > -1:
                        tex = scene_graph.texture(mat_desc.diffuse_texture)
                        model.set_texture(tex.filename)

            body_node.flatten_light()
            body_node.reparent_to(self.root_node)
            self.node_dict[uid] = body_node

    def set_state(self, scene_state):
        """ Update node positions
        """
        for uid, body_node in self.node_dict.items():
            pose = scene_state.pose(uid)
            body_node.set_pos_quat_scale(Vec3(*pose.origin), Quat(*pose.quat), Vec3(*pose.scale))
