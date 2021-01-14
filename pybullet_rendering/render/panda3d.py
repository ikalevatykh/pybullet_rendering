import os

import numpy as np
import panda3d.core as p3d

import pybullet_rendering as pr

from .utils import decompose, depth_from_zbuffer, primitive_mesh

__all__ = ('P3dRenderer')


class P3dRenderer(pr.BaseRenderer):
    """Panda3D-based renderer.

    Note: You cannot use more than one renderer per process at the same time.
    """

    def __init__(self,
                 callback_fn=None,
                 multisamples=0,
                 srgb_color=False,
                 show_window=False):
        """Construct a Renderer.

        Keyword Arguments:
            callback_fn {callable} -- call a function with rendered images instead of passing to bullet (default: {None})
            multisamples {bool} -- antialiasing multisamples: 0 (disabled), 2, 4, etc. (default: {0})
            srgb_color {bool} -- enable sRGB recoloring (default: False)
            show_window {bool} -- open a window (mostly for debug purposes) (default: False)
        """
        pr.BaseRenderer.__init__(self)
        self._callback_fn = callback_fn
        self._scene = Scene()
        self._renderer = Renderer(multisamples, srgb_color, show_window)

    @property
    def scene(self):
        """Scene representation.

        Returns:
            NodePath -- scene root node path
        """
        return self._scene

    def update_scene(self, scene_graph, materials_only):
        """Update a scene graph

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        self._scene.update_graph(scene_graph, materials_only)

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} -- scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        self._scene.update_state(scene_state)
        self._scene.update_view(scene_view)

        color, depth, mask = self._renderer.render_frame(
            self._scene, *scene_view.viewport)

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


class Renderer:
    """Internal renderer implementation."""

    def __init__(self, multisamples=0, srgb_color=False, show_window=False):
        """Construct a Renderer.

        Keyword Arguments:
            multisamples {bool} -- antialiasing multisamples: 0 (disabled), 2, 4, etc. (default: {0})
            srgb_color {bool} -- enable sRGB recoloring (default: False)
            show_window {bool} -- open a window (default: False)
        """
        self._multisamples = multisamples
        self._srgb_color = srgb_color
        self._show_window = show_window

        p3d.ConfigVariableBool('allow-incomplete-render').set_value(False)

        self._engine = p3d.GraphicsEngine.get_global_ptr()
        self._pipe = p3d.GraphicsPipeSelection.get_global_ptr().make_default_pipe()
        self._buffer_size = None
        self._buffer = None
        self._region = None
        self._depth_tex = None
        self._color_tex = None

    def render_frame(self, scene, width, height):
        """Render one frame.

        Arguments:
            scene {Scene} -- scene to render
            width {int} -- target frame size
            height {int} -- target frame size
        """
        if self._buffer is None:
            self._make_buffer(width, height)
        elif self._buffer.get_size() != (width, height):
            self._remove_buffer()
            self._make_buffer(width, height)

        if self._region.camera != scene.camera:
            self._region.camera = scene.camera
            # TODO: shadows don't appear without this dummy rendering
            self._engine.render_frame()

        self._buffer.set_clear_color(scene.bg_color)
        self._engine.render_frame()

        data = self._color_tex.getRamImageAs('RGBA')
        color_image = np.frombuffer(data, np.uint8)
        color_image.shape = (height, width, 4)
        color_image = np.flipud(color_image)

        data = self._depth_tex.getRamImage()
        depth_image = np.frombuffer(data, np.float32)
        depth_image.shape = height, width
        lens = scene.camera.node().get_lens()
        depth_image = depth_from_zbuffer(depth_image, lens.near, lens.far)
        depth_image = np.flipud(depth_image)

        return color_image, depth_image, None

    def destroy(self):
        """Clean up resources."""
        self._remove_buffer()

    def _make_buffer(self, width, height):
        """Make an offscreen buffer.

        Arguments:
            width {int} -- target buffer width
            height {int} -- target buffer height
        """
        fb_prop = p3d.FrameBufferProperties(p3d.FrameBufferProperties.get_default())
        fb_prop.set_multisamples(self._multisamples)
        fb_prop.set_srgb_color(self._srgb_color)

        self._buffer = self._engine.make_output(
            self._pipe, name="offscreen", sort=0,
            fb_prop=p3d.FrameBufferProperties.get_default(),
            win_prop=p3d.WindowProperties(size=(width, height)),
            flags=p3d.GraphicsPipe.BFRefuseWindow)

        self._region = self._buffer.make_display_region()

        self._depth_tex = p3d.Texture()
        self._depth_tex.setFormat(p3d.Texture.FDepthComponent)
        self._buffer.addRenderTexture(
            self._depth_tex, p3d.GraphicsOutput.RTMCopyRam, p3d.GraphicsOutput.RTPDepth)

        self._color_tex = p3d.Texture()
        self._color_tex.setFormat(p3d.Texture.FRgba8)
        self._buffer.addRenderTexture(
            self._color_tex, p3d.GraphicsOutput.RTMCopyRam, p3d.GraphicsOutput.RTPColor)

    def _remove_buffer(self):
        """Remove existing offscreen buffer."""
        if self._buffer is not None:
            self._engine.remove_window(self._buffer)
            self._buffer = None
            self._region = None

    def __del__(self):
        """Clean up resources."""
        self.destroy()


class Scene:
    """Internal scene implementation."""

    def __init__(self):
        """Construct a Scene."""
        self._nodes = {}
        self._seg_node_map = {}
        self._loader = p3d.Loader.get_global_ptr()
        self._render = p3d.NodePath('#scene')
        self._bg_color = (0.7, 0.7, 0.8, 0.0)

        # setup attributes
        self._render.set_attrib(p3d.RescaleNormalAttrib.makeDefault(), 1)
        self._render.set_two_sided(False, 1)
        self._render.set_antialias(p3d.AntialiasAttrib.MAuto, 1)
        self._render.set_shader_auto(1)
        self._render.set_depth_offset(1, 1)

        # setup default camera
        camera = p3d.Camera('#camera', p3d.PerspectiveLens())
        self._camera_np = self._render.attach_new_node(camera)
        self._camera_np.set_pos(0.0, -2.0, 3.0)
        self._camera_np.look_at(0.0, 0.0, 0.0)

        # setup ambient light
        alight = p3d.AmbientLight('#alight')
        alight.set_color((0.7, 0.7, 0.7, 0.0))
        self._alight_np = self._render.attach_new_node(alight)
        self._render.set_light(self._alight_np)

        # setup directional light
        dlight = p3d.DirectionalLight('#dlight')
        dlight.set_color((0.3, 0.3, 0.3, 0.0))
        dlight.get_lens().set_film_size(2, 2)
        dlight.get_lens().set_near_far(0.1, 10)
        dlight.set_shadow_caster(True, 512, 512)
        self._dlight_np = self._render.attach_new_node(dlight)
        self._dlight_np.set_pos(-0.8, -0.2, 2.0)
        self._dlight_np.look_at(0.0, 0.0, 0.0)
        self._render.set_light(self._dlight_np)

    def update_graph(self, scene_graph, materials_only):
        """Update scene graph.

        This function rebuild scene completely each time when something changed.
        TODO: update changed nodes instead.

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for uid, model_np in self._nodes.items():
            model_np.detach_node()
        self._nodes = {}
        self._seg_node_map = {}

        for uid, link in scene_graph.nodes.items():
            model_np = self._render.attach_new_node(
                p3d.ModelNode(f'#link_{link.body}_{link.link}'))
            model_np.node().set_preserve_transform(p3d.ModelNode.PTLocal)
            self._nodes[uid] = model_np

            for shape in link.shapes:
                if shape.mesh is None:
                    mesh = Mesh.from_trimesh(primitive_mesh(shape))
                elif shape.mesh.data is None:
                    filename = os.path.abspath(shape.mesh.filename)
                    mesh = self._loader.load_sync(filename)
                else:
                    mesh = Mesh.from_mesh_data(shape.mesh.data)

                mesh_np = model_np.attach_new_node(mesh)
                mesh_np.set_mat((*shape.pose.matrix.ravel(),))

                if shape.material is not None:
                    mesh_np.set_color((*shape.material.diffuse_color,))

                    material = p3d.Material()
                    material.set_diffuse((*shape.material.diffuse_color,))
                    material.set_ambient((*shape.material.diffuse_color,))
                    material.set_specular((*shape.material.specular_color, 0.0))
                    material.set_roughness(0.4)
                    mesh_np.set_material(material, 1)

                    if shape.material.diffuse_color[3] < 1.0:
                        mesh_np.set_transparency(p3d.TransparencyAttrib.M_alpha)

                    if shape.material.diffuse_texture:
                        filename = os.path.abspath(shape.material.diffuse_texture.filename)
                        texture = p3d.TexturePool.load_texture(filename)
                        mesh_np.set_texture(texture, 1)

    def update_state(self, scene_state):
        """Apply scene state.

        Arguments:
            scene_state {SceneState} -- transformations of all objects in the scene
        """
        for uid, node in self._nodes.items():
            node.set_mat(p3d.Mat4(*scene_state.matrix(uid).ravel()))

    def update_view(self, scene_view):
        """Apply scene state.

        Arguments:
            settings {SceneView} -- view settings, e.g. camera, light, viewport parameters
        """
        if scene_view.camera is not None:
            yfov, znear, zfar, aspect = decompose(scene_view.camera.projection_matrix)
            conv_mat = p3d.Mat4.convert_mat(p3d.CSZupRight, p3d.CSYupRight)
            self._camera_np.set_mat(conv_mat * p3d.Mat4(*scene_view.camera.pose_matrix.ravel(), ))
            self._camera_np.node().get_lens().set_near_far(znear, zfar)
            self._camera_np.node().get_lens().set_fov(np.rad2deg(yfov*aspect), np.rad2deg(yfov))

        if scene_view.light is not None:
            self._alight_np.node().set_color((*scene_view.light.ambient_color, 0.0))
            self._dlight_np.node().set_color((*scene_view.light.diffuse_color, 0.0))
            self._dlight_np.node().set_specular_color((*scene_view.light.specular_color, 0.0))
            self._dlight_np.node().set_shadow_caster(scene_view.light.shadow_caster)
            self._dlight_np.set_pos(*scene_view.light.position)
            self._dlight_np.look_at(0, 0, 0)

    @property
    def render(self):
        """Scene root node.

        Returns:
            NodePath -- scene root node path
        """
        return self._render

    @property
    def camera(self):
        """Camera node.

        Returns:
            NodePath -- camera node path
        """
        return self._camera_np

    @property
    def bg_color(self):
        """Background color.

        Returns:
            tuple -- RGBA color
        """
        return self._bg_color

    @bg_color.setter
    def bg_color(self, value):
        self._bg_color = value


class Mesh:
    """Mesh helper class."""

    @staticmethod
    def from_mesh_data(mesh):
        """Procedurally generating 3D mesh.

        Arguments:
            mesh {bindings.Mesh} -- input mesh data

        Returns:
            NodePath -- created geometry node
        """
        if len(mesh.normals) > 0 and len(mesh.uvs) > 0:
            vformat = p3d.GeomVertexFormat.get_v3n3t2()
            vertices = np.column_stack((mesh.vertices, mesh.normals, mesh.uvs))
        elif len(mesh.normals) > 0:
            vformat = p3d.GeomVertexFormat.get_v3n3()
            vertices = np.column_stack((mesh.vertices, mesh.normals))
        elif len(mesh.uvs) > 0:
            vformat = p3d.GeomVertexFormat.get_v3t2()
            vertices = np.column_stack((mesh.vertices, mesh.uvs))
        else:
            vformat = p3d.GeomVertexFormat.get_v3()
            vertices = mesh.vertices
        return Mesh._make(vformat, vertices, mesh.faces)

    @staticmethod
    def from_trimesh(mesh):
        """Procedurally generating 3D mesh.

        Arguments:
            mesh {trimesh.Trimesh} -- input mesh

        Returns:
            NodePath -- created geometry node
        """
        vformat = p3d.GeomVertexFormat.get_v3n3()
        vertices = np.column_stack((mesh.vertices, mesh.vertex_normals))
        # TODO: uvs
        return Mesh._make(vformat, vertices, mesh.faces)

    @staticmethod
    def _make(vformat, vertices, faces):
        vdata = p3d.GeomVertexData('#vdata', vformat, p3d.Geom.UHStatic)
        vdata.unclean_set_num_rows(len(vertices))
        vdata.modify_array_handle(0).set_subdata(0, len(vertices), vertices.astype(np.float32))

        prim = p3d.GeomTriangles(p3d.Geom.UHStatic)
        prim.clear_vertices()
        for v1, v2, v3 in faces:
            prim.add_vertices(v1, v2, v3)
        prim.close_primitive()

        geom = p3d.Geom(vdata)
        geom.add_primitive(prim)

        node = p3d.GeomNode('#geom')
        node.add_geom(geom)
        return node
