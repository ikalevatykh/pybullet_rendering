import argparse
import numpy as np
import pybullet as pb
import pybullet_data

from pybullet_utils.bullet_client import BulletClient
from pybullet_rendering import RenderingPlugin, BaseRenderer, ShapeType
from pybullet_rendering.render.panda3d import PbMaterial
from pybullet_rendering.render.utils import shape_filename

from direct.filter.CommonFilters import CommonFilters
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import Vec3, Vec4, Quat, Vec2
from panda3d.core import AmbientLight, DirectionalLight
from panda3d.core import PointLight, Spotlight, PerspectiveLens
from panda3d.core import Material, Texture, AntialiasAttrib
from panda3d.core import FrameBufferProperties, WindowProperties, GraphicsPipe
from panda3d.core import loadPrcFileData


parser = argparse.ArgumentParser("Example of using Panda3D for rendering")
parser.add_argument("--multisamples", type=int, default=0, help="The minimum number of samples requested")
parser.add_argument("--srgb", action="store_true", help="Enable gamma-correction")
parser.add_argument("--shadow_resolution", type=int, default=1024, help="Shadow buffer resolution")
parser.add_argument("--ambient_occlusion", action="store_true", help="Ambient occlusion filter")
parser.add_argument("--debug", action="store_true", help="Debug scene mode")
args = parser.parse_args()


loadPrcFileData("",
    f"""
    framebuffer-srgb  {1 if args.srgb else 0}
    framebuffer-multisample {1 if args.multisamples else 0}
    multisamples {args.multisamples}
    model-path {pybullet_data.getDataPath()}
    show-frame-rate-meter 1
    gl-compile-and-execute 1
    gl-use-bindless-texture 1
    prefer-texture-buffer 1
    audio-library-name null
    """)


class MyApp(BaseRenderer, ShowBase):

    def __init__(self):
        BaseRenderer.__init__(self)  # <- important
        ShowBase.__init__(self)

        client = BulletClient(pb.DIRECT)
        client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.client = client

        # bind external renderer
        plugin = RenderingPlugin(client, self)

        # setup scene
        self.nodes = {}
        self.camLens.setNearFar(.1, 20)
        self.camLens.setFilmSize(Vec2(0.030, 0.030))
        self.render.setAntialias(AntialiasAttrib.MAuto)
        self.render.setDepthOffset(1)
        self.render.setShaderAuto()
        self.setupScene(client)
        self.setupLights()

        # setup filters
        if args.ambient_occlusion:
            filters = CommonFilters(self.win, self.cam)
            filters.setAmbientOcclusion()

        # setup periodic tasks
        self.time = 0
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        self.taskMgr.add(self.stepSimulationTask, "StepSimulationTask")

        if args.debug:
            self.oobe()

    def setupScene(self, client):
        """Init pybullet scene"""
        table = client.loadURDF("table/table.urdf")
        client.resetBasePositionAndOrientation(
            table, [0.4, 0.04, -0.7], [0, 0, 0, 1])

        human = client.loadURDF("humanoid/humanoid.urdf", globalScaling=.25, 
                                basePosition=(0, -.6, .15), baseOrientation=(1,0,0,1))
        kuka = client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        client.resetBasePositionAndOrientation(
            kuka, [0.0, 0.0, 0.0], [0, 0, 0, 1])

        Q = [
            0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
            -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
        ]
        for i, q in enumerate(Q):
            client.setJointMotorControl2(bodyIndex=kuka,
                                        jointIndex=i,
                                        controlMode=pb.POSITION_CONTROL,
                                        targetPosition=q,
                                        targetVelocity=0,
                                        force=100,
                                        positionGain=0.01,
                                        velocityGain=1)

    def setupLights(self):
        """Setup extrnal lights"""

        # ambient light
        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # directional light
        dlight = DirectionalLight('dlight')
        dlight.setColor((0.8, 0.8, 0.5, 1))
        lens = PerspectiveLens()
        lens.setNearFar(1, 5)
        dlight.setLens(lens)
        dlight.setShadowCaster(True, args.shadow_resolution, args.shadow_resolution)
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setPos(2, -2, 3)
        dlnp.lookAt(0, 0, 0)
        self.render.setLight(dlnp)

        # spotlight
        slight = Spotlight('slight')
        slight.setColor((0.7, 0.7, 1.0, 1))
        lens = PerspectiveLens()
        lens.setNearFar(1, 5)
        slight.setLens(lens)
        slight.setShadowCaster(True, args.shadow_resolution, args.shadow_resolution)
        slnp = self.render.attachNewNode(slight)
        slnp.setPos(1, 1, 2)
        slnp.lookAt(0, 0, 0)
        self.render.setLight(slnp)

    def spinCameraTask(self, task):
        """Update camera position
        """
        deg = task.time * 6.0
        rad = deg * (np.pi / 180.0)
        dist = 5.
        self.camera.setPos(0.4 + dist * np.sin(rad), -dist * np.cos(rad), 1.5)
        self.camera.setHpr(deg, -15, 0)
        return Task.cont

    def stepSimulationTask(self, task):
        """Update light position
        """
        if task.time - self.time > 1 / 240.:
            self.client.stepSimulation()
            # this call trigger updateScene (if necessary) and draw methods
            self.client.getCameraImage(1, 1)
            self.time = task.time
        return Task.cont

    def update_scene(self, scene_graph, materials_only):
        """Update a scene using scene_graph description

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for k, v in scene_graph.nodes.items():
            node = self.render.attachNewNode(f'node_{k:02d}')
            self.nodes[k] = node

            for j, pb_shape in enumerate(v.shapes):
                filename = shape_filename(pb_shape)
                if not filename:
                    continue

                # load model
                shape = self.loader.load_model(filename)

                if shape.has_material:
                    # set material
                    material = PbMaterial(pb_shape.material)
                    shape.setMaterial(material, 1)
                    texture_id = pb_shape.material.diffuse_texture
                    # set texture
                    if texture_id > -1:
                        tex = scene_graph.texture(texture_id)
                        shape.setTexture(tex.filename)

                # set relative position
                pose = pb_shape.pose
                shape.reparentTo(node)
                shape.setPos(*pose.origin)
                shape.setQuat(Quat(*pose.quat))
                shape.setScale(*pose.scale)

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} --  scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer, ignore
        """
        for k, node in self.nodes.items():
            pose = scene_state.pose(k)
            node.setPos(*pose.origin)
            node.setQuat(Quat(*pose.quat))
            node.setScale(*pose.scale)

        self.setBackgroundColor(*scene_view.bg_color)
        return False


app = MyApp()
app.run()
