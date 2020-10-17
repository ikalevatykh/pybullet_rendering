from os import times
import numpy as np
import pyrender
import pybullet as pb

from pybullet_utils.bullet_client import BulletClient
import pybullet_data
from pybullet_rendering import RenderingPlugin, BaseRenderer, ShapeType
from pybullet_rendering.render.pyrender import Loader, PbLightNode, PbMaterial, PbCameraNode
from pybullet_rendering.render.utils import shape_filename

from pyrender import RenderFlags

import time
from collections import OrderedDict
import pprint


class PyrenderGUI(BaseRenderer):
    
    def __init__(self, client: BulletClient) -> None:
        BaseRenderer.__init__(self)
        self.return_to_bullet = False
        
        self.client = client
        self.plugin = RenderingPlugin(client, self)

        self._scene = pyrender.Scene(ambient_light=np.array([0.5, 0.5, 0.5, 1.]))
        # self._light = PbLightNode(self._scene)
        # self._camera = PbCameraNode(self._scene)
        self._node_dict = OrderedDict()
        self._loader = Loader()


        self._flags = pyrender.RenderFlags.RGBA
        self._color, self._depth = None, None

        self.setup_viewer()

    @property
    def scene(self):
        return self._scene

    def setup_viewer(self):
        render_flags = {
            "shadows": True,
        }
        viewer_flags = {
            "window_title": "Pyrender Bullet GUI",
        }
        viewer = pyrender.Viewer(self._scene,
                                 viewport_size=(1600, 900),
                                 run_in_thread=True,
                                 render_flags=render_flags,
                                 viewer_flags=viewer_flags)
        self._viewer = viewer


    def update_scene(self, scene_graph, materials_only):
        """Update a scene graph

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        self._viewer.render_lock.acquire()
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
                pose_mat = np.asarray(shape.pose.matrix).reshape(4, 4).T
                self._scene.add(mesh, pose=pose_mat, parent_node=node)
        self._viewer.render_lock.release()

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} -- scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        self._viewer.render_lock.acquire()
        for uid, node in self._node_dict.items():
            pose = np.asarray(scene_state.matrix(uid)).reshape(4, 4).T
            self._scene.set_pose(node, pose)
        self._viewer.render_lock.release()

        return False

    def close(self):
        return self._viewer.close_external()

if __name__ == "__main__":
    client = BulletClient(pb.DIRECT)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    grav = np.array([0., 0., -9.91])
    client.setGravity(*grav)
    gui = PyrenderGUI(client)
    
    
    plane = client.loadURDF("plane.urdf", useMaximalCoordinates=True)
    table = client.loadURDF("table/table.urdf",
                            basePosition=[0.4, 0., 0.],
                            flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    ball  = client.loadURDF("soccerball.urdf", basePosition=[0, 0, 1], globalScaling=.25)
    laikago = client.loadURDF("laikago/laikago.urdf",
                              basePosition=[0, 1.2, 0.5],
                              baseOrientation=[1, 0, 0, 1])
    
    
    ball_vel = np.array([1.6, 0., 1.])
    client.resetBaseVelocity(ball, ball_vel)
    
    client.getCameraImage(1, 1)

    num_steps = 30 * 2048
    dt = 1./240
    client.setTimeStep(dt)
    render_fps = 30
    # render every no. of steps
    num_steps_render = 1. / (dt * render_fps)

    for t in range(num_steps):
        time.sleep(dt)
        client.stepSimulation()
        if t % num_steps_render == 0:
            client.getCameraImage(1, 1)
            print(f"Time t={t}")
        
        t += 1
    gui.close()
