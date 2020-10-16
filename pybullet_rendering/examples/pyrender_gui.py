import numpy as np
import pybullet as pb

from pybullet_utils.bullet_client import BulletClient
import pybullet_data
from pybullet_rendering import RenderingPlugin, BaseRenderer, ShapeType
from pybullet_rendering.render.pyrender import Loader, PbMaterial, PbCameraNode
from pybullet_rendering.render.utils import shape_filename

import pyrender
from pyrender import RenderFlags

from collections import OrderedDict



class PyrenderGUI(BaseRenderer):
    
    def __init__(self, client: BulletClient) -> None:
        BaseRenderer.__init__(self)
        self.return_to_bullet = True
        
        self.client = client
        plugin = RenderingPlugin(client, self)
        self.plugin = plugin
        
        self._scene = pyrender.Scene(ambient_light=np.array([1, 1, 1, 1]))
        self._camera = PbCameraNode(self._scene)
        self._node_dict = OrderedDict()
        self._loader = Loader()
        self._flags = pyrender.RenderFlags.NONE
        self._color, self._depth = None, None

    def get_viewer(self):
        viewer = pyrender.Viewer(self._scene,
                                 viewport_size=(1920, 1080),
                                 run_in_thread=True)
        return viewer
    
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

        if self._camera.is_enabled:
            self._camera.update(scene_view.camera)

if __name__ == "__main__":
    client = BulletClient(pb.DIRECT)


    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    client.setGravity(*[0, 0, -9.81])
    
    gui = PyrenderGUI(client)
    viewer = gui.get_viewer()
    
    plane = client.loadURDF("plane.urdf")
    table = client.loadURDF("table/table.urdf", basePosition=[0.4, 0.04, 1.4])
    
    t = 0
    
    while True:
        client.stepSimulation()
        t += 1
        print(f"Time t={t}")
        if t % 30 == 0:
            input("press enter")
    
    
