"""An example of recording an MP4 video using the offscreen Pyrender renderer."""
import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient

from pybullet_rendering.plugin import RenderingPlugin
from pybullet_rendering.render.pyrender import Renderer

import imageio
import tqdm
import pprint


frame_size = (1280, 720)
client = BulletClient(pb.DIRECT)
client.setAdditionalSearchPath(pybullet_data.getDataPath())
renderer = Renderer()
renderer.enable_shadows(True)
renderer.return_to_bullet = False
plugin = RenderingPlugin(client, renderer)

v_up = np.array([0, 0, 1.])
grav = -9.81 * v_up
client.setGravity(*grav)


plane = client.loadURDF('plane.urdf', useMaximalCoordinates=True)
table = client.loadURDF("table/table.urdf",
                        basePosition=(0, 1., 0.),
                        flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)


class Robot:
    def __init__(self, client) -> None:
        self._client = client
        robot = client.loadURDF('laikago/laikago.urdf', basePosition=(0, 0, 0.5),
                                baseOrientation=(-1, 1, 1, -1))
        self.uid = robot
        self._num_joints = client.getNumJoints(self.uid)
        self._joint_types = []
        self._joint_names = []
        self._joint_name_to_ids = {}
        self._joint_indices = range(self._num_joints)
        
        for j in range(self._num_joints):
            j_info = client.getJointInfo(self.uid, j)
            j_name = j_info[1].decode()
            self._joint_names.append(j_name)
            self._joint_types.append(j_info[2])
            self._joint_name_to_ids[j_name] = j
        pprint.pprint(self._joint_name_to_ids)

    def actuate(self, positions, indices=None):
        if indices is None:
            indices = self._joint_indices
        client.setJointMotorControlMultiDofArray(
            self.uid,
            indices,
            pb.POSITION_CONTROL,
            positions
        )
        

robot = Robot(client)


def get_matrices():
    cam_target = client.getBasePositionAndOrientation(robot.uid)[0]
    aspect = float(frame_size[0]) / frame_size[1]
    # view_matrix = pb.computeViewMatrixFromYawPitchRoll(cam_target, 2.0, 0, -40, 0, 2)
    cam_pos = np.array([0, -1, 1])
    view_matrix = pb.computeViewMatrix(cam_pos, cam_target, v_up)
    # view_matrix = np.array(view_matrix).reshape(4, 4)
    
    proj_matrix = pb.computeProjectionMatrixFOV(fov=60, aspect=aspect, nearVal=.1, farVal=10.0)
    # proj_matrix = np.array(proj_matrix).reshape(4, 4)
    return view_matrix, proj_matrix


view_matrix, proj_matrix = get_matrices()
print(proj_matrix)
print(view_matrix)

images = []
timestep = 1./240
vid_fps = 30
clock = 0.

writer = imageio.get_writer('render.mp4', fps=vid_fps)

total_duration = 6.
num_steps = int(total_duration / timestep)


def pose_traj(s):
    target_pose = [
        [np.pi/10 * np.sin(3 * s * np.pi/4)],
        [-np.pi/4 * (1 - 1./(1+s))],
        [ np.pi/6],
        [np.pi/8 * (1 - np.exp(-s / 2))],
        [np.pi/8 * (1 - np.exp(-s / 2))]
    ]
    indices = [3, 5, 0, 10, 7]
    return target_pose, indices

elapsed_vid_time = 0.
for t in tqdm.trange(num_steps):
    client.stepSimulation()
    view_matrix, proj_matrix = get_matrices()
    clock += timestep
    elapsed_vid_time += timestep

    if elapsed_vid_time >= 1./vid_fps:
        target_pose, indices = pose_traj(clock)
        robot.actuate(target_pose, indices)
        
        ret = client.getCameraImage(*frame_size,
                                    projectionMatrix=proj_matrix,
                                    viewMatrix=view_matrix,
                                    flags=pb.ER_NO_SEGMENTATION_MASK)
        # img = ret[2]
        # depth = ret[3]
        elapsed_vid_time = 0.
        img = renderer.color
        img = img[..., :3]
        images.append(img)
        writer.append_data(img)
