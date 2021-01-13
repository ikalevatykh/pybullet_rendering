import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient

from pybullet_rendering import RenderingPlugin
from pybullet_rendering.render.panda3d import P3dRenderer
from pybullet_rendering.render.pyrender import PyrRenderer, PyrViewer


def loadPrimitive(client, collision=False, **kwargs):
    if collision:
        col_id = client.createCollisionShape(**kwargs)
        vis_id = -1
    else:
        col_id = -1
        vis_id = client.createVisualShape(**kwargs)

    return client.createMultiBody(
        baseCollisionShapeIndex=col_id,
        baseVisualShapeIndex=vis_id,
        baseInertialFramePosition=(0, 0, 0),
        baseInertialFrameOrientation=(0, 0, 0, 1))


def main():
    client = BulletClient(pb.DIRECT)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())

    # render = PyrViewer()
    # render = PyrRenderer(platform='egl', render_mask=True)  # Viewer() #
    render = P3dRenderer(show_window=False, multisamples=4, srgb_color=True)
    plugin = RenderingPlugin(client, render)

    tex = client.loadTexture('table/table.png')
    # 'changeTexture'

    # table = client.loadURDF("table/table.urdf", flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    # client.resetBasePositionAndOrientation(table, (0.4, 0.04, -0.6), (0.0, 0.0, 0.0, 1.0))

    table = client.loadURDF("table/table.urdf", flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    client.resetBasePositionAndOrientation(table, [0.0, 0.0, -0.7], [0, 0, 0, 1])

    # table = client.loadURDF("table/table.urdf")
    # client.resetBasePositionAndOrientation(
    #     table, [0.4, 0.04, -0.7], [0, 0, 0, 1])

    # kuka = client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
    # client.resetBasePositionAndOrientation(
    #     kuka, [0.0, 0.0, 0.0], [0, 0, 0, 1])

    # Q = [
    #     0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
    #     -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
    # ]
    # for i, q in enumerate(Q):
    #     client.setJointMotorControl2(bodyIndex=kuka,
    #                                 jointIndex=i,
    #                                 controlMode=pb.POSITION_CONTROL,
    #                                 targetPosition=q,
    #                                 targetVelocity=0,
    #                                 force=100,
    #                                 positionGain=0.01,
    #                                 velocityGain=1)


    cube = loadPrimitive(client, shapeType=pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=(1.0, 0.0, 0.0, 1.0))
    client.resetBasePositionAndOrientation(cube, (-0.6, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))

    sphere = loadPrimitive(client, shapeType=pb.GEOM_SPHERE, radius=0.1, rgbaColor=(0.0, 1.0, 0.0, 1.0))
    client.resetBasePositionAndOrientation(sphere, (-0.3, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))

    cylinder = loadPrimitive(client, shapeType=pb.GEOM_CYLINDER, radius=0.1, length=0.2, rgbaColor=(0.0, 0.0, 1.0, 1.0))
    client.resetBasePositionAndOrientation(cylinder, (0.3, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))

    capsule = loadPrimitive(client, shapeType=pb.GEOM_CAPSULE, radius=0.1, length=0.2, rgbaColor=(1.0, 0.0, 1.0, 1.0))
    client.resetBasePositionAndOrientation(capsule, (0.6, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))

    client.changeVisualShape(cube, -1, textureUniqueId=tex)

    # shape = loadPrimitive(
    #             client,
    #             collision=True,
    #             shapeType=pb.GEOM_HEIGHTFIELD,
    #             meshScale=[.05, .05, 0.25],
    #             fileName="heightmaps/ground0.txt",
    #             heightfieldTextureScaling=128)


    proj_mat = pb.computeProjectionMatrixFOV(
        fov=60, aspect=320.0/240.0, nearVal=0.1, farVal=10)
    view_mat = pb.computeViewMatrixFromYawPitchRoll(
        (0, 0, 0), 2.0, 0, -40, 0, 2)

    w, h, color1, depth1, mask1 = client.getCameraImage(
        512, 512, projectionMatrix=proj_mat, viewMatrix=view_mat)

    # view_mat = pb.computeViewMatrixFromYawPitchRoll(
    #     (0, 0, 0), 2.0, 0, -90, 0, 2)

    plugin.unload()

    w, h, color2, depth2, mask2 = client.getCameraImage(
        512, 512, projectionMatrix=proj_mat, viewMatrix=view_mat)

    plt.subplot(2, 3, 1)
    plt.imshow(color1[:, :, :3])
    plt.subplot(2, 3, 2)
    plt.imshow(depth1)
    plt.subplot(2, 3, 3)
    plt.imshow(mask1)

    plt.subplot(2, 3, 4)
    plt.imshow(color2[:, :, :3])
    plt.subplot(2, 3, 5)
    plt.imshow(depth2)
    plt.subplot(2, 3, 6)
    plt.imshow(mask2)
    plt.show()

    plugin.unload()


if __name__ == '__main__':
    main()
