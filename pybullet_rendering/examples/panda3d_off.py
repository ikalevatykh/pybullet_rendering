import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient
from pybullet_rendering import RenderingPlugin
from pybullet_rendering.renderer import RendererPanda3D
from PIL import Image

FRAME_SIZE = (512, 512)


def make_scene(client):
    """ Build example scene """
    table = client.loadURDF("table/table.urdf")
    client.resetBasePositionAndOrientation(table, [0.4, 0.04, -0.7], [0, 0, 0, 1])

    kuka = client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
    client.resetBasePositionAndOrientation(kuka, [0.0, 0.0, 0.0], [0, 0, 0, 1])

    Q = [
        0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
        -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
    ]
    for i, q in enumerate(Q):
        client.resetJointState(kuka, i, q)


def get_camera_image(client):
    """ Get image using standart bullet interface """
    proj_mat = pb.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=10)
    view_mat = pb.computeViewMatrixFromYawPitchRoll((0.4, 0, 0), 2.0, 0, -40, 0, 2)
    w, h, color, depth, mask = client.getCameraImage(*FRAME_SIZE,
                                                     projectionMatrix=proj_mat,
                                                     viewMatrix=view_mat)
    return color


def main():
    client = BulletClient(pb.DIRECT)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    make_scene(client)
    color = get_camera_image(client)
    img = Image.fromarray(color[:, :, :3])
    img.save('color_tiny.jpg')
    print('saved color_tiny.jpg')

    client = BulletClient(pb.DIRECT)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())
    # it's nesessary to load plugin before any scene modifications
    plugin = RenderingPlugin(client)
    # can change renderer at any moment
    render = RendererPanda3D(frame_size_max=FRAME_SIZE, MSAA_samples=8)
    plugin.set_local_renderer(render)
    make_scene(client)
    color = get_camera_image(client)
    img = Image.fromarray(color[:, :, :3])
    img.save('color_panda3d.jpg')
    print('saved color_panda3d.jpg')


if __name__ == '__main__':
    main()
