"""Renderers performance test."""

import argparse
import multiprocessing as mp
import pkgutil
from timeit import default_timer as timer

import numpy as np
import pybullet as pb
import pybullet_data
from PIL import Image
from pybullet_utils.bullet_client import BulletClient

from pybullet_rendering import RenderingPlugin
from pybullet_rendering.render.panda3d import P3dRenderer
from pybullet_rendering.render.pyrender import PyrRenderer
from pybullet_rendering.render.utils import depth_from_zbuffer

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-n', '--num_frames', type=int, default=1000,
                    help='Render num frames to compute average fps')
parser.add_argument('-f', '--frame_size', nargs='+', default=[256, 256], help='Frame size')
parser.add_argument('-e', '--engines', nargs='+',
                    default=['tiny', 'egl', 'pyrender', 'panda3d'], help='Engines to test')


def run_test(num_frames, frame_size, engine, fps_out):
    """Compute fps of a specific engine."""
    client = BulletClient(pb.DIRECT)
    client.setAdditionalSearchPath(pybullet_data.getDataPath())

    # load renderer
    if 'tiny' == engine:
        pass
    elif 'egl' == engine:
        egl = pkgutil.get_loader('eglRenderer')
        client.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
    elif 'panda3d' == engine:
        RenderingPlugin(client, P3dRenderer(multisamples=4))
    elif 'pyrender' == engine:
        RenderingPlugin(client, PyrRenderer(platform='egl'))

    # sample scene
    table = client.loadURDF("table/table.urdf", flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    client.resetBasePositionAndOrientation(table, [0.4, 0.04, -0.7], [0, 0, 0, 1])

    kuka = client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
    client.resetBasePositionAndOrientation(kuka, [0.0, 0.0, 0.0], [0, 0, 0, 1])
    client.resetJointState(kuka, 3, -1.57)

    proj_mat = pb.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=10)
    view_mat = pb.computeViewMatrixFromYawPitchRoll((0.4, 0, 0), 2.0, 0, -40, 0, 2)

    # warming up
    _, _, color, depth, mask = client.getCameraImage(
        *frame_size, projectionMatrix=proj_mat, viewMatrix=view_mat)
    if engine in ('tiny', 'egl'):
        depth = depth_from_zbuffer(depth, 0.1, 10.0)
    img = Image.fromarray(color[:, :, :3])
    img.save(f'color_{engine}.png')
    img = Image.fromarray(((depth / depth.max()) * 255).astype(np.uint8))
    img.save(f'depth_{engine}.png')
    img = Image.fromarray(mask.astype(np.uint16))
    img.save(f'mask_{engine}.png')

    # compute fps
    start = timer()
    for _ in range(num_frames):
        client.getCameraImage(
            *frame_size, projectionMatrix=proj_mat, viewMatrix=view_mat)
    end = timer()
    fps_out.value = num_frames / (end - start)


def main(args):
    results = {}

    for engine in args.engines:
        print(f'Testing {engine}...')
        fps = mp.Value('d', 0.0)
        proc = mp.Process(target=run_test, args=(args.num_frames, args.frame_size, engine, fps))
        proc.start()
        proc.join()
        results[engine] = fps.value

    print('Results:')
    for engine, fps in results.items():
        print(f'{engine}: {fps:.2f} fps')


if __name__ == '__main__':
    args = parser.parse_args()
    main(args)
