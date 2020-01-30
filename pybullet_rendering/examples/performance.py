import argparse
import numpy as np
import pkgutil
import pybullet as pb
import pybullet_data

from PIL import Image
from timeit import default_timer as timer
from pybullet_utils.bullet_client import BulletClient
from pybullet_rendering import RenderingPlugin

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-f', '--frame_size', nargs='+', default=[256, 256], help='Frame size')
parser.add_argument('-e',
                    '--engines',
                    nargs='+',
                    default=['tiny', 'egl', 'pyrender', 'panda3d'],
                    help='Engines to test')


class PerformanceTest:
    """Performance test
    """

    def __init__(self, engine, frame_size):
        self._engine = engine
        self._frame_size = frame_size
        self._client = BulletClient(pb.DIRECT)
        self._client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._prepare_render(engine, frame_size)
        self._prepare_scene()

    def run(self):
        # warmup
        color, depth = self._render_frame()
        img = Image.fromarray(color[:, :, :3])
        img.save(f'color_{self._engine}.png')
        img = Image.fromarray(((depth / depth.max()) * 255).astype(np.uint8))
        img.save(f'depth_{self._engine}.png')
        # count fps
        num_frames = 1000
        start = timer()
        for i in range(num_frames):
            self._render_frame()
        end = timer()
        self._client.disconnect()
        fps = num_frames / (end - start)
        return fps

    def _render_frame(self):
        """Render one frame
        """
        ret = self._client.getCameraImage(*self._frame_size,
                                          projectionMatrix=self._proj_mat,
                                          viewMatrix=self._view_mat,
                                          flags=pb.ER_NO_SEGMENTATION_MASK)
        if self._renderer is None:
            return ret[2], ret[3]
        return self._renderer.color, self._renderer.depth

    def _prepare_render(self, engine, frame_size):
        """Load renderer
        """
        if engine == 'tiny':
            self._renderer = None
        elif engine == 'egl':
            egl = pkgutil.get_loader('eglRenderer')
            if (egl):
                self._client.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            else:
                self._client.loadPlugin("eglRendererPlugin")
            self._renderer = None
        elif engine == 'panda3d':
            from pybullet_rendering.render.panda3d import Renderer
            renderer = Renderer(MSAA_samples=4)
            renderer.return_to_bullet = False
            plugin = RenderingPlugin(self._client, renderer)
            self._renderer = renderer
        elif engine == 'pyrender':
            from pybullet_rendering.render.pyrender import Renderer
            renderer = Renderer()
            renderer.return_to_bullet = False
            plugin = RenderingPlugin(self._client, renderer)
            self._renderer = renderer

    def _prepare_scene(self):
        """Build an example scene
        """
        table = self._client.loadURDF("table/table.urdf",
                                      flags=pb.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        self._client.resetBasePositionAndOrientation(table, [0.4, 0.04, -0.7], [0, 0, 0, 1])

        kuka = self._client.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]
        self._client.resetBasePositionAndOrientation(kuka, [0.0, 0.0, 0.0], [0, 0, 0, 1])
        Q = [
            0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
            -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
        ]
        for i, q in enumerate(Q):
            self._client.resetJointState(kuka, i, q)

        self._proj_mat = pb.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=10)
        self._view_mat = pb.computeViewMatrixFromYawPitchRoll((0.4, 0, 0), 2.0, 0, -40, 0, 2)


def main(args):
    results = {}
    for engine in args.engines:
        print(f'Test {engine}...')
        test = PerformanceTest(engine, args.frame_size)
        fps = test.run()
        results[engine] = fps
    print('Results:')
    for engine, fps in results.items():
        print(f'{engine}: {fps:.2f} fps')


if __name__ == '__main__':
    args = parser.parse_args()
    main(args)
