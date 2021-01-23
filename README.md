# pybullet_rendering
External rendering for [PyBullet](https://github.com/bulletphysics/bullet3/) simulator.

## Install

### Using pip

```
pip install pybullet_rendering
```

### From source code

Install NumPy

```
pip install numpy
```

Install PyBullet from source code

```
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
python setup.py install
export BULLET_ROOT_DIR="$PWD"
```

Install the package

```
cd ..
git clone https://github.com/ikalevatykh/pybullet_rendering.git
cd pybullet_rendering
python setup.py install --bullet_dir "$BULLET_ROOT_DIR"
```

Run tests

```
python -m unittest discover tests -v
```

## Run examples

Example of using [Panda3D](https://www.panda3d.org/) for rendering in GUI mode:

```
python -m pybullet_rendering.examples.panda3d_gui --multisamples 8
```
    
Test performance of diferent renderers:

```
python -m pybullet_rendering.examples.performance
```

## Renderers

This package provide example renderers based on [Panda3D](https://www.panda3d.org/) and [pyrender](https://github.com/mmatl/pyrender).

### Example of usage

```python
import pybullet as pb
from pybullet_rendering import RenderingPlugin
from pybullet_rendering.render.panda3d import P3dRenderer # panda3d-based renderer
from pybullet_rendering.render.pyrender import PyrRenderer # pyrender-based renderer

client_id = pb.connect(pb.DIRECT)

# bind your renderer to pybullet
renderer = P3dRenderer(multisamples=4) # or PyrRenderer(platform='egl', egl_device=1)
plugin = RenderingPlugin(client_id, renderer)

# render thru the standard pybullet API
w, h, rgba, depth, mask = pb.getCameraImage(w, h, projectionMatrix=..., viewMatrix=...)
```

### Implement your own renderer in Python

Your renderer should inherit from the `BaseRenderer` class and implement its `update_scene` and `render_frame` methods. To get an idea of their parameters, see examples and tests.

```python
from pybullet_rendering import BaseRenderer

class MyRenderer(BaseRenderer):

    def __init__(self):
        """The base class initializer have to be called first."""
        BaseRenderer.__init__(self)

    def update_scene(self, scene_graph, materials_only):
        """Update a scene using scene_graph description

        Arguments:
            scene_graph {SceneGraph} -- scene description
            materials_only {bool} -- update only shape materials
        """
        for uid, pb_node in scene_graph.nodes.items():
            "update nodes of your scene"

    def render_frame(self, scene_state, scene_view, frame):
        """Render a scene at scene_state with a scene_view settings

        Arguments:
            scene_state {SceneState} --  scene state, e.g. transformations of all objects
            scene_view {SceneView} -- view settings, e.g. camera, light, viewport parameters
            frame {FrameData} -- output image buffer
        """
        w, h = scene_view.viewport        
        if return_to_bullet:
            frame.color_img[:] = ... # np.uint8, (h,w,4)
            frame.depth_img[:] = ... # np.float32 (h,w)
            frame.mask_img[:] = ... # np.int32 (h,w)
            return True
            
        return False  
```

## Citation
If you find pybullet_rendering useful in your research, please cite the repository using the following BibTeX entry.
```
@Misc{kalevatykh2020pybullet_rendering,
  author =       {Kalevatykh, Igor et al.},
  title =        {pybullet_rendering - External rendering for the PyBullet simulator},
  howpublished = {Github},
  year =         {2020},
  url =          {https://github.com/ikalevatykh/pybullet_rendering}
}
```
## License
mano_pybullet is released under the [GPLv3](https://github.com/ikalevatykh/pybullet_rendering/blob/master/LICENSE) license.