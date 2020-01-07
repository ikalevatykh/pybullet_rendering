# pybullet_rendering
External rendering for [PyBullet](https://github.com/bulletphysics/bullet3/) simulator

## Install

### Install PyBullet

```
cd $BULLET_ROOT_DIR
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
python setup.py install
```

### Install package

```
git clone https://github.com/ikalevatykh/pybullet_rendering.git
cd pybullet_rendering
python setup.py install --bullet_dir "$BULLET_ROOT_DIR/bullet3"
```

## Examples

Example of using [Panda3D](https://www.panda3d.org/) for rendering in GUI mode:

    python -m pybullet_rendering.examples.panda3d_gui --multisamples 8
