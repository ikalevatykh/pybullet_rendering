# pybullet_rendering
External rendering for PyBullet

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


