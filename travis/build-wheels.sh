#!/bin/bash
set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
        auditwheel repair "$wheel" --plat "$PLAT" -w /io/wheelhouse/
    fi
}

# Clone bullet
git clone --depth 1 --branch 3.08 https://github.com/bulletphysics/bullet3.git
export BULLET_ROOT_DIR="/bullet3"

# Compile wheels
for PYBIN in /opt/python/cp3*/bin; do
    "${PYBIN}/pip" wheel /io/ --no-deps -w wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*.whl; do
    repair_wheel "$whl"
done

# Install packages and test
for PYBIN in /opt/python/cp3*/bin/; do
    "${PYBIN}/pip" install numpy pybullet
    "${PYBIN}/pip" install pybullet_rendering --no-index -f /io/wheelhouse
    (cd "$HOME"; "${PYBIN}/python" -m unittest discover -v /io/tests)
done
