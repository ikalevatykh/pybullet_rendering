
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
namespace py = pybind11;
using namespace py::literals;

#include "utils/utils.h"
#include "plugin/plugin.h"
#include "render/render.h"
#include "scene/scene.h"

#include <map>
#include <vector>

PYBIND11_MODULE(bindings, m)
{
    m.doc() = "PyBullet rendering plugin binding";

    bindUtils(m);
    bindRender(m);
    bindScene(m);
    bindPlugin(m);
}
