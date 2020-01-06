#pragma once

#include <scene/SceneState.h>

void bindSceneState(py::module& m)
{
    using namespace scene;

    py::class_<SceneState>(m, "SceneState")
        .def("pose", py::overload_cast<int>(&SceneState::pose, py::const_), R"(Node pose)",
             py::return_value_policy::reference_internal);
}
