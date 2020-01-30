#pragma once

#include <scene/SceneState.h>

void bindSceneState(py::module& m)
{
    using namespace scene;

    // SceneState
    py::class_<SceneState>(m, "SceneState")
        .def("pose", &SceneState::pose, "Node pose", py::return_value_policy::reference_internal)
        .def("matrix", [](SceneState* self, int uid) { return self->pose(uid).matrix(); },
             "Node transformation matrix 4x4", py::return_value_policy::reference_internal)
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneState>());
}
