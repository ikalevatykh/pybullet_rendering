#pragma once

#include <scene/SceneState.h>

void bindSceneState(py::module& m)
{
    using namespace scene;

    // SceneState
    py::class_<SceneState, std::shared_ptr<SceneState>>(m, "SceneState")
        .def("pose", &SceneState::pose, "Node pose", py::return_value_policy::reference_internal)
        .def(
            "matrix",
            [](const SceneState& self, int uid) {
                return py::array_t<float>({ssize_t(4), ssize_t(4)}, self.pose(uid).matrix().data());
            },
            "Node transformation matrix 4x4")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self)
        // pickle
        .def(pickle<SceneState>());
}
