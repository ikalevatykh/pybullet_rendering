#pragma once

#include <pybind11/numpy.h>

#include <utils/math.h>
#include <utils/serialization.h>

void bindUtils(py::module& m)
{
    // Affine3f
    py::class_<Affine3f>(m, "Affine")
        .def_readonly("origin", &Affine3f::origin, "Origin")
        .def_readonly("quat", &Affine3f::quat, "Rotation")
        .def_readonly("scale", &Affine3f::scale, "Scale")
        .def_property_readonly("euler", &Affine3f::euler, "Yaw pitch roll")
        .def_property_readonly(
            "matrix",
            [](const Affine3f& self) {
                return py::array_t<float>({ssize_t(4), ssize_t(4)}, self.matrix().data());
            },
            "Homogenous transform matrix 4x4")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

}

template <class T>
auto pickle()
{
    return py::pickle(
        [](const T& p) { //
            return py::bytes(BinarySerialize(p));
        },
        [](py::bytes& t) {
            auto p = T();
            BinaryDeserialize(t.cast<std::string>(), p);
            return p;
        });
}
