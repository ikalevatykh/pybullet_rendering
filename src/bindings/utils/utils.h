#pragma once

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
        .def_property_readonly("matrix", &Affine3f::matrix, "Homogenous transform matrix 4x4")
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
