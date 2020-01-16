#pragma once

#include <utils/math.h>
#include <utils/serialization.h>

void bindUtils(py::module& m)
{
    // Affine3f
    py::class_<Affine3f>(m, "Affine")
        .def_readonly("origin", &Affine3f::origin, "Origin")
        .def_readonly("quat", &Affine3f::quaternion, "Rotation")
        .def_readonly("scale", &Affine3f::scale, "Scale")
        .def_property_readonly("euler", &Affine3f::euler, "Yaw pitch roll")
        // operators
        .def(py::self == py::self)
        .def(py::self != py::self);

    py::class_<Matrix4f>(m, "Matrix4", py::buffer_protocol())
        .def_buffer([](Matrix4f& m) -> py::buffer_info {
            return py::buffer_info(m.data(), sizeof(float), py::format_descriptor<float>::format(),
                                   2, {4, 4}, {sizeof(float) * 4, sizeof(float)});
        });
}

template <class T>
auto pickle()
{
    return py::pickle(
        [](const T& p) {
            const auto& str = Serialize(p);
            return py::bytes(str);
        },
        [](py::bytes& t) {
            auto p = T();
            if (!Deserialize(&p, t.cast<std::string>()))
                throw std::runtime_error("Invalid state!");
            return p;
        });
}
