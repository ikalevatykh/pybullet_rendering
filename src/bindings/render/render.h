#pragma once

#include "PyRenderer.h"

void bindRender(py::module& m)
{
    using namespace render;

    // Renderer
    py::class_<BaseRenderer, PyRenderer, std::shared_ptr<BaseRenderer>>(m, "BaseRenderer")
        .def(py::init<>())
        .def("update_scene", &BaseRenderer::updateScene,
             "Update a scene using scene graph description")
        .def("render_frame", &BaseRenderer::renderFrame,
             "Render a scene using scene state and view settings");

    // FrameData
    py::class_<FrameData>(m, "FrameData")
        .def_property_readonly(
            "color_img",
            [](FrameData& self) {
                return py::array_t<uint8_t>({self.rows, self.cols, 4}, self.color, py::cast(self));
            },
            py::return_value_policy::reference_internal, "Color image memory buffer")
        .def_property_readonly(
            "depth_img",
            [](FrameData& self) {
                return py::array_t<float>({self.rows, self.cols}, self.depth, py::cast(self));
            },
            py::return_value_policy::reference_internal, "Depth image memory buffer")
        .def_property_readonly(
            "mask_img",
            [](FrameData& self) {
                return py::array_t<int>({self.rows, self.cols}, self.mask, py::cast(self));
            },
            py::return_value_policy::reference_internal, "Mask image memory buffer");
}
