#pragma once

#include "PyRenderer.h"

extern void gSetRenderer(const std::shared_ptr<render::BaseRenderer>& renderer,
                         int physicsClientId);

void bindRender(py::module& m)
{
    using namespace render;

    py::class_<render::BaseRenderer, PyRenderer, std::shared_ptr<BaseRenderer>>(m, "Renderer")
        .def(py::init<>())
        .def("updateScene", &BaseRenderer::updateScene,
             R"(Update a scene using scene graph description)")
        .def("draw", &BaseRenderer::draw,
             R"(Draw a scene using scene state and view description)");

    m.def("setRenderer", //
          [](std::shared_ptr<BaseRenderer>& render, int physicsClientId) {
              gSetRenderer(render, physicsClientId);
          },
          R"(Set renderer for a specific client)");
}
