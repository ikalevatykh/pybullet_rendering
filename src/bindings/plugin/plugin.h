#pragma once

#include <render/BaseRenderer.h>

extern void gSetRenderer(const std::shared_ptr<render::BaseRenderer>& renderer,
                         int physicsClientId);

void bindPlugin(py::module& m)
{
    using namespace render;

    // Module-level function
    m.def("set_renderer", //
          [](std::shared_ptr<BaseRenderer>& render, int physicsClientId) {
              gSetRenderer(render, physicsClientId);
          },
          "Set renderer for a specific client");
}
