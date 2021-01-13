#pragma once

#include <render/BaseRenderer.h>

#include <scene/SceneGraph.h>
#include <scene/SceneState.h>
#include <scene/SceneView.h>

class PyRenderer : public render::BaseRenderer
{
  public:
    /* Default constructor */
    PyRenderer() {}

    /**
     * @brief Update a scene using \p sceneGraph description
     *
     * @param sceneGraph - scene description
     * @param materialsOnly - update only shape materials
     */
    void updateScene(const std::shared_ptr<scene::SceneGraph>& sceneGraph,
                     bool materialsOnly) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, render::BaseRenderer, "update_scene", updateScene,
                                    sceneGraph, materialsOnly);
    };

    /**
     * @brief Render a scene at state \p sceneState with a view settings \p sceneView
     *
     * @param sceneState - scene state, e.g. transformations of all objects
     * @param sceneView - view settings, e.g. camera, light, viewport size
     * @param outputFrame - rendered images
     *
     * @return True if rendered
     */
    bool renderFrame(const std::shared_ptr<scene::SceneState>& sceneState,
                     const std::shared_ptr<scene::SceneView>& sceneView,
                     render::FrameData& outputFrame) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, render::BaseRenderer, "render_frame", renderFrame,
                                    sceneState, sceneView, outputFrame);
        return false;
    };
};

/*
Workaround to keep reference to an associated Python object.
Copy-pasted from https://github.com/pybind/pybind11/issues/1546
*/
namespace pybind11::detail {

template <>
struct type_caster<std::shared_ptr<render::BaseRenderer>> {
    PYBIND11_TYPE_CASTER(std::shared_ptr<render::BaseRenderer>, _("render::BaseRenderer"));

    using BaseCaster =
        copyable_holder_caster<render::BaseRenderer, std::shared_ptr<render::BaseRenderer>>;

    bool load(pybind11::handle src, bool b)
    {
        BaseCaster bc;
        bool success = bc.load(src, b);
        if (!success) {
            return false;
        }

        auto py_obj = py::reinterpret_borrow<py::object>(src);
        auto base_ptr = static_cast<std::shared_ptr<render::BaseRenderer>>(bc);

        // Construct a shared_ptr to the py::object
        auto py_obj_ptr = std::shared_ptr<object>{new object{py_obj}, [](auto py_object_ptr) {
                                                      gil_scoped_acquire gil;
                                                      delete py_object_ptr;
                                                  }};

        value = std::shared_ptr<render::BaseRenderer>(py_obj_ptr, base_ptr.get());
        return true;
    }

    static handle cast(std::shared_ptr<render::BaseRenderer> base, return_value_policy rvp,
                       handle h)
    {
        return BaseCaster::cast(base, rvp, h);
    }
};

template <>
struct is_holder_type<render::BaseRenderer, std::shared_ptr<render::BaseRenderer>>
    : std::true_type {
};
} // namespace pybind11::detail
