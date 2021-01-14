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
